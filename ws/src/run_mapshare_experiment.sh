#!/usr/bin/env bash
# Host-side orchestrator for the MULTI-ROBOT MAP-SHARING BANDWIDTH experiment.
#
# Derived from run_fused_experiment.sh. Same base stack (EKF + NDT + static
# extrinsics + seg), same fused LiDAR+RGB-D scovox params — the ONLY differences:
#   (a) N scovox mappers instead of 1, each namespaced /robotK, each in
#       mode:=rolling (creates bin_pub_) and integration_frame:=rK_map (so the
#       merger keys them as N distinct sources — dscovox keys per header.frame_id),
#   (b) N identity  map->rK_map  static TFs (rK_map == map; lets the merger fold
#       every source back into the common map frame),
#   (c) a dscovox_mapping_node MERGER, started FIRST (the scovox_bin publish is
#       subscriber-gated: no subscriber => deltas are drained, never sent),
#   (d) ros2 topic bw / hz probes + an optional bag record of the scovox_bin
#       streams — THE map-sharing channel and the only thing to measure.
#
# What is measured (see ws/src/map_share_bandwidth_experiment.md for the full plan):
#   The LZ4-compressed ScovoxMapBinary delta stream on /robotK/scovox_node/scovox_bin.
#   ros2 topic bw reports serialized bytes/s ~= the exact bytes a robot-to-robot
#   radio link would carry (transport here is SHM/loopback, so a NIC tcpdump sees
#   nothing — ros2 topic bw is the correct transport-independent figure).
#
#   Per-frame wire budget (pre-LZ4, K_TOP=2, share_tsdf=false):
#     S_raw = 28 + 20*N_beta + 28*N_dir  bytes/frame  (+20*N_tsdf if share_tsdf).
#
# Usage (HOST; needs docker, NOT ROS):
#   ./run_mapshare_experiment.sh [playback_duration_s]        # empty = full bag (~500 s)
#   NROBOTS=2 ./run_mapshare_experiment.sh 120                # 2-robot, first 120 s
#
# Sweep knobs (env vars; defaults reproduce the shipped fused pipeline = baseline A0):
#   NROBOTS=2            # 1..3 robots. 2+ = duplicate-source (byte-identical deltas):
#                        #   clean N x BW_1 upper bound. WARNING: each robot is a full
#                        #   fused mapper — N mappers = N x CPU/GPU on one host.
#   RESOLUTION=0.10      # B1: voxel size (m). Dominant knob (~1/res^2..1/res^3).
#   DOWNSAMPLE=0.1       # B2: per-scan downsample_voxel_size (m). 0 = off.
#   CARVE_BAND=-1.0      # B3: free-space carve length (m) before each hit; <=0 = full-ray.
#   MAX_RANGE=20.0       # B4: LiDAR max_range (m).
#   SHARE_TSDF=0         # B5: 1 => share_tsdf:=true + enable_tsdf:=true (~2x geometry).
#   NUM_CLASSES=14       # B7: Dirichlet dimension (header-only; ~no BW effect. control).
#   SEMANTIC_TOP_K=2     # B6: wire K_TOP is COMPILE-LOCKED; this only clamps <= built K_TOP.
#                        #   To truly sweep K_TOP you must rebuild scovox_core. (see plan B6)
#   KERNEL_L=0.4         # RGB-D->LiDAR BKI spread radius (m); matches run_fused.
#   MODEL=<hf-id>        # override seg model_name (matches run_fused).
#   LIDAR_ONLY=1         # occupancy-only arm: no seg / no camera topics (Beta stream only).
#   RECORD=1             # also `ros2 bag record` the scovox_bin streams (per-message sizes).
#   COVERAGE_SPLIT=1     # stagger robot2's mapper by STAGGER_S so deltas diverge (realistic,
#                        #   sub-linear aggregate) instead of duplicate-source. NROBOTS>=2.
#   STAGGER_S=40         # robot>=2 mapper start delay (s) under COVERAGE_SPLIT.
#   RVIZ=1               # open RViz on the merged /dscovox/pointcloud and leave nodes up.
#
# After the bag finishes the script prints per-robot ros2 topic bw / hz averages and,
# if RECORD=1, the recorded scovox_bin bag size. Compute r_LZ4 = mean(len(data))/S_raw
# from beta_count/dir_count (grep the mapper logs) offline.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOC_DIR="$HERE/hmr_localisation"
SEG_DIR="$HERE/scovox/src/seg_pipeline"
SCOVOX_DIR="$HERE/scovox"
BAG="/ws/bags/2026_06_19_18_19_06__kalhan-map-test-2_"
DUR="${1:-}"
DUR_ARG=""; [ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

# --- experiment knobs (defaults = run_fused_experiment.sh shipped values) ---
NROBOTS="${NROBOTS:-2}"
RESOLUTION="${RESOLUTION:-0.10}"
DOWNSAMPLE="${DOWNSAMPLE:-0.1}"
CARVE_BAND="${CARVE_BAND:--1.0}"
MAX_RANGE="${MAX_RANGE:-20.0}"
SHARE_TSDF="${SHARE_TSDF:-0}"
NUM_CLASSES="${NUM_CLASSES:-14}"
SEMANTIC_TOP_K="${SEMANTIC_TOP_K:-2}"
KERNEL_L="${KERNEL_L:-0.4}"
MODEL="${MODEL:-}"
LIDAR_ONLY="${LIDAR_ONLY:-}"
RECORD="${RECORD:-1}"
COVERAGE_SPLIT="${COVERAGE_SPLIT:-0}"
STAGGER_S="${STAGGER_S:-40}"
RVIZ="${RVIZ:-}"
RVIZ_CFG="$HERE/seg_experiment.rviz"

case "$NROBOTS" in 1|2|3) ;; *) echo "NROBOTS must be 1..3 (got $NROBOTS)"; exit 2;; esac
# share_tsdf must co-enable the local TSDF grid (you can't ship a TSDF you never built).
if [ "$SHARE_TSDF" = "1" ]; then STSDF=true;  ETSDF=true; else STSDF=false; ETSDF=false; fi

dc_loc()    { docker compose -f "$LOC_DIR/compose.yaml"    "$@"; }
dc_seg()    { docker compose -f "$SEG_DIR/compose.yaml"    "$@"; }
dc_scovox() { docker compose -f "$SCOVOX_DIR/compose.yaml" "$@"; }

# scovox-container ros2 preamble (shared by every exec below)
SRC='source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash'

# build the merger input-topics YAML list + the bag-record topic list
INPUT_TOPICS="["; REC_TOPICS=""
for i in $(seq 1 "$NROBOTS"); do
  t="/robot$i/scovox_node/scovox_bin"
  INPUT_TOPICS="$INPUT_TOPICS'$t'"; [ "$i" -lt "$NROBOTS" ] && INPUT_TOPICS="$INPUT_TOPICS,"
  REC_TOPICS="$REC_TOPICS $t"
done
INPUT_TOPICS="$INPUT_TOPICS]"

LEAVE_UP=""
stop_cmds() {
  echo "    docker compose -f $SCOVOX_DIR/compose.yaml stop scovox"   # kills mappers+merger+bw+RViz
  echo "    docker compose -f $SEG_DIR/compose.yaml stop seg"
  echo "    docker compose -f $LOC_DIR/compose.yaml stop ros"
}
cleanup() {
  if [ -n "$LEAVE_UP" ]; then
    echo "[orch] leaving nodes up. stop everything with:"; stop_cmds; return 0
  fi
  echo "[orch] stopping nodes (docker compose stop)…"
  dc_scovox stop scovox 2>/dev/null || true
  dc_seg    stop seg    2>/dev/null || true
  dc_loc    stop ros    2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[orch] map-sharing bandwidth run: NROBOTS=$NROBOTS res=$RESOLUTION ds=$DOWNSAMPLE carve=$CARVE_BAND max_range=$MAX_RANGE share_tsdf=$STSDF split=${COVERAGE_SPLIT}"
echo "[orch] bringing up containers…"
dc_loc up -d
if [ -z "$LIDAR_ONLY" ]; then dc_seg up -d; fi
dc_scovox up -d
if [ -n "$RVIZ" ]; then xhost +local:root >/dev/null 2>&1 || true; fi

# kill any stray bag play left over from manual probing
dc_loc exec -T ros bash -lc 'pkill -f "[r]os2 bag play" 2>/dev/null; sleep 1; true' || true

# 1) localizer stack (EKF + NDT map->odom + 3 static extrinsics) — verbatim from run_fused.
echo "[orch] launching EKF + NDT (map->odom) + static extrinsics in hmr_loc…"
dc_loc exec -d ros bash -lc '
  source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml
  ros2 launch /ws/launch/ekf_odom.launch.py use_sim_time:=true > /tmp/ekf.log 2>&1 &
  ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
    localization_param_dir:=/ws/config/gt_ouster_ndt_tree_realtime.yaml \
    cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
    global_frame_id:=map odom_frame_id:=odom base_frame_id:=base_link \
    use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
    publish_lidar_tf:=false publish_imu_tf:=false \
    > /tmp/ndt.log 2>&1 &
  ros2 run tf2_ros static_transform_publisher \
    --x 0.1105 --y 0.0 --z 0.404 --qx 0.0 --qy 0.0 --qz 1.0 --qw 0.0 \
    --frame-id base_link --child-frame-id os_lidar > /tmp/lidartf.log 2>&1 &
  ros2 run tf2_ros static_transform_publisher \
    --x 0.062 --y 0.0 --z 0.015 --qx 0.0 --qy 0.0 --qz 0.7071068 --qw 0.7071068 \
    --frame-id base_link --child-frame-id imu > /tmp/imutf.log 2>&1 &
  ros2 run tf2_ros static_transform_publisher \
    --x 0.270676 --y 0.049297 --z 0.279109 \
    --qx -0.013514 --qy 0.000986 --qz 0.000780 --qw 0.999908 \
    --frame-id base_link --child-frame-id camera_color_frame > /tmp/camtf.log 2>&1 &
  wait
'

# 1b) N identity  map->rK_map  static TFs so the merger folds each source into map.
echo "[orch] publishing $NROBOTS identity map->rK_map static TFs…"
IDENT_CMDS=""
for i in $(seq 1 "$NROBOTS"); do
  IDENT_CMDS="$IDENT_CMDS ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id r${i}_map > /tmp/r${i}maptf.log 2>&1 & "
done
dc_loc exec -d ros bash -lc "source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; $IDENT_CMDS wait"

echo "[orch] waiting for NDT map load + activation…"
dc_loc exec -T ros bash -lc 'for i in $(seq 1 60); do grep -aq "Activating end" /tmp/ndt.log 2>/dev/null && { echo NDT_ACTIVE; exit 0; }; sleep 1; done; echo NDT_TIMEOUT; tail -5 /tmp/ndt.log'

# 2) online seg node (default output_frame=camera_color_frame) — verbatim from run_fused.
if [ -z "$LIDAR_ONLY" ]; then
  echo "[orch] launching seg node in hmr_seg${MODEL:+ (model=$MODEL)}…"
  dc_seg exec -d seg bash -lc "
    source /opt/ros/jazzy/setup.bash
    cd /seg && exec python3 -m seg_pipeline.seg_node --ros-args -p use_sim_time:=true ${MODEL:+-p model_name:=$MODEL} > /root/seg.log 2>&1
  "
else
  echo "[orch] LIDAR_ONLY=1 → occupancy-only (no seg node; Beta stream only)."
fi

# 3) MERGER FIRST (dscovox). scovox_bin publish is subscriber-gated: bring the sink up
#    before the sources so no early delta is drained. It subscribes to all N streams.
echo "[orch] launching dscovox merger (input_topics=$INPUT_TOPICS)…"
dc_scovox exec -d -e IT="$INPUT_TOPICS" scovox bash -lc "$SRC
  exec ros2 run scovox_mapping dscovox_mapping_node --ros-args -r __node:=dscovox_node \
    -p use_sim_time:=true -p map_frame:=map \
    -p \"input_topics:=\$IT\" \
    -p pointcloud_topic:=/dscovox/pointcloud \
    -p semantic_top_k:=$SEMANTIC_TOP_K \
    > /tmp/dscovox.log 2>&1"

# 4) N fused rolling mappers. Params = run_fused's fused block, with ONLY:
#    __ns:=/robotK, mode:=rolling, integration_frame:=rK_map, + the swept knobs.
launch_mapper() {
  local i="$1"
  echo "[orch] launching mapper robot$i (ns=/robot$i, integration_frame=r${i}_map, mode=rolling)…"
  dc_scovox exec -d \
    -e RNS="/robot$i" -e IFRAME="r${i}_map" -e LOGF="/tmp/scovox_r$i.log" \
    -e RES="$RESOLUTION" -e DS="$DOWNSAMPLE" -e MRANGE="$MAX_RANGE" -e CB="$CARVE_BAND" \
    -e KL="$KERNEL_L" -e NC="$NUM_CLASSES" -e STK="$SEMANTIC_TOP_K" \
    -e STSDF="$STSDF" -e ETSDF="$ETSDF" \
    scovox bash -lc "$SRC"'
    exec ros2 run scovox_mapping scovox_mapping_node --ros-args -r __ns:=$RNS -r __node:=scovox_node \
      -p use_sim_time:=true \
      -p fuse_lidar_rgbd:=true \
      -p depth_topic:=/scovox/depth/image_raw \
      -p depth_info_topic:=/scovox/depth/camera_info \
      -p seg_topic:=/scovox/segmentation/colored \
      -p input_pointcloud_topic:=/ouster/points \
      -p imu_topic:=/imu/data \
      -p dataset_mode:=false \
      -p integration_frame:=$IFRAME -p map_frame:=map -p base_frame:=base_link \
      -p lidar_base_frame:=os_lidar -p rgbd_base_frame:=camera_color_frame \
      -p lidar_w_occ:=8.0 -p lidar_w_free:=4.0 \
      -p rgbd_w_occ:=0.0 -p rgbd_w_free:=0.0 -p rgbd_geometry_off:=true \
      -p rgbd_dirichlet_min_p_occ:=0.55 \
      -p rgbd_kernel_radius:=$KL \
      -p resolution:=$RES -p carve_band:=$CB \
      -p num_classes:=$NC -p max_semantic_classes:=$NC -p semantic_top_k:=$STK \
      -p semantic_mode:=dirichlet -p dirichlet_prior:=0.01 \
      -p mode:=rolling -p enable_tsdf:=$ETSDF -p share_tsdf:=$STSDF \
      -p min_depth:=0.3 -p max_depth:=6.0 \
      -p min_range:=1.0 -p max_range:=$MRANGE -p range_decay_length:=-1.0 \
      -p deskew_mode:=auto -p downsample_voxel_size:=$DS \
      -p tf_lookup_timeout_sec:=1.0 -p tf_require_exact:=true \
      -p startup_tf_stable_sec:=0.0 -p startup_tf_jump_threshold:=10.0 -p runtime_tf_gate:=false \
      -p "semantic_color_map_keys:=[8405120,15999976,10025880,4605510,7048739,10066329,16427550,14423100,16711680,142,4620980,12491161,6710940,0]" \
      -p "semantic_color_map_classes:=[1,2,3,4,5,6,7,8,9,10,11,12,13,0]" \
      > $LOGF 2>&1'
}

# robot1 always starts now; robot>=2 optionally staggered for a coverage-split run.
launch_mapper 1
for i in $(seq 2 "$NROBOTS"); do
  if [ "$COVERAGE_SPLIT" = "1" ]; then
    ( sleep "$STAGGER_S"; launch_mapper "$i" ) &
    echo "[orch] robot$i mapper will start after ${STAGGER_S}s (COVERAGE_SPLIT)."
  else
    launch_mapper "$i"
  fi
done

# 5) bandwidth + rate probes on each scovox_bin (the map-sharing channel).
echo "[orch] starting ros2 topic bw / hz probes on the scovox_bin streams…"
for i in $(seq 1 "$NROBOTS"); do
  dc_scovox exec -d -e T="/robot$i/scovox_node/scovox_bin" scovox bash -lc "$SRC; exec ros2 topic bw \$T > /tmp/bw_r$i.log 2>&1"
  dc_scovox exec -d -e T="/robot$i/scovox_node/scovox_bin" scovox bash -lc "$SRC; exec ros2 topic hz \$T > /tmp/hz_r$i.log 2>&1"
done
if [ "$RECORD" = "1" ]; then
  echo "[orch] recording scovox_bin streams (AUTHORITATIVE wire measurement):$REC_TOPICS"
  # NOT detached-exec: keep the record PID so we can finalize with SIGINT later
  # (a SIGKILL'd rosbag has no footer/metadata.yaml and needs a manual reindex).
  dc_scovox exec -d scovox bash -lc "$SRC; rm -rf /tmp/binbag
    ros2 bag record -o /tmp/binbag$REC_TOPICS > /tmp/rec.log 2>&1 & echo \$! > /tmp/rec.pid; wait"
fi

# 5b) optional RViz on the merged cloud.
if [ -n "$RVIZ" ]; then
  echo "[orch] launching RViz inside the scovox container…"
  docker cp "$RVIZ_CFG" scovox:/tmp/seg_experiment.rviz 2>/dev/null || true
  GLENV='export __NV_PRIME_RENDER_OFFLOAD=1; export __GLX_VENDOR_LIBRARY_NAME=nvidia'
  [ "${SOFTGL:-0}" = "1" ] && GLENV='export LIBGL_ALWAYS_SOFTWARE=1; unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD'
  dc_scovox exec -d scovox bash -lc "$SRC
    export DISPLAY=\"\${DISPLAY:-:1}\"; $GLENV
    exec rviz2 -d /tmp/seg_experiment.rviz --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1"
fi

echo "[orch] letting merger + mappers + probes subscribe (8 s)…"
sleep 8

# 6) play the bag (identical to run_fused).
CAM_TOPICS="/camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info /camera/color/image_raw/compressed"
[ -n "$LIDAR_ONLY" ] && CAM_TOPICS=""
echo "[orch] playing bag ${DUR:+(first ${DUR}s)}${LIDAR_ONLY:+ [LIDAR_ONLY]}…"
dc_loc exec -T ros bash -lc "
  source /opt/ros/jazzy/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml
  ros2 bag play $BAG --clock --rate 1.0 $DUR_ARG \
    --topics /ouster/points /imu/data $CAM_TOPICS \
    --qos-profile-overrides-path /ws/config/ouster_reliable_qos.yaml
"

# 6b) finalize the recorder BEFORE teardown so rosbag2 writes metadata.yaml and the
#     bag is directly readable. NOTE: for a backgrounded `ros2 bag record` under the
#     ros2 CLI wrapper, SIGINT does NOT finalize (verified) — SIGTERM does. A SIGKILL'd
#     bag has no footer → needs a manual reindex. This is a targeted recorder finalize,
#     NOT an experiment teardown (teardown still = docker compose stop).
if [ "$RECORD" = "1" ]; then
  echo "[orch] finalizing recorder (SIGTERM)…"
  dc_scovox exec -T scovox bash -lc '
    kill -TERM $(cat /tmp/rec.pid 2>/dev/null) 2>/dev/null || true
    for i in $(seq 1 30); do [ -f /tmp/binbag/metadata.yaml ] && { echo REC_FINALIZED; break; }; sleep 1; done' || true
fi

# 7) results.
echo
echo "================ MAP-SHARING BANDWIDTH RESULTS ================"
echo "config: NROBOTS=$NROBOTS res=$RESOLUTION ds=$DOWNSAMPLE carve_band=$CARVE_BAND max_range=$MAX_RANGE share_tsdf=$STSDF top_k=$SEMANTIC_TOP_K classes=$NUM_CLASSES split=$COVERAGE_SPLIT lidar_only=${LIDAR_ONLY:-0}"
if [ "$RECORD" = "1" ]; then
  echo "--- AUTHORITATIVE wire measurement (recorded bag; reliable QoS catches every sample) ---"
  dc_scovox exec -T scovox bash -lc "$SRC"'
cat > /tmp/bagstats.py <<"PYEOF"
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import statistics, collections
r = SequentialReader()
r.open(StorageOptions(uri="/tmp/binbag", storage_id="mcap"), ConverterOptions("", ""))
by = collections.defaultdict(list)
while r.has_next():
    topic, data, t = r.read_next()
    by[topic].append((t, len(data)))
grand = 0.0
for topic, msgs in sorted(by.items()):
    msgs.sort()
    sizes = [s for _, s in msgs]
    n = len(sizes); tot = sum(sizes)
    span = max((msgs[-1][0] - msgs[0][0]) / 1e9, 1e-9)
    deltas = sizes[1:] or sizes
    bps = tot / span; grand += bps
    print(f"  {topic}")
    print(f"    msgs={n}  rate={n/span:.2f} Hz  total={tot/1e6:.1f} MB over {span:.1f} s")
    print(f"    WIRE = {bps/1e6:.2f} MB/s  ({bps*8/1e6:.1f} Mbps)")
    print(f"    snapshot(msg0)={sizes[0]/1e6:.2f} MB | delta mean={statistics.mean(deltas)/1e6:.2f} MB  min={min(deltas)/1e3:.0f} KB  max={max(deltas)/1e6:.2f} MB")
if len(by) > 1:
    print(f"  AGGREGATE fleet wire = {grand/1e6:.2f} MB/s ({grand*8/1e6:.1f} Mbps)")
PYEOF
python3 /tmp/bagstats.py 2>/dev/null || echo "  (bag not finalized — run: ros2 bag reindex -s mcap /tmp/binbag)"' || true
fi
echo "--- indicative live probes (ros2 topic bw/hz are BEST-EFFORT: they DROP big samples, so they UNDER-report) ---"
for i in $(seq 1 "$NROBOTS"); do
  dc_scovox exec -T scovox bash -lc "tail -2 /tmp/bw_r$i.log 2>/dev/null | tr '\n' ' ' | sed 's/^/  bw[r$i]  /'; echo; grep -aE 'average rate' /tmp/hz_r$i.log 2>/dev/null | tail -1 | sed 's/^/  hz[r$i]  /'" || true
done
echo "note: trust WIRE MB/s from the recorded bag; DISCARD msg0 (new-subscriber snapshot) for steady-state."
echo "      per-frame budget check: delta mean vs 28+20*N_beta+28*N_dir (grep beta_count/dir_count in /tmp/scovox_r*.log)."
echo "=============================================================="

# tear down (unless RVIZ). Use docker compose stop — never pkill.
if [ -n "$RVIZ" ]; then LEAVE_UP=1; fi
# cleanup() runs on EXIT
