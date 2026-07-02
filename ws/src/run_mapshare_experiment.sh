#!/usr/bin/env bash
# Host-side orchestrator for the DISTRIBUTED MULTI-ROBOT MAPPING experiment:
# every robot runs its own scovox mapper AND its own dscovox merger, the robots
# exchange scovox_bin delta streams, and each robot's dscovox becomes that
# robot's copy of the GLOBAL map. One run both (i) tests that topology
# end-to-end (post-run FUSION VERIFICATION, see step 8) and (ii) measures the
# map-sharing bandwidth (this script's original purpose).
#
# Derived from run_fused_experiment.sh. Same base stack (EKF + NDT + static
# extrinsics + seg), same fused LiDAR+RGB-D scovox params — the differences:
#   (a) N scovox mappers instead of 1, each namespaced /robotK, each in
#       mode:=rolling (creates bin_pub_) and integration_frame:=rK_map (so the
#       mergers key them as N distinct sources — dscovox keys per header.frame_id),
#   (b) N identity  map->rK_map  static TFs (rK_map == map; lets each merger fold
#       every source back into the common map frame),
#   (c) N dscovox_mapping_node MERGERS — one per robot namespace, each fusing
#       ALL robots' streams (the real fleet topology; no central merger) —
#       started FIRST (the scovox_bin publish is subscriber-gated: no
#       subscriber => deltas are drained, never sent),
#   (d) ros2 topic bw / hz probes + an optional bag record of the scovox_bin
#       streams — THE map-sharing channel and the bandwidth measurement,
#   (e) SPLIT_BAND (default on; needs NROBOTS>=2): each robot SHARES only a
#       disjoint z-slice of [SPLIT_ZMIN,SPLIT_ZMAX] (it still maps everything
#       locally), so any voxel a robot's fused map holds inside a PEER's slice
#       can ONLY have arrived over the wire from that peer — step 8 queries
#       every /robotK/dscovox_node/get_region and asserts exactly that.
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
#   ./run_mapshare_experiment.sh 120                # 2-robot distributed-mapping test, 120 s
#   SPLIT_BAND=0 ./run_mapshare_experiment.sh 120   # bandwidth baseline (full-map shares)
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
#   SHARE_RATE_HZ=0.0    # C1: >0 = timer-coalesced scovox_bin publish at this rate (Hz);
#                        #   <=0 = legacy per-scan publish inline in the sensor callbacks.
#   SHARE_CHANGE_GATE=1  # C2: 1 = per-voxel change gate vs LAST-EMITTED wire state (default);
#                        #   0 = legacy wire (every touched voxel re-sent every publish).
#   SHARE_GATE_P_EPS=0.02  # C2a: |dp_occ| re-emit threshold.
#   SHARE_GATE_EV_REL=0.10 # C2b: relative evidence-growth re-emit threshold.
#   SHARE_Z_MIN=0.0      # C3: shared-ROI z-band (m); min>=max = off. Applied on the sender
#   SHARE_Z_MAX=0.0      #   wire AND mirrored as the dscovox ingest clip. Must be a
#                        #   SUPERSET of explo_planner roi_min_z/roi_max_z (see the
#                        #   KEEP IN SYNC comments in exploration_params.yaml).
#                        #   Only honored when SPLIT_BAND=0 (SPLIT_BAND assigns the bands).
#   SPLIT_BAND=1         # fusion-proof mode (default; forced off when NROBOTS=1): robot K
#                        #   SHARES only slice K of [SPLIT_ZMIN,SPLIT_ZMAX]; the mergers
#                        #   ingest-clip to the full band. Peer-slice voxels in a robot's
#                        #   fused map == proven cross-robot fusion (checked in step 8).
#   SPLIT_ZMIN=-0.5      # full shared band carved into NROBOTS sender slices under
#   SPLIT_ZMAX=2.0       #   SPLIT_BAND (defaults = the real-robot band in scovox_robot_share.yaml).
#   KERNEL_L=0.4         # RGB-D->LiDAR BKI spread radius (m); matches run_fused.
#   MODEL=<hf-id>        # override seg model_name (matches run_fused).
#   LIDAR_ONLY=1         # occupancy-only arm: no seg / no camera topics (Beta stream only).
#   RECORD=1             # also `ros2 bag record` the scovox_bin streams (per-message sizes).
#   COVERAGE_SPLIT=1     # stagger robot2's mapper by STAGGER_S so deltas diverge (realistic,
#                        #   sub-linear aggregate) instead of duplicate-source. NROBOTS>=2.
#   STAGGER_S=40         # robot>=2 mapper start delay (s) under COVERAGE_SPLIT.
#   RVIZ=1               # open RViz on robot1's fused /robot1/dscovox_node/pointcloud; leave nodes up.
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
SHARE_RATE_HZ="${SHARE_RATE_HZ:-0.0}"
SHARE_CHANGE_GATE="${SHARE_CHANGE_GATE:-1}"
SHARE_GATE_P_EPS="${SHARE_GATE_P_EPS:-0.02}"
SHARE_GATE_EV_REL="${SHARE_GATE_EV_REL:-0.10}"
SHARE_Z_MIN="${SHARE_Z_MIN:-0.0}"
SHARE_Z_MAX="${SHARE_Z_MAX:-0.0}"
SPLIT_BAND="${SPLIT_BAND:-1}"
SPLIT_ZMIN="${SPLIT_ZMIN:--0.5}"
SPLIT_ZMAX="${SPLIT_ZMAX:-2.0}"
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
if [ "$SHARE_CHANGE_GATE" = "1" ]; then SGATE=true; else SGATE=false; fi
if [ "$NROBOTS" = "1" ] && [ "$SPLIT_BAND" = "1" ]; then
  echo "[orch] NROBOTS=1 -> no peer to prove fusion against; SPLIT_BAND off."
  SPLIT_BAND=0
fi
# Per-robot SENDER z-band + the mergers' ingest clip. SPLIT_BAND carves the
# full band into NROBOTS disjoint sender slices (mergers clip to the union);
# otherwise every robot shares SHARE_Z_MIN/MAX and the mergers mirror it.
sender_band() {  # $1 = robot index -> "zmin zmax"
  if [ "$SPLIT_BAND" = "1" ]; then
    awk -v i="$1" -v n="$NROBOTS" -v a="$SPLIT_ZMIN" -v b="$SPLIT_ZMAX" \
      'BEGIN{w=(b-a)/n; printf "%.3f %.3f", a+(i-1)*w, a+i*w}'
  else
    echo "$SHARE_Z_MIN $SHARE_Z_MAX"
  fi
}
if [ "$SPLIT_BAND" = "1" ]; then MERGE_Z_MIN="$SPLIT_ZMIN"; MERGE_Z_MAX="$SPLIT_ZMAX"
else MERGE_Z_MIN="$SHARE_Z_MIN"; MERGE_Z_MAX="$SHARE_Z_MAX"; fi

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

echo "[orch] distributed-mapping run: NROBOTS=$NROBOTS res=$RESOLUTION ds=$DOWNSAMPLE carve=$CARVE_BAND max_range=$MAX_RANGE share_tsdf=$STSDF split=${COVERAGE_SPLIT} share_rate_hz=$SHARE_RATE_HZ change_gate=$SGATE split_band=$SPLIT_BAND z_band=[$SHARE_Z_MIN,$SHARE_Z_MAX]"
echo "[orch] bringing up containers…"
dc_loc up -d
if [ -z "$LIDAR_ONLY" ]; then dc_seg up -d; fi
dc_scovox up -d
if [ -n "$RVIZ" ]; then xhost +local:root >/dev/null 2>&1 || true; fi

# kill any stray bag play left over from manual probing + clear STALE /tmp logs
# from earlier runs (compose stop preserves container /tmp; the NDT wait below
# and the step-8 verification grep these files — leftovers make both lie:
# a stale "Activating end" once green-lit a run whose NDT never activated).
dc_loc exec -T ros bash -lc 'pkill -f "[r]os2 bag play" 2>/dev/null;
  rm -f /tmp/ndt.log /tmp/ekf.log /tmp/lidartf.log /tmp/imutf.log /tmp/camtf.log /tmp/r*maptf.log; sleep 1; true' || true
dc_scovox exec -T scovox bash -lc 'rm -f /tmp/dscovox_r*.log /tmp/scovox_r*.log /tmp/bw_r*.log /tmp/hz_r*.log /tmp/rec.log /tmp/rec.pid' || true

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
# The launch file drives configure->activate via OnStateTransition events; if
# the launch process misses the node's configuring->inactive transition event
# (startup race), ACTIVATE is never emitted and the node sits inactive forever.
# After 15 s stuck at "Configuring end", nudge the lifecycle ourselves.
# ABORT the run on timeout — without map->odom every mapper drops every scan.
dc_loc exec -T ros bash -lc '
  source /opt/ros/jazzy/setup.bash
  for i in $(seq 1 90); do
    grep -aq "Activating end" /tmp/ndt.log 2>/dev/null && { echo NDT_ACTIVE; exit 0; }
    if [ "$i" -gt 15 ] && grep -aq "Configuring end" /tmp/ndt.log 2>/dev/null; then
      ros2 lifecycle set /lidar_localization activate >/dev/null 2>&1 || true
    fi
    sleep 1
  done
  echo NDT_TIMEOUT; tail -5 /tmp/ndt.log; exit 1' \
  || { echo "[orch] NDT never activated — aborting run."; exit 1; }

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

# 3) MERGERS FIRST — one dscovox PER ROBOT (the real fleet topology: every robot
#    fuses all peers' streams into its own copy of the global map; no central
#    merger). scovox_bin publish is subscriber-gated: bring the sinks up before
#    the sources so no early delta is drained. Base params come from the shipped
#    dscovox_params.yaml; only the experiment knobs are overridden on top.
DSCOVOX_PARAMS=/scovox/src/scovox_mapping/config/dscovox_params.yaml
for i in $(seq 1 "$NROBOTS"); do
  echo "[orch] launching dscovox merger robot$i (ns=/robot$i, input_topics=$INPUT_TOPICS)…"
  dc_scovox exec -d -e IT="$INPUT_TOPICS" -e RNS="/robot$i" -e LOGF="/tmp/dscovox_r$i.log" \
    scovox bash -lc "$SRC
    exec ros2 run scovox_mapping dscovox_mapping_node --ros-args \
      -r __ns:=\$RNS -r __node:=dscovox_node \
      --params-file $DSCOVOX_PARAMS \
      -p use_sim_time:=true \
      -p \"input_topics:=\$IT\" \
      -p semantic_top_k:=$SEMANTIC_TOP_K \
      -p share_roi_z_min:=$MERGE_Z_MIN -p share_roi_z_max:=$MERGE_Z_MAX \
      > \$LOGF 2>&1"
done

# 4) N fused rolling mappers. Params = run_fused's fused block, with ONLY:
#    __ns:=/robotK, mode:=rolling, integration_frame:=rK_map, + the swept knobs.
launch_mapper() {
  local i="$1" bzmin bzmax
  read -r bzmin bzmax <<<"$(sender_band "$i")"
  echo "[orch] launching mapper robot$i (ns=/robot$i, integration_frame=r${i}_map, mode=rolling, share_z=[$bzmin,$bzmax])…"
  dc_scovox exec -d \
    -e RNS="/robot$i" -e IFRAME="r${i}_map" -e LOGF="/tmp/scovox_r$i.log" \
    -e RES="$RESOLUTION" -e DS="$DOWNSAMPLE" -e MRANGE="$MAX_RANGE" -e CB="$CARVE_BAND" \
    -e KL="$KERNEL_L" -e NC="$NUM_CLASSES" -e STK="$SEMANTIC_TOP_K" \
    -e STSDF="$STSDF" -e ETSDF="$ETSDF" \
    -e SRHZ="$SHARE_RATE_HZ" -e SGATE="$SGATE" \
    -e SPEPS="$SHARE_GATE_P_EPS" -e SEREL="$SHARE_GATE_EV_REL" \
    -e SZMIN="$bzmin" -e SZMAX="$bzmax" \
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
      -p share_rate_hz:=$SRHZ -p share_change_gate:=$SGATE \
      -p share_gate_p_eps:=$SPEPS -p share_gate_evidence_rel:=$SEREL \
      -p share_roi_z_min:=$SZMIN -p share_roi_z_max:=$SZMAX \
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

# 5b) optional RViz on robot1's fused cloud (per-robot mergers publish ~/pointcloud).
if [ -n "$RVIZ" ]; then
  echo "[orch] launching RViz inside the scovox container…"
  docker cp "$RVIZ_CFG" scovox:/tmp/seg_experiment.rviz 2>/dev/null || true
  dc_scovox exec -T scovox bash -lc \
    "sed -i 's#/dscovox/pointcloud#/robot1/dscovox_node/pointcloud#g' /tmp/seg_experiment.rviz" || true
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
echo "config: NROBOTS=$NROBOTS res=$RESOLUTION ds=$DOWNSAMPLE carve_band=$CARVE_BAND max_range=$MAX_RANGE share_tsdf=$STSDF top_k=$SEMANTIC_TOP_K classes=$NUM_CLASSES split=$COVERAGE_SPLIT lidar_only=${LIDAR_ONLY:-0} share_rate_hz=$SHARE_RATE_HZ change_gate=$SGATE gate_eps=$SHARE_GATE_P_EPS/$SHARE_GATE_EV_REL split_band=$SPLIT_BAND z_band=[$SHARE_Z_MIN,$SHARE_Z_MAX]"
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

# 8) DISTRIBUTED-FUSION VERIFICATION — does every robot's dscovox now hold the
#    global map? Fused totals come from each merger's dscovox_diag log line;
#    under SPLIT_BAND we additionally query each merger's get_region service
#    (services still answer after /clock stops) with one thin probe slab per
#    sender slice: content a robot holds inside a PEER's slab can only have
#    arrived over its scovox_bin stream.
echo
echo "============== DISTRIBUTED FUSION VERIFICATION =============="
dc_scovox exec -T -e NROBOTS="$NROBOTS" -e SPLIT_BAND="$SPLIT_BAND" \
  -e ZA="$SPLIT_ZMIN" -e ZB="$SPLIT_ZMAX" scovox bash -lc "$SRC"'
cat > /tmp/verify_fusion.py <<"PYEOF"
import os, re, sys, rclpy
from scovox_msgs.srv import GetRegion

N      = int(os.environ["NROBOTS"])
SPLIT  = os.environ["SPLIT_BAND"] == "1"
ZA, ZB = float(os.environ["ZA"]), float(os.environ["ZB"])
BIG    = 1.0e6
ok     = True

# Fused totals: last dscovox_diag line per merger (publish-timer driven, so it
# reflects the state at the end of playback).
totals = {}
for i in range(1, N + 1):
    try:
        txt = open(f"/tmp/dscovox_r{i}.log", "rb").read().decode("utf8", "replace")
        m = re.findall(r"sources=(\d+).*fused_voxels=(\d+)", txt)
        srcs, totals[i] = (int(m[-1][0]), int(m[-1][1])) if m else (0, 0)
    except OSError:
        srcs, totals[i] = 0, 0
    print(f"  [r{i}] merger saw {srcs}/{N} sources; fused global map = {totals[i]} voxels")
    if srcs < N or totals[i] == 0: ok = False
if totals and min(totals.values()) > 0:
    sym = min(totals.values()) / max(totals.values())
    print(f"  fused-total symmetry across robots (min/max) = {sym:.3f}")
    if sym < 0.8: ok = False

if SPLIT and N > 1:
    rclpy.init(); node = rclpy.create_node("fusion_verifier")
    W = (ZB - ZA) / N
    def count(cli, zlo, zhi):
        rq = GetRegion.Request()
        rq.min_corner.x = rq.min_corner.y = -BIG; rq.min_corner.z = zlo
        rq.max_corner.x = rq.max_corner.y =  BIG; rq.max_corner.z = zhi
        fut = cli.call_async(rq)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=300.0)
        return len(fut.result().map.voxels) if fut.done() else None
    # one 0.4 m probe slab at the centre of each sender slice
    slabs = [(ZA + (k + 0.5) * W - 0.2, ZA + (k + 0.5) * W + 0.2) for k in range(N)]
    for i in range(1, N + 1):
        cli = node.create_client(GetRegion, f"/robot{i}/dscovox_node/get_region")
        if not cli.wait_for_service(timeout_sec=20.0):
            print(f"  [r{i}] get_region service NOT reachable"); ok = False; continue
        for k, (lo, hi) in enumerate(slabs, start=1):
            n = count(cli, lo, hi)
            tag = "own slice" if k == i else f"r{k} slice -> CROSS-ROBOT content"
            print(f"  [r{i}]   probe slab z=[{lo:+.2f},{hi:+.2f}]: {n} voxels ({tag})")
            if k != i and not n: ok = False
    node.destroy_node(); rclpy.shutdown()

print()
print("  FUSION VERIFY: " + ("PASS — every robot dscovox holds the global fused map"
      if ok else "FAIL — see counts above"))
sys.exit(0 if ok else 1)
PYEOF
python3 /tmp/verify_fusion.py' || true
echo "=============================================================="

# tear down (unless RVIZ). Use docker compose stop — never pkill.
if [ -n "$RVIZ" ]; then LEAVE_UP=1; fi
# cleanup() runs on EXIT
