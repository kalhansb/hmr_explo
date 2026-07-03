#!/usr/bin/env bash
# Host-side orchestrator for the DISTRIBUTED MULTI-ROBOT LiDAR-ONLY mapping
# experiment against a bag — the config-file-driven, geometry-only sibling of
# run_lidar_experiment.sh (single robot) and run_mapshare_experiment.sh (the full
# semantic/bandwidth sweep harness).
#
# Topology (one sensor stream stands in for every robot; see
# scovox/docs/distributed_mapping_lidar.md for the real-fleet runbook):
#   * N scovox_mapping_node MAPPERS, each namespaced /robotK, each in mode:=rolling
#     (creates the ~/scovox_bin LZ4 delta stream) and integrating into a UNIQUE
#     frame rK_map (the merger keys sources by the bin header.frame_id == the
#     integration_frame, so two robots sharing one frame would collapse to one
#     source — see [[dscovox-source-frame-keying]]).
#   * N identity  map->rK_map  static TFs so each merger folds every source back
#     into the common map frame.
#   * N dscovox_mapping_node MERGERS, one per /robotK, each fusing ALL robots'
#     streams into that robot's copy of the global map (real fleet topology; no
#     central merger). Started FIRST — the bin publish is subscriber-gated, so a
#     mapper with no merger listening throws its deltas away.
#
# "CONFIG FILES ONLY": unlike run_mapshare_experiment.sh (which sets ~30 mapper
# params inline via -p), every node here loads its params from the COMMITTED,
# fleet-identical config files. The only per-node command-line args are the ones
# that CANNOT live in a shared file:
#   mapper  : -r __ns:=/robotK  -r __node:=scovox_node
#             --params-file scovox_lidar_geometric.yaml   (LiDAR-only base: Beta
#                occupancy, full-ray carve, in-node deskew; fuse_lidar_rgbd:false)
#             --params-file scovox_robot_share.yaml       (share overlay: rolling
#                mode + change-gate + 2 Hz coalescing + z-band)
#             -p integration_frame:=rK_map                (unique per robot)
#   merger  : -r __ns:=/robotK  -r __node:=dscovox_node
#             --params-file dscovox_params.yaml           (input_topics + z-clip)
#   ...plus -p use_sim_time:=true on every scovox node: the configs ship
#   use_sim_time:false for LIVE robots; bag replay drives sim time off /clock.
#   (If NROBOTS != 2 the merger input_topics is also overridden, since the
#   committed file lists exactly robot1+robot2 — extend it in the yaml for a
#   permanent >2 fleet, per the runbook.)
#
# Frames come STRAIGHT FROM THE BAG's own TF (same as run_lidar_experiment.sh):
#   map --NDT--> odom --EKF--> base_link --STATIC(bag /tf_static)--> os_lidar.
#   The bag /tf has only wheel joints (no map/odom leg), so it never fights the
#   live NDT+EKF tree. One local alias imu_link->imu (identity) bridges the frame
#   name /imu/data is stamped with; the identity map->rK_map statics are added on
#   top. Nothing geometric is hand-entered.
#
# Usage (HOST; needs docker, NOT ROS):
#   ./run_dist_lidar_experiment.sh [playback_duration_s]      # empty = full bag (~500 s)
#   ./run_dist_lidar_experiment.sh 120                        # 2-robot fusion test, 120 s
#   RVIZ=1 ./run_dist_lidar_experiment.sh 120                 # + RViz on robot1's fused cloud
#   NROBOTS=3 ./run_dist_lidar_experiment.sh 120              # 3 mappers + 3 mergers
#
# Env knobs (defaults reproduce the shipped LiDAR-only distributed pipeline):
#   NROBOTS=2       # 1..3. 2+ = duplicate-source: each merger reaching sources=N
#                   #   means it received every peer's UNIQUE-frame stream = fusion.
#   RATE=0.5        # bag playback rate. 0.5 gives N full-ray mappers room on one host.
#   CARVE_BAND=     # override the mapper carve_band (m); empty = config default (-1.0 full-ray).
#   RVIZ=1          # open RViz on /robot1/dscovox_node/pointcloud; leave nodes up at end.
#   BW=1            # attach ros2 topic bw/hz probes to each scovox_bin and print the tail.
#   PC_VIZ_S=2.0    # merger viz-cloud min publish interval (s); the fused cloud is large.
#   SOFTGL=1        # force software GL for RViz (no-GPU fallback).
#
# The run ends with a DISTRIBUTED FUSION VERIFICATION: every merger's dscovox_diag
# must reach sources=N and the per-robot fused_voxels totals must be ~symmetric.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOC_DIR="$HERE/hmr_localisation"
SCOVOX_DIR="$HERE/scovox"
BAG="/ws/bags/2026_06_19_18_19_06__kalhan-map-test-2_"
DUR="${1:-}"
DUR_ARG=""; [ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

NROBOTS="${NROBOTS:-2}"
RATE="${RATE:-0.5}"
CARVE_BAND="${CARVE_BAND:-}"               # empty = keep the config's carve_band (-1.0)
RVIZ="${RVIZ:-}"
BW="${BW:-}"
PC_VIZ_S="${PC_VIZ_S:-2.0}"
RVIZ_CFG="$HERE/mapshare_experiment.rviz"  # subscribes /robot1/dscovox_node/pointcloud

case "$NROBOTS" in 1|2|3) ;; *) echo "NROBOTS must be 1..3 (got $NROBOTS)"; exit 2;; esac

# Committed, fleet-identical config files (paths inside the scovox container).
MAP_BASE=/scovox/config/scovox_lidar_geometric.yaml            # LiDAR-only base
MAP_SHARE=/scovox/config/scovox_robot_share.yaml               # rolling-share overlay
DSCOVOX_PARAMS=/scovox/src/scovox_mapping/config/dscovox_params.yaml

dc_loc()    { docker compose -f "$LOC_DIR/compose.yaml"    "$@"; }
dc_scovox() { docker compose -f "$SCOVOX_DIR/compose.yaml" "$@"; }
# scovox-container ros2 preamble (shared by every exec below)
SRC='source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash'

# Merger input-topics list. Default N=2 uses the committed file verbatim (pure
# config); any other N overrides it (the file lists exactly robot1+robot2).
INPUT_TOPICS="["
for i in $(seq 1 "$NROBOTS"); do
  INPUT_TOPICS="$INPUT_TOPICS'/robot$i/scovox_node/scovox_bin'"
  [ "$i" -lt "$NROBOTS" ] && INPUT_TOPICS="$INPUT_TOPICS,"
done
INPUT_TOPICS="$INPUT_TOPICS]"

LEAVE_UP=""
stop_cmds() {
  echo "    docker compose -f $SCOVOX_DIR/compose.yaml stop scovox"   # mappers+mergers+probes+RViz
  echo "    docker compose -f $LOC_DIR/compose.yaml stop ros"
}
cleanup() {
  if [ -n "$LEAVE_UP" ]; then
    echo "[orch] RVIZ mode: leaving nodes + RViz running so you can inspect the fused map."
    echo "[orch] stop everything when done with:"; stop_cmds; return 0
  fi
  echo "[orch] stopping nodes (docker compose stop)…"
  dc_scovox stop scovox 2>/dev/null || true
  dc_loc    stop ros    2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[orch] distributed LiDAR-only mapping: NROBOTS=$NROBOTS rate=$RATE carve_band=${CARVE_BAND:-config} rviz=${RVIZ:-0}"
echo "[orch] bringing up containers…"
dc_loc    up -d
dc_scovox up -d
if [ -n "$RVIZ" ]; then
  echo "[orch] RVIZ=1 → RViz runs inside the scovox container (GPU/hardware GL); allowing local X11…"
  xhost +local:root >/dev/null 2>&1 || true
fi

# kill stray bag play + clear STALE /tmp logs (the NDT wait and the fusion verify
# grep these files — leftovers from an earlier run would make both lie).
dc_loc exec -T ros bash -lc 'pkill -f "[r]os2 bag play" 2>/dev/null;
  rm -f /tmp/ndt.log /tmp/ekf.log /tmp/imualias.log /tmp/r*maptf.log; sleep 1; true' || true
dc_scovox exec -T scovox bash -lc 'rm -f /tmp/dscovox_r*.log /tmp/scovox_r*.log /tmp/bw_r*.log /tmp/hz_r*.log' || true

# 1) localizer stack in hmr_loc (verbatim from run_lidar_experiment.sh): EKF +
#    NDT (map->odom ONLY; publish_lidar_tf/imu_tf:=false so the sensor legs come
#    from the bag TF) + the one imu_link->imu identity alias.
echo "[orch] launching EKF + NDT (map->odom) + imu_link->imu alias in hmr_loc…"
dc_loc exec -d ros bash -lc '
  source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml   # SHM for big clouds
  ros2 launch /ws/launch/ekf_odom.launch.py use_sim_time:=true > /tmp/ekf.log 2>&1 &
  ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
    localization_param_dir:=/ws/config/gt_ouster_ndt_tree_fused.yaml \
    cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
    global_frame_id:=map odom_frame_id:=odom base_frame_id:=base_link \
    use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
    publish_lidar_tf:=false publish_imu_tf:=false \
    > /tmp/ndt.log 2>&1 &
  ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.0 --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 \
    --frame-id imu_link --child-frame-id imu > /tmp/imualias.log 2>&1 &
  wait
'

# 1b) identity map->rK_map static TFs — one per robot, so each merger folds that
#     robot's rK_map-tagged stream back into the common map frame. Latched
#     /tf_static; the mergers cache each on first sight.
echo "[orch] publishing identity map->rK_map static TFs…"
IDENT_CMDS=""
for i in $(seq 1 "$NROBOTS"); do
  IDENT_CMDS="$IDENT_CMDS ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id r${i}_map > /tmp/r${i}maptf.log 2>&1 & "
done
dc_loc exec -d ros bash -lc "source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; $IDENT_CMDS wait"

echo "[orch] waiting for NDT map load + activation…"
dc_loc exec -T ros bash -lc 'for i in $(seq 1 60); do grep -aq "Activating end" /tmp/ndt.log 2>/dev/null && { echo NDT_ACTIVE; exit 0; }; sleep 1; done; echo NDT_TIMEOUT; tail -5 /tmp/ndt.log'

# 2) MERGERS FIRST — one dscovox per robot, all params from dscovox_params.yaml.
#    Only use_sim_time (bag replay) + the viz-cloud throttle are overridden; N!=2
#    additionally overrides input_topics (see header).
IT_ARG=""; [ "$NROBOTS" != "2" ] && IT_ARG="-p input_topics:=$INPUT_TOPICS"
for i in $(seq 1 "$NROBOTS"); do
  echo "[orch] launching dscovox merger robot$i (ns=/robot$i, params=dscovox_params.yaml)…"
  dc_scovox exec -d -e RNS="/robot$i" -e LOGF="/tmp/dscovox_r$i.log" -e IT_ARG="$IT_ARG" \
    scovox bash -lc "$SRC
    exec ros2 run scovox_mapping dscovox_mapping_node --ros-args \
      -r __ns:=\$RNS -r __node:=dscovox_node \
      --params-file $DSCOVOX_PARAMS \
      -p use_sim_time:=true \
      -p pointcloud_min_interval_s:=$PC_VIZ_S \
      \$IT_ARG \
      > \$LOGF 2>&1"
done

# 3) N rolling mappers — params entirely from the two committed config files.
#    Per-robot: __ns + unique integration_frame rK_map. use_sim_time:=true for
#    bag replay; CARVE_BAND appended only when set (else the config's -1.0 wins).
CB_ARG=""; [ -n "$CARVE_BAND" ] && CB_ARG="-p carve_band:=$CARVE_BAND"
for i in $(seq 1 "$NROBOTS"); do
  echo "[orch] launching mapper robot$i (ns=/robot$i, integration_frame=r${i}_map, mode=rolling)…"
  dc_scovox exec -d -e RNS="/robot$i" -e IFRAME="r${i}_map" -e LOGF="/tmp/scovox_r$i.log" -e CB_ARG="$CB_ARG" \
    scovox bash -lc "$SRC
    exec ros2 run scovox_mapping scovox_mapping_node --ros-args \
      -r __ns:=\$RNS -r __node:=scovox_node \
      --params-file $MAP_BASE \
      --params-file $MAP_SHARE \
      -p integration_frame:=\$IFRAME \
      -p use_sim_time:=true \
      \$CB_ARG \
      > \$LOGF 2>&1"
done

# 4) optional bandwidth/rate probes on each scovox_bin (the map-sharing channel).
if [ -n "$BW" ]; then
  echo "[orch] attaching ros2 topic bw/hz probes to the scovox_bin streams…"
  for i in $(seq 1 "$NROBOTS"); do
    dc_scovox exec -d -e T="/robot$i/scovox_node/scovox_bin" scovox bash -lc "$SRC; exec ros2 topic bw \$T > /tmp/bw_r$i.log 2>&1"
    dc_scovox exec -d -e T="/robot$i/scovox_node/scovox_bin" scovox bash -lc "$SRC; exec ros2 topic hz \$T > /tmp/hz_r$i.log 2>&1"
  done
fi

# 4b) optional RViz on robot1's fused cloud (per-robot mergers publish ~/pointcloud).
if [ -n "$RVIZ" ]; then
  echo "[orch] launching RViz inside the scovox container…"
  docker cp "$RVIZ_CFG" scovox:/tmp/mapshare_experiment.rviz 2>/dev/null || true
  GLENV='export __NV_PRIME_RENDER_OFFLOAD=1; export __GLX_VENDOR_LIBRARY_NAME=nvidia'
  [ "${SOFTGL:-0}" = "1" ] && GLENV='export LIBGL_ALWAYS_SOFTWARE=1; unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD'
  dc_scovox exec -d scovox bash -lc "$SRC
    export DISPLAY=\"\${DISPLAY:-:1}\"; $GLENV
    exec rviz2 -d /tmp/mapshare_experiment.rviz --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1"
fi

echo "[orch] letting mergers + mappers subscribe (8 s)…"
sleep 8

# 5) play the bag: LiDAR cloud + /imu (deskew) + the bag's OWN TF. No camera
#    topics (LiDAR-only). Best-effort /ouster/points connects directly (no qos
#    override); SHM (the FASTRTPS profile) is the large-cloud transport.
echo "[orch] playing bag ${DUR:+(first ${DUR}s)} at rate $RATE [distributed LiDAR-only, bag TF]…"
dc_loc exec -T ros bash -lc "
  source /opt/ros/jazzy/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml
  ros2 bag play $BAG --clock --rate $RATE $DUR_ARG \
    --topics /ouster/points /imu/data /tf /tf_static
"

# 6) DISTRIBUTED FUSION VERIFICATION — every merger's dscovox_diag must reach
#    sources=N (it received every peer's UNIQUE-frame stream = cross-robot
#    exchange) and the per-robot fused_voxels totals must be ~symmetric.
echo
echo "============== DISTRIBUTED FUSION VERIFICATION =============="
dc_scovox exec -T -e NROBOTS="$NROBOTS" scovox bash -lc "$SRC"'
cat > /tmp/verify_dist.py <<"PYEOF"
import os, re, sys
N = int(os.environ["NROBOTS"])
ok = True; totals = {}
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
print()
print("  FUSION VERIFY: " + ("PASS — every robot dscovox holds the global fused map"
      if ok else "FAIL — see counts above (sources<N ⇒ a peer stream never arrived)"))
sys.exit(0 if ok else 1)
PYEOF
python3 /tmp/verify_dist.py' || true
echo "=============================================================="

# 7) optional bandwidth readout (live probes UNDER-report — big samples drop —
#    but give an indicative per-robot wire figure for the scovox_bin channel).
if [ -n "$BW" ]; then
  echo "--- per-robot scovox_bin wire (indicative; ros2 topic bw/hz under-report) ---"
  for i in $(seq 1 "$NROBOTS"); do
    dc_scovox exec -T scovox bash -lc "tail -2 /tmp/bw_r$i.log 2>/dev/null | tr '\n' ' ' | sed 's/^/  bw[r$i]  /'; echo; grep -aE 'average rate' /tmp/hz_r$i.log 2>/dev/null | tail -1 | sed 's/^/  hz[r$i]  /'" || true
  done
fi

if [ -n "$RVIZ" ]; then LEAVE_UP=1; fi
# cleanup() runs on EXIT
