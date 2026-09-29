#!/usr/bin/env bash
# Two-robot (bunker + curt) distributed exploration stack, NATIVE, pinned,
# FastDDS SHM by default (DDS=cyclone for CycloneDDS), both bags playing SIMULTANEOUSLY. The native counterpart of
# run_explo_dual.sh (Docker + full raw bags, neither available on this box),
# built from run_explo_pinned.sh's single-robot wiring.
#
# Usage:
#   run_explo_dual_native.sh [run_name] [duration_s]
#
# PER ROBOT (R = bunker | curt):
#   /R/lidar_localization   SMALL_VGICP vs the shared gt_map  -> map->odom_R
#   static odom_R -> base_R (identity; the localizer carries the pose)
#   static map -> map_int_R (identity alias; becomes the merger's source key)
#   /R/scovox_node          rolling mapper   -> /R/scovox_node/scovox_bin
#   /R/dscovox_node         merger fed by BOTH robots' bins (distributed fusion)
#   explo_planner_R         reads ONLY /R/dscovox_node/scovox, coordination on
#
# ONE CLOCK: the bags were recorded 27 min apart and only one player may publish
# /clock. bunker is replayed from bunker_jetson_on_curt_clock — the same bag
# re-stamped onto curt's timeline by restamp_bag_onto.py (log time, headers, TF
# and the Hesai per-point timestamps all shifted by the same integer ns) — so
# curt drives /clock and both robots' stamps agree with it.
#
# Frames are disjoint (bunker: odom/base_link/hesai_lidar, curt: odom_curt/
# base_link_curt/os_lidar), and every localizer topic is relative, so the /R
# namespace is all that keeps the two instances apart.
#
# Open loop, as in every bag run: each robot replays its recorded path and the
# planners' goals are not followed.
set -e
LWS=/home/jetsondevkit/jetbot-slam/hmr_localisation
SCOVOX_WS=/home/jetsondevkit/hmr_explo/ws
OVL=/home/jetsondevkit/hmr_explo/ovl_field
source /opt/ros/humble/setup.bash
source "$LWS/install/setup.bash"
source "$SCOVOX_WS/install/setup.bash"
source "$OVL/install/setup.bash"
export AMENT_PREFIX_PATH=/home/jetsondevkit/ros-extra/prefix/opt/ros/humble:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/home/jetsondevkit/ros-extra/prefix/opt/ros/humble/lib:/home/jetsondevkit/ros-extra/prefix/opt/ros/humble/lib/aarch64-linux-gnu:/home/jetsondevkit/small_gicp-install/lib:$LD_LIBRARY_PATH

# DDS=fastdds (default): shared-memory FastDDS, 64 MB segment — the multi-MB
# clouds go over SHM, so net.core.rmem_max does not matter.
# DDS=cyclone: the config benchmarked in docs/cyclonedds_transport_study.md
# (UDP; needs rmem_max >= 10 MB or the clouds drop at the socket).
DDS="${DDS:-fastdds}"
case "$DDS" in
  fastdds)
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE="$LWS/config/fastdds_shm.xml"
    unset CYCLONEDDS_URI ;;
  cyclone)
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://$LWS/scripts/cyclonedds_study/cyc_user.xml"
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    RMEM=$(sysctl -n net.core.rmem_max)
    [ "$RMEM" -ge 10485760 ] || echo "WARNING: net.core.rmem_max=$RMEM < 10485760 (sudo sysctl -w net.core.rmem_max=10485760)" ;;
  *) echo "DDS must be fastdds or cyclone"; exit 1 ;;
esac
export RCUTILS_COLORIZED_OUTPUT=0

# PINNING — 12 cores, two full stacks. Single-robot pinned runs measured each
# node at ~0.6-0.9 cores avg; each merger now folds two bin streams.
#            loc    scovox  dscovox planner
# bunker     0-1    2       3       4
# curt       5-6    7       8       9
# players    10-11
BUNKER_CORES=(${BUNKER_CORES:-0-1 2 3 4})
CURT_CORES=(${CURT_CORES:-5-6 7 8 9})
PLAYER_CORES="${PLAYER_CORES:-10-11}"

NAME="${1:-dual_$(date +%Y%m%d_%H%M%S)}"
DUR="${2:-}"
OUT="${OUT:-/home/jetsondevkit/explo-output/dual}"
mkdir -p "$OUT"

BAG_BUNKER="$LWS/bags/bunker_jetson_on_curt_clock"
BAG_CURT="$LWS/bags/curtmini_jetson/curtmini_jetson_0.mcap"
[ -d "$BAG_BUNKER" ] || { echo "no re-stamped bunker bag at $BAG_BUNKER — run:
  restamp_bag_onto.py $LWS/bags/bunker_jetson $LWS/bags/curtmini_jetson $BAG_BUNKER"; exit 1; }
[ -f "$BAG_CURT" ] || { echo "no bag at $BAG_CURT"; exit 1; }

SCOVOX_CFG="$SCOVOX_WS/install/scovox_mapping/share/scovox_mapping/config/bunker_jetson.yaml"
PLANNER_CFG="$OVL/install/explo_planner/share/explo_planner/config/exploration_fused_bag.yaml"
LOC_NODE="$LWS/install/lidar_localization_ros2/lib/lidar_localization_ros2/lidar_localization_node"
SCOVOX_BIN="$SCOVOX_WS/install/scovox_mapping/lib/scovox_mapping/scovox_mapping_node"
DSCOVOX_BIN="$SCOVOX_WS/install/scovox_mapping/lib/scovox_mapping/dscovox_mapping_node"
PLANNER_BIN="$OVL/install/explo_planner/lib/explo_planner/explo_planner_node"
STATIC_TF="$(ros2 pkg prefix tf2_ros)/lib/tf2_ros/static_transform_publisher"
for f in "$SCOVOX_CFG" "$PLANNER_CFG"; do [ -f "$f" ] || { echo "missing $f"; exit 1; }; done
for f in "$LOC_NODE" "$SCOVOX_BIN" "$DSCOVOX_BIN" "$PLANNER_BIN"; do [ -x "$f" ] || { echo "missing $f"; exit 1; }; done
MAP_ABS="$LWS/${MAP:-gt_map/gt_map_us050.pcd}"
[ -f "$MAP_ABS" ] || { echo "no map at $MAP_ABS"; exit 1; }

# A background compile corrupts pinned timing (bench-box rule): refuse to start.
# -x on the compiler's process name, and the [c] bracket on colcon: a plain -f
# pattern matches whatever shell launched this script with that text in it.
if pgrep -x cc1plus >/dev/null || pgrep -f "[c]olcon build" >/dev/null; then
  echo "a compiler is running — timing would be contaminated"; exit 1
fi

for _p in "scovox_mapping_nod[e]" "dscovox_mapping_nod[e]" "explo_planner_nod[e]" \
          "lidar_localization_nod[e]" "static_transform_publishe[r]" "ros2 bag pla[y]"; do
  pgrep -f "$_p" | xargs -r kill -9 2>/dev/null
done
sleep 3

echo "rmw: $RMW_IMPLEMENTATION  profile=${FASTRTPS_DEFAULT_PROFILES_FILE:-${CYCLONEDDS_URI:-}}"
echo "pins: bunker=${BUNKER_CORES[*]} curt=${CURT_CORES[*]} players=$PLAYER_CORES"

PIDS=()
cleanup() {
  kill -TERM "${PIDS[@]}" 2>/dev/null || true
  for _ in $(seq 1 40); do
    alive=0
    for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ "$alive" -eq 0 ] && return 0
    sleep 0.5
  done
  kill -KILL "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sample_proc() {  # $1 = pid, $2 = csv path (0.5 s: planner bursts are 250-750 ms)
  echo "time_sec,cpu_ticks,rss_kb" > "$2"
  while [ -r "/proc/$1/stat" ]; do
    printf '%s,%s,%s\n' "$(date +%s.%N)" \
      "$(awk '{print $14+$15}' "/proc/$1/stat" 2>/dev/null)" \
      "$(awk '/^VmRSS:/{print $2}' "/proc/$1/status" 2>/dev/null)" >> "$2"
    sleep 0.5
  done
}

# ---- per-robot stack ---------------------------------------------------------
# $1 robot  $2 cloud  $3 imu  $4 base  $5 odom  $6-$9 cores(loc scovox dscovox planner)
# $10 nav speed  ${@:11} initial pose x y z qz qw  (+ loc overrides via LOC_SET_<robot>)
SAMPS=()
launch_robot() {
  local R=$1 CLOUD=$2 IMU=$3 BASE=$4 ODOM=$5 C_LOC=$6 C_SC=$7 C_DS=$8 C_PL=$9 SPEED=${10}
  local SEED=("${@:11:5}") P="$OUT/${NAME}_$R"

  local CFG="/tmp/explo_dual_loc_$R.yaml"
  sed -e "s/^\(\s*initial_pose_x:\).*/\1 ${SEED[0]}/" \
      -e "s/^\(\s*initial_pose_y:\).*/\1 ${SEED[1]}/" \
      -e "s/^\(\s*initial_pose_z:\).*/\1 ${SEED[2]}/" \
      -e 's/^\(\s*initial_pose_qx:\).*/\1 0.0/' \
      -e 's/^\(\s*initial_pose_qy:\).*/\1 0.0/' \
      -e "s/^\(\s*initial_pose_qz:\).*/\1 ${SEED[3]}/" \
      -e "s/^\(\s*initial_pose_qw:\).*/\1 ${SEED[4]}/" \
      -e "s/^\(\s*base_frame_id:\).*/\1 $BASE/" \
      "$LWS/config/gt_ouster_ndt_tree_realtime.yaml" > "$CFG"
  local set_var="LOC_SET_$R"
  for kv in map_path=\"$MAP_ABS\" ${!set_var:-}; do
    local k="${kv%%=*}" v="${kv#*=}"
    grep -q "^\s*$k:" "$CFG" || { echo "override '$k' is not a parameter in the config"; exit 1; }
    sed -i "s|^\(\s*$k:\)[^#]*\(#.*\)\?$|\1 $v  \2|" "$CFG"
  done
  echo "[$R] registration: $(grep -E '^\s*registration_method:' "$CFG" | sed 's/.*: *//')"

  "$STATIC_TF" --x 0 --y 0 --z 0 --frame-id "$ODOM" --child-frame-id "$BASE" \
    --ros-args -r __node:=odom_to_base_$R > /dev/null 2>&1 &
  PIDS+=($!)
  "$STATIC_TF" --x 0 --y 0 --z 0 --frame-id map --child-frame-id "map_int_$R" \
    --ros-args -r __node:=map_to_int_$R > /dev/null 2>&1 &
  PIDS+=($!)

  # The node itself, not lidar_localization.launch.py: that launch hardcodes
  # namespace '' so two instances would collide on node name and topics.
  taskset -c "$C_LOC" "$LOC_NODE" --ros-args \
    -r __ns:=/$R -r __node:=lidar_localization \
    --params-file "$CFG" \
    -p use_sim_time:=true -p global_frame_id:=map -p odom_frame_id:="$ODOM" \
    -p base_frame_id:="$BASE" -p use_imu_preintegration:=true \
    -p imu_preintegration_use_base_frame_transform:=true \
    -r cloud:="$CLOUD" -r imu:="$IMU" -r twist:=/$R/twist \
    > "$P.loc.log" 2>&1 &
  local LOC_PID=$!
  PIDS+=($LOC_PID)
  # Lifecycle: configure (loads the map) then activate — what the launch does.
  for _ in $(seq 1 30); do ros2 lifecycle get /$R/lidar_localization >/dev/null 2>&1 && break; sleep 1; done
  ros2 lifecycle set /$R/lidar_localization configure > /dev/null
  ros2 lifecycle set /$R/lidar_localization activate > /dev/null
  for _ in $(seq 1 300); do grep -aq "Activating end" "$P.loc.log" && break; sleep 1; done
  grep -aq "Activating end" "$P.loc.log" || { echo "[$R] localizer failed to activate:"; tail -25 "$P.loc.log"; exit 1; }

  taskset -c "$C_SC" "$SCOVOX_BIN" --ros-args \
    -r __ns:=/$R -r __node:=scovox_node \
    --params-file "$SCOVOX_CFG" \
    -p use_sim_time:=true \
    -p input_pointcloud_topic:="$CLOUD" \
    -p imu_topic:="$IMU" \
    -p base_frame:="$BASE" \
    -p integration_frame:="map_int_$R" \
    -p mode:=rolling \
    -p share_rate_hz:=2.0 \
    > "$P.scovox.log" 2>&1 &
  local SC_PID=$!
  PIDS+=($SC_PID)

  taskset -c "$C_DS" "$DSCOVOX_BIN" --ros-args \
    -r __ns:=/$R -r __node:=dscovox_node \
    -p use_sim_time:=true \
    -p "input_topics:=['/bunker/scovox_node/scovox_bin', '/curt/scovox_node/scovox_bin']" \
    -p map_frame:=map \
    -p pointcloud_min_interval_s:=0.5 \
    > "$P.dscovox.log" 2>&1 &
  local DS_PID=$!
  PIDS+=($DS_PID)

  # map_resolution must match the mapper's 0.20; roi_min_z -5.5 per run_explo_pinned.sh.
  taskset -c "$C_PL" "$PLANNER_BIN" --ros-args \
    -r __node:=explo_planner_$R \
    --params-file "$PLANNER_CFG" \
    -p use_sim_time:=true \
    -p robot_name:=$R \
    -p dscovox_topic:=/$R/dscovox_node/scovox \
    -p map_frame:=map \
    -p map_resolution:=0.20 \
    -p base_frame:="$BASE" \
    -p goal_topic:=/$R/goal_pose \
    -p coordination_enabled:=true \
    -p nav_speed_estimate_mps:=$SPEED \
    -p roi_min_z:=-5.5 \
    -p output_csv:="$P.planner.csv" \
    > "$P.planner.log" 2>&1 &
  local PL_PID=$!
  PIDS+=($PL_PID)

  sleep 3
  for p in $SC_PID $DS_PID $PL_PID; do
    kill -0 "$p" 2>/dev/null || { echo "[$R] a node died at startup (pid $p); logs in $P.*"; exit 1; }
  done
  echo "[$R] pids: loc=$LOC_PID scovox=$SC_PID dscovox=$DS_PID planner=$PL_PID"
  sample_proc $LOC_PID "$P.loc.cpu.csv" & SAMPS+=($!)
  sample_proc $SC_PID "$P.scovox.cpu.csv" & SAMPS+=($!)
  sample_proc $DS_PID "$P.dscovox.cpu.csv" & SAMPS+=($!)
  sample_proc $PL_PID "$P.planner.cpu.csv" & SAMPS+=($!)
}

# Seeds and bunker's scan_channel_count are run_explo_pinned.sh's; speeds are
# run_explo_dual.sh's measured per-platform means.
LOC_SET_bunker="scan_channel_count=128 ${LOC_SET_bunker:-}"
launch_robot bunker /hesai/points  /imu/data      base_link      odom      "${BUNKER_CORES[@]}" 0.22 \
  7.82 -4.11 -1.17 0.8550024006656964 0.5186240399903342
launch_robot curt   /ouster/points /curt/imu/data base_link_curt odom_curt "${CURT_CORES[@]}" 0.20 \
  8.33 -3.78 -1.39 0.853050270749362 0.5218287416139898

QOS=/tmp/explo_dual_qos.yaml
cat > "$QOS" <<EOF
/hesai/points: {history: keep_last, depth: 5, reliability: best_effort, durability: volatile}
/ouster/points: {history: keep_last, depth: 5, reliability: best_effort, durability: volatile}
/imu/data: {history: keep_last, depth: 100, reliability: best_effort, durability: volatile}
/curt/imu/data: {history: keep_last, depth: 100, reliability: best_effort, durability: volatile}
/tf_static: {history: keep_last, depth: 100, reliability: reliable, durability: transient_local}
EOF

# curt drives /clock; bunker (re-stamped onto curt's timeline) must not.
echo "playing BOTH bags at rate ${PLAY_RATE:-1.0}${DUR:+ (first ${DUR}s)}..."
PLAY_B=(taskset -c "$PLAYER_CORES" ros2 bag play "$BAG_BUNKER" -s mcap
        --topics /hesai/points /imu/data /tf_static
        --rate "${PLAY_RATE:-1.0}" --qos-profile-overrides-path "$QOS")
PLAY_C=(taskset -c "$PLAYER_CORES" ros2 bag play "$BAG_CURT" -s mcap
        --topics /ouster/points /curt/imu/data /tf_static
        --clock --rate "${PLAY_RATE:-1.0}" --qos-profile-overrides-path "$QOS")
if [ -n "$DUR" ]; then
  timeout "$DUR" "${PLAY_B[@]}" > "$OUT/${NAME}_bunker.play.log" 2>&1 &
  PB=$!
  timeout "$DUR" "${PLAY_C[@]}" > "$OUT/${NAME}_curt.play.log" 2>&1 || true
else
  "${PLAY_B[@]}" > "$OUT/${NAME}_bunker.play.log" 2>&1 &
  PB=$!
  "${PLAY_C[@]}" > "$OUT/${NAME}_curt.play.log" 2>&1 || true
fi
wait $PB || true

# Drain: wait until both mappers stop logging scans.
last=0; idle=0
while [ $idle -lt 10 ]; do
  sleep 2
  cur=$(cat "$OUT/${NAME}"_{bunker,curt}.scovox.log 2>/dev/null | grep -ac 'recv=' || true)
  if [ "$cur" -gt "$last" ]; then last=$cur; idle=0; else idle=$((idle+2)); fi
done
kill "${SAMPS[@]}" 2>/dev/null || true

echo
for R in bunker curt; do
  echo "================ $R ================"
  python3 "$(dirname "$0")/explo_pin_summary.py" "$OUT/${NAME}_$R" || true
  echo "--- merger /$R/dscovox_node (want sources=2) ---"
  grep -aE 'dscovox_diag|deserialize failed|dropping' "$OUT/${NAME}_$R.dscovox.log" | tail -2 || true
  echo "--- localization divergences: $(grep -ac 'localization diverged' "$OUT/${NAME}_$R.scovox.log" || true)"
done
echo "files -> $OUT/${NAME}_*"
