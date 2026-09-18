#!/usr/bin/env bash
# Profile the FULL pinned exploration stack on a recorded bag, CycloneDDS.
#
# Usage:
#   run_explo_pinned.sh [bunker|curtmini] [run_name] [duration_s]
#
# WHAT RUNS
#   hmr_localisation  lidar_localization_ros2 (SMALL_VGICP)  -> map->odom
#   scovox            scovox_mapping_node, mode=rolling      -> scovox_bin
#   scovox            dscovox_mapping_node (merger)          -> /robot1/dscovox_node/scovox
#   explo_planner     explo_planner_node (EIG, straight-line)
# plus a static identity odom->base_link and the bag player. The planner's map
# is the MERGER's transient_local ~/scovox — scovox_node's own ~/scovox is
# VOLATILE and never matches the planner's transient_local subscription (the
# committed exploration_fused_bag.yaml default is unusable; see
# run_explo_curtmini.sh's header).
#
# DDS: CycloneDDS with scripts/cyclonedds_study/cyc_user.xml — the exact config
# benchmarked in docs/cyclonedds_transport_study.md. No SHM: UDP on wlP1p1s0.
#
# PINNING — every measured node has its own EXCLUSIVE cores; the player and
# samplers live on the leftover cores so they never contend with a node:
#   localizer   cores 0-2   (measured ~2.4 cores on this box)
#   scovox      cores 3-4   (~1.0 when integrating; +bin publish)
#   dscovox     core  5
#   planner     cores 6-7   (single-threaded ~1.1-core bursts)
#   bag player  cores 8-11
# Override with LOC_CORES/SCOVOX_CORES/DSCOVOX_CORES/PLANNER_CORES/PLAYER_CORES.
#
# The mapping config is the VERIFIED field set (scovox bunker_jetson.yaml:
# res 0.20, full-ray carve, deskew on @ ref_frac 0.5, max_range 20) with only
# what the dscovox path needs on top: mode:=rolling (makes scovox_bin exist)
# and share_rate_hz:=2.0 (coalesced binary deltas, dscovox_single_robot.launch
# reference values). The underlying voxel grid is fully persistent in both
# modes, so mapping cost stays comparable to the persistent-mode benchmarks.
set -e
LWS=/home/jetsondevkit/jetbot-slam/hmr_localisation
SCOVOX_WS=/home/jetsondevkit/scovox_new_experiments/scovox
OVL=/home/jetsondevkit/hmr_explo/ovl_field
source /opt/ros/humble/setup.bash
source "$LWS/install/setup.bash"
source "$SCOVOX_WS/install/setup.bash"
source "$OVL/install/setup.bash"
export AMENT_PREFIX_PATH=/home/jetsondevkit/ros-extra/prefix/opt/ros/humble:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/home/jetsondevkit/ros-extra/prefix/opt/ros/humble/lib:/home/jetsondevkit/ros-extra/prefix/opt/ros/humble/lib/aarch64-linux-gnu:/home/jetsondevkit/small_gicp-install/lib:$LD_LIBRARY_PATH

# CycloneDDS, always — this harness exists to measure the stack on this config.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$LWS/scripts/cyclonedds_study/cyc_user.xml"
RMEM=$(sysctl -n net.core.rmem_max)
[ "$RMEM" -ge 10485760 ] || echo "WARNING: net.core.rmem_max=$RMEM < 10485760 (sudo sysctl -w net.core.rmem_max=10485760)"
unset FASTRTPS_DEFAULT_PROFILES_FILE
export RCUTILS_COLORIZED_OUTPUT=0

LOC_CORES="${LOC_CORES:-0-2}"
SCOVOX_CORES="${SCOVOX_CORES:-3-4}"
DSCOVOX_CORES="${DSCOVOX_CORES:-5}"
PLANNER_CORES="${PLANNER_CORES:-6-7}"
PLAYER_CORES="${PLAYER_CORES:-8-11}"

PROFILE="${1:-bunker}"
NAME="${2:-explo_${PROFILE}_$(date +%Y%m%d_%H%M%S)}"
DUR="${3:-}"
case "$PROFILE" in
  bunker)
    BAG="$LWS/bags/bunker_jetson/bunker_jetson_0.mcap"
    CLOUD=/hesai/points; IMU=/imu/data; BASE=base_link
    SEED=(7.82 -4.11 -1.17 0.8550024006656964 0.5186240399903342)
    SET="scan_channel_count=128 ${SET:-}" ;;
  curtmini)
    BAG="$LWS/bags/curtmini_jetson/curtmini_jetson_0.mcap"
    CLOUD=/ouster/points; IMU=/curt/imu/data; BASE=base_link_curt
    SEED=(8.33 -3.78 -1.39 0.853050270749362 0.5218287416139898) ;;
  *) echo "unknown bag profile '$PROFILE' (bunker|curtmini)"; exit 1 ;;
esac
OUT="${OUT:-/home/jetsondevkit/explo-output/pinned}"
mkdir -p "$OUT"
[ -f "$BAG" ] || { echo "no bag at $BAG"; exit 1; }

SCOVOX_CFG="$SCOVOX_WS/install/scovox_mapping/share/scovox_mapping/config/bunker_jetson.yaml"
PLANNER_CFG="$OVL/install/explo_planner/share/explo_planner/config/exploration_fused_bag.yaml"
[ -f "$SCOVOX_CFG" ] || { echo "no scovox config at $SCOVOX_CFG"; exit 1; }
[ -f "$PLANNER_CFG" ] || { echo "no planner config at $PLANNER_CFG"; exit 1; }

# Reap leftovers FIRST (a survivor re-publishes latched state into this run).
for _p in "scovox_mapping_nod[e]" "dscovox_mapping_nod[e]" "explo_planner_nod[e]" \
          "lidar_localization_nod[e]" "static_transform_publishe[r]" "ros2 bag pla[y]"; do
  pgrep -f "$_p" | xargs -r kill -9 2>/dev/null
done
sleep 3

echo "rmw: $RMW_IMPLEMENTATION  uri=$CYCLONEDDS_URI"
echo "pins: loc=$LOC_CORES scovox=$SCOVOX_CORES dscovox=$DSCOVOX_CORES planner=$PLANNER_CORES player=$PLAYER_CORES"
echo "bag: $BAG"

# ---- localizer config: same construction as scovox_bag_test.sh --------------
CFG=/tmp/explo_pin_loc.yaml
sed -e "s/^\(\s*initial_pose_x:\).*/\1 ${SEED[0]}/" \
    -e "s/^\(\s*initial_pose_y:\).*/\1 ${SEED[1]}/" \
    -e "s/^\(\s*initial_pose_z:\).*/\1 ${SEED[2]}/" \
    -e 's/^\(\s*initial_pose_qx:\).*/\1 0.0/' \
    -e 's/^\(\s*initial_pose_qy:\).*/\1 0.0/' \
    -e "s/^\(\s*initial_pose_qz:\).*/\1 ${SEED[3]}/" \
    -e "s/^\(\s*initial_pose_qw:\).*/\1 ${SEED[4]}/" \
    -e "s/^\(\s*base_frame_id:\).*/\1 $BASE/" \
    "$LWS/config/gt_ouster_ndt_tree_realtime.yaml" > "$CFG"
MAP="${MAP:-gt_map/gt_map_us050.pcd}"
case "$MAP" in /*) MAP_ABS="$MAP" ;; *) MAP_ABS="$LWS/$MAP" ;; esac
[ -f "$MAP_ABS" ] || { echo "no map at $MAP_ABS"; exit 1; }
SET="map_path=\"$MAP_ABS\" ${SET:-}"
for kv in ${SET:-}; do
  k="${kv%%=*}"; v="${kv#*=}"
  grep -q "^\s*$k:" "$CFG" || { echo "override '$k' is not a parameter in the config"; exit 1; }
  sed -i "s|^\(\s*$k:\)[^#]*\(#.*\)\?$|\1 $v  \2|" "$CFG"
  echo "loc override: $k = $v"
done
echo "registration: $(grep -E '^\s*registration_method:' "$CFG" | sed 's/.*: *//')"

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

# ---- odometry: static identity odom->base + SMALL_VGICP localizer -----------
"$(ros2 pkg prefix tf2_ros)/lib/tf2_ros/static_transform_publisher" --x 0 --y 0 --z 0 \
  --frame-id odom --child-frame-id "$BASE" > /dev/null 2>&1 &
PIDS+=($!)
taskset -c "$LOC_CORES" ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
  localization_param_dir:="$CFG" cloud_topic:="$CLOUD" imu_topic:="$IMU" \
  use_sim_time:=true global_frame_id:=map odom_frame_id:=odom base_frame_id:="$BASE" \
  use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
  publish_lidar_tf:=false publish_imu_tf:=false > "$OUT/$NAME.loc.log" 2>&1 &
PIDS+=($!)
LAUNCH_PID=$!
echo "waiting for map load + localizer activation..."
for _ in $(seq 1 300); do grep -aq "Activating end" "$OUT/$NAME.loc.log" && break; sleep 1; done
grep -aq "Activating end" "$OUT/$NAME.loc.log" || { echo "localizer failed to activate:"; tail -25 "$OUT/$NAME.loc.log"; exit 1; }
LOC_PID=$(pgrep -P "$LAUNCH_PID" -f lidar_localization_node | head -1)

# ---- scovox mapper: verified field config + the rolling/share knobs ---------
SCOVOX_BIN="$SCOVOX_WS/install/scovox_mapping/lib/scovox_mapping/scovox_mapping_node"
[ -x "$SCOVOX_BIN" ] || { echo "no scovox binary at $SCOVOX_BIN"; exit 1; }
taskset -c "$SCOVOX_CORES" "$SCOVOX_BIN" --ros-args \
  -r __ns:=/robot1 -r __node:=scovox_node \
  --params-file "$SCOVOX_CFG" \
  -p use_sim_time:=true \
  -p input_pointcloud_topic:="$CLOUD" \
  -p imu_topic:="$IMU" \
  -p base_frame:="$BASE" \
  -p integration_frame:=map \
  -p mode:=rolling \
  -p share_rate_hz:=2.0 \
  ${SCOVOX_SET:+$(for kv in $SCOVOX_SET; do printf -- '-p %s ' "$kv"; done)} \
  > "$OUT/$NAME.scovox.log" 2>&1 &
SCOVOX_PID=$!
PIDS+=($SCOVOX_PID)

# ---- dscovox merger: the only map source the planner can hear ---------------
DSCOVOX_BIN="$SCOVOX_WS/install/scovox_mapping/lib/scovox_mapping/dscovox_mapping_node"
[ -x "$DSCOVOX_BIN" ] || { echo "no dscovox binary at $DSCOVOX_BIN"; exit 1; }
taskset -c "$DSCOVOX_CORES" "$DSCOVOX_BIN" --ros-args \
  -r __ns:=/robot1 -r __node:=dscovox_node \
  -p use_sim_time:=true \
  -p "input_topics:=['/robot1/scovox_node/scovox_bin']" \
  -p map_frame:=map \
  -p pointcloud_min_interval_s:=0.5 \
  > "$OUT/$NAME.dscovox.log" 2>&1 &
DSCOVOX_PID=$!
PIDS+=($DSCOVOX_PID)

# ---- planner: EIG exploration, straight-line mode, open loop ----------------
# roi_min_z -5.5 not the yaml's -5.0: at -5.0 groundZAt can return NaN at the
# slab edge and vantages get rejected (run_explo_curtmini.sh, verified there).
# map_resolution must match the mapper's 0.20 (the yaml says 0.10).
PLANNER_BIN="$OVL/install/explo_planner/lib/explo_planner/explo_planner_node"
[ -x "$PLANNER_BIN" ] || { echo "no planner binary at $PLANNER_BIN"; exit 1; }
taskset -c "$PLANNER_CORES" "$PLANNER_BIN" --ros-args \
  --params-file "$PLANNER_CFG" \
  -p use_sim_time:=true \
  -p dscovox_topic:=/robot1/dscovox_node/scovox \
  -p map_resolution:=0.20 \
  -p base_frame:="$BASE" \
  -p roi_min_z:=-5.5 \
  -p output_csv:="$OUT/$NAME.planner.csv" \
  > "$OUT/$NAME.planner.log" 2>&1 &
PLANNER_PID=$!
PIDS+=($PLANNER_PID)

sleep 4
for p in $SCOVOX_PID $DSCOVOX_PID $PLANNER_PID; do
  kill -0 "$p" 2>/dev/null || { echo "a node died at startup (pid $p); logs in $OUT/$NAME.*"; exit 1; }
done
echo "pids: loc=${LOC_PID:-?} scovox=$SCOVOX_PID dscovox=$DSCOVOX_PID planner=$PLANNER_PID"

# ---- samplers: 0.5 s /proc utime+stime + VmRSS (0.5 s not 2 s: the planner --
# bursts for 250-750 ms and a 2 s sampler smears those bursts by ~3x) ---------
sample_proc() {  # $1 = pid, $2 = csv path
  echo "time_sec,cpu_ticks,rss_kb" > "$2"
  while [ -r "/proc/$1/stat" ]; do
    printf '%s,%s,%s\n' "$(date +%s.%N)" \
      "$(awk '{print $14+$15}' "/proc/$1/stat" 2>/dev/null)" \
      "$(awk '/^VmRSS:/{print $2}' "/proc/$1/status" 2>/dev/null)" >> "$2"
    sleep 0.5
  done
}
sample_proc "$SCOVOX_PID"  "$OUT/$NAME.scovox.cpu.csv"  & SAMPS=($!)
sample_proc "$DSCOVOX_PID" "$OUT/$NAME.dscovox.cpu.csv" & SAMPS+=($!)
sample_proc "$PLANNER_PID" "$OUT/$NAME.planner.cpu.csv" & SAMPS+=($!)
[ -n "$LOC_PID" ] && { sample_proc "$LOC_PID" "$OUT/$NAME.loc.cpu.csv" & SAMPS+=($!); }

QOS=/tmp/explo_pin_qos.yaml
cat > "$QOS" <<EOF
$CLOUD:
  history: keep_last
  depth: 5
  reliability: best_effort
  durability: volatile
$IMU:
  history: keep_last
  depth: 100
  reliability: best_effort
  durability: volatile
/tf_static:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: transient_local
EOF

echo "playing at rate ${PLAY_RATE:-1.0}${DUR:+ (first ${DUR}s)}..."
PLAY=(taskset -c "$PLAYER_CORES" ros2 bag play "$BAG" -s mcap
      --topics "$CLOUD" "$IMU" /tf_static
      --clock --rate "${PLAY_RATE:-1.0}" --qos-profile-overrides-path "$QOS")
if [ -n "$DUR" ]; then
  timeout "$DUR" "${PLAY[@]}" > "$OUT/$NAME.play.log" 2>&1 || true
else
  "${PLAY[@]}" > "$OUT/$NAME.play.log" 2>&1
fi

# Drain: wait until scovox stops logging scans.
last=0; idle=0
while [ $idle -lt 10 ]; do
  sleep 2
  cur=$(grep -ac 'recv=' "$OUT/$NAME.scovox.log" 2>/dev/null || echo 0)
  if [ "$cur" -gt "$last" ]; then last=$cur; idle=0; else idle=$((idle+2)); fi
done
kill "${SAMPS[@]}" 2>/dev/null || true
echo
python3 "$(dirname "$0")/explo_pin_summary.py" "$OUT/$NAME"
echo "files -> $OUT/$NAME.*"
