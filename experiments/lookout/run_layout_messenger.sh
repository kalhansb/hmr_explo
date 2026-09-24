#!/bin/bash
# The procedure before 2026-09-24 (drive-out + messenger trips); kept, not used.
# One layout end to end (in the container):
#   run_layout_messenger.sh <layout> <out> <pilot|full>
# 1. Gazebo + bridge, lookouts spawned beside the mulcher (sim_up.sh).
# 2. Per lookout: TF glue, the 25 m lidar crop, simple_nav_3d and explo_planner
#    in lookout mode (immediate start, messenger on); radio_node.py.
# 3. trips.py: drive-out, then 1 (pilot) or 3 (full) messenger trips per lookout.
# 4. Stacks stopped, zero cmd_vel sent, then the walks (walks.py pauses the
#    world, checks the Huskies are at rest and logs their poses every step):
#    pilot = 6 walks + the check walks that match them; full = every walk,
#    then the 12 check walks (lookouts removed).
set -u
LAYOUT=$1; OUT=$2; MODE=$3
case $MODE in pilot) NTRIPS=1 ;; full) NTRIPS=3 ;; *) echo "mode: pilot|full"; exit 2 ;; esac
unset AT_POST
source /lookout/sim_up.sh
SIM=/ws/src/explo_planner/sim
PLANNER_SHARE=$(ros2 pkg prefix explo_planner)/share/explo_planner
PIDS=()
start() { # start <name> <cmd...>
  local name=$1; shift
  setsid "$@" > "$OUT/$name.log" 2>&1 &
  PIDS+=("$!")
  echo "[run_layout] started $name pid=$!"
}

# per-robot lookout parameters (params-file YAML, one block per namespaced node)
python3 - "$LAYOUT" > $OUT/lookout_params.yaml <<'EOF'
import sys, yaml
c = yaml.safe_load(open(f"/lookout/config/{sys.argv[1]}.yaml"))
out = {}
for l in c["lookouts"]:
    out[f"/{l['name']}/explo_planner"] = {"ros__parameters": {
        "exploit_mode": "lookout", "lookout_start": "immediate",
        "lookout_x": float(l["x"]), "lookout_y": float(l["y"]), "lookout_yaw": float(l["yaw"]),
        "lookout_messenger": True,
        "lookout_mulcher_x": float(c["mulcher"]["x"]), "lookout_mulcher_y": float(c["mulcher"]["y"])}}
print(yaml.safe_dump(out, sort_keys=False))
EOF

for r in $LOOKOUTS; do
  start tf_$r python3 $SIM/sim_tf_publisher.py $r
  start crop_$r python3 $SIM/lidar_crop.py $r --max-range 25.0 --ros-args -p use_sim_time:=true
  start nav_$r ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=$r mode:=ugv \
    mapping:=dscovox_lidar voxel_resolution_m:=0.20 \
    global_planning_map_size_m:=200.0 global_planning_map_resolution:=0.40 \
    nav_global_map_size_m:=200.0 lidar_points_topic:=/$r/velodyne_points_25m
done
for r in $LOOKOUTS; do
  start planner_$r ros2 run explo_planner explo_planner_node --ros-args \
    -r __ns:=/$r -r __node:=explo_planner \
    --params-file "$PLANNER_SHARE/config/shared_params.yaml" \
    --params-file $OUT/lookout_params.yaml \
    -p use_sim_time:=true -p robot_name:=$r -p terrain_relative_z:=false \
    -p mission_start_hold_sec:=0.0 -p exploitation_enabled:=false \
    -p experiment_log_path:=$OUT/$r.events.jsonl -p output_csv:=$OUT/planner_$r.csv
done
start radio python3 /lookout/radio_node.py --layout $LAYOUT --out $OUT

# wall-clock guard against hangs only: trips.py times out each leg in sim time,
# and with the nav stacks up the sim runs at ~0.1 real time (L3 full: ~12 h)
timeout 172800 python3 /lookout/trips.py --layout $LAYOUT --out $OUT --trips $NTRIPS 2>&1 | tee $OUT/trips.log
TRIPS_RC=${PIPESTATUS[0]}

# stop every stack (process groups), then make zero velocity the last command
for p in "${PIDS[@]}"; do kill -INT -- -$p 2>/dev/null; done
sleep 8
for p in "${PIDS[@]}"; do kill -KILL -- -$p 2>/dev/null; done
for r in $LOOKOUTS; do
  timeout 20 ros2 topic pub --times 10 -r 10 /$r/cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
done
echo "[run_layout] stacks stopped (trips rc=$TRIPS_RC)"
[ "$TRIPS_RC" = "0" ] || { echo "[run_layout] trips failed; walks not run"; exit 3; }

if [ "$MODE" = pilot ]; then
  timeout 20000 python3 /lookout/walks.py --layout $LAYOUT --out $OUT --mode pilot 2>&1 | tee -a $OUT/walks.log
  timeout 20000 python3 /lookout/walks.py --layout $LAYOUT --out $OUT --mode pilot_check 2>&1 | tee -a $OUT/walks.log
else
  timeout 86400 python3 /lookout/walks.py --layout $LAYOUT --out $OUT --mode full 2>&1 | tee -a $OUT/walks.log
  timeout 20000 python3 /lookout/walks.py --layout $LAYOUT --out $OUT --mode check 2>&1 | tee -a $OUT/walks.log
fi
echo "[run_layout] done"
