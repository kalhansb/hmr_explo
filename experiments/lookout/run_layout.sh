#!/bin/bash
# One layout end to end (in the container):
#   run_layout.sh <layout> <out> <pilot|full>
# 1. Gazebo + bridge, each lookout Husky spawned on its post (sim_up.sh with
#    AT_POST=1; the posts are calculated from the map by posts.py and written
#    to the layout config by build_world.py), static, since a parked Husky
#    rocks (spawn_static.py).
# 2. The walks (walks.py pauses the world, waits until the Huskies are at rest,
#    logs each one's distance from its post and its radio link there, and logs
#    their poses every step): pilot = 6 walks, full = every walk.
# 3. The check walks (the pilot's: those matching its 6 walks; full: 12), run
#    twice, since removing a lidar model crashes Fortress (2026-09-24):
#    "away" in the same sim, the lookouts spawned then moved 1 km away; "absent"
#    in a fresh sim (sim_up.sh NO_LOOKOUTS=1) where they were never spawned.
# No planner, nav stack or radio node runs: the lookouts do not move, and every
# post is linked (posts.py), so a lookout's alarm reaches the mulcher at once.
# (Until 2026-09-24 the lookouts drove out and carried warnings:
# run_layout_messenger.sh, trips.py, radio_node.py.)
set -u
LAYOUT=$1; OUT=$2; MODE=$3
case $MODE in pilot|full) ;; *) echo "mode: pilot|full"; exit 2 ;; esac
AT_POST=1
source /lookout/sim_up.sh

if [ "$MODE" = pilot ]; then MAIN=pilot; CHECK=pilot_check; T_MAIN=20000
else MAIN=full; CHECK=check; T_MAIN=86400; fi
timeout $T_MAIN python3 /lookout/walks.py --layout $LAYOUT --out $OUT --mode $MAIN 2>&1 | tee -a $OUT/walks.log
timeout 20000 python3 /lookout/walks.py --layout $LAYOUT --out $OUT --mode $CHECK --check-how away 2>&1 | tee -a $OUT/walks.log

# a fresh sim without the lookouts
kill $BRIDGE_PID $GZ_PID 2>/dev/null
pkill -f parameter_bridge; pkill -f "ign gazebo"
for i in $(seq 1 60); do pgrep -f "ign gazebo|parameter_bridge" > /dev/null || break; sleep 1; done
pkill -9 -f "ign gazebo|parameter_bridge"; sleep 2
NO_LOOKOUTS=1 SIM_TAG=_absent source /lookout/sim_up.sh
timeout 20000 python3 /lookout/walks.py --layout $LAYOUT --out $OUT --mode $CHECK --check-how absent 2>&1 | tee -a $OUT/walks.log
echo "[run_layout] done"
