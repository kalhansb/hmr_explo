#!/bin/bash
# Why does a parked lookout Husky tip? (lk2b, L2 pilot 2026-09-24: 0.52 deg in
# one 0.1 s step ~45 s after it had settled, nothing near it.) Ten Huskies on
# open flat ground (no trees), two at each lookout's heading, spawned as in the
# experiment (z 0.3, spawn_robot.launch.py); the world runs free and every
# ground-truth odom message is logged. <detector> is the DART collision
# detector: bullet (the experiment's worlds) or ode.
#   wobble.sh <detector> <out> [sim seconds, default 600]
set -u
DET=$1; OUT=$2; T=${3:-600}
W=lookout_wobble_$DET
mkdir -p $OUT
python3 - <<EOF
import sys; sys.path.insert(0, "/lookout")
import build_world as bw
s = bw.world_sdf("$W", [], bw.husky_lidar_block(), with_mulcher=False)
s = s.replace("<collision_detector>bullet</collision_detector>", "<collision_detector>$DET</collision_detector>")
assert "<collision_detector>$DET</collision_detector>" in s
open("/runs/lookout/worlds/$W.sdf", "w").write(s)
EOF
ign gazebo -s -r -v 3 ${GZ_HEADLESS:-} /runs/lookout/worlds/$W.sdf > $OUT/gz.log 2>&1 &
for i in $(seq 1 120); do ign service -l 2>/dev/null | grep -q "/world/$W/create" && break; sleep 1; done
# the five lookout headings (L2 lk2a lk2b, L3 lk3a lk3b lk3c), two robots each
YAWS=(3.12639 0.30752 1.58799 -2.84805 -0.30804)
B=$OUT/bridge.yaml; : > $B; NAMES=""
for i in $(seq 0 9); do
  n=wb$i; x=$(( (i % 5) * 10 - 20 )); y=$(( (i / 5) * 10 - 5 )); yaw=${YAWS[$((i % 5))]}
  ros2 launch hmr_sim spawn_robot.launch.py robot_name:=$n sdf_file:=COSTAR_HUSKY_SENSOR_CONFIG_LIDAR/model.sdf \
    world:=$W x:=$x y:=$y z:=0.3 roll:=0.0 pitch:=0.0 yaw:=$yaw use_imu:=true > $OUT/spawn_$n.log 2>&1
  echo "- {ros_topic_name: $n/odom_ground_truth, gz_topic_name: $n/odom_ground_truth, ros_type_name: nav_msgs/msg/Odometry, gz_type_name: ignition.msgs.Odometry, direction: GZ_TO_ROS}" >> $B
  NAMES="$NAMES,$n"
done
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$B > $OUT/bridge.log 2>&1 &
python3 /lookout/smoke/wobble_log.py ${NAMES#,} $T $OUT/odom.csv
echo "[wobble] done"
