#!/bin/bash
# Do static lookouts (spawn_static.py) work? On the flat bullet world of
# wobble.sh: two static Huskies at lookout headings. Checks that ground-truth
# odom and lidar clouds arrive, that the pose holds (odom logged to 600 s sim),
# and that set_pose still moves one (the "away" check walks).
#   static.sh <out>
set -u
OUT=$1; W=lookout_wobble_bullet
mkdir -p $OUT
ign gazebo -s -r -v 3 ${GZ_HEADLESS:-} /runs/lookout/worlds/$W.sdf > $OUT/gz.log 2>&1 &
for i in $(seq 1 120); do ign service -l 2>/dev/null | grep -q "/world/$W/create" && break; sleep 1; done
B=$OUT/bridge.yaml; : > $B
for a in "st0 -20 -5 3.12639" "st1 0 -5 0.30752"; do
  set -- $a
  python3 /lookout/spawn_static.py $W $1 $2 $3 $4 > $OUT/spawn_$1.log 2>&1 || echo "[static] spawn $1 failed"
  cat >> $B <<YAML
- {ros_topic_name: $1/velodyne_points, gz_topic_name: $1/laser_scan/points, ros_type_name: sensor_msgs/msg/PointCloud2, gz_type_name: ignition.msgs.PointCloudPacked, direction: GZ_TO_ROS}
- {ros_topic_name: $1/odom_ground_truth, gz_topic_name: $1/odom_ground_truth, ros_type_name: nav_msgs/msg/Odometry, gz_type_name: ignition.msgs.Odometry, direction: GZ_TO_ROS}
YAML
done
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$B > $OUT/bridge.log 2>&1 &
sleep 5
for n in st0 st1; do
  timeout 20 ros2 topic echo /$n/odom_ground_truth --once > $OUT/odom_$n.txt 2>&1 && echo "[static] $n odom ok" || echo "[static] $n NO odom"
  timeout 20 ros2 topic hz /$n/velodyne_points > $OUT/hz_$n.txt 2>&1; echo "[static] $n cloud rate: $(grep average $OUT/hz_$n.txt | tail -1)"
done
python3 - <<EOF
import sys; sys.path.insert(0, "/lookout")
import gz
for n in ("st0", "st1"):
    mp, sp = gz.model_info(n, "front_laser")
    print(f"[static] {n} model pose {mp}  lidar offset {sp}")
EOF
python3 /lookout/smoke/wobble_log.py st0,st1 600 $OUT/odom.csv   # sim time; logging starts ~115 s
python3 - <<EOF
import sys; sys.path.insert(0, "/lookout")
import gz
gz.set_pose("$W", "st1", 1000.0, 1000.0, 0.2, 0.0)
EOF
sleep 3
timeout 20 ros2 topic echo /st1/odom_ground_truth --once --field pose.pose.position 2>&1 | head -4 | sed 's/^/[static] st1 after set_pose: /'
echo "[static] done"
