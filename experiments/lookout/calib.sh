#!/bin/bash
# Part 5 calibration (in the container). Usage: calib.sh [out]
set -u
OUT=${1:-/runs/lookout/calib}; mkdir -p $OUT
W=lookout_calib
ign gazebo -s -r -v 3 ${GZ_HEADLESS:-} /runs/lookout/worlds/$W.sdf > $OUT/gz.log 2>&1 &
for i in $(seq 1 120); do ign service -l 2>/dev/null | grep -q "/world/$W/create" && break; sleep 1; done
ros2 launch hmr_sim spawn_robot.launch.py robot_name:=lk0 \
  sdf_file:=COSTAR_HUSKY_SENSOR_CONFIG_LIDAR/model.sdf world:=$W \
  x:=0.0 y:=-60.0 z:=0.3 roll:=0.0 pitch:=0.0 yaw:=-1.5708 use_imu:=true > $OUT/spawn.log 2>&1
cat > $OUT/bridge.yaml <<YAML
- {ros_topic_name: clock, gz_topic_name: clock, ros_type_name: rosgraph_msgs/msg/Clock, gz_type_name: ignition.msgs.Clock, direction: GZ_TO_ROS}
- {ros_topic_name: lk0/velodyne_points, gz_topic_name: lk0/laser_scan/points, ros_type_name: sensor_msgs/msg/PointCloud2, gz_type_name: ignition.msgs.PointCloudPacked, direction: GZ_TO_ROS}
- {ros_topic_name: lk0/odom_ground_truth, gz_topic_name: lk0/odom_ground_truth, ros_type_name: nav_msgs/msg/Odometry, gz_type_name: ignition.msgs.Odometry, direction: GZ_TO_ROS}
- {ros_topic_name: mulcher/velodyne_points, gz_topic_name: mulcher/laser_scan/points, ros_type_name: sensor_msgs/msg/PointCloud2, gz_type_name: ignition.msgs.PointCloudPacked, direction: GZ_TO_ROS}
YAML
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$OUT/bridge.yaml > $OUT/bridge.log 2>&1 &
sleep 3
timeout 7200 python3 /lookout/calib.py --out $OUT 2>&1 | tee $OUT/calib.log
