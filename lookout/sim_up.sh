#!/bin/bash
# Source from a layout script (in the container): starts Gazebo on the layout
# world (server, running), spawns the lookout Huskies and starts the bridge.
#   LAYOUT=L2 OUT=/runs/lookout/L2_x [AT_POST=1] source /lookout/sim_up.sh
# AT_POST=1 spawns each lookout on its post, static (the experiment, since
# 2026-09-24; spawn_static.py says why); unset, beside the mulcher and free to
# drive (the drive-out of run_layout_messenger.sh). NO_LOOKOUTS=1 spawns none (the "absent" check
# walks); SIM_TAG suffixes this sim's log files (a second sim in one run).
# Sets W (world name), LOOKOUTS (names), GZ_PID and BRIDGE_PID.
W=lookout_$LAYOUT
mkdir -p $OUT
T=${SIM_TAG:-}
ign gazebo -s -r -v 3 ${GZ_HEADLESS:-} /runs/lookout/worlds/$W.sdf > $OUT/gz$T.log 2>&1 &
GZ_PID=$!
for i in $(seq 1 120); do ign service -l 2>/dev/null | grep -q "/world/$W/create" && break; sleep 1; done
ign service -l 2>/dev/null | grep -q "/world/$W/create" || { echo "[sim_up] no world $W"; exit 1; }

# name x y z yaw, one line per lookout (spawn pose, or the post with AT_POST=1)
SPAWNS=$(python3 - <<EOF
import yaml
c = yaml.safe_load(open("/lookout/config/$LAYOUT.yaml"))
for l in ([] if "${NO_LOOKOUTS:-0}" == "1" else c["lookouts"]):
    s = l["spawn"]
    if "${AT_POST:-0}" == "1":
        s = {"x": l["x"], "y": l["y"], "z": 0.3, "yaw": l["yaw"]}
    print(l["name"], s["x"], s["y"], s["z"], s["yaw"])
EOF
)
LOOKOUTS=$(echo "$SPAWNS" | awk '{print $1}' | xargs)
echo "[sim_up] $W lookouts: ${LOOKOUTS:-none} (AT_POST=${AT_POST:-0}, NO_LOOKOUTS=${NO_LOOKOUTS:-0})"

B=$OUT/bridge$T.yaml
cat > $B <<YAML
- {ros_topic_name: clock, gz_topic_name: clock, ros_type_name: rosgraph_msgs/msg/Clock, gz_type_name: ignition.msgs.Clock, direction: GZ_TO_ROS}
- {ros_topic_name: mulcher/velodyne_points, gz_topic_name: mulcher/laser_scan/points, ros_type_name: sensor_msgs/msg/PointCloud2, gz_type_name: ignition.msgs.PointCloudPacked, direction: GZ_TO_ROS}
YAML
while read -r n x y z yaw; do
  [ -z "$n" ] && continue
  if [ "${AT_POST:-0}" = "1" ]; then
    # static on the post: a parked Husky rocks in DART+bullet (spawn_static.py)
    python3 /lookout/spawn_static.py $W $n $x $y $yaw > $OUT/spawn_$n$T.log 2>&1 || { echo "[sim_up] spawn $n failed"; exit 1; }
  else
    ros2 launch hmr_sim spawn_robot.launch.py robot_name:=$n \
      sdf_file:=COSTAR_HUSKY_SENSOR_CONFIG_LIDAR/model.sdf world:=$W \
      x:=$x y:=$y z:=$z roll:=0.0 pitch:=0.0 yaw:=$yaw use_imu:=true > $OUT/spawn_$n$T.log 2>&1
  fi
  cat >> $B <<YAML
- {ros_topic_name: $n/velodyne_points, gz_topic_name: $n/laser_scan/points, ros_type_name: sensor_msgs/msg/PointCloud2, gz_type_name: ignition.msgs.PointCloudPacked, direction: GZ_TO_ROS}
- {ros_topic_name: $n/imu/data, gz_topic_name: $n/imu/data, ros_type_name: sensor_msgs/msg/Imu, gz_type_name: ignition.msgs.IMU, direction: GZ_TO_ROS}
- {ros_topic_name: $n/odom_ground_truth, gz_topic_name: $n/odom_ground_truth, ros_type_name: nav_msgs/msg/Odometry, gz_type_name: ignition.msgs.Odometry, direction: GZ_TO_ROS}
- {ros_topic_name: $n/cmd_vel, gz_topic_name: $n/cmd_vel, ros_type_name: geometry_msgs/msg/Twist, gz_type_name: ignition.msgs.Twist, direction: ROS_TO_GZ}
YAML
done <<< "$SPAWNS"
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$B > $OUT/bridge$T.log 2>&1 &
BRIDGE_PID=$!
for n in $LOOKOUTS; do
  for i in $(seq 1 60); do timeout 8 ros2 topic echo /$n/odom_ground_truth --once > /dev/null 2>&1 && break; sleep 2; done
  echo "[sim_up] $n odom up"
done
