#!/bin/bash
# Renderer smoke (in the container): layout world with its mulcher only (no
# Huskies), mulcher lidar bridged; times stepped scans and counts returns.
#   gpu_smoke.sh <layout> <out>
set -u
LAYOUT=$1; OUT=$2; mkdir -p $OUT
W=lookout_$LAYOUT
ign gazebo -s -r -v 4 $GZ_HEADLESS /runs/lookout/worlds/$W.sdf > $OUT/gz.log 2>&1 &
for i in $(seq 1 120); do ign service -l 2>/dev/null | grep -q "/world/$W/create" && break; sleep 1; done
cat > $OUT/bridge.yaml <<YAML
- {ros_topic_name: clock, gz_topic_name: clock, ros_type_name: rosgraph_msgs/msg/Clock, gz_type_name: ignition.msgs.Clock, direction: GZ_TO_ROS}
- {ros_topic_name: mulcher/velodyne_points, gz_topic_name: mulcher/laser_scan/points, ros_type_name: sensor_msgs/msg/PointCloud2, gz_type_name: ignition.msgs.PointCloudPacked, direction: GZ_TO_ROS}
YAML
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$OUT/bridge.yaml > $OUT/bridge.log 2>&1 &
sleep 3
timeout 15 ign topic -e -t /stats -n 1 2>/dev/null | grep -E "real_time_factor" | head -1
timeout 300 python3 - "$W" <<'PY'
import sys, time
import numpy as np
sys.path.insert(0, "/lookout")
import rclpy, gz
from scans import ScanNode
W = sys.argv[1]
rclpy.init()
node = ScanNode(W, {"mulcher": "/mulcher/velodyne_points"}, odoms=[], name="gpu_smoke")
print("clouds:", node.wait_all(odoms=[]), flush=True)
gz.pause(W, True); node.drain(0.5)
node.step_fresh()
ts, fin = [], []
for i in range(20):
    t0 = time.time(); msgs, info = node.step_fresh(); ts.append(time.time() - t0)
    r = gz.decode(msgs["mulcher"])["range"]; fin.append(int(np.isfinite(r).sum()))
print(f"step wall s: median {np.median(ts):.3f} min {min(ts):.3f} max {max(ts):.3f}", flush=True)
print(f"finite returns per scan: median {np.median(fin):.0f} min {min(fin)} max {max(fin)} of {r.size}", flush=True)
r = gz.decode(msgs["mulcher"])["range"]
PY
grep -iE "renderer|vendor|GL_VERSION|EGL|headless|llvmpipe|nvidia" /gzhome/.ignition/rendering/ogre2.log 2>/dev/null | tail -8
grep -iE "headless|egl|error" $OUT/gz.log | head -8
