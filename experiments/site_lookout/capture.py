#!/usr/bin/env python3
"""Save one PointCloud2 per topic to npz (xyz in the sensor frame).
Usage: capture.py <out.npz> <topic>..."""
import sys, time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

out, topics = sys.argv[1], sys.argv[2:]
rclpy.init(); n = rclpy.create_node('capture'); got = {}
for t in topics:
    n.create_subscription(PointCloud2, t, lambda m, t=t: got.setdefault(t, m), qos_profile_sensor_data)
t0 = time.time()
while len(got) < len(topics) and time.time() - t0 < 300:
    rclpy.spin_once(n, timeout_sec=0.5)
arr = {}
for t, m in got.items():
    p = pc2.read_points_numpy(m, field_names=['x', 'y', 'z'], skip_nans=True)
    p = p[np.isfinite(p).all(1)]
    arr[t.strip('/').replace('/', '_')] = p.astype(np.float32)
    print(t, len(p), 'points')
np.savez_compressed(out, **arr)
print('missing', [t for t in topics if t not in got])
