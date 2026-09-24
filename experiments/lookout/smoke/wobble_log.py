#!/usr/bin/env python3
"""Log every ground-truth odom message of the given robots until sim time T
(or 45 min wall): name, t (sim s), x, y, z, roll, pitch, yaw (deg).
  wobble_log.py wb0,wb1,... <T> <out.csv>"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

names, T, out = sys.argv[1].split(","), float(sys.argv[2]), sys.argv[3]
rclpy.init()
node = Node("wobble_log")
f = open(out, "w")
f.write("name,t,x,y,z,roll,pitch,yaw\n")
t_max = [0.0]


def cb(m, r):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    f.write(f"{r},{t:.3f},{p.x:.6f},{p.y:.6f},{p.z:.6f},{roll:.5f},{pitch:.5f},{yaw:.5f}\n")
    t_max[0] = max(t_max[0], t)


for r in names:
    node.create_subscription(Odometry, f"/{r}/odom_ground_truth", lambda m, r=r: cb(m, r), qos_profile_sensor_data)
w0, nxt = time.time(), 0.0
while t_max[0] < T and time.time() - w0 < 2700:
    rclpy.spin_once(node, timeout_sec=0.05)
    if t_max[0] >= nxt:
        print(f"[wobble_log] sim {t_max[0]:.0f} s, wall {time.time() - w0:.0f} s", flush=True)
        nxt += 60.0
f.close()
