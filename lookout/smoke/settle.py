#!/usr/bin/env python3
"""Husky settling on flat ground: sample ground-truth odom (z, roll, pitch)
of every lookout for a while with the world running."""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

names = sys.argv[1].split(","); dur = float(sys.argv[2])
rclpy.init(); n = Node("settle"); last = {}
for r in names:
    n.create_subscription(Odometry, f"/{r}/odom_ground_truth", lambda m, r=r: last.__setitem__(r, m), qos_profile_sensor_data)
t0 = time.time(); nxt = 0
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - t0 >= nxt and len(last) == len(names):
        nxt += 5
        out = []
        for r in names:
            m = last[r]; p = m.pose.pose.position; q = m.pose.pose.orientation
            roll = math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)))
            pitch = math.degrees(math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x)))))
            out.append(f"{r} t={m.header.stamp.sec+m.header.stamp.nanosec*1e-9:7.2f} x={p.x:.4f} y={p.y:.4f} z={p.z:.5f} roll={roll:.4f} pitch={pitch:.4f}")
        print(f"wall {time.time()-t0:5.0f} | " + " | ".join(out), flush=True)
