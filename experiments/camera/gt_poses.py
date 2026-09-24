#!/usr/bin/env python3
"""Ground-truth robot poses from a run's bag -> <run>/gt_poses.npz.

Container python. Usage: gt_poses.py <run_dir> <bag copy> [--hz 10]
Keys per robot r: r_t (sim s), r_xyz (N,3), r_rpy (N,3), decimated to --hz.
Also t0/t_end (run_start/run_end from atlas.events.jsonl) when present.
"""
import json, math, os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

run, bag = sys.argv[1], sys.argv[2]
hz = float(sys.argv[sys.argv.index("--hz") + 1]) if "--hz" in sys.argv else 10.0
t0 = t1 = None
ev = os.path.join(run, "atlas.events.jsonl")
if os.path.exists(ev):
    for l in open(ev):
        e = json.loads(l)
        if e.get("event") == "run_start": t0 = e["t0_sim_sec"]
        if e.get("event") == "run_end": t1 = e["t_sim_sec"]
r = rosbag2_py.SequentialCompressionReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
topics = {f"/{x}/odom_ground_truth": x for x in ("atlas", "bestla")}
r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
buf = {x: [] for x in topics.values()}
last = {x: -1e9 for x in topics.values()}
while r.has_next():
    tp, data, _ = r.read_next()
    m = deserialize_message(data, Odometry)
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    rb = topics[tp]
    if t - last[rb] < 1.0 / hz - 1e-6:
        continue
    last[rb] = t
    p = m.pose.pose.position; q = m.pose.pose.orientation
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    buf[rb].append((t, p.x, p.y, p.z, roll, pitch, yaw))
out = {"t0": t0 if t0 is not None else np.nan, "t_end": t1 if t1 is not None else np.nan}
for rb, v in buf.items():
    a = np.array(v)
    out[f"{rb}_t"] = a[:, 0]; out[f"{rb}_xyz"] = a[:, 1:4]; out[f"{rb}_rpy"] = a[:, 4:7]
np.savez_compressed(os.path.join(run, "gt_poses.npz"), **out)
# Cumulative GT path per robot at 1 s (host scripts have no numpy).
cum = {}
for rb, v in buf.items():
    a = np.array(v); d = np.r_[0.0, np.cumsum(np.hypot(*np.diff(a[:, 1:3], axis=0).T))]
    tt = np.arange(math.ceil(a[0, 0]), a[-1, 0], 1.0)
    cum[rb] = {"t": tt.round(1).tolist(), "m": np.interp(tt, a[:, 0], d).round(2).tolist()}
json.dump(cum, open(os.path.join(run, "gt_path_cum.json"), "w"))
print(os.path.basename(run.rstrip("/")), {k: len(v) for k, v in buf.items()}, "t0", t0, "t_end", t1)
