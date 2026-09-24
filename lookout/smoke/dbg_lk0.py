import argparse, math, os, sys, time
import numpy as np
sys.path.insert(0, "/lookout"); sys.path.insert(0, "/lookout/smoke")
import gz, rclpy
from scans import ScanNode
W = "lookout_smoke"
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); a = ap.parse_args()
rclpy.init()
n = ScanNode(W, {"lk0": "/lk0/velodyne_points", "mulcher": "/mulcher/velodyne_points"}, odoms=["lk0"])
assert n.wait_all(odoms=["lk0"])
time.sleep(3.0); gz.pause(W, True); n.drain(0.5)
out = {}
for i, (px, py) in enumerate([(1000, 1000), (0, -30), (0, -10), (5, -20)]):
    gz.set_pose(W, "person", px, py, 0, 0.0)
    sc, info = n.step_fresh()
    n.spin_until(lambda: False, 0.3)
    o = n.odom["lk0"].pose.pose
    d = gz.decode(sc["lk0"])
    out.update({f"c{i}_{k}": d[k] for k in ("x", "y", "z", "range")})
    out[f"c{i}_odom"] = np.array([o.position.x, o.position.y, o.position.z, o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w])
    out[f"c{i}_spot"] = np.array([px, py])
    print("DBG", i, px, py, info["stamps"], out[f"c{i}_odom"].round(3).tolist(), "frame", sc["lk0"].header.frame_id)
np.savez_compressed(os.path.join(a.out, "dbg_lk0.npz"), **out)
print("DBG DONE")
