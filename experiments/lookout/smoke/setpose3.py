#!/usr/bin/env python3
"""set_pose on the (dynamic) mulcher, with the per-lidar freshness test.

Per case (mulcher x, y, yaw): Gazebo readback; lk0's circular-mean azimuth and
range in the mulcher cloud vs the value expected from the commanded pose;
head near hits (< 3 m, rings 5-13) must stay at azimuth 0; lk0's view of the
body (world x/y extent near the commanded origin).
"""
import argparse, json, math, os, subprocess, sys, time
import numpy as np
sys.path.insert(0, "/lookout"); sys.path.insert(0, "/lookout/smoke")
import gz
import rclpy
from scans import ScanNode
from probe import husky_sensor_pose, to_world

W = "lookout_smoke"


def cmean(deg):
    a = np.radians(deg)
    return float(np.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean())))


def readback():
    out = subprocess.run(["ign", "model", "-m", "mulcher", "-p"], capture_output=True, text=True, timeout=30)
    return out.stdout.strip()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); a = ap.parse_args()
    rclpy.init()
    n = ScanNode(W, {"lk0": "/lk0/velodyne_points", "mulcher": "/mulcher/velodyne_points"}, odoms=["lk0"])
    assert n.wait_all(odoms=["lk0"])
    time.sleep(3.0); gz.pause(W, True); n.drain(0.5)
    n.spin_until(lambda: False, 0.5)
    R_h, t_h = husky_sensor_pose(n.odom["lk0"])
    gz.set_pose(W, "person", 1000, 1000, 0, 0)
    n.step_fresh()
    rows, bad = [], 0
    for (mx, my, yd) in [(0, 0, 0), (0, 0, 90), (0, 0, 30), (0, 0, -60), (0, 0, 180), (5, 0, 0), (0, 0, 330), (0, 0, 0)]:
        yaw = math.radians(yd)
        gz.set_pose(W, "mulcher", mx, my, 0, yaw)
        sc, info = n.step_fresh()
        h, m = gz.decode(sc["lk0"]), gz.decode(sc["mulcher"])
        # lk0 in the mulcher lidar frame, expected
        dx, dy = t_h[0] - mx, t_h[1] - my
        exp_az = math.degrees(math.remainder(math.atan2(dy, dx) - yaw, 2 * math.pi))
        exp_r = math.hypot(dx, dy)
        r = m["range"]; fin = np.isfinite(r)
        zw = m["z"] + 2.1
        obj = fin & (r > 3) & (zw > 0.15)
        az = np.degrees(np.arctan2(m["y"], m["x"]))
        head = fin & (r < 3.0); head[:5] = False; head[14:] = False
        Pw = to_world(h, R_h, t_h)
        near = np.isfinite(Pw[..., 2]) & (np.hypot(Pw[..., 0] - mx, Pw[..., 1] - my) < 4) & (Pw[..., 2] > 0.1)
        row = {"case": [mx, my, yd], "readback": [s.strip() for s in readback().split("\n")[-2:]],
               "lk0_az": cmean(az[obj]) if obj.any() else None, "lk0_az_exp": exp_az,
               "lk0_r": float(np.median(r[obj])) if obj.any() else None, "lk0_r_exp_xy": exp_r, "lk0_n": int(obj.sum()),
               "head_az": cmean(az[head]), "head_n": int(head.sum()),
               "body_seen_by_lk0": {"n": int(near.sum()),
                                    "x": [float(Pw[near][:, 0].min()), float(Pw[near][:, 0].max())] if near.any() else None,
                                    "y_min": float(Pw[near][:, 1].min()) if near.any() else None},
               "stamps": info["stamps"], "retries": info["retries"], "wall_s": round(info["wall_s"], 2)}
        ok = row["lk0_az"] is not None and abs(math.remainder(math.radians(row["lk0_az"] - exp_az), 2 * math.pi)) < math.radians(1.0) \
            and abs(row["head_az"]) < 0.5 and row["head_n"] == rows[0]["head_n"] if rows else True
        row["ok"] = bool(ok); bad += (not ok)
        rows.append(row)
        print("SP3", json.dumps(row))
    json.dump(rows, open(os.path.join(a.out, "setpose3.json"), "w"), indent=1)
    print("SP3 DONE bad", bad)


if __name__ == "__main__":
    main()
