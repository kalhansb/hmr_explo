#!/usr/bin/env python3
"""Part 5 calibration (open ground, before any walk). In the container, with
lookout_calib running, a Husky `lk0` spawned at (0, -60) facing -y and the
bridge up (lookout/calib.sh).

For each lidar mount (Husky front_laser, mulcher roof) the person stands
5, 7.5, ..., 40 m from the lidar (horizontal distance from the lidar's
actual pose), 10 scans at each distance: in front of the Husky, behind the
mulcher (its head blocks the front). Two orientations: `toward` (walking
straight at the sensor, the main case: narrowest profile) and `across`
(walking across the line of sight, side-on). Each scan is run through
detect.py exactly as in the walks: background learned with the person parked
far away, new points, 3D clusters (and the horizontal-linkage variant),
candidate per variant, alarm = candidate at two scans in a row within 1 m,
hit within 1 m of the person; best case >= 5 non-static returns in the box.

Outputs in --out:
  calib_scans.csv    one row per mount x orientation x distance x scan
  calib_summary.csv  per mount x orientation x distance: rate of each rule
  calib_ranges.json  per mount x orientation x rule: reliable range (the rule
                     fires in >= 90 % of scans at that distance and every
                     shorter one) and maximum range (fires at least once)
  events.jsonl
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, "/lookout")
import detect as D  # noqa: E402
import gz  # noqa: E402
import walks as WK  # noqa: E402  (variants, settle tolerances)

W = "lookout_calib"
DISTS = [5.0 + 2.5 * i for i in range(15)]
N_SCANS = 10
ORIENTS = {"toward": 0.0, "across": math.pi / 2}   # person heading relative to "towards the sensor"
YAW_OFF = math.pi / 2                               # model yaw = heading + pi/2 (mesh faces -y)
PARK = (1000.0, 1000.0)
RELIABLE = 0.9


def main():
    import rclpy
    from scans import ScanNode
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    ev = open(os.path.join(a.out, "events.jsonl"), "a")

    def log(kind, **kw):
        ev.write(json.dumps({"t_wall": time.time(), "kind": kind, **kw}, default=str) + "\n"); ev.flush()
        print(f"[calib] {kind} {json.dumps(kw, default=str)[:300]}", flush=True)

    rclpy.init()
    node = ScanNode(W, {"lk0": "/lk0/velodyne_points", "mulcher": "/mulcher/velodyne_points"}, odoms=["lk0"],
                    name="calib")
    if not node.wait_all(odoms=["lk0"]):
        raise RuntimeError("no clouds/odom")
    gz.pause(W, True)
    node.drain(0.5)
    sensor = {"lk0": gz.model_info("lk0", "front_laser")[1], "mulcher": gz.model_info("mulcher", "roof_lidar")[1]}
    mp, _ = gz.model_info("mulcher")
    mul = gz.compose(mp, sensor["mulcher"])

    def pose(n):
        return gz.odom_sensor(node.odom["lk0"], sensor["lk0"]) if n == "lk0" else mul

    # settle the Husky (see walks.settle)
    n_win = int(round(WK.SETTLE_WINDOW_S / 0.1))
    hist, t_sim = [], 0.0
    while True:
        node.step_fresh()
        t_sim += 0.1
        hist.append(pose("lk0"))
        if len(hist) > n_win:
            (R0, t0), (R1, t1) = hist[-1 - n_win], hist[-1]
            dp = float(np.linalg.norm(t1 - t0))
            da = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R0.T @ R1) - 1) / 2))))
            if dp <= WK.SETTLE_TOL_M and da <= WK.SETTLE_TOL_DEG:
                log("settled", sim_s=round(t_sim, 1), window=[dp, da])
                break
            if t_sim > WK.SETTLE_MAX_S:
                raise RuntimeError(f"lk0 not at rest: {dp} m {da} deg")

    # backgrounds, person parked
    gz.set_pose(W, "person", PARK[0], PARK[1], 0.0, 0.0)
    node.step_fresh()
    acc = {n: [] for n in ("lk0", "mulcher")}
    for _ in range(10):
        msgs, _ = node.step_fresh()
        for n in acc:
            acc[n].append(gz.decode(msgs[n])["range"])
    bg = {n: D.learn_background(v) for n, v in acc.items()}
    poses = {n: pose(n) for n in acc}
    for n in acc:
        np.savez_compressed(os.path.join(a.out, f"bg_{n}.npz"), bg=bg[n].astype(np.float32), R=poses[n][0],
                            t=poses[n][1])
    log("backgrounds", poses={n: poses[n][1].tolist() for n in poses},
        yaw_deg={n: math.degrees(math.atan2(poses[n][0][1, 0], poses[n][0][0, 0])) for n in poses})

    rules = [f"alarm_{v}" for v in WK.VARIANTS] + [f"cand_{v}" for v in WK.VARIANTS] + ["best"]
    fs = ["mount", "orient", "d_set_m", "scan", "d_m", "px", "py", "lx", "ly", "lz", "stamp", "n_new",
          "best_all", "best_ns", "best_rings", "best_ring_list", "near_n", "near_rings", "near_d_m",
          "nearxy_n", "nearxy_rings", "nearxy_d_m"] + rules
    fc = open(os.path.join(a.out, "calib_scans.csv"), "w", newline="")
    wc = csv.DictWriter(fc, fieldnames=fs)
    wc.writeheader()
    summary = []
    for mount in ("lk0", "mulcher"):
        R, t = poses[mount]
        fwd = math.atan2(R[1, 0], R[0, 0])
        away = fwd if mount == "lk0" else fwd + math.pi          # Husky: in front; mulcher: behind
        for orient, rel in ORIENTS.items():
            for d in DISTS:
                px, py = t[0] + d * math.cos(away), t[1] + d * math.sin(away)
                heading = away + math.pi + rel
                gz.set_pose(W, "person", px, py, 0.0, heading + YAW_OFF)
                det = {v: D.Detector(mount, min_pts=mp_) for v, (mp_, _) in WK.VARIANTS.items()}
                rows = []
                for s in range(N_SCANS):
                    msgs, info = node.step_fresh()
                    dd = gz.decode(msgs[mount])
                    Rn, tn = pose(mount)
                    Pw = gz.to_world(dd, Rn, tn)
                    r, rings = dd["range"], dd["ring"].astype(np.int64)
                    m = D.new_mask(r, bg[mount], D.RULE.closer)
                    cls = {False: D.clusters(Pw[m], rings[m]), True: D.clusters(Pw[m], rings[m], xy=True)}
                    row = {"mount": mount, "orient": orient, "d_set_m": d, "scan": s,
                           "d_m": round(math.hypot(px - tn[0], py - tn[1]), 3), "px": round(px, 3),
                           "py": round(py, 3), "lx": round(tn[0], 4), "ly": round(tn[1], 4), "lz": round(tn[2], 4),
                           "stamp": info["stamps"][mount], "n_new": int(m.sum())}
                    for v, (mp_, xy) in WK.VARIANTS.items():
                        alarms, cands = det[v].step_clusters(cls[xy], (px, py))
                        row[f"cand_{v}"] = int(any(math.hypot(c["centroid"][0] - px, c["centroid"][1] - py)
                                                   <= D.RULE.hit for c in cands))
                        row[f"alarm_{v}"] = int(any(x["hit"] for x in alarms))
                    b_all, b_ns, _, b_rings = D.best_case_count(Pw, r, bg[mount], px, py, heading + YAW_OFF,
                                                                 rings=rings)
                    near = D.nearest(cls[False], (px, py))
                    nearxy = D.nearest(cls[True], (px, py))
                    row.update(best_all=b_all, best_ns=b_ns, best_rings=len(b_rings),
                               best_ring_list=" ".join(map(str, b_rings)), best=int(b_ns >= D.RULE.box_min),
                               near_n=near[0], near_rings=near[1], near_d_m=None if near[2] is None else round(near[2], 3),
                               nearxy_n=nearxy[0], nearxy_rings=nearxy[1],
                               nearxy_d_m=None if nearxy[2] is None else round(nearxy[2], 3))
                    wc.writerow(row)
                    rows.append(row)
                fc.flush()
                srow = {"mount": mount, "orient": orient, "d_m": d,
                        "box_ns_median": float(np.median([x["best_ns"] for x in rows])),
                        "box_rings_median": float(np.median([x["best_rings"] for x in rows])),
                        "near_n_median": float(np.median([x["near_n"] for x in rows])),
                        "near_rings_median": float(np.median([x["near_rings"] for x in rows]))}
                for rl in rules:
                    vals = [x[rl] for x in rows[1:]] if rl.startswith("alarm") else [x[rl] for x in rows]
                    srow[rl] = sum(vals) / len(vals)          # alarms: scans 2..10 (need a previous scan)
                summary.append(srow)
                log("distance", mount=mount, orient=orient, d=d, box=srow["box_ns_median"],
                    rings=srow["box_rings_median"], cand_m5=srow["cand_m5"], alarm_m5=srow["alarm_m5"],
                    alarm_xy5=srow["alarm_xy5"], best=srow["best"])
    fc.close()
    with open(os.path.join(a.out, "calib_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0]))
        w.writeheader()
        w.writerows(summary)
    ranges = {}
    for mount in ("lk0", "mulcher"):
        for orient in ORIENTS:
            ss = sorted((x for x in summary if x["mount"] == mount and x["orient"] == orient), key=lambda x: x["d_m"])
            for rl in rules:
                rel = None
                for x in ss:
                    if x[rl] >= RELIABLE:
                        rel = x["d_m"]
                    else:
                        break
                fired = [x["d_m"] for x in ss if x[rl] > 0]
                ranges[f"{mount}/{orient}/{rl}"] = {"reliable_m": rel, "max_m": max(fired) if fired else None}
    json.dump(ranges, open(os.path.join(a.out, "calib_ranges.json"), "w"), indent=1)
    for k, v in ranges.items():
        if "/cand_" not in k:
            print(f"[calib] {k:28s} reliable {v['reliable_m']}  max {v['max_m']}", flush=True)
    log("done", retries=node.retries, svc_retries=len(gz.SVC_RETRIES))


if __name__ == "__main__":
    main()
