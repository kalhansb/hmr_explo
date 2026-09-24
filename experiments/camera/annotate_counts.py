#!/usr/bin/env python3
"""Add view geometry to the rendered camera counts (container python).

Usage: annotate_counts.py <run_dir> [lesions.json]
Reads <run>/cam/counts.csv and <run>/gt_poses.npz; writes
<run>/cam/counts_geo.csv = counts + dist (m, camera to plate face) and
angle (deg between the plate normal and the direction to the camera;
> 90 = the camera is behind the plate). Poses are interpolated at the frame
time (yaw unwrapped); camera = base_link + (0.30, 0, 0.55) as in the SDF.
Bark rows (labels 130-148) get dist/angle of the trunk axis, angle blank.
"""
import csv, json, math, os, sys
import numpy as np

run = sys.argv[1]
lj = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "lesions.json")
L = json.load(open(lj))
les = {l["label"]: l for l in L["lesions"]}
P = np.load(os.path.join(run, "gt_poses.npz"))
MOUNT = np.array([0.30, 0.0, 0.55])


def rotm(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


pose = {}
for rb in ("atlas", "bestla"):
    pose[rb] = (P[f"{rb}_t"], P[f"{rb}_xyz"], np.unwrap(P[f"{rb}_rpy"], axis=0))
cache = {}


def cam_at(rb, t):
    k = (rb, t)
    if k not in cache:
        tt, xyz, rpy = pose[rb]
        x = np.array([np.interp(t, tt, xyz[:, c]) for c in range(3)])
        a = np.array([np.interp(t, tt, rpy[:, c]) for c in range(3)])
        cache[k] = x + rotm(*a) @ MOUNT
    return cache[k]


src = os.path.join(run, "cam", "counts.csv")
with open(src) as f, open(os.path.join(run, "cam", "counts_geo.csv"), "w") as g:
    rd = csv.DictReader(f)
    g.write(",".join(rd.fieldnames) + ",dist,angle\n")
    for r in rd:
        lab = int(r["label"]); t = float(r["t"])
        cam = cam_at(r["robot"], t)
        if lab in les:
            d = cam - np.array(les[lab]["face_world"])
            dist = float(np.linalg.norm(d))
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(d @ np.array(les[lab]["normal"])) / dist))))
            g.write(",".join(r[k] for k in rd.fieldnames) + f",{dist:.2f},{ang:.1f}\n")
        else:
            g.write(",".join(r[k] for k in rd.fieldnames) + ",,\n")
print("wrote", os.path.join(run, "cam", "counts_geo.csv"))
