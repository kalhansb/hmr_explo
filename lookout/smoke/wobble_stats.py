#!/usr/bin/env python3
"""Summarise wobble.sh logs: per robot, when it first passes walks.py's settle
test (pose change over 2 s <= 0.2 mm and 0.002 deg), then after that: the
largest orientation change in any 0.1 s, how many 0.1 s changes exceed
0.05 deg, and the largest drift from the settled pose (walks.py relearns a
background beyond 5 mm / 0.01 deg).
  wobble_stats.py <odom.csv> [...]"""
import collections
import csv
import sys

import numpy as np

WIN, TOL_M, TOL_DEG = 2.0, 0.0002, 0.002


def rot_deg(a, b):
    """Rotation angle between orientations given as (roll, pitch, yaw) deg
    arrays (small angles: the difference vector's norm)."""
    d = a - b
    d[..., 2] = (d[..., 2] + 180.0) % 360.0 - 180.0
    return np.linalg.norm(d, axis=-1)


for path in sys.argv[1:]:
    rows = collections.defaultdict(list)
    for r in csv.DictReader(open(path)):
        rows[r["name"]].append([float(r[k]) for k in ("t", "x", "y", "z", "roll", "pitch", "yaw")])
    print(f"== {path}")
    for n in sorted(rows, key=lambda s: int(s[2:])):
        A = np.array(sorted(rows[n]))
        A = A[np.r_[True, np.diff(A[:, 0]) > 1e-9]]            # one sample per sim time
        t, P, O = A[:, 0], A[:, 1:4], A[:, 4:7]
        j = np.searchsorted(t, t - WIN)                          # sample WIN s earlier
        ok = (t - WIN >= t[0]) & (np.linalg.norm(P - P[j], axis=1) <= TOL_M) & (rot_deg(O, O[j]) <= TOL_DEG)
        if not ok.any():
            print(f"  {n}: never settled in {t[-1]:.0f} s")
            continue
        k0 = int(np.argmax(ok))
        ts = t[k0]
        s = slice(k0, None)
        j1 = np.searchsorted(t, t - 0.1)
        d01 = rot_deg(O, O[j1])[s]
        drift_deg = rot_deg(O[s], O[k0])
        drift_m = np.linalg.norm(P[s] - P[k0], axis=1)
        big = t[s][d01 > 0.05]
        first_big = f"{big[0]:.1f}" if len(big) else "-"
        print(f"  {n} yaw {O[k0, 2]:7.1f}: settled at {ts:5.1f} s; after that ({t[-1] - ts:.0f} s): "
              f"max 0.1 s change {d01.max():.4f} deg, {int((d01 > 0.05).sum())} samples > 0.05 deg "
              f"(first at {first_big} s); max drift {drift_deg.max():.4f} deg / {1000 * drift_m.max():.2f} mm; "
              f"end z {P[-1, 2]:.4f} m, tilt {np.hypot(O[-1, 0], O[-1, 1]):.3f} deg")
