#!/usr/bin/env python3
"""Were any scans stale (showing the person one step back)? For every walk,
step and lidar: the returns within 1.5 m (horizontal) of the person's true
position, and their mean offset along the walking direction. A fresh scan sees
the body's near surface, roughly -0.2..+0.2 m; a scan rendered before the
person's last 0.5 m step sits ~0.5 m further back. Offset < -0.3 m with >= 3
returns counts as stale.
  stale_audit.py <run dir> [walk substring]"""
import glob
import os
import sys
import collections

import numpy as np

d = sys.argv[1]
sub = sys.argv[2] if len(sys.argv) > 2 else ""
tot = collections.Counter()
for f in sorted(glob.glob(os.path.join(d, "scans", "*.npz"))):
    w = os.path.basename(f).split("__")[0]
    if sub not in w:
        continue
    Z = np.load(f)
    xy = Z["steps_xy"]
    dv = np.gradient(xy, axis=0)
    dv /= np.linalg.norm(dv, axis=1)[:, None]
    line = []
    for li, n in enumerate(Z["lidars"]):
        st = n_ok = 0
        for k in range(len(xy)):
            s = (Z["step"] == k) & (Z["lidar"] == li)
            P = Z["xyz"][s][:, :2].astype(float) - xy[k]
            near = np.hypot(P[:, 0], P[:, 1]) < 1.5
            if near.sum() < 3:
                continue
            a = (P[near] @ dv[k]).mean()
            n_ok += 1
            st += a < -0.3
        tot[(n, "steps")] += n_ok
        tot[(n, "stale")] += st
        line.append(f"{n} {st}/{n_ok}")
    print(w, " ".join(line))
print("total:", {n: f"{tot[(n, 'stale')]}/{tot[(n, 'steps')]}" for n in sorted({k[0] for k in tot})})
