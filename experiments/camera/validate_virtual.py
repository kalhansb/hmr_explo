#!/usr/bin/env python3
"""Virtual vs rendered camera, same run, same frame times (host python).

Usage: validate_virtual.py <run_dir> [--virtual virtual_frames.csv]
Needs <run>/cam/counts.csv (rendered) and the virtual CSV made with
virtual_cam.py --frames <run>/cam/frames.csv. PLAN.md acceptance: median
per-lesion best-pixel ratio (virtual / rendered) within 0.8-1.25 and
resolved/unresolved (25 px) agreement >= 90 % over lesions.
"""
import csv, os, sys

run = sys.argv[1]
vname = sys.argv[sys.argv.index("--virtual") + 1] if "--virtual" in sys.argv else "virtual_frames.csv"


def load(p):
    d = {}
    with open(p) as f:
        for r in csv.DictReader(f):
            L = int(r["label"])
            if r.get("angle") not in (None, "") and float(r["angle"]) > 90.0:
                continue            # back views (PLAN.md deviation 1)
            if 10 <= L <= 123:
                d[(round(float(r["t"]), 1), r["robot"], L)] = float(r["npix"])
    return d


R = load(os.path.join(run, "cam", "counts_geo.csv"))
V = load(os.path.join(run, vname))
keys = set(R) | set(V)
# frame level
pairs = [(R.get(k, 0.0), V.get(k, 0.0)) for k in keys]
both = [(r, v) for r, v in pairs if r >= 25 or v >= 25]
agree = sum((r >= 25) == (v >= 25) for r, v in both)
rat = sorted(v / r for r, v in pairs if r >= 25 and v > 0)
print(f"frame-lesion pairs with either >= 25 px: {len(both)}; agree on >=25: {agree / max(1, len(both)):.3f}")
if rat:
    q = lambda p: rat[min(len(rat) - 1, int(p * len(rat)))]
    print(f"  ratio virtual/rendered where rendered >= 25 (n={len(rat)}): p10 {q(.1):.2f}  median {q(.5):.2f}  p90 {q(.9):.2f}")
only_r = sum(1 for r, v in pairs if r >= 25 and v < 1); only_v = sum(1 for r, v in pairs if v >= 25 and r < 1)
print(f"  rendered >=25 but virtual 0: {only_r}; virtual >=25 but rendered 0: {only_v}")
# lesion level (best over the run)
bestR, bestV = {}, {}
for (t, rb, L), n in R.items(): bestR[L] = max(bestR.get(L, 0), n)
for (t, rb, L), n in V.items(): bestV[L] = max(bestV.get(L, 0), n)
# lesion 90 is out of scoring (PLAN.md deviation 2); report it separately
print(f"  lesion 90 (dropped): rendered {bestR.get(90, 0):.0f} px, virtual {bestV.get(90, 0):.0f} px")
labs = [L for L in range(10, 124) if L != 90]
ag = sum((bestR.get(L, 0) >= 25) == (bestV.get(L, 0) >= 25) for L in labs)
lr = sorted(bestV[L] / bestR[L] for L in labs if bestR.get(L, 0) >= 25 and bestV.get(L, 0) > 0)
med = lr[len(lr) // 2] if lr else float("nan")
print(f"lesions: resolved(25) agreement {ag}/{len(labs)} = {ag / len(labs):.3f}; best-px ratio median {med:.2f} (n={len(lr)})")
dis = [(L, bestR.get(L, 0), bestV.get(L, 0)) for L in labs if (bestR.get(L, 0) >= 25) != (bestV.get(L, 0) >= 25)]
print("  disagreements (label, rendered, virtual):", [(L, round(a), round(b)) for L, a, b in dis])
ok = 0.8 <= med <= 1.25 and ag / len(labs) >= 0.90
print("ACCEPT" if ok else "REJECT", "(median ratio in 0.8-1.25 and agreement >= 0.90)")
