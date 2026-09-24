#!/usr/bin/env python3
"""Where newly resolved lesions were resolved from (host python, descriptive).

Usage: cam_mechanism.py <run> [<run> ...] [--thr 25]
For each lesion unresolved at the cue and resolved by the end (rendered camera,
front views only, lesion 90 dropped, as cam_score.py), the first frame after the
cue that shows it with >= thr px: range to the plate, view angle off the plate
normal, and seconds after the cue. Targets vs undrawn pool oaks.
"""
import csv, json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
POOL = {4, 18, 36, 39, 40, 42, 58, 69, 73}
thr = float(sys.argv[sys.argv.index("--thr") + 1]) if "--thr" in sys.argv else 25.0
runs = [a for a in sys.argv[1:] if not a.startswith("--") and os.path.isdir(a)]
L = json.load(open(os.path.join(HERE, "lesions.json")))
oak_of = {l["label"]: int(l["oak"].split("_")[-1]) for l in L["lesions"]}


def med(v):
    v = sorted(v)
    return v[len(v) // 2] if len(v) % 2 else 0.5 * (v[len(v) // 2 - 1] + v[len(v) // 2])


pooled = {"T": [], "Cp": []}
print(f"first frame after the cue with >= {thr:.0f} px, lesions newly resolved by the end")
print("run      group  n   range m median [min-max]   angle deg median   s after cue median")
for run in runs:
    S = json.load(open(os.path.join(run, "cam_score_rendered.json")))
    t0, cue, tg = S["t0"], S["cue_t_rel"], set(S["targets"])
    first = {}
    for r in csv.DictReader(open(os.path.join(run, "cam", "counts_geo.csv"))):
        lab = int(r["label"])
        if lab not in oak_of or lab == 90 or r["angle"] in (None, "") or float(r["angle"]) > 90.0:
            continue
        t = float(r["t"]) - t0
        if t > cue and float(r["npix"]) >= thr and lab not in first:
            first[lab] = (float(r["dist"]), float(r["angle"]), t - cue)
    for g, oaks in (("T", tg), ("Cp", POOL - tg)):
        rows = [first[int(l)] for l, v in S["per_lesion"].items()
                if oak_of[int(l)] in oaks and v["cue"] < thr and v["end"] >= thr and int(l) in first]
        pooled[g] += rows
        if rows:
            d = [x[0] for x in rows]
            print(f"{os.path.basename(run.rstrip('/')):8s} {g:5s} {len(rows):2d}  {med(d):5.1f} [{min(d):.1f}-{max(d):.1f}]"
                  f"          {med([x[1] for x in rows]):5.0f}              {med([x[2] for x in rows]):5.0f}")
for g, rows in pooled.items():
    if rows:
        d = [x[0] for x in rows]
        print(f"{'pooled':8s} {g:5s} {len(rows):2d}  {med(d):5.1f} [{min(d):.1f}-{max(d):.1f}]"
              f"          {med([x[1] for x in rows]):5.0f}              {med([x[2] for x in rows]):5.0f}")
