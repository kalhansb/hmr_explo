#!/usr/bin/env python3
"""Per-tree best frames of a camera run -> false-colour NDVI tiles (container python).

Usage: ndvi_tiles.py <run_dir> <out_dir> [--scale 0.5]
For each tree k in lesions.json, takes whichever robot's best_<robot>_<k>.npz
shows more lesion pixels, renders it with ndvi_png.render (front-view filter
not applied: these are the raw rendered label frames) and writes
<out_dir>/tree_<k>.png plus tiles.json (oak, t, lesion px, lesion labels seen).
"""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ndvi_png import render

run, out = sys.argv[1], sys.argv[2]
sc = float(sys.argv[sys.argv.index("--scale") + 1]) if "--scale" in sys.argv else 0.5
L = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lesions.json")))
os.makedirs(out, exist_ok=True)
meta = {}
for k, tr in enumerate(L["trees"]):
    best = None
    for rb in ("atlas", "bestla"):
        f = os.path.join(run, "cam", f"best_{rb}_{k}.npz")
        if not os.path.exists(f):
            continue
        lab = np.load(f)["labels"]
        n = int(((lab >= 10) & (lab <= 123)).sum())
        if best is None or n > best[1]:
            best = (f, n, rb, sorted(int(x) for x in np.unique(lab) if 10 <= x <= 123))
    if best is None:
        continue
    im = render(best[0])
    im = im.resize((int(im.width * sc), int(im.height * sc)))
    im.save(os.path.join(out, f"tree_{k}.png"), optimize=True)
    meta[k] = {"oak": tr[1], "robot": best[2],
               "t": float(np.load(best[0])["t"]), "lesion_px": best[1], "labels": best[3]}
json.dump(meta, open(os.path.join(out, "tiles.json"), "w"), indent=1)
print("wrote", len(meta), "tiles to", out)
