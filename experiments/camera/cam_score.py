#!/usr/bin/env python3
"""Per-run readout for the NDVI-camera experiment (host python, no numpy).

Usage: cam_score.py <run_dir> [--counts cam/counts.csv] [--targets <yaml>]
                    [--at <t_rel,...>] [--tag rendered] [--quiet]
Counts: rendered <run>/cam/counts.csv or virtual <run>/virtual_cam.csv (any CSV
with t,robot,label,npix). Horizons default to the explore-done cue (manifest)
and the end; --at overrides (t_rel s, 'cue', 'end').
Writes <run>/cam_score_<tag>.json.

Per PLAN.md: a lesion is resolved at h if some frame up to h shows it with
>= 25 px (sensitivity 9 and 100). Primary metric: conditional gain = of the
lesions not resolved at the cue, the fraction resolved by the end, for
targets vs the undrawn pool oaks (primary control) and vs all non-targets.
"""
import csv, json, math, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
POOL = {4, 18, 36, 39, 40, 42, 58, 69, 73}
THR = (9, 25, 100)
SIG_PX = 0.05
# PLAN.md deviation 2: lesion 90 never renders (0 px in every frame of every
# camera run while the virtual camera sees it), so it is out of all scoring.
DROP = {90}


def arg(k, d=None):
    return sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d


def oak_num(name):
    return int(name.split("_")[-1])


def load_targets(path, trees):
    s = open(path).read()
    xs = [float(v) for v in re.search(r"target_x:\s*\[([^\]]*)\]", s).group(1).split(",")]
    ys = [float(v) for v in re.search(r"target_y:\s*\[([^\]]*)\]", s).group(1).split(",")]
    out = []
    for x, y in zip(xs, ys):
        best = min(trees, key=lambda t: (t[1] - x) ** 2 + (t[2] - y) ** 2)
        assert (best[1] - x) ** 2 + (best[2] - y) ** 2 < 0.5, (x, y)
        out.append(oak_num(best[0]))
    return out


def main():
    run = sys.argv[1].rstrip("/")
    tag = arg("--tag", "rendered")
    dflt = os.path.join(run, "cam", "counts_geo.csv")
    if not os.path.exists(dflt):
        dflt = os.path.join(run, "cam", "counts.csv")
        if "--counts" not in sys.argv:
            print("WARNING: no counts_geo.csv, back views not filtered", file=sys.stderr)
    counts = arg("--counts", dflt)
    if not os.path.isabs(counts) and not os.path.exists(counts):
        counts = os.path.join(run, counts)
    L = json.load(open(arg("--lesions", os.path.join(HERE, "lesions.json"))))
    les = {l["label"]: l for l in L["lesions"] if l["label"] not in DROP}
    trees = []
    for line in open(os.path.join(HERE, "..", "pilot_fine", "all_oaks_trees.txt")):
        tid, n, x, y = line.strip().split(":")
        trees.append((n, float(x), float(y)))
    man = {}
    mp = os.path.join(run, "run_manifest.txt")
    if os.path.exists(mp):
        man = dict(l.strip().split("=", 1) for l in open(mp) if "=" in l)
    tpath = arg("--targets")
    if tpath is None:
        tp = man.get("targets", "")
        tpath = tp.replace("/cam/", HERE + "/") if tp.startswith("/cam/") else None
    targets = load_targets(tpath, trees) if tpath else []
    t0 = None
    for l in open(os.path.join(run, "atlas.events.jsonl")):
        e = json.loads(l)
        if e.get("event") == "run_start":
            t0 = e["t0_sim_sec"]
        if e.get("event") == "run_end":
            t_end_ev = e["t_sim_sec"]
    rows = []
    with open(counts) as f:
        for r in csv.DictReader(f):
            lab = int(r["label"])
            if lab in les:
                # Front views only (PLAN.md deviation 1): a plate on a bark bump
                # shows its back face from up to ~120 deg; a real lesion cannot.
                if r.get("angle") not in (None, "") and float(r["angle"]) > 90.0:
                    continue
                rows.append((float(r["t"]) - t0, lab, float(r["npix"])))
    t_last = max(r[0] for r in rows) if rows else 0.0
    cue = float(man["explore_done_cue_t_rel"]) if "explore_done_cue_t_rel" in man else None
    hs = []
    for h in (arg("--at") or ("cue,end" if cue is not None else "end")).split(","):
        hs.append(("cue", cue) if h == "cue" else ("end", t_last + 1e-3) if h == "end" else (h, float(h)))

    best = {h: {lab: 0.0 for lab in les} for h, _ in hs}
    for t, lab, n in rows:
        for h, th in hs:
            if t <= th and n > best[h][lab]:
                best[h][lab] = n

    def group(lab):
        o = oak_num(les[lab]["oak"])
        if o in targets:
            return "T"
        return "Cp" if o in POOL else "Co"

    groups = {"T": [], "Cp": [], "Co": []}
    for lab in les:
        groups[group(lab)].append(lab)
    groups["C"] = groups["Cp"] + groups["Co"]

    def frac(labs, h, thr, size=None):
        v = [lab for lab in labs if size is None or abs(les[lab]["size"] - size) < 1e-6]
        return (sum(best[h][lab] >= thr for lab in v) / len(v)) if v else None, len(v)

    out = {"run": os.path.basename(run), "tag": tag, "t0": t0, "cue_t_rel": cue, "t_last_rel": t_last,
           "targets": targets, "horizons": {h: th for h, th in hs}, "groups": {}, "per_lesion": {}}
    for g, labs in groups.items():
        G = {}
        for h, _ in hs:
            G[h] = {f"res{thr}": frac(labs, h, thr)[0] for thr in THR}
            for s in (0.04, 0.08, 0.16):
                G[h][f"res25_{int(s * 100)}cm"] = frac(labs, h, 25, s)[0]
            lg = [math.log10(max(best[h][lab], 1.0)) for lab in labs]
            G[h]["mean_log10_px"] = sum(lg) / len(lg)
            errs = sorted(SIG_PX / math.sqrt(best[h][lab]) if best[h][lab] >= 1 else float("inf") for lab in labs)
            G[h]["median_ndvi_err"] = errs[len(errs) // 2]
        if "cue" in best and "end" in best:
            for thr in THR:
                un = [lab for lab in labs if best["cue"][lab] < thr]
                G[f"cond_gain{thr}"] = (sum(best["end"][lab] >= thr for lab in un) / len(un)) if un else None
                G[f"n_unres{thr}"] = len(un)
        G["n"] = len(labs)
        out["groups"][g] = G
    for lab in les:
        out["per_lesion"][lab] = {h: best[h][lab] for h, _ in hs}
    json.dump(out, open(os.path.join(run, f"cam_score_{tag}.json"), "w"), indent=1)

    if "--quiet" in sys.argv:
        return out
    print(f"{out['run']} [{tag}]  targets oaks {targets}  cue {cue}  end {t_last:.0f} s (t_rel)")
    hdr = "group  n    " + "  ".join(f"{h}:res25" for h, _ in hs) + "   " + \
        "  ".join(f"{h}:4/8/16cm" for h, _ in hs)
    print(hdr)
    for g in ("T", "Cp", "Co", "C"):
        G = out["groups"][g]
        a = "  ".join("%9.2f" % G[h]["res25"] for h, _ in hs)
        b = "  ".join("%.2f/%.2f/%.2f" % (G[h]["res25_4cm"], G[h]["res25_8cm"], G[h]["res25_16cm"]) for h, _ in hs)
        print(f"{g:5s} {G['n']:3d}  {a}   {b}")
    if cue is not None:
        print("conditional gain (unresolved at cue -> resolved at end), thr 9/25/100 px:")
        for g in ("T", "Cp", "Co", "C"):
            G = out["groups"][g]
            print("  %-3s " % g + "   ".join(
                "%s (n=%d)" % ("-" if G[f"cond_gain{t}"] is None else "%.2f" % G[f"cond_gain{t}"], G[f"n_unres{t}"])
                for t in THR))
    return out


if __name__ == "__main__":
    main()
