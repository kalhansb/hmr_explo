#!/usr/bin/env python3
"""PLAN.md metric 4: best bark pixels per tree, cue -> end (host python).

Usage: cam_bark.py <run> [<run> ...] [--sims 100000]
Reads <run>/cam/counts.csv (bark labels 130 + k) and the times from
<run>/cam_score_rendered.json. Per tree: log10 of the most bark pixels in any
single frame up to the cue and up to the end, and the change. Per run:
D_bark = mean change on the targets - mean change on the undrawn pool oaks.
Same randomisation as cam_pool.py (within each run, all 84 ways to call 3 pool
oaks "targets"). Oak 36 is left out: its bark label never renders (Deviation 2).
No front-view filter: a trunk is seen from every side.
Writes <run>/cam_bark.json and bark_pool.json next to this script.
"""
import csv, itertools, json, math, os, random, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from cam_pool import exact_p  # noqa: E402
POOL = [4, 18, 36, 39, 40, 42, 58, 69, 73]
NO_BARK = {36}


def per_run(run, L):
    S = json.load(open(os.path.join(run, "cam_score_rendered.json")))
    t0, cue, end = S["t0"], S["cue_t_rel"], S["t_last_rel"] + 1e-3
    oak_of = {130 + k: int(oak.split("_")[-1]) for k, (tid, oak) in enumerate(L["trees"])}
    best = {o: [0.0, 0.0] for o in oak_of.values()}
    with open(os.path.join(run, "cam", "counts.csv")) as f:
        for r in csv.DictReader(f):
            lab = int(r["label"])
            if lab not in oak_of:
                continue
            t, n, b = float(r["t"]) - t0, float(r["npix"]), best[oak_of[lab]]
            if t <= cue:
                b[0] = max(b[0], n)
            if t <= end:
                b[1] = max(b[1], n)
    trees = {o: {"cue_px": b[0], "end_px": b[1],
                 "dlog": math.log10(max(b[1], 1)) - math.log10(max(b[0], 1))}
             for o, b in best.items() if o not in NO_BARK}
    out = {"run": os.path.basename(run), "targets": S["targets"], "cue": cue, "trees": trees}
    json.dump(out, open(os.path.join(run, "cam_bark.json"), "w"), indent=1)
    return out


def D(trees, tg):
    T = [trees[o]["dlog"] for o in tg if o in trees]
    C = [trees[o]["dlog"] for o in POOL if o not in tg and o in trees]
    return sum(T) / len(T) - sum(C) / len(C)


def med(v):
    v = sorted(v)
    return v[len(v) // 2] if len(v) % 2 else 0.5 * (v[len(v) // 2 - 1] + v[len(v) // 2])


def main():
    sims = int(sys.argv[sys.argv.index("--sims") + 1]) if "--sims" in sys.argv else 100000
    runs = [a for a in sys.argv[1:] if not a.startswith("--") and os.path.isdir(a)]
    L = json.load(open(os.path.join(HERE, "lesions.json")))
    rows, nulls = [], []
    print("metric 4: best single-frame bark px per tree (Oak 36 excluded), median over trees")
    print("run      targets        T cue -> end        Cp cue -> end       D_bark (dlog10)  rank-p")
    for r in runs:
        R = per_run(r, L)
        tr, tg = R["trees"], R["targets"]
        d = D(tr, tg)
        null = [D(tr, s) for s in itertools.combinations(POOL, 3)]
        nulls.append(null)
        rank = sum(x >= d - 1e-12 for x in null) / len(null)
        T = [tr[o] for o in tg if o in tr]; C = [tr[o] for o in POOL if o not in tg and o in tr]
        row = {"run": R["run"], "targets": tg, "D": d, "rank_p": rank,
               "T_cue": med([x["cue_px"] for x in T]), "T_end": med([x["end_px"] for x in T]),
               "Cp_cue": med([x["cue_px"] for x in C]), "Cp_end": med([x["end_px"] for x in C])}
        rows.append(row)
        print(f"{row['run']:8s} {','.join(map(str, tg)):14s} {row['T_cue']:7.0f} -> {row['T_end']:7.0f}   "
              f"{row['Cp_cue']:7.0f} -> {row['Cp_end']:7.0f}   {d:+.3f}           {rank:.3f}")
    obs = sum(r["D"] for r in rows) / len(rows)
    rng = random.Random(1)
    ge = sum(sum(rng.choice(n) for n in nulls) / len(nulls) >= obs - 1e-12 for _ in range(sims))
    p_mc = (ge + 1) / (sims + 1)
    p = exact_p(nulls, obs * len(nulls))
    print(f"mean D_bark = {obs:+.3f} (x{10 ** obs:.2f} px); randomisation p (one-sided, exact) = {p:.4f}"
          f"  [Monte Carlo, {sims} draws: {p_mc:.4f}]")
    json.dump({"runs": rows, "mean_D": obs, "p_one_sided": p, "p_exact": True, "p_mc": p_mc, "sims": sims},
              open(os.path.join(HERE, "bark_pool.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
