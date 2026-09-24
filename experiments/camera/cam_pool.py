#!/usr/bin/env python3
"""Pool the camera replicates and run the randomisation test (host python).

Usage: cam_pool.py <run> [<run> ...] [--tag rendered] [--thr 25] [--sims 200000]
Reads <run>/cam_score_<tag>.json (cam_score.py). Writes pool_<tag>.json next
to this script and prints the readout.

Statistic per run (PLAN.md primary): D = conditional gain on the 3 targets
minus conditional gain on the 6 undrawn pool oaks, at the given threshold.
Pooled statistic = mean of D over runs. Null: within each run, every one of
the C(9,3) = 84 ways to call 3 pool oaks "targets" is equally likely under the
sharp null (targeting changes no tree's outcome), independently across runs.
Subsets where D is undefined (no unresolved lesion in a group) are dropped
from that run's null. p = P(mean D_null >= mean D_obs), exact (every
assignment counted); the Monte Carlo estimate is kept as p_mc.
"""
import itertools, json, math, os, random, sys

HERE = os.path.dirname(os.path.abspath(__file__))
POOL = [4, 18, 36, 39, 40, 42, 58, 69, 73]


def arg(k, d):
    return sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d


def exact_p(nulls, obs_sum, eps=1e-9):
    """P(sum over runs of an independently re-drawn D >= obs_sum), exactly: every
    one of prod(len(n)) assignments counted, meet-in-the-middle (half the runs'
    sums sorted, the other half bisected), so 6 runs cost 2 x 84^3 sums."""
    import bisect
    def sums(ns):
        out = [0.0]
        for n in ns:
            out = [a + b for a in out for b in n]
        return out
    h = len(nulls) // 2
    A, B = sums(nulls[:h]), sorted(sums(nulls[h:]))
    ge = sum(len(B) - bisect.bisect_left(B, obs_sum - a - eps) for a in A)
    return ge / (len(A) * len(B))


def main():
    tag = arg("--tag", "rendered"); thr = float(arg("--thr", "25")); sims = int(arg("--sims", "200000"))
    runs = [a for a in sys.argv[1:] if not a.startswith("--") and os.path.isdir(a)]
    L = json.load(open(os.path.join(HERE, "lesions.json")))
    oak_of = {str(l["label"]): int(l["oak"].split("_")[-1]) for l in L["lesions"]}
    size_of = {str(l["label"]): l["size"] for l in L["lesions"]}
    per_run, nulls = [], []
    for r in runs:
        S = json.load(open(os.path.join(r, f"cam_score_{tag}.json")))
        pl = S["per_lesion"]
        # per oak: (#unresolved at cue, #of those resolved at end)
        stat = {}
        for lab, v in pl.items():
            o = oak_of[lab]
            a = stat.setdefault(o, [0, 0, 0, 0])      # unres_cue, gained, n, resolved_cue
            a[2] += 1
            if v["cue"] < thr:
                a[0] += 1
                if v["end"] >= thr:
                    a[1] += 1
            else:
                a[3] += 1

        def D(tset):
            ct = [stat[o] for o in tset]; cc = [stat[o] for o in POOL if o not in tset]
            ut, gt = sum(a[0] for a in ct), sum(a[1] for a in ct)
            uc, gc = sum(a[0] for a in cc), sum(a[1] for a in cc)
            if ut == 0 or uc == 0:
                return None
            return gt / ut - gc / uc
        tg = sorted(S["targets"])
        d_obs = D(tg)
        null = [d for s in itertools.combinations(POOL, 3) if (d := D(s)) is not None]
        rank = sum(d >= d_obs for d in null) / len(null) if d_obs is not None else None
        others = [o for o in stat if o not in POOL]
        uo, go = sum(stat[o][0] for o in others), sum(stat[o][1] for o in others)
        ut, gt = sum(stat[o][0] for o in tg), sum(stat[o][1] for o in tg)
        up = sum(stat[o][0] for o in POOL if o not in tg); gp = sum(stat[o][1] for o in POOL if o not in tg)
        # by lesion size, all three groups
        by_size = {}
        for s in (0.04, 0.08, 0.16):
            row = {}
            for gname, oaks in (("T", tg), ("Cp", [o for o in POOL if o not in tg]), ("Co", others)):
                labs = [lab for lab in pl if oak_of[lab] in oaks and abs(size_of[lab] - s) < 1e-6]
                row[gname] = (sum(pl[lab]["cue"] >= thr for lab in labs) / len(labs),
                              sum(pl[lab]["end"] >= thr for lab in labs) / len(labs), len(labs))
            by_size[f"{int(s * 100)}cm"] = row
        per_run.append({"run": os.path.basename(r.rstrip("/")), "targets": tg, "D": d_obs,
                        "within_run_p": rank, "T": (gt, ut), "Cp": (gp, up), "Co": (go, uo),
                        "cue": S["cue_t_rel"], "end": S["t_last_rel"], "by_size": by_size})
        nulls.append(null)
    ok = [p for p in per_run if p["D"] is not None]
    obs = sum(p["D"] for p in ok) / len(ok)
    rng = random.Random(12345)
    usable = [n for p, n in zip(per_run, nulls) if p["D"] is not None]
    ge = sum(1 for _ in range(sims) if sum(rng.choice(n) for n in usable) / len(usable) >= obs - 1e-12)
    p_mc = (ge + 1) / (sims + 1)
    p = exact_p(usable, obs * len(usable))
    out = {"tag": tag, "thr": thr, "runs": per_run, "mean_D": obs, "p_one_sided": p, "p_exact": True,
           "p_mc": p_mc, "sims": sims, "n_assignments": math.prod(len(n) for n in usable)}
    json.dump(out, open(os.path.join(HERE, f"pool_{tag}_thr{int(thr)}.json"), "w"), indent=1)
    print(f"[{tag}] threshold {thr:.0f} px, {len(ok)} run(s)")
    print("run      targets        cue   end    T gained/unres   pool ctrl      other ctrl     D      within-run rank-p")
    for q in per_run:
        f = lambda a: "%2d/%-2d = %.2f" % (a[0], a[1], a[0] / a[1]) if a[1] else "%2d/%-2d =  -  " % a
        print("%-8s %-13s %5.0f %5.0f   %s   %s   %s   %s   %s" % (
            q["run"], ",".join(map(str, q["targets"])), q["cue"] or -1, q["end"], f(q["T"]), f(q["Cp"]), f(q["Co"]),
            "  -  " if q["D"] is None else "%+.2f" % q["D"], "-" if q["within_run_p"] is None else "%.3f" % q["within_run_p"]))
    print(f"mean D = {obs:+.3f}; randomisation p (one-sided, exact over {out['n_assignments']} assignments) = {p:.2e}"
          f"  [Monte Carlo, {sims} draws: {p_mc:.4f}]")
    print("resolved fraction by size, cue -> end (pooled over runs):")
    for s in ("4cm", "8cm", "16cm"):
        parts = []
        for g in ("T", "Cp", "Co"):
            n = sum(q["by_size"][s][g][2] for q in per_run)
            a = sum(q["by_size"][s][g][0] * q["by_size"][s][g][2] for q in per_run) / n
            b = sum(q["by_size"][s][g][1] * q["by_size"][s][g][2] for q in per_run) / n
            parts.append(f"{g} {a:.2f}->{b:.2f} (n={n})")
        print(f"  {s:5s} " + "   ".join(parts))


if __name__ == "__main__":
    main()
