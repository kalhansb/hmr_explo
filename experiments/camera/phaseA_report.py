#!/usr/bin/env python3
"""Phase A readout: the virtual camera on the campaign on/off pairs and the
pilot runs (host python). Usage: phaseA_report.py [--thr 25]

Per run: cam_score.py --tag virtual at fixed horizons (targets from the run's
manifest, mapped to the overlay config). Campaign pairs: per rep and horizon,
DiD = (T_on - T_off) - (Cp_on - Cp_off) on resolved fraction, the mean over
reps, and an exact sign-flip p over the 2^n assignments (PLAN.md, Phase A
statistic). Writes phaseA_report.json (incl. resolved-vs-time curves).
"""
import itertools, json, os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..")
CFG = os.path.join(RUNS, "pilot_fine", "overlay", "explo_planner", "explo_planner", "config")
THR = int(sys.argv[sys.argv.index("--thr") + 1]) if "--thr" in sys.argv else 25
H_CAMP = [300, 600, 900, 1200, 1800, 2400, 3000, 3600]
H_PILOT = [300, 600, 900, 1200, 1800, 2400]
REPS = [1, 2, 3, 4, 5]
PILOT = ["pilot_fine/off", "pilot_fine/on", "pilot_fine/noisy_off", "pilot_fine/noisy_on", "pilot_fine/seq_clean"]


def score(run, hs):
    d = os.path.join(RUNS, run)
    man = dict(l.strip().split("=", 1) for l in open(os.path.join(d, "run_manifest.txt")) if "=" in l)
    tgt = os.path.join(CFG, os.path.basename(man["targets"]))
    subprocess.run([sys.executable, os.path.join(HERE, "cam_score.py"), d, "--tag", "virtual_h",
                    "--counts", "virtual_cam.csv", "--targets", tgt, "--quiet",
                    "--at", ",".join(str(h) for h in hs) + ",end"], check=True)
    return json.load(open(os.path.join(d, "cam_score_virtual_h.json")))


def curve(run, S, step=30):
    """Resolved fraction (>= THR px) vs t_rel for T and Cp, from the virtual counts."""
    d = os.path.join(RUNS, run)
    first = {}
    for l in open(os.path.join(d, "virtual_cam.csv")).read().splitlines()[1:]:
        t, rb, lab, n, _ = l.split(",")
        if float(n) >= THR:
            t = float(t) - S["t0"]; lab = int(lab)
            first[lab] = min(first.get(lab, 1e9), t)
    groups = {g: [int(x) for x in S["per_lesion"] if grp(S, int(x)) == g] for g in ("T", "Cp", "Co")}
    tmax = S["t_last_rel"]
    ts = list(range(0, int(tmax) + step, step))
    return {"t": ts, **{g: [sum(first.get(x, 1e9) <= t for x in labs) / len(labs) for t in ts]
                        for g, labs in groups.items()}}


LES = {l["label"]: l for l in json.load(open(os.path.join(HERE, "lesions.json")))["lesions"]}
POOL = {4, 18, 36, 39, 40, 42, 58, 69, 73}


def grp(S, lab):
    o = int(LES[lab]["oak"].split("_")[-1])
    return "T" if o in S["targets"] else ("Cp" if o in POOL else "Co")


def res(S, g, h):
    return S["groups"][g][str(h)][f"res{THR}"]


def signflip(xs):
    obs = sum(xs) / len(xs)
    n = 0; k = 0
    for s in itertools.product((1, -1), repeat=len(xs)):
        m = sum(a * b for a, b in zip(s, xs)) / len(xs)
        n += 1; k += m >= obs - 1e-12
    return k / n


def main():
    out = {"thr": THR, "campaign": {}, "pilot": {}, "curves": {}}
    for arm in ("on", "off"):
        for r in REPS:
            c = f"{arm}_rep{r}"
            if not os.path.exists(os.path.join(RUNS, c, "virtual_cam.csv")):
                print("missing", c); continue
            S = score(c, H_CAMP)
            out["campaign"][c] = {h: {g: res(S, g, h) for g in ("T", "Cp", "Co", "C")} for h in H_CAMP + ["end"]}
            out["campaign"][c]["t_last_rel"] = S["t_last_rel"]
            out["curves"][c] = curve(c, S)
    for c in PILOT:
        if not os.path.exists(os.path.join(RUNS, c, "virtual_cam.csv")):
            print("missing", c); continue
        S = score(c, H_PILOT)
        out["pilot"][c] = {h: {g: res(S, g, h) for g in ("T", "Cp", "Co", "C")} for h in H_PILOT + ["end"]}
        out["pilot"][c]["t_last_rel"] = S["t_last_rel"]
        out["curves"][c] = curve(c, S)

    # paired DiD over reps
    did = {}
    print(f"Campaign pairs, virtual camera, resolved fraction at {THR} px (T = oaks 42/4/39)")
    print("rep  h      T_on  T_off  Cp_on Cp_off   DiD_Cp   DiD_C")
    for h in H_CAMP + ["end"]:
        rows = []
        for r in REPS:
            on, off = out["campaign"].get(f"on_rep{r}"), out["campaign"].get(f"off_rep{r}")
            if not on or not off:
                continue
            a, b = on[h], off[h]
            dcp = (a["T"] - b["T"]) - (a["Cp"] - b["Cp"])
            dc = (a["T"] - b["T"]) - (a["C"] - b["C"])
            rows.append((r, dcp, dc))
            print(f"{r:3d}  {str(h):5s}  {a['T']:.2f}  {b['T']:.2f}   {a['Cp']:.2f}  {b['Cp']:.2f}   {dcp:+.2f}    {dc:+.2f}")
        if rows:
            xs = [x[1] for x in rows]; ys = [x[2] for x in rows]
            did[str(h)] = {"per_rep": {x[0]: {"Cp": x[1], "C": x[2]} for x in rows},
                           "mean_Cp": sum(xs) / len(xs), "p_Cp": signflip(xs), "n_pos_Cp": sum(x > 1e-9 for x in xs),
                           "mean_C": sum(ys) / len(ys), "p_C": signflip(ys), "n": len(xs)}
            D = did[str(h)]
            print(f"     {str(h):5s}  mean DiD_Cp {D['mean_Cp']:+.3f} ({D['n_pos_Cp']}/{D['n']} > 0, sign-flip p {D['p_Cp']:.3f})"
                  f"   mean DiD_C {D['mean_C']:+.3f} (p {D['p_C']:.3f})")
        print()
    out["did"] = did
    print("Pilot runs (T = oaks 42/4/39):")
    for c, v in out["pilot"].items():
        print(f"  {c:22s} " + "  ".join(f"{h}:{v[h]['T']:.2f}/{v[h]['Cp']:.2f}" for h in H_PILOT + ["end"]) + "   (T/Cp)")
    json.dump(out, open(os.path.join(HERE, f"phaseA_report_thr{THR}.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
