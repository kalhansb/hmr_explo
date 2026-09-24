#!/usr/bin/env python3
"""Collect the camera experiment's results into page_data.json for the results page
(host python). Usage: build_page_data.py [rep dirs...]  (default: every rep*/
with a cam_score_rendered.json). Runs cam_pool.py over those reps first.
"""
import base64, glob, json, os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
POOL = {4, 18, 36, 39, 40, 42, 58, 69, 73}
L = json.load(open(os.path.join(HERE, "lesions.json")))
OAK_K = {int(oak.split("_")[-1]): k for k, (tid, oak) in enumerate(L["trees"])}


def J(p):
    return json.load(open(p))


def man(run):
    return dict(l.strip().split("=", 1) for l in open(os.path.join(run, "run_manifest.txt")) if "=" in l)


def path_after(run, t0, cue, end):
    g = J(os.path.join(run, "gt_path_cum.json"))
    out = {}
    for rb, v in g.items():
        def at(t):
            best = 0.0
            for tt, m in zip(v["t"], v["m"]):
                if tt <= t0 + t:
                    best = m
            return best
        out[rb] = round(at(end) - at(cue), 1)
    return out


def lidar_m2(run, targets):
    p = os.path.join(run, "zband.json")
    if not os.path.exists(p):
        return None
    z = J(p)
    out = {}
    for h, e in z["horizons"].items():
        key = "end" if h == "end" else "cue"
        tr = e["trunks"]
        def m2(o):
            t = tr.get(f"Oak tree_{o}")
            return None if t is None else t["bands"]["0.2-1.2"]["M2"]
        T = [m2(o) for o in targets]
        C = [m2(o) for o in sorted(POOL - set(targets))]
        out[key] = {"T": T, "Cp": C, "T_mean": sum(T) / len(T), "Cp_mean": sum(C) / len(C)}
    return out


def unfiltered(run):
    """Rendered score without the front-view filter (sensitivity check); made on first use."""
    p = os.path.join(run, "cam_score_rendered_unfiltered.json")
    if not os.path.exists(p):
        subprocess.run([sys.executable, os.path.join(HERE, "cam_score.py"), run, "--tag", "rendered_unfiltered",
                        "--counts", "cam/counts.csv", "--quiet"], check=True)
    return p


def rep_entry(run):
    name = os.path.basename(run)
    M = man(run)
    R = J(os.path.join(run, "cam_score_rendered.json"))
    V = J(os.path.join(run, "cam_score_virtual.json"))
    U = J(unfiltered(run))
    cue, end = R["cue_t_rel"], R["t_last_rel"]
    val = open(os.path.join(run, "validate_virtual.txt")).read() if os.path.exists(os.path.join(run, "validate_virtual.txt")) else ""

    def grp(S):
        return {g: {k: S["groups"][g].get(k) for k in ("cond_gain9", "cond_gain25", "cond_gain100",
                                                         "n_unres9", "n_unres25", "n_unres100", "n")}
                   | {"cue": S["groups"][g]["cue"], "end": S["groups"][g]["end"]} for g in ("T", "Cp", "Co", "C")}
    return {"run": name, "targets": R["targets"], "cue": cue, "end": end,
            "end_reason": M.get("run_end_reason"), "cost_s": end - cue,
            "path_after_cue": path_after(run, R["t0"], cue, end),
            "rendered": grp(R), "virtual": grp(V), "unfiltered": grp(U),
            "lidar_M2": lidar_m2(run, R["targets"]),
            "validate": val.strip().splitlines()}


def tiles(run, oaks):
    d = os.path.join(run, "ndvi_tiles")
    if not os.path.isdir(d):
        subprocess.run(["docker", "run", "--rm", "--network", "none", "-u", f"{os.getuid()}:{os.getgid()}",
                        "-v", f"{HERE}:/cam", "hmrexplo:humble", "python3", "/cam/ndvi_tiles.py",
                        f"/cam/{os.path.basename(run)}", f"/cam/{os.path.basename(run)}/ndvi_tiles"], check=True)
    meta = J(os.path.join(d, "tiles.json"))
    out = []
    for o in oaks:
        k = str(OAK_K[o])
        if k not in meta:
            continue
        b = base64.b64encode(open(os.path.join(d, f"tree_{k}.png"), "rb").read()).decode()
        out.append({**meta[k], "oak": o, "png": "data:image/png;base64," + b})
    return out


def main():
    reps = [a.rstrip("/") for a in sys.argv[1:]] or sorted(
        (os.path.dirname(p) for p in glob.glob(os.path.join(HERE, "rep*", "cam_score_rendered.json"))),
        key=lambda s: (int("".join(c for c in os.path.basename(s) if c.isdigit())), s))
    reps = [os.path.join(HERE, r) if not os.path.isabs(r) else r for r in reps]
    for r in reps:
        unfiltered(r)
    # pre-registered set (PLAN.md: reps 1-4; 5-6 only if time allows), shown separately once extra reps exist
    pre = [r for r in reps if os.path.basename(r) in ("rep1", "rep2", "rep3", "rep4")]
    prereg = None
    if len(pre) == 4 and len(reps) > 4:
        subprocess.run([sys.executable, os.path.join(HERE, "cam_pool.py"), *pre, "--tag", "rendered",
                        "--thr", "25", "--sims", "100000"], check=True, stdout=subprocess.DEVNULL)
        prereg = J(os.path.join(HERE, "pool_rendered_thr25.json"))
    pooled = {}
    for tag in ("rendered", "virtual", "rendered_unfiltered"):
        for thr in (9, 25, 100):
            subprocess.run([sys.executable, os.path.join(HERE, "cam_pool.py"), *reps, "--tag", tag,
                            "--thr", str(thr), "--sims", "100000"], check=True, stdout=subprocess.DEVNULL)
            pooled[f"{tag}_{thr}"] = J(os.path.join(HERE, f"pool_{tag}_thr{thr}.json"))
    subprocess.run([sys.executable, os.path.join(HERE, "cam_bark.py"), *reps, "--sims", "100000"],
                   check=True, stdout=subprocess.DEVNULL)
    bark = J(os.path.join(HERE, "bark_pool.json"))
    data = {"reps": [rep_entry(r) for r in reps], "pooled": pooled, "prereg": prereg, "bark": bark,
            "phaseA": {thr: J(os.path.join(HERE, f"phaseA_report_thr{thr}.json")) for thr in (9, 25, 100)}}
    # example images from the first rep: its three targets and three undrawn pool oaks
    if reps:
        r0 = data["reps"][0]
        ctrl = [o for o in sorted(POOL) if o not in r0["targets"]]
        data["tiles"] = {"run": r0["run"], "cue": r0["cue"],
                         "T": tiles(reps[0], r0["targets"]), "Cp": tiles(reps[0], ctrl)}
    for e in data["reps"]:
        e["bark"] = next((b for b in bark["runs"] if b["run"] == e["run"]), None)
    json.dump(data, open(os.path.join(HERE, "page_data.json"), "w"))
    print("reps:", [r["run"] for r in data["reps"]], "->", os.path.join(HERE, "page_data.json"),
          f"{os.path.getsize(os.path.join(HERE, 'page_data.json')) / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
