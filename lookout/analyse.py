#!/usr/bin/env python3
"""Part 7 analysis (offline, in the container: numpy + matplotlib).

  analyse.py --layout L2=/runs/lookout/L2_full --layout L3=/runs/lookout/L3_full \
             --calib /runs/lookout/calib --out /runs/lookout/results

Reads only the logs (walks.csv, first_detect.csv, alarms.csv, steps.csv,
events.jsonl, calib_summary.csv); nothing is re-run.
Walks marked "-chk-away" (lookouts spawned, then moved 1 km away) and "-chk-absent"
(fresh sim, lookouts never spawned) are the two kinds of check walk; every other
walk in walks.csv is a main walk. Main rule = alarm_m5; best case = the brief's
comparison rule; m3/m10/m20 = the point-count sensitivity; xy5 = the
horizontal-linkage sensitivity (declared before the pilot; see PLAN.md).

Outputs in --out: summary.json (every number below), tables/*.csv,
figures/*.png, results.md (the numbers in text form).

Definitions
  post line       where an entry's path crosses the radius of its lookout's
                  post (r_post in the layout config, from posts.py)
  warn_s          path distance still to go to the post line / 1.3 m/s at
                  the first alarm (negative once past it)
  caught >= T     first alarm with warn_s >= T
  before post     first alarm with warn_s >= 0
  before 50       first alarm while the person is > 50 m from the mulcher
                  (straight-line)
  team            mulcher + every lookout, earliest first alarm. Every post is
                  linked to the mulcher (posts.py; checked on the post and at
                  every lookout alarm), so a lookout's alarm is the machine's
                  warning at once.
Intervals are Wilson 95 %; walks share headings and entries (and paired
detectors share walks), so the intervals are approximate.
"""
import argparse
import collections
import csv
import json
import math
import os
import sys

import numpy as np
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

MAIN = "alarm_m5"
VARIANTS = ["alarm_m3", "alarm_m5", "alarm_m10", "alarm_m20", "alarm_xy5", "best"]
VLABEL = {"alarm_m3": "rule, 3 pts", "alarm_m5": "rule, 5 pts (main)", "alarm_m10": "rule, 10 pts",
          "alarm_m20": "rule, 20 pts", "alarm_xy5": "rule, 5 pts, xy linkage", "best": "best case"}
TS = list(range(0, 31))
Z = 1.959964


# ------------------------------------------------------------------ stats --
def wilson(k, n):
    if n == 0:
        return (None, None, None)
    p = k / n
    d = 1 + Z * Z / n
    c = (p + Z * Z / (2 * n)) / d
    h = Z * math.sqrt(p * (1 - p) / n + Z * Z / (4 * n * n)) / d
    return (p, max(0.0, c - h), min(1.0, c + h))


def mcnemar_exact(b, c):
    """Two-sided exact McNemar p (binomial on the discordant pairs)."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)


def rate(k, n):
    p, lo, hi = wilson(k, n)
    return {"k": k, "n": n, "p": p, "lo": lo, "hi": hi}


def fnum(v):
    return None if v in (None, "") else float(v)


def med(xs):
    xs = [x for x in xs if x is not None]
    return float(np.median(xs)) if xs else None


def quart(xs):
    xs = [x for x in xs if x is not None]
    return [float(np.percentile(xs, q)) for q in (25, 50, 75)] if xs else None


def qfmt(xs):
    return "-" if xs is None else "[" + ", ".join(f"{x:.1f}" for x in xs) + "]"


def pct(r):
    return "n/a" if r["p"] is None else f"{100 * r['p']:.0f} % ({r['k']}/{r['n']}; {100 * r['lo']:.0f}-{100 * r['hi']:.0f})"


# --------------------------------------------------------------- loading --
def read(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


KINDS = ("main", "check-away", "check-absent")


def kind_of(walk):
    """main, check-away or check-absent (walks.py pick_check ids)."""
    return "main" if "-chk-" not in walk else "check-" + walk.rsplit("-chk-", 1)[1]


class Layout:
    def __init__(self, name, d):
        self.name, self.d = name, d
        self.cfg = yaml.safe_load(open(f"/lookout/config/{name}.yaml"))
        self.lk = [l["name"] for l in self.cfg["lookouts"]]
        self.speed = self.cfg["walk"]["speed"]
        self.dt = self.cfg["walk"]["step"] / self.speed
        self.r_post = {e["id"]: e["r_post"] for e in self.cfg["entries"]}
        walks = read(os.path.join(d, "walks.csv"))
        self.main = [w for w in walks if kind_of(w["walk"]) == "main"]
        self.check = [w for w in walks if kind_of(w["walk"]) != "main"]
        # a walk interrupted (crash, stop) is re-run: only the session that
        # finished it (its walks.csv row) counts, in every log
        runs = {(w["walk"], w["session"]) for w in walks}
        self.first = collections.defaultdict(dict)      # walk -> (detector, variant) -> row
        for r in read(os.path.join(d, "first_detect.csv")):
            if (r["walk"], r["session"]) in runs:
                self.first[r["walk"]][(r["detector"], r["variant"])] = r
        self.alarms = [a for a in read(os.path.join(d, "alarms.csv")) if (a["walk"], a["session"]) in runs]
        ev = os.path.join(d, "events.jsonl")
        self.at_post = ([e for e in map(json.loads, open(ev)) if e["kind"] == "at_post"]
                        if os.path.exists(ev) else [])
        self.n_scans = collections.Counter()            # (lidar, kind_of(walk)) -> scans
        with open(os.path.join(d, "steps.csv")) as f:
            for r in csv.DictReader(f):
                if (r["walk"], r["session"]) not in runs:
                    continue
                self.n_scans[(r["lidar"], kind_of(r["walk"]))] += 1

    def fd(self, walk, det, var):
        """First alarm of detector `det` (mulcher, lookout name or team) under
        variant `var`: dict(t, warn, dist, before_post, before50) or None."""
        r = self.first[walk].get((det, var))
        if r is None or r["step"] in ("", None):
            return None
        return {"step": int(r["step"]), "t": float(r["t_s"]), "warn": float(r["warn_s"]), "dist": float(r["dist_m"]),
                "x": float(r["x"]), "y": float(r["y"]),
                "before_post": int(r["before_post"]), "before50": int(r["before50"])}

    def earliest(self, walk, dets, var):
        c = [x for x in (self.fd(walk, d, var) for d in dets) if x is not None]
        return min(c, key=lambda x: x["t"]) if c else None



# -------------------------------------------------------------- analysis --
def detection(L):
    """Per detector x variant: caught before the post line / 50 m, warning times."""
    out, rows = {}, []
    dets = ["mulcher"] + L.lk + ["team"]
    for det in dets:
        for var in VARIANTS:
            f = [L.fd(w["walk"], det, var) for w in L.main]
            n = len(f)
            kpost = sum(1 for x in f if x and x["warn"] >= 0)
            k50 = sum(1 for x in f if x and x["before50"])
            kany = sum(1 for x in f if x)
            e = {"n": n, "caught_any": rate(kany, n), "caught_before_post": rate(kpost, n),
                 "caught_before50": rate(k50, n), "warn_s_q": quart([x["warn"] for x in f if x]),
                 "dist_m_q": quart([x["dist"] for x in f if x]),
                 "curve": [rate(sum(1 for x in f if x and x["warn"] >= T), n)["p"] for T in TS]}
            out[f"{det}/{var}"] = e
            rows.append({"layout": L.name, "detector": det, "variant": var, "n": n,
                         "caught_any": kany, "caught_before_post": kpost,
                         "ppost": e["caught_before_post"]["p"], "ppost_lo": e["caught_before_post"]["lo"],
                         "ppost_hi": e["caught_before_post"]["hi"], "caught_before50": k50,
                         "p50": e["caught_before50"]["p"], "p50_lo": e["caught_before50"]["lo"],
                         "p50_hi": e["caught_before50"]["hi"],
                         "warn_s_median": med([x["warn"] for x in f if x]),
                         "dist_m_median": med([x["dist"] for x in f if x])})
    return out, rows


def paired(L, var=MAIN):
    """McNemar team vs mulcher alone, and leave-one-lookout-out."""
    res = {}
    for line, key in (("_post", lambda x: x is not None and x["warn"] >= 0),
                      ("50", lambda x: x is not None and x["before50"] == 1)):
        b = c = both = neither = 0
        for w in L.main:
            m = key(L.fd(w["walk"], "mulcher", var))
            t = key(L.fd(w["walk"], "team", var))
            b += t and not m; c += m and not t; both += t and m; neither += not t and not m
        res[f"mcnemar_before{line}"] = {"team_only": b, "mulcher_only": c, "both": both, "neither": neither,
                                         "p_exact": mcnemar_exact(b, c)}
    loo = {}
    for drop in L.lk:
        dets = ["mulcher"] + [x for x in L.lk if x != drop]
        f = [L.earliest(w["walk"], dets, var) for w in L.main]
        loo[drop] = {"caught_before_post": rate(sum(1 for x in f if x and x["warn"] >= 0), len(f)),
                     "caught_before50": rate(sum(1 for x in f if x and x["before50"]), len(f))}
    res["leave_one_out"] = loo
    fb = collections.Counter(w["team_alarm_first_by"] or "none" for w in L.main)
    res["first_catcher"] = dict(fb)
    best_lk = [max((x["warn"] for x in (L.fd(w["walk"], n, var) for n in L.lk) if x), default=None)
               for w in L.main]
    res["best_lookout_warn_s_q"] = quart(best_lk)
    res["best_lookout_warn_s"] = best_lk
    # the same walk, team vs mulcher alone: how much earlier the team's first
    # alarm came (s, along the path), and where each first alarm was
    for v in (var, "best"):
        gain, dm, dt, m_never = [], [], [], 0
        for w in L.main:
            m, t = L.fd(w["walk"], "mulcher", v), L.fd(w["walk"], "team", v)
            if t is None:
                continue
            dt.append(t["dist"])
            if m is None:
                m_never += 1
                continue
            dm.append(m["dist"])
            gain.append(t["warn"] - m["warn"])
        res[f"extra_warning/{v}"] = {"n": len(gain), "s_q": quart(gain), "s_min": min(gain, default=None),
                                    "mulcher_never": m_never, "dist_mulcher_m_q": quart(dm),
                                    "dist_team_m_q": quart(dt)}
    return res


def wedge(L, var=MAIN):
    out = {}
    for inw in (1, 0):
        ws = [w for w in L.main if int(w["in_wedge"]) == inw]
        f = [L.fd(w["walk"], "mulcher", var) for w in ws]
        out["in_wedge" if inw else "outside_wedge"] = {
            "n": len(ws), "detected": sum(1 for x in f if x),
            "dist_m_q": quart([x["dist"] for x in f if x]),
            "dist_m": [x and x["dist"] for x in f]}
    return out


def false_alarms(L):
    out, pts = {}, []
    for kind in KINDS:
        for lid in ["mulcher"] + L.lk:
            n_sc = L.n_scans[(lid, kind)]
            hours = n_sc * L.dt / 3600.0
            for var in VARIANTS[:-1]:
                fa = [a for a in L.alarms if a["lidar"] == lid and a["variant"] == var.replace("alarm_", "")
                      and a["hit"] == "0" and kind_of(a["walk"]) == kind]
                out[f"{kind}/{lid}/{var}"] = {
                    "false_alarm_scans": len(fa), "walks_with_fa": len({a["walk"] for a in fa}),
                    "scans": n_sc, "walk_hours": hours,
                    "per_hour": len(fa) / hours if hours else None,
                    "per_1000_scans": 1000 * len(fa) / n_sc if n_sc else None,
                    "size_n_median": med([float(a["n"]) for a in fa]),
                    "rings_median": med([len(a["rings"].split()) for a in fa])}
                if var == MAIN:
                    pts += [{"layout": L.name, "kind": kind, "lidar": lid, "walk": a["walk"], "t_s": a["t_s"],
                             "x": a["cx"], "y": a["cy"], "z": a["cz"], "n": a["n"], "rings": len(a["rings"].split())}
                            for a in fa]
    return out, pts


def check_walks(L, var=MAIN):
    rows = []
    for c in L.check:
        m = c["matches"]
        a, b = L.fd(c["walk"], "mulcher", var), L.fd(m, "mulcher", var)
        rows.append({"layout": L.name, "check": c["walk"], "how": kind_of(c["walk"]), "matches": m,
                     "mulcher_dist_check": a and a["dist"], "mulcher_dist_main": b and b["dist"],
                     "same_step": int((a is None and b is None) or (a is not None and b is not None
                                                                     and a["step"] == b["step"])),
                     # repeating a walk moved the first detection by one step (lidar range
                     # noise, sigma 0.02 m; L2 pilot runs 1 and 2), so this is reported too
                     "within_1_step": int((a is None and b is None) or (a is not None and b is not None
                                                                        and abs(a["step"] - b["step"]) <= 1)),
                     "fa_mulcher_check": int(c["fa_mulcher"] or 0),
                     "fa_mulcher_main": int(next((w["fa_mulcher"] for w in L.main if w["walk"] == m), 0) or 0)})
    return rows


def links(L):
    """The radio link on each post (walks.py at_post events, static budget of
    the unchanged comms model) and at every lookout alarm (linked_at_alarm)."""
    out = {}
    for n in L.lk:
        la = [w.get(f"linked_at_alarm_{n}") for w in L.main]
        out[n] = {"at_post": [e for e in L.at_post if e["robot"] == n],
                  "alarms_linked": sum(1 for v in la if v == "1"),
                  "alarms_not_linked": sum(1 for v in la if v == "0")}
    return out


# --------------------------------------------------------------- figures --
COL = {"mulcher": "#444444", "team": "#1f77b4"}
LKCOL = ["#d62728", "#2ca02c", "#9467bd", "#ff7f0e"]


def trees_of(L):
    with open(f"/runs/lookout/worlds/{L.cfg['forest']['csv']}") as f:
        next(f)
        return np.array([tuple(map(float, l.split(",")[:2])) for l in f if l.strip()])


def fig_layout(L, figdir):
    T = trees_of(L)
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(T[:, 0], T[:, 1], s=6, c="#6b8e23", lw=0, label="oak trunks")
    for p in L.cfg["paths"]:
        P = np.array(p["polyline"])
        ax.plot(P[:, 0], P[:, 1], c="#8b5a2b", lw=2)
    th = np.linspace(0, 2 * np.pi, 361)
    for r, ls, lab in ((50, ":", "50 m"), (30, "-.", "30 m (radio horizon)")):
        ax.plot(r * np.cos(th), r * np.sin(th), ls, c="k", lw=0.8)
        ax.text(r * 0.707, r * 0.707, lab, fontsize=7)
    for i, n in enumerate(L.lk):
        c = LKCOL[i % len(LKCOL)]
        l = next(x for x in L.cfg["lookouts"] if x["name"] == n)
        ax.plot([0, l["x"]], [0, l["y"]], c="#17becf", lw=1.2, label="radio link" if i == 0 else None)
        ax.plot(l["x"], l["y"], "^", c=c, ms=9, label=f"{n} post ({l['r']} m, {l['side']})")
        ax.arrow(l["x"], l["y"], 4 * math.cos(l["yaw"]), 4 * math.sin(l["yaw"]), color=c, width=0.2)
    for e in L.cfg["entries"]:
        ax.plot(*e["post_cross"], "o", mfc="none", c="k", ms=7)
    ax.plot(0, 0, "s", c="k", ms=8, label="mulcher")
    ax.set_aspect("equal")
    ax.set_xlim(-80, 80); ax.set_ylim(-80, 80)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(f"{L.name}: paths, lookout posts (posts.py), radio links; o = post line")
    ax.legend(fontsize=7, loc="lower left")
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, f"layout_{L.name}.png"), dpi=130)
    plt.close(fig)


def fig_calib(calib_dir, figdir):
    S = read(os.path.join(calib_dir, "calib_summary.csv"))
    if not S:
        return
    fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True, sharey=True)
    for i, mount in enumerate(("lk0", "mulcher")):
        for j, orient in enumerate(("toward", "across")):
            ax = axes[i, j]
            ss = sorted((r for r in S if r["mount"] == mount and r["orient"] == orient), key=lambda r: float(r["d_m"]))
            d = [float(r["d_m"]) for r in ss]
            for v, st in (("alarm_m3", dict(c="#9ecae1", lw=1)), ("alarm_m5", dict(c="#08519c", lw=2.5)),
                          ("alarm_m10", dict(c="#6baed6", lw=1)), ("alarm_m20", dict(c="#c6dbef", lw=1)),
                          ("alarm_xy5", dict(c="#e6550d", lw=1.5)), ("best", dict(c="k", lw=1.5, ls="--"))):
                ax.plot(d, [100 * float(r[v]) for r in ss], label=VLABEL[v], **st)
            ax.axvspan(30, 38, color="#dddddd", alpha=0.5, lw=0)
            ax.set_title(f"{'Husky (lookout)' if mount == 'lk0' else 'mulcher roof'}, person walking {orient}")
            ax.grid(alpha=0.3)
            if i == 1:
                ax.set_xlabel("horizontal distance from the lidar (m)")
            if j == 0:
                ax.set_ylabel("% of scans the rule fires")
    axes[0, 0].legend(fontsize=7)
    fig.suptitle("Calibration, open ground (grey band: ~30-38 m real-world sanity range)")
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, "calibration.png"), dpi=130)
    plt.close(fig)


def fig_curves(res, figdir):
    lays = list(res)
    fig, axes = plt.subplots(1, len(lays), figsize=(6 * len(lays), 4.5), sharey=True, squeeze=False)
    for ax, ln in zip(axes[0], lays):
        det = res[ln]["detection"]
        ax.plot(TS, [100 * p for p in det[f"mulcher/{MAIN}"]["curve"]], c=COL["mulcher"], lw=2,
                label="control: mulcher alone")
        ax.plot(TS, [100 * p for p in det[f"team/{MAIN}"]["curve"]], c=COL["team"], lw=2,
                label="team: mulcher + lookouts")
        ax.plot(TS, [100 * p for p in det["mulcher/best"]["curve"]], c=COL["mulcher"], lw=1.2, ls="--",
                label="control, best case")
        ax.plot(TS, [100 * p for p in det["team/best"]["curve"]], c=COL["team"], lw=1.2, ls="--",
                label="team, best case")
        ax.plot(TS, [100 * p for p in det["team/alarm_xy5"]["curve"]], c="#e6550d", lw=1, ls=":",
                label="team, xy-linkage variant")
        ax.set_title(f"{ln}: walkers caught at least T s before the post line")
        ax.set_xlabel("T (s)"); ax.grid(alpha=0.3); ax.set_ylim(-2, 102)
    axes[0][0].set_ylabel("% of walks")
    # below the panels: inside, the legend covered the control's best-case line
    fig.legend(*axes[0][0].get_legend_handles_labels(), loc="lower center", ncol=3, fontsize=8)
    fig.tight_layout(rect=(0, 0.1, 1, 1))
    fig.savefig(os.path.join(figdir, "warning_curve.png"), dpi=130)
    plt.close(fig)


def fig_bars(res, layouts, figdir):
    lays = list(res)
    fig, axes = plt.subplots(1, len(lays), figsize=(6 * len(lays), 4.5), sharey=True, squeeze=False)
    for ax, ln in zip(axes[0], lays):
        L = layouts[ln]
        dets = ["mulcher"] + L.lk + ["team"]
        det = res[ln]["detection"]
        x = np.arange(len(dets))
        for off, key, lab, col in ((-0.2, "caught_before_post", "before the post line", "#08519c"),
                                   (0.2, "caught_before50", "before 50 m", "#6baed6")):
            r = [det[f"{d}/{MAIN}"][key] for d in dets]
            p = np.array([100 * (q["p"] or 0) for q in r])
            err = np.array([[100 * ((q["p"] or 0) - (q["lo"] or 0)) for q in r],
                            [100 * ((q["hi"] or 0) - (q["p"] or 0)) for q in r]])
            ax.bar(x + off, p, 0.4, yerr=err, capsize=3, color=col, label=f"main rule, {lab}")
            b = [100 * (det[f"{d}/best"][key]["p"] or 0) for d in dets]
            ax.plot(x + off, b, "_", c="k", ms=18, mew=2, label="best case (tick on each bar)" if off < 0 else None)
        ax.set_xticks(x); ax.set_xticklabels(dets)
        ax.set_title(f"{ln}: % of walks caught (Wilson 95 %, approximate)")
        ax.grid(axis="y", alpha=0.3); ax.set_ylim(0, 105)
    axes[0][0].set_ylabel("% of walks")
    fig.legend(*axes[0][0].get_legend_handles_labels(), loc="lower center", ncol=4, fontsize=8)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(os.path.join(figdir, "catch_rates.png"), dpi=130)
    plt.close(fig)


def fig_fa(layouts, fa_pts, figdir):
    lays = list(layouts)
    fig, axes = plt.subplots(1, len(lays), figsize=(7 * len(lays), 7), squeeze=False)
    for ax, ln in zip(axes[0], lays):
        L = layouts[ln]
        T = trees_of(L)
        ax.scatter(T[:, 0], T[:, 1], s=2, c="#bbbbbb", lw=0)
        for p in L.cfg["paths"]:
            P = np.array(p["polyline"])
            ax.plot(P[:, 0], P[:, 1], c="#d2b48c", lw=1.5)
        for i, lid in enumerate(["mulcher"] + L.lk):
            P = [(float(a["x"]), float(a["y"])) for a in fa_pts if a["layout"] == ln and a["lidar"] == lid]
            c = "k" if lid == "mulcher" else LKCOL[(i - 1) % len(LKCOL)]
            if lid != "mulcher":
                l = next(x for x in L.cfg["lookouts"] if x["name"] == lid)
                ax.plot(l["x"], l["y"], "^", c=c, ms=8)
            if P:
                P = np.array(P)
                ax.scatter(P[:, 0], P[:, 1], s=14, c=c, marker="o", label=f"{lid} ({len(P)} false-alarm scans)")
            else:
                ax.scatter([], [], c=c, label=f"{lid} (0)")
        ax.plot(0, 0, "s", c="k", ms=7)
        lim = L.cfg["forest"]["radius"]
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_aspect("equal")
        ax.set_title(f"{ln}: false alarms, main rule (object centroids)")
        ax.legend(fontsize=7, loc="lower left")
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, "false_alarms.png"), dpi=130)
    plt.close(fig)


def write_csv(path, rows):
    if not rows:
        return
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


# ------------------------------------------------------------------ text --
def results_md(res, layouts, calib):
    o = ["# Lookout experiment: results (generated by analyse.py)", "",
         "Main rule = the brief's rule as written (5 points on 2 rings, 3D 0.5 m linkage). Intervals: Wilson 95 %, "
         "approximate (walks share headings and entries).", ""]
    if calib:
        o += ["## Calibration (open ground): reliable / max range, m", "",
              "| mount / orientation | main rule | xy5 | best case |", "|---|---|---|---|"]
        for mount in ("lk0", "mulcher"):
            for orient in ("toward", "across"):
                g = lambda r: calib.get(f"{mount}/{orient}/{r}", {})
                o.append(f"| {'Husky' if mount == 'lk0' else 'mulcher'} {orient} | "
                         + " | ".join(f"{g(r).get('reliable_m')} / {g(r).get('max_m')}"
                                      for r in ("alarm_m5", "alarm_xy5", "best")) + " |")
        o.append("")
    for ln, r in res.items():
        L = layouts[ln]
        det, pr = r["detection"], r["paired"]
        o += [f"## {ln} ({len(L.main)} walks, {len(L.check)} check walks)", "",
              "Post lines: " + ", ".join(f"entry {e['id']} {e['r_post']} m" for e in L.cfg["entries"]) + ".", "",
              "| detector | rule | caught before the post line | caught before 50 m | median warning, walks detected (s) |",
              "|---|---|---|---|---|"]
        for d in ["mulcher"] + L.lk + ["team"]:
            for v in VARIANTS:
                e = det[f"{d}/{v}"]
                q = e["warn_s_q"]
                o.append(f"| {d} | {VLABEL[v]} | {pct(e['caught_before_post'])} | {pct(e['caught_before50'])} | "
                         f"{'-' if q is None else f'{q[1]:.1f}'} |")
        o += [""]
        for line, lab in (("_post", "the post line"), ("50", "50 m")):
            mc = pr[f"mcnemar_before{line}"]
            o.append(f"- McNemar exact, team vs mulcher alone, caught before {lab} (main rule): team only "
                     f"{mc['team_only']}, mulcher only {mc['mulcher_only']}, both {mc['both']}, neither "
                     f"{mc['neither']}, p = {mc['p_exact']:.3g}.")
        o += [
              f"- First catcher (main rule): {pr['first_catcher']}.",
              f"- Best lookout warning time (s, quartiles): {pr['best_lookout_warn_s_q']}.",
              *(f"- Team vs mulcher alone, same walk ({VLABEL[v]}): first alarm {qfmt(g['s_q'])} s earlier "
                f"(quartiles; least {g['s_min']:.1f} s) over the {g['n']} walks both caught; the mulcher never "
                f"raised it in {g['mulcher_never']}. Distance from the mulcher at the first alarm (m, quartiles): "
                f"mulcher {qfmt(g['dist_mulcher_m_q'])}, team {qfmt(g['dist_team_m_q'])}."
                for v, g in ((v, pr[f"extra_warning/{v}"]) for v in (MAIN, "best"))),
              "- Leave one lookout out (caught before the post line): " + "; ".join(
                  f"without {k}: {pct(v['caught_before_post'])}" for k, v in pr["leave_one_out"].items()) + ".",
              f"- Mulcher first-detection distance by wedge (m, quartiles): " + "; ".join(
                  f"{k}: {v['detected']}/{v['n']} detected, {v['dist_m_q']}" for k, v in r["wedge"].items()) + ".", ""]
        o += ["False alarms (main rule), per lidar: false-alarm scans / walk hours:", ""]
        for k, v in r["false_alarms"].items():
            if k.endswith(MAIN):
                o.append(f"- {k}: {v['false_alarm_scans']} in {v['scans']} scans ({v['walk_hours']:.2f} h), "
                         f"{v['per_hour'] or 0:.1f} /h, {v['walks_with_fa']} walks")
        o += ["", "Radio link (every post is linked by construction; checked):", ""]
        for n in L.lk:
            k = r["links"][n]
            ap = "; ".join(f"{e['d_post_m']} m from the point, linked {e['linked']}, {e['link_d_m']} m, "
                           f"{e['trees_on_link']} trunks, SNR {e['snr_db']} dB" for e in k["at_post"])
            o.append(f"- {n}: on the post: {ap or 'n/a'}; lookout alarms on a linked post "
                     f"{k['alarms_linked']}, not linked {k['alarms_not_linked']}")
        o += [""]
        for how, lab in (("check-away", "lookouts spawned, then moved 1 km away"),
                         ("check-absent", "fresh sim, lookouts never spawned")):
            cw = [c for c in r["check"] if c["how"] == how]
            o.append(f"- Check walks ({lab}): mulcher's first detection at the same step as in the matching "
                     f"walk in {sum(c['same_step'] for c in cw)}/{len(cw)}; within one step (lidar noise) in "
                     f"{sum(c['within_1_step'] for c in cw)}/{len(cw)}.")
        o += [""]
    return "\n".join(o)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", action="append", required=True, help="NAME=DIR")
    ap.add_argument("--calib", default="/runs/lookout/calib")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    figdir, tabdir = os.path.join(a.out, "figures"), os.path.join(a.out, "tables")
    os.makedirs(figdir, exist_ok=True); os.makedirs(tabdir, exist_ok=True)
    layouts = {}
    for s in a.layout:
        n, d = s.split("=", 1)
        layouts[n] = Layout(n, d)
        print(f"[analyse] {n}: {len(layouts[n].main)} walks, {len(layouts[n].check)} check walks, "
              f"{len(layouts[n].alarms)} alarm rows", flush=True)
    res, det_rows, chk_rows, fa_pts = {}, [], [], []
    for n, L in layouts.items():
        det, dr = detection(L)
        fa, pts = false_alarms(L)
        cw = check_walks(L)
        res[n] = {"detection": det, "paired": paired(L), "wedge": wedge(L), "false_alarms": fa,
                  "links": links(L), "check": cw}
        det_rows += dr; chk_rows += cw; fa_pts += pts
    calib = {}
    cp = os.path.join(a.calib, "calib_ranges.json")
    if os.path.exists(cp):
        calib = json.load(open(cp))
    write_csv(os.path.join(tabdir, "detection.csv"), det_rows)
    write_csv(os.path.join(tabdir, "check_walks.csv"), chk_rows)
    write_csv(os.path.join(tabdir, "false_alarms.csv"), fa_pts)
    json.dump({"calibration": calib, "layouts": res}, open(os.path.join(a.out, "summary.json"), "w"), indent=1,
              default=str)
    for L in layouts.values():
        fig_layout(L, figdir)
    fig_calib(a.calib, figdir)
    fig_curves(res, figdir)
    fig_bars(res, layouts, figdir)
    fig_fa(layouts, fa_pts, figdir)
    md = results_md(res, layouts, calib)
    open(os.path.join(a.out, "results.md"), "w").write(md)
    print(md)


if __name__ == "__main__":
    sys.exit(main())
