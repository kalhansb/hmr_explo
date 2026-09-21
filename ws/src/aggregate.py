#!/usr/bin/env python3
"""Aggregate scored experiment cells into the readouts of doc §5.5 and §6.1-§6.3.

Input is a run directory per cell, each holding what the runner and the scorer
left there:

    score.json            per-horizon, per-trunk M1/M1_sec12/M2/M3/M5 and the
                          gate-O1 ROI count (written by score_cell.py)
    run_manifest.txt      the arm, and every threshold the run was made at
    planner_<robot>.csv   per-step unknown_fraction / observed voxels
    planner_<robot>.log   the target closure lines
    <robot>.events.jsonl  the clock anchors, and the mission origin

Nothing here re-derives a metric: score_cell.py owns the map metrics and this
owns only the arithmetic over cells. The one exception is the target closures
and the completion time, which live in the run's own logs rather than in the
scored map, and are read here.

Everything it prints is a quantity the plan names. Where the plan's gate cannot
be evaluated as written -- and one cannot, see `gate_P1` -- it says so and shows
the number, rather than quietly substituting a test that passes.
"""

import argparse
import csv
import glob
import json
import math
import os
import random
import re
import sys

# §5.2. The three released trunks, by the scheduler's target id.
TARGET_IDS = (1, 2, 3)
# §6.3's primary horizon, and the matched-horizon grid it sits in.
PRIMARY_H = "1500"
HORIZON_ORDER = ["600", "900", "1200", "1500", "1800", "2400", "end"]
# §5.5: a target must close before this for the cell to count as treated.
TREAT_BY_S = 1500.0
# §7 gate P3.
P3_SD_LIMIT = 0.20
# §7 gate P2.
P2_SAT_LIMIT = 0.90
# §7 gate P1, "consistent with the comms control pilot".
P1_MAKESPAN = (1300.0, 2100.0)
# §7 gate O1: the two maps must agree to this.
O1_TOL = 0.01
# §5.4 H2 budgets, decided on a CI rather than a p-value (§6.4).
#
# RETIRED SCALE (plan §6.6). 0.03 was set as half the 0.55 - 0.4922 headroom
# and §6.5's calibrated 0.031 came from the 3D SD; both are 2.5D column-coverage
# numbers. Coverage is now the 2D planning map, where the same run ends at 0.061
# unknown rather than 0.502, so neither number transfers. None means "not yet
# re-derived": the readout refuses to print an H2 verdict rather than score the
# 2D series against a 3D budget. Re-derive from the exploit-off arm alone,
# before any treated cell is scored, and pin the value here.
H2_UNK_BUDGET = None
# The horizons at which H2's verdict is taken.
#
# On the retired 3D scale this was ("600","900","1200"), because the 3D measure
# stopped moving at ~1500 s and a pass at a later horizon was arithmetic rather
# than evidence. That saturation was an artifact of unobservable z-column
# volume, NOT of exploration finishing (§6.6): on the 2D scale the same run is
# still gaining map at 1500 s (0.078) and does not flatten until ~2400 s
# (0.065 -> 0.061 by 3600 s). The deciding set widens accordingly.
H2_DECIDING_H = ("600", "900", "1200", "1500", "1800")
# The headline H2 verdict: the last horizon that still carries information.
H2_HEADLINE_H = "1800"
H2_DELAY_BUDGET = 900.0
# §3: the manifest lines a campaign cell is ALLOWED to differ on. Everything
# else must match the reference cell exactly, or the cell is not a member of
# this campaign whatever directory it sits in. Prefixes, not exact keys,
# because the runner writes several timestamps and end-state lines.
CONFORMANCE_EXEMPT = (
    "exploitation_enabled", "outdir", "run_end", "run_start", "started",
    "ended", "host", "pid", "mtime_", "git_/", "sim_t0", "seed",
    # Written only when a run ends, so an in-flight cell is missing them and
    # would otherwise fail conformance against a finished reference for no
    # reason that concerns the configuration.
    "finished_utc", "started_utc", "done_drain", "run_duration",
)
# The achievable ROI unknown fraction in this world. A horizon where BOTH arms
# sit within FLOOR_TOL of it has no coverage left to lose, so the H2 budget is
# met there by arithmetic rather than by evidence -- see `h2_cost`.
#
# C4 (2026-09-21): was 0.486, which was never a property of this world. It was
# where the 3D COLUMN measure saturated, and that saturation is unobservable
# z-column volume -- on off_rep1 the 3D fraction sticks at 0.502 from t=1500s
# while the 2D map over the very same sensor data keeps falling to 0.061. On
# the 2D scale 0.486 is above every horizon the campaign reaches, so leaving it
# would mark EVERY horizon AT FLOOR and suppress the entire H2 table.
#
# The 2D floor is real but is ROI GEOMETRY, not sensing: of off_rep1's 3837
# residual unknown cells, 3808 lie in blobs touching the ROI boundary and only
# 29 are interior, and insetting the rim by 10 m leaves 8 unknown cells of
# 40000. So it is near 0.06 -- but that is one cell, measured under the offline
# two-mapper census rather than the production config, and pinning a constant
# to it would repeat the error being corrected. It is derived from the
# re-scored exploit-off arm, with the budget, and is None until then.
EXPLORE_FLOOR = None
FLOOR_TOL = 0.03
# §5.3: the M1 estimator's own worst-case error on synthetic bark. A difference
# smaller than this is not a difference, which is what makes it the threshold
# for M4's disagreement trigger (§6.4).
INSTRUMENT_ERR = 0.083

CLOSE_RE = re.compile(
    r"\[(\d+\.\d+)\].*Target (\d+) exploitation (COMPLETE|PARTIAL) "
    r"\((\d+)/(\d+) clear-LoS vantages dwelled\)")
DWELL_RE = re.compile(
    r"\[(\d+\.\d+)\].*Dwelled ([\d.]+)s at vantage (\d+) of target (\d+) "
    r"\(LoS (\w+)")


# ---------------------------------------------------------------- loading

def read_manifest(path):
    m = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, _, v = line.partition("=")
                m[k] = v
    return m


def clock_offset(events_path):
    """ROS-log stamp -> sim seconds, and the mission origin.

    The planner's log stamps and the events' `t_wall_sec` are the same clock
    (both are the node's ROS clock, which is sim time on this stack), while the
    CSV and the horizons are in `t_sim_sec`. The two differ by a constant that
    every `clock_anchor` carries both halves of, so one anchor fixes it exactly
    -- no fitting, no drift term.

    Returns (offset, t0) with sim = stamp - offset. `run_start.t0_sim_sec` is
    the mission origin the scheduler also latched its release schedule against.
    """
    off = t0 = None
    with open(events_path) as f:
        for line in f:
            try:
                e = json.loads(line)
            except ValueError:
                continue
            if e.get("event") == "run_start":
                t0 = e.get("t0_sim_sec")
            if off is None and e.get("t_wall_sec") and e.get("t_sim_sec") is not None:
                off = e["t_wall_sec"] - e["t_sim_sec"]
            if off is not None and t0 is not None:
                break
    return off, t0


def read_closures(cell, robot):
    """Target closures and clear-LoS dwells for one robot, on the sim clock.

    Both live only in the planner's text log -- the CSV's `target_id` column is
    populated on LOG_STEP rows alone, so a closure that lands between two steps
    leaves no CSV trace at all. Reading the log is not a convenience here; it is
    the only complete record.
    """
    ev = os.path.join(cell, f"{robot}.events.jsonl")
    log = os.path.join(cell, f"planner_{robot}.log")
    if not (os.path.exists(ev) and os.path.exists(log)):
        return [], []
    off, _ = clock_offset(ev)
    if off is None:
        return [], []
    closes, dwells = [], []
    with open(log, errors="replace") as f:
        for line in f:
            m = CLOSE_RE.search(line)
            if m:
                closes.append({"t": float(m.group(1)) - off, "target": int(m.group(2)),
                               "status": m.group(3), "clear": int(m.group(4)),
                               "required": int(m.group(5)), "robot": robot})
                continue
            m = DWELL_RE.search(line)
            if m:
                dwells.append({"t": float(m.group(1)) - off, "dwell_s": float(m.group(2)),
                               "vantage": int(m.group(3)), "target": int(m.group(4)),
                               "los_clear": m.group(5).lower() in ("clear", "true", "yes"),
                               "robot": robot})
    return closes, dwells


def load_cell(cell):
    name = os.path.basename(cell.rstrip("/"))
    man = read_manifest(os.path.join(cell, "run_manifest.txt"))
    out = {"name": name, "dir": cell, "manifest": man,
           "arm": "on" if man.get("exploitation_enabled") == "true" else "off",
           "run_end_reason": man.get("run_end_reason", "?"),
           "closes": [], "dwells": [], "planner": {}}
    sj = os.path.join(cell, "score.json")
    out["score"] = json.load(open(sj)) if os.path.exists(sj) else None
    for robot in ("atlas", "bestla"):
        c, d = read_closures(cell, robot)
        out["closes"] += c
        out["dwells"] += d
        csvp = os.path.join(cell, f"planner_{robot}.csv")
        if os.path.exists(csvp):
            with open(csvp) as f:
                out["planner"][robot] = list(csv.DictReader(f))
    # C4: the per-step `unknown_fraction` in planner_<robot>.csv is the RETIRED
    # 3D column measure -- it is what the planner computed while the cell ran,
    # and it is an artifact (plan §6.6). coverage_2d.csv is the same cell
    # re-scored off its own bagged scovox_bin streams under the 2D planning-map
    # definition. When it exists it REPLACES the planner column; it is never
    # merged with it, because the two are not the same quantity on the same
    # scale.
    cov = os.path.join(cell, "coverage_2d.csv")
    if os.path.exists(cov):
        with open(cov) as f:
            out["cov2d"] = list(csv.DictReader(f))
    else:
        out["cov2d"] = None
    ev = os.path.join(cell, "atlas.events.jsonl")
    out["t0"] = clock_offset(ev)[1] if os.path.exists(ev) else None
    return out


# The done_coverage_source values that mean the planner's own per-step
# unknown_fraction is ALREADY on the 2D scale. A cell run after C4 measures the
# 2D map live, so re-scoring it off its bag would be doing the same arithmetic
# twice. "scovox" is the retired 3D measure and is deliberately absent.
NATIVE_2D_SOURCES = ("coverage_map", "planning_map", "planning_map_inflated")


def unknown_series(c):
    """The cell's ROI unknown-fraction series, on the 2D scale, or None.

    One series per CELL, not per robot: the unknown fraction is read off the
    FUSED team map, so both robots' planner rows carried the same number and
    averaging them only ever averaged two copies of one quantity.

    Two ways a cell can have one. A cell run after C4 measured the 2D map while
    it ran, so planner_<robot>.csv is already the right quantity. A cell run
    before C4 measured the 3D column, and has to be re-scored off its bag into
    coverage_2d.csv (ws/src/rescore_2d).

    Which one applies is decided by the cell's RECORDED provenance, not by
    which files happen to exist: `done_coverage_source` in the manifest says
    what the planner actually measured. A cell whose manifest says "scovox" and
    that has not been re-scored returns None, and callers must report it as
    missing. Falling back to the planner column there is exactly the error
    §6.6 corrects, and it would be invisible -- the 3D series is a
    plausible-looking number in the same units.
    """
    if c.get("cov2d"):
        return c["cov2d"]
    src = (c.get("manifest") or {}).get("done_coverage_source")
    if src in NATIVE_2D_SOURCES:
        # The team map is identical for both robots, so any one robot's rows
        # carry it; take the longest in case one planner was cut short.
        rows = list(c.get("planner", {}).values())
        return max(rows, key=len) if rows else None
    return None


# ------------------------------------------------------- derived quantities

def completion_time(rows, thresh, consec=3):
    """First sim time at which the ROI unknown fraction has been at or below
    `thresh` for `consec` consecutive logged steps.

    This is the planner's own DONE predicate (done_unknown_fraction /
    done_min_consecutive_steps), applied after the fact. The campaign runs with
    the stop latch disabled so that a cell lasts long enough to reach the plan's
    horizons at all; that removes the run's `all_done` end, not the quantity --
    the same crossing is still in the record, and reading it here rather than
    acting on it keeps both arms measured by one identical rule.
    """
    run = 0
    for r in rows:
        try:
            unk = float(r["unknown_fraction"])
            t = float(r["sim_time_sec"])
        except (KeyError, ValueError):
            continue
        if unk < 0:
            continue
        run = run + 1 if unk <= thresh else 0
        if run >= consec:
            return t
    return None


def series_at(rows, t, field):
    """Value of `field` at the last logged step at or before sim time `t`.

    Last-at-or-before, never interpolated and never the nearest: these series
    are step functions sampled when the planner happens to log, and a horizon
    must read what the run actually had by then, not a value blended from a
    sample that had not happened yet.
    """
    best = last_t = None
    for r in rows:
        try:
            rt = float(r["sim_time_sec"])
        except (KeyError, ValueError):
            continue
        last_t = rt
        if rt <= t:
            try:
                best = float(r[field])
            except (KeyError, ValueError):
                pass
    # A horizon the run never reached has no value, and MUST NOT report the
    # last one it had. Carrying the final row forward would let a cell that
    # died at 520 s contribute a flat 0.636 at every horizon out to 2400 s,
    # indistinguishable from a cell that genuinely held that value there --
    # and it would enter the arm mean as if it were data.
    if last_t is None or t > last_t:
        return None
    return best


def trunks_at(cell, horizon):
    h = (cell.get("score") or {}).get("horizons", {}).get(horizon)
    return h.get("trunks", {}) if h else {}


def target_means(cell, horizon, metric):
    """Mean of `metric` over the three target trunks at a horizon."""
    tr = trunks_at(cell, horizon)
    vals = [v[metric] for v in tr.values()
            if v.get("target_id") in TARGET_IDS and metric in v]
    return sum(vals) / len(vals) if len(vals) == len(TARGET_IDS) else None


def control_means(cell, horizon, metric):
    """Mean of `metric` over the near-control trunks at a horizon.

    Targets are excluded because they are the treatment, and
    `excluded_control` trunks because §5.2 excludes them for a reason that has
    nothing to do with the arm (a visitor model stands 0.12 m from Oak tree_19,
    so its surface is not the planner's to see).
    """
    tr = trunks_at(cell, horizon)
    vals = [v[metric] for v in tr.values()
            if v.get("target_id") is None and not v.get("excluded_control")
            and metric in v]
    return sum(vals) / len(vals) if vals else None


# ------------------------------------------------------------- statistics

def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


def sd(xs):
    """Sample SD (n-1). The plan's floor is an estimate of the population
    spread from a handful of repeats, which is what n-1 is for; n would bias it
    down by 18% at the pilot's n=3, right where gate P3 reads it."""
    xs = [x for x in xs if x is not None]
    if len(xs) < 2:
        return None
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def bootstrap_ci(xs, n=10000, alpha=0.05, rng=None):
    xs = [x for x in xs if x is not None]
    if len(xs) < 2:
        return (None, None)
    rng = rng or random.Random(12345)
    boots = []
    for _ in range(n):
        boots.append(sum(rng.choice(xs) for _ in xs) / len(xs))
    boots.sort()
    return (boots[int(alpha / 2 * n)], boots[int((1 - alpha / 2) * n) - 1])


def bootstrap_ci_diff(on, off, n=10000, alpha=0.05, rng=None):
    """95% CI on the difference of arm means, resampling each arm separately.

    The arms are independent samples, not a paired series (§3, SEED is inert),
    so each is resampled within itself and the difference taken per replicate.
    Pooling first would build the null into the interval.
    """
    on = [x for x in on if x is not None]
    off = [x for x in off if x is not None]
    if len(on) < 2 or len(off) < 2:
        return (None, None)
    rng = rng or random.Random(12345)
    boots = []
    for _ in range(n):
        a = sum(rng.choice(on) for _ in on) / len(on)
        b = sum(rng.choice(off) for _ in off) / len(off)
        boots.append(a - b)
    boots.sort()
    return (boots[int(alpha / 2 * n)], boots[int((1 - alpha / 2) * n) - 1])


def budget_verdict(lo, hi, budget):
    """§6.4: a cost claim is decided on the interval, not on a p-value.

    ACCEPT only if the whole interval is under the budget, REJECT only if the
    whole interval is over it, UNDECIDED whenever it straddles -- which is the
    honest answer at this n and the one a non-significant p would have
    disguised as a pass.
    """
    if budget is None:
        return "NO VERDICT (H2's budget is not derived on the 2D scale yet)"
    if lo is None or hi is None:
        return "UNDECIDED (no interval)"
    if hi < budget:
        return "ACCEPT (cost within budget)"
    if lo > budget:
        return "REJECT (cost exceeds budget)"
    return "UNDECIDED (interval straddles the budget)"


def permutation_p(on, off, greater=True):
    """One-sided permutation test over arm labels, §6.3.

    Exhaustive, not sampled: 10 cells give C(10,5)=252 labelings, so the exact
    null is small enough to enumerate and there is no reason to approximate it.
    The floor this implies is reported alongside -- with 252 labelings the
    smallest attainable p is 1/252 = 0.004, and an effect can be real and still
    not go below it, so the number means nothing without it.
    """
    on = [x for x in on if x is not None]
    off = [x for x in off if x is not None]
    if not on or not off:
        return None, None
    pool = on + off
    k, n = len(on), len(pool)
    obs = sum(on) / k - sum(off) / (n - k)
    from itertools import combinations
    count = tot = 0
    for idx in combinations(range(n), k):
        s = set(idx)
        a = sum(pool[i] for i in idx) / k
        b = sum(pool[i] for i in range(n) if i not in s) / (n - k)
        d = a - b
        tot += 1
        if (d >= obs - 1e-12) if greater else (d <= obs + 1e-12):
            count += 1
    return count / tot, 1.0 / tot


def conformance(cells, reference):
    """§3: one variable. Reject any cell whose manifest differs from the
    reference on any line but the exempt ones.

    This is the check that keeps a stray directory out of an arm. A pilot or
    shakeout cell run at a different `done_unknown_fraction` sits in the same
    `runs/` tree, carries a valid manifest, and would otherwise be averaged
    into the exploit-off arm as though it were a replicate -- which is exactly
    how a campaign silently acquires a second variable.
    """
    ref = reference["manifest"]
    out = []
    for c in cells:
        man = c["manifest"]
        diffs = []
        for k in sorted(set(ref) | set(man)):
            if any(k.startswith(e) for e in CONFORMANCE_EXEMPT):
                continue
            if ref.get(k) != man.get(k):
                diffs.append((k, ref.get(k), man.get(k)))
        out.append({"cell": c["name"], "conforms": not diffs, "diffs": diffs})
    return out


# ------------------------------------------------------------------ gates

def manipulation_check(cell):
    """§5.5. A cell counts as TREATED only if every target was really worked.

    Two conditions, both per target: at least one dwell whose line of sight to
    the trunk was clear, and a closure (COMPLETE or PARTIAL) before 1500 s. A
    dwell with LoS blocked is a robot that stood in the right place and saw
    nothing, which is not the treatment; a target still open at the primary
    horizon was not delivered by the time the endpoint is read.

    A cell that fails is reported with its reason and then EXCLUDED from H1 --
    kept in the record, kept out of the average. It is not evidence about what
    exploitation does to the map, because on that cell exploitation did not
    happen.
    """
    if cell["arm"] != "on":
        return {"treated": None, "reason": "exploit-off cell; no targets released"}
    per = {}
    for tid in TARGET_IDS:
        cl = [c for c in cell["closes"] if c["target"] == tid]
        dw = [d for d in cell["dwells"] if d["target"] == tid and d["los_clear"]]
        first = min((c["t"] for c in cl), default=None)
        per[tid] = {"closed": bool(cl), "closed_at": first,
                    "status": cl[0]["status"] if cl else None,
                    "clear_dwells": len(dw),
                    "in_time": first is not None and first <= TREAT_BY_S}
    bad = []
    for tid, p in per.items():
        if p["clear_dwells"] == 0:
            bad.append(f"target {tid}: no clear-LoS dwell")
        elif not p["closed"]:
            bad.append(f"target {tid}: never closed")
        elif not p["in_time"]:
            bad.append(f"target {tid}: closed at {p['closed_at']:.0f}s > {TREAT_BY_S:.0f}s")
    return {"treated": not bad, "reason": "; ".join(bad) or "all three targets worked",
            "per_target": per}


def gate_O1(cell):
    """The offline oracle and the robots' own maps must be the same map.

    Compared against the LARGER of the two robots' final observed-voxel counts,
    not their mean: the two are the same team map seen through two mergers, so
    the larger is the one that had integrated more of it by the end, and the
    smaller only says that robot's merger was behind. Taking the mean would
    charge the oracle for the lagging robot.
    """
    sc = cell.get("score")
    if not sc:
        return None
    last = None
    for h in reversed(HORIZON_ORDER):
        e = sc.get("horizons", {}).get(h)
        if e and "roi_observed_voxels" in e:
            last, label = e["roi_observed_voxels"], h
            break
    if last is None:
        return None
    planner = []
    for rows in cell["planner"].values():
        v = [float(r["total_observed_voxels"]) for r in rows
             if r.get("total_observed_voxels")]
        if v:
            planner.append(max(v))
    if not planner:
        return None
    ref = max(planner)
    rel = abs(last - ref) / ref if ref else None
    return {"horizon": label, "oracle": last, "planner_max": ref,
            "rel_diff": rel, "pass": rel is not None and rel <= O1_TOL}


def gate_P1(cells):
    """Pilot cells terminate the way the plan expects.

    As written the gate reads `run_end_reason == all_done` with a makespan of
    1300-2100 s. Neither half is evaluable as written on this campaign, and for
    the same single reason: the stop latch is disabled, so no run declares DONE
    and every cell ends at its duration. The crossing itself is still there, so
    what is reported is the DERIVED completion time -- the first moment the
    unknown fraction stayed at or below the run's own done threshold for three
    steps -- together with whether it lands in the plan's band.

    The band was calibrated before the coverage measure changed, so read it as
    provisional. On the 2D map 90 % of the ROI is known near t_sim 1300 in the
    pilot cell -- just inside P1's 1300-2100 s band, where the retired 3D
    criterion was met near 485 s, a third of the lower bound. If the band is
    wrong it is reported as missed, not silently widened.
    """
    out = []
    for c in cells:
        # 0.10 = 90 % of the ROI known on the 2D map (plan §6.6). The old 0.64
        # default is a 3D column-coverage number; applied to a 2D series it is
        # crossed in the first few minutes of every cell and makes every
        # makespan meaningless. The manifest value still wins when present.
        thresh = float(c["manifest"].get("done_unknown_fraction") or 0) or 0.10
        rows = unknown_series(c)
        mk = completion_time(rows, thresh) if rows else None
        out.append({"cell": c["name"], "makespan": mk, "threshold": thresh,
                    "rescored": rows is not None,
                    "end_reason": c["run_end_reason"],
                    "in_band": mk is not None and P1_MAKESPAN[0] <= mk <= P1_MAKESPAN[1]})
    return out


def gate_P2(cells):
    """Is M1 saturated on the untreated arm? If it is, M3 becomes primary.

    Read on exploit-off cells only, and the plan requires it to be settled
    before any exploit-on cell is scored: a primary endpoint chosen after
    seeing the treated arm is chosen partly by the effect it is meant to test.
    """
    vals = [target_means(c, PRIMARY_H, "M1") for c in cells if c["arm"] == "off"]
    m = mean(vals)
    alt = mean([target_means(c, PRIMARY_H, "M3") for c in cells if c["arm"] == "off"])
    # No off-arm data is UNDECIDED, never a verdict. Returning "saturated" from
    # an empty average would flip the primary endpoint to M3 on the strength of
    # having measured nothing.
    if m is None:
        return {"off_mean_M1": None, "off_mean_M3": alt, "per_cell_M1": vals,
                "pass": None, "primary": "M1",
                "note": "UNDECIDED: no scored exploit-off cell yet; "
                        "primary stays M1 until the pilot decides it"}
    return {"off_mean_M1": m, "off_mean_M3": alt, "per_cell_M1": vals,
            "pass": m < P2_SAT_LIMIT,
            "primary": "M1" if m < P2_SAT_LIMIT else "M3"}


def resolvable(sd_noise, budget, n_per_arm=5, z=1.96):
    """Can a difference this small be told from zero at this n?

    Half-width of the 95% CI on a difference of two arm means, each of
    `n_per_arm` cells drawn from a population with SD `sd_noise`:
    z * sd * sqrt(2/n). If that exceeds the budget, the budget sits inside the
    noise and the H2 verdict can only come back UNDECIDED however the cells
    fall -- which is worth knowing from the PILOT, before five treated cells
    are spent measuring something the design cannot resolve.
    """
    if sd_noise is None:
        return None
    half = z * sd_noise * math.sqrt(2.0 / n_per_arm)
    if budget is None:
        # C4: the 3D budget was retired with the 3D coverage measure and the 2D
        # one is not derived yet. The half-width is still worth reporting -- it
        # is a property of the design, not of the budget -- but there is nothing
        # to compare it against, and inventing one here would be the same error
        # as carrying 0.03 across the scale change.
        return {"ci_half_width": half, "budget": None, "resolvable": None}
    return {"ci_half_width": half, "budget": budget,
            "resolvable": half <= budget}


def calibrated_budget(sd_noise, n_per_arm=5, z=1.96):
    """The smallest budget this design can actually resolve, from the pilot.

    Same arithmetic as `resolvable`, read the other way round: the CI
    half-width IS the smallest difference that can be told from zero at this n,
    so it is the smallest budget against which a verdict can be anything but
    UNDECIDED. It is computed from the EXPLOIT-OFF arm alone, before any
    treated cell is scored, and it never replaces the plan's H2_UNK_BUDGET --
    a budget moved to fit its own noise floor is not a pre-registered budget
    (plan §6.5). It is reported beside it so the gap between the two is on the
    page: that gap is what n = 5 an arm buys.
    """
    if sd_noise is None:
        return None
    return z * sd_noise * math.sqrt(2.0 / n_per_arm)


def gate_P3(cells):
    """The noise floor, from the exploit-off pilot."""
    tgt = [target_means(c, PRIMARY_H, "M1") for c in cells if c["arm"] == "off"]
    ctl = [control_means(c, PRIMARY_H, "M1") for c in cells if c["arm"] == "off"]
    unk = []
    for c in cells:
        if c["arm"] != "off":
            continue
        rows = unknown_series(c)
        unk.append(series_at(rows, 1200.0, "unknown_fraction") if rows else None)
    s = sd(tgt)
    n = len([x for x in tgt if x is not None])
    su = sd(unk)
    # The unknown-fraction SD needs no scoring, so it is available from the
    # pilot's CSVs alone, before any oracle merge has been run.
    h2 = resolvable(su, H2_UNK_BUDGET)
    out = {"sd_target_M1": s, "sd_control_M1": sd(ctl), "sd_unknown_1200": su,
           "n_unknown": len([x for x in unk if x is not None]),
           "h2_resolvable": h2, "n": n}
    if s is None:
        out.update({"pass": None, "action": None,
                    "note": f"UNDECIDED: {n} scored exploit-off cell(s); "
                            "an SD needs at least 2"})
        return out
    out.update({"pass": s <= P3_SD_LIMIT,
                "action": None if s <= P3_SD_LIMIT
                          else "extend BOTH arms by 3 cells and rescore the whole "
                               "campaign (post-hoc symmetric rule, plan §6.5)"})
    return out


# ---------------------------------------------------------------- readouts

def endpoint(cells, horizon, metric, which="targets"):
    f = target_means if which == "targets" else control_means
    on = [f(c, horizon, metric) for c in cells if c["arm"] == "on" and c.get("include", True)]
    off = [f(c, horizon, metric) for c in cells if c["arm"] == "off"]
    on = [x for x in on if x is not None]
    off = [x for x in off if x is not None]
    p, floor = permutation_p(on, off)
    return {"horizon": horizon, "metric": metric, "which": which,
            "on_mean": mean(on), "off_mean": mean(off),
            "on_ci": bootstrap_ci(on), "off_ci": bootstrap_ci(off),
            "delta": (mean(on) - mean(off)) if on and off else None,
            "n_on": len(on), "n_off": len(off), "p": p, "p_floor": floor}


def h3_did(cells, horizon, metric):
    """H3 as a difference-in-differences: what exploitation did to the trunks it
    was aimed at, over and above what it did to the trunks it was not.

    The near controls absorb whatever moved the whole map that cell -- a lucky
    route, a noisier lidar, a robot that covered more ground. Only the excess
    over them is attributable to having been targeted.
    """
    def did(cs):
        t = mean([target_means(c, horizon, metric) for c in cs])
        n = mean([control_means(c, horizon, metric) for c in cs])
        return (t - n) if (t is not None and n is not None) else None
    on = [c for c in cells if c["arm"] == "on" and c.get("include", True)]
    off = [c for c in cells if c["arm"] == "off"]
    per_on = [(target_means(c, horizon, metric), control_means(c, horizon, metric)) for c in on]
    per_off = [(target_means(c, horizon, metric), control_means(c, horizon, metric)) for c in off]
    d_on = [a - b for a, b in per_on if a is not None and b is not None]
    d_off = [a - b for a, b in per_off if a is not None and b is not None]
    p, floor = permutation_p(d_on, d_off)
    return {"horizon": horizon, "metric": metric,
            "on_gap": mean(d_on), "off_gap": mean(d_off),
            "did": (mean(d_on) - mean(d_off)) if d_on and d_off else None,
            "on_gap_ci": bootstrap_ci(d_on), "off_gap_ci": bootstrap_ci(d_off),
            "p": p, "p_floor": floor}


def h2_cost(cells, horizon, calibrated=None):
    """H2: what exploitation cost exploration, at a matched horizon.

    `unknown_fraction` at the horizon is the direct reading. Completion time is
    reported beside it because the plan asks for it, derived as in `gate_P1`.
    """
    def unk(c):
        rows = unknown_series(c)
        if rows is None or horizon == "end":
            return None
        return series_at(rows, float(horizon), "unknown_fraction")
    on = [unk(c) for c in cells if c["arm"] == "on" and c.get("include", True)]
    off = [unk(c) for c in cells if c["arm"] == "off"]
    on = [x for x in on if x is not None]
    off = [x for x in off if x is not None]
    # Higher unknown = worse, so the one-sided alternative is on > off.
    p, floor = permutation_p(on, off, greater=True)
    on_m, off_m = mean(on), mean(off)
    d_lo, d_hi = bootstrap_ci_diff(on, off)
    # Secondary, and labelled as such wherever it is printed.
    cal_v = budget_verdict(d_lo, d_hi, calibrated) if calibrated else None
    # A horizon at which both arms have already hit the world's floor cannot
    # show a cost even if one exists: there is no unknown volume left for
    # exploitation to have denied the explorer. Reading "within budget" off
    # such a horizon would be reporting the world's geometry as a result about
    # the treatment. Flagged so the verdict is read only where it means
    # something. On the 2D map that is every horizon this campaign reaches:
    # coverage is still climbing at 1800 s and the floor is not approached
    # until ~2800 s.
    at_floor = (EXPLORE_FLOOR is not None
                and on_m is not None and off_m is not None
                and max(on_m, off_m) <= EXPLORE_FLOOR + FLOOR_TOL)
    return {"horizon": horizon, "at_floor": at_floor,
            "on_mean": mean(on), "off_mean": mean(off),
            "delta": (mean(on) - mean(off)) if on and off else None,
            "on_ci": bootstrap_ci(on), "off_ci": bootstrap_ci(off),
            "delta_ci": (d_lo, d_hi), "budget": H2_UNK_BUDGET,
            "verdict": budget_verdict(d_lo, d_hi, H2_UNK_BUDGET),
            "deciding": horizon in H2_DECIDING_H,
            "calibrated_budget": calibrated,
            "calibrated_verdict": cal_v,
            "n_on": len(on), "n_off": len(off), "p": p, "p_floor": floor}


def h2_delay(cells):
    """H2's other budget: how much later exploitation finishes the sweep.

    With the stop latch disabled no run declares DONE, so this reads the
    derived completion time of `gate_P1` -- the same crossing, read after the
    fact by the planner's own predicate. A cell whose unknown fraction never
    meets the threshold within T has no completion time: it is counted and
    never averaged, which is §5.4's rule kept in intent. Its own blanket rule,
    that no delta is printed if any cell in an arm is censored, cannot be kept
    in letter -- under DONE_UNKNOWN=0 every cell is censored at T, so it would
    print nothing at all.
    """
    times = {r["cell"]: r["makespan"] for r in gate_P1(cells)}
    def arm(a):
        cs = [c for c in cells if c["arm"] == a and (a == "off" or c.get("include", True))]
        vals = [times[c["name"]] for c in cs]
        return [v for v in vals if v is not None], sum(1 for v in vals if v is None)
    on, on_missing = arm("on")
    off, off_missing = arm("off")
    d_lo, d_hi = bootstrap_ci_diff(on, off)
    p, floor = permutation_p(on, off, greater=True)
    return {"on_mean": mean(on), "off_mean": mean(off),
            "delta": (mean(on) - mean(off)) if on and off else None,
            "delta_ci": (d_lo, d_hi), "budget": H2_DELAY_BUDGET,
            "verdict": budget_verdict(d_lo, d_hi, H2_DELAY_BUDGET),
            "n_on": len(on), "n_off": len(off),
            "no_completion_on": on_missing, "no_completion_off": off_missing,
            "p": p, "p_floor": floor}


def m4_trigger(cells, metric):
    """§6.4: whether M4 has to be computed, decided by a stated condition.

    Two triggers, both fixed before the data: the primary metric and M2 point
    in OPPOSITE directions with neither difference inside the instrument's own
    error, or the primary is saturated in both arms at gate P2's ceiling. The
    plan's original wording -- compute M4 "if M1-M3 disagree" -- would have let
    the decision be made after seeing which metric helped.
    """
    prim = endpoint(cells, PRIMARY_H, metric)
    m2 = endpoint(cells, PRIMARY_H, "M2")
    dp, d2 = prim["delta"], m2["delta"]
    reasons = []
    if dp is not None and d2 is not None:
        big = abs(dp) > INSTRUMENT_ERR and abs(d2) > INSTRUMENT_ERR
        if big and (dp > 0) != (d2 > 0):
            reasons.append(f"{metric} and M2 disagree in sign "
                           f"({metric} {dp:+.3f}, M2 {d2:+.3f}), both beyond "
                           f"the {INSTRUMENT_ERR} instrument error")
    if (prim["on_mean"] is not None and prim["off_mean"] is not None
            and min(prim["on_mean"], prim["off_mean"]) >= P2_SAT_LIMIT):
        reasons.append(f"{metric} is saturated in BOTH arms "
                       f"(on {prim['on_mean']:.3f}, off {prim['off_mean']:.3f} "
                       f">= {P2_SAT_LIMIT})")
    return {"required": bool(reasons), "reasons": reasons,
            "primary_delta": dp, "m2_delta": d2}


# ------------------------------------------------------------------ output

def fmt(x, n=3):
    return "n/a" if x is None else f"{x:.{n}f}"


def ci(pair):
    return "n/a" if pair[0] is None else f"[{pair[0]:.3f}, {pair[1]:.3f}]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", help="directory holding the cell directories")
    ap.add_argument("--metric", default=None,
                    help="primary metric; default is whatever gate P2 selects")
    ap.add_argument("--json", default=None, help="also write the full readout here")
    ap.add_argument("--reference", default=None,
                    help="cell whose manifest is the §3 reference; "
                         "default is the first campaign cell")
    ap.add_argument("--include-unlisted", action="store_true",
                    help="score cells absent from index.csv (they are not "
                         "campaign cells; this is for inspecting a pilot)")
    args = ap.parse_args()

    dirs = sorted(d for d in glob.glob(os.path.join(args.runs, "*"))
                  if os.path.exists(os.path.join(d, "run_manifest.txt")))
    if not dirs:
        sys.exit(f"no cells with a run_manifest.txt under {args.runs}")
    cells = [load_cell(d) for d in dirs]

    # Campaign membership, before anything is averaged. The driver appends one
    # row per finished cell to index.csv, so that file -- not the directory
    # listing -- is the campaign's own record of which cells it ran.
    idx = os.path.join(args.runs, "index.csv")
    listed = None
    if os.path.exists(idx) and not args.include_unlisted:
        with open(idx) as f:
            listed = {r["cell"] for r in csv.DictReader(f) if r.get("cell")}
    if listed:
        outside = [c["name"] for c in cells if c["name"] not in listed]
        cells = [c for c in cells if c["name"] in listed]
        if outside:
            print("not in index.csv, not part of this campaign: "
                  + ", ".join(outside))
    if not cells:
        sys.exit("no campaign cells: index.csv lists none of the directories "
                 "found (pass --include-unlisted to score them anyway)")

    # §3: one variable. A cell that differs from the reference on any other
    # manifest line is not a replicate of it.
    ref = next((c for c in cells if c["name"] == args.reference), cells[0]) \
        if args.reference else cells[0]
    nonconforming = []
    for r in conformance(cells, ref):
        if not r["conforms"]:
            nonconforming.append(r)
    if nonconforming:
        print()
        print("=" * 78)
        print(f"MANIFEST CONFORMANCE (§3) -- reference {ref['name']}")
        print("=" * 78)
        for r in nonconforming:
            print(f"  {r['cell']:<12} DIFFERS, excluded from every average:")
            for k, a, b in r["diffs"][:12]:
                print(f"      {k}: reference={a!r} cell={b!r}")
            if len(r["diffs"]) > 12:
                print(f"      ... and {len(r['diffs']) - 12} more")
    bad = {r["cell"] for r in nonconforming}
    cells = [c for c in cells if c["name"] not in bad]
    if not cells:
        sys.exit("no cells conform to the reference manifest")

    print("=" * 78)
    print("CELLS")
    print("=" * 78)
    for c in cells:
        scored = "scored" if c["score"] else "NOT SCORED"
        print(f"  {c['name']:<12} arm={c['arm']:<3} end={c['run_end_reason']:<12} "
              f"t0={fmt(c['t0'], 2):<8} {scored}")

    # §5.5 manipulation check. Runs before any average is taken, because it
    # decides which cells are allowed into one.
    print()
    print("=" * 78)
    print("MANIPULATION CHECK (§5.5)")
    print("=" * 78)
    for c in cells:
        mc = manipulation_check(c)
        c["include"] = mc["treated"] is not False
        if mc["treated"] is None:
            print(f"  {c['name']:<12} -- {mc['reason']}")
            continue
        verdict = "TREATED" if mc["treated"] else "EXCLUDED from H1"
        print(f"  {c['name']:<12} {verdict}: {mc['reason']}")
        for tid, p in sorted(mc["per_target"].items()):
            print(f"      target {tid}: status={p['status'] or '-':<8} "
                  f"closed_at={fmt(p['closed_at'], 0):<8} clear_dwells={p['clear_dwells']}")

    print()
    print("=" * 78)
    print("GATES")
    print("=" * 78)
    for r in gate_O1_all(cells):
        print(f"  O1 {r['cell']:<12} oracle={r['oracle']:<10} planner={r['planner_max']:<12.0f} "
              f"rel={fmt(r['rel_diff'], 4)}  {'PASS' if r['pass'] else 'FAIL'}")
    print()
    unscored = []
    for r in gate_P1(cells):
        if not r["rescored"]:
            unscored.append(r["cell"])
            band = "NOT RE-SCORED on the 2D map -- no verdict"
        elif r["makespan"] is None:
            band = f"never reached unknown<={r['threshold']} within T"
        else:
            band = ("in band" if r["in_band"]
                    else f"OUTSIDE {P1_MAKESPAN[0]:.0f}-{P1_MAKESPAN[1]:.0f}s")
        print(f"  P1 {r['cell']:<12} derived completion={fmt(r['makespan'], 0):<8} "
              f"(unknown<={r['threshold']}) {band}")
    print("     P1 note: the stop latch is disabled campaign-wide, so no run ends"
          "\n     'all_done'; the completion time above is the same crossing read"
          "\n     off the logged series instead of acted on.")
    if unscored:
        print(f"     {len(unscored)} cell(s) carry only the retired 3D column series:"
              f"\n     {', '.join(unscored)}. Their unknown fractions are LEFT BLANK"
              "\n     rather than filled from planner_<robot>.csv, which measures a"
              "\n     different quantity on a different scale (§6.6). Re-score them"
              "\n     off their bags into <cell>/coverage_2d.csv.")
    p2 = gate_P2(cells)
    print()
    verdict = "UNDECIDED" if p2["pass"] is None else ("PASS" if p2["pass"] else "SATURATED")
    print(f"  P2 exploit-off mean M1 @{PRIMARY_H}s = {fmt(p2['off_mean_M1'])} "
          f"(limit {P2_SAT_LIMIT}) -> primary metric {p2['primary']}  {verdict}")
    if p2.get("note"):
        print(f"     {p2['note']}")
    print(f"     off-arm M3 for comparison: {fmt(p2['off_mean_M3'])}")
    p3 = gate_P3(cells)
    v3 = "UNDECIDED" if p3["pass"] is None else ("PASS" if p3["pass"] else "FAIL")
    print(f"  P3 SD(target M1)={fmt(p3['sd_target_M1'])} over n={p3['n']} off cells "
          f"(limit {P3_SD_LIMIT})  {v3}")
    if p3.get("note"):
        print(f"     {p3['note']}")
    print(f"     SD(control M1)={fmt(p3['sd_control_M1'])}  "
          f"SD(unknown@1200s)={fmt(p3['sd_unknown_1200'])} "
          f"over n={p3['n_unknown']} off cells")
    h2r = p3.get("h2_resolvable")
    if h2r and h2r["resolvable"] is None:
        print(f"     the 95% CI half-width this noise implies at n=5 an arm is "
              f"{fmt(h2r['ci_half_width'])},\n     but H2's budget is not derived on"
              " the 2D scale yet, so whether it\n     is resolvable CANNOT BE"
              " ASSESSED. This half-width is the floor any\n     re-derived budget"
              " has to clear to be worth pre-registering.")
    elif h2r:
        print(f"     H2 budget {h2r['budget']} vs the 95% CI half-width this "
              f"noise implies\n     at n=5 an arm, {fmt(h2r['ci_half_width'])}: "
              f"{'RESOLVABLE' if h2r['resolvable'] else 'NOT RESOLVABLE'}")
        if not h2r["resolvable"]:
            print("     The budget sits inside the replicate noise, so H2 can only"
                  "\n     come back UNDECIDED however the cells fall. That is a"
                  "\n     property of the design read off the PILOT, not a result;"
                  "\n     report it as a limit on H2 rather than as no cost.")
    if p3["action"]:
        print(f"     ACTION: {p3['action']}")

    metric = args.metric or p2["primary"]
    n_on = sum(1 for c in cells if c["arm"] == "on" and c.get("include", True))
    n_off = sum(1 for c in cells if c["arm"] == "off")

    print()
    print("=" * 78)
    print(f"H1 PRIMARY ENDPOINT -- mean {metric} over the 3 targets @ t={PRIMARY_H}s")
    print("=" * 78)
    e = endpoint(cells, PRIMARY_H, metric)
    print(f"  exploit-on  {fmt(e['on_mean'])}  95% CI {ci(e['on_ci'])}  n={e['n_on']}")
    print(f"  exploit-off {fmt(e['off_mean'])}  95% CI {ci(e['off_ci'])}  n={e['n_off']}")
    print(f"  delta       {fmt(e['delta'])}")
    if e["p"] is not None:
        print(f"  one-sided permutation p = {e['p']:.4f} "
              f"(smallest attainable with these n: {e['p_floor']:.4f})")

    print()
    print("=" * 78)
    print(f"MATCHED-HORIZON CURVE -- mean {metric} over the 3 targets")
    print("=" * 78)
    print(f"  {'horizon':<9} {'on':>8} {'off':>8} {'delta':>8} {'p':>8}")
    for h in HORIZON_ORDER:
        r = endpoint(cells, h, metric)
        if r["on_mean"] is None and r["off_mean"] is None:
            continue
        print(f"  {h:<9} {fmt(r['on_mean']):>8} {fmt(r['off_mean']):>8} "
              f"{fmt(r['delta']):>8} {fmt(r['p'], 4) if r['p'] is not None else 'n/a':>8}")

    print()
    print("=" * 78)
    print("H2 COST TO EXPLORATION -- ROI unknown fraction at matched horizons")
    print("=" * 78)
    cal = calibrated_budget(gate_P3(cells)["sd_unknown_1200"])
    print(f"  {'horizon':<9} {'on':>8} {'off':>8} {'delta':>8} "
          f"{'95% CI on delta':>20} {'p':>8}")
    for h in HORIZON_ORDER:
        if h == "end":
            continue
        r = h2_cost(cells, h, calibrated=cal)
        if r["on_mean"] is None and r["off_mean"] is None:
            continue
        print(f"  {h:<9} {fmt(r['on_mean']):>8} {fmt(r['off_mean']):>8} "
              f"{fmt(r['delta']):>8} {ci(r['delta_ci']):>20} "
              f"{fmt(r['p'], 4) if r['p'] is not None else 'n/a':>8}"
              f"{'   AT FLOOR' if r['at_floor'] else ''}"
              f"{'   [deciding]' if r['deciding'] else ''}")
    # NOT PRIMARY_H. On the retired 3D measure 1500 s was past saturation, so
    # the H1 horizon could not show a cost at all. On the 2D measure it CAN --
    # the map is still filling there -- but the headline stays at 1800 s, which
    # is where the arms have had the longest to diverge while both are still
    # above the floor (§6.6).
    rp = h2_cost(cells, H2_HEADLINE_H, calibrated=cal)
    if H2_UNK_BUDGET is None:
        print("\n  H2's budget is NOT SET. The pre-registered 0.03 was fixed on the"
              "\n     retired 3D column measure, whose unknown fraction floors near"
              "\n     0.50; the 2D planning map runs the same cells down to ~0.06, so"
              "\n     a delta on this scale means something different in kind, not"
              "\n     merely in size. It is re-derived from the re-scored exploit-off"
              "\n     arm alone, before any treated cell is scored (plan §6.6). Until"
              "\n     then the deltas below are reported and NO H2 verdict is printed.")
    else:
        print(f"\n  budget {H2_UNK_BUDGET} on the delta at t={H2_HEADLINE_H}s "
              f"-> {rp['verdict']}")
        for h in H2_DECIDING_H:
            if h == H2_HEADLINE_H:
                continue
            print(f"     and at t={h}s -> {h2_cost(cells, h)['verdict']}")
    if rp["at_floor"]:
        print(f"     BUT BOTH ARMS ARE AT THE FLOOR ({EXPLORE_FLOOR}) HERE, so this"
              "\n     verdict is arithmetic, not evidence: there is no unknown volume"
              "\n     left for exploitation to have denied the explorer. Read the cost"
              "\n     off the horizons above that are NOT marked AT FLOOR.")
    live = [h for h in HORIZON_ORDER if h != "end"
            and not h2_cost(cells, h)["at_floor"]]
    if live and rp["on_mean"] is not None and rp["off_mean"] is not None:
        print(f"     horizons that can still show a cost: {', '.join(live)}")
    print(f"     H2 is decided on t={', '.join(str(h) for h in H2_DECIDING_H)}s "
          "(§6.6). The horizons were widened past"
          "\n     1200s because the ~1500s saturation that justified truncating"
          "\n     them was the 3D artifact: on the 2D map the same cells are still"
          "\n     gaining coverage out to ~2800s, so those horizons DO carry"
          "\n     information about a cost and are no longer discarded.")
    if cal is not None:
        print(f"\n  calibrated budget (SECONDARY, §6.5): {fmt(cal)} -- the smallest"
              "\n     difference this design can tell from zero, from the exploit-off"
              f"\n     arm alone. At t={H2_HEADLINE_H}s -> "
              f"{rp['calibrated_verdict']}."
              + ("\n     This is a NOISE FLOOR, not the budget: it is what n=5 an arm"
                 "\n     can resolve, and adopting it as the budget would be choosing"
                 "\n     a threshold to fit the noise. It bounds the re-derivation."
                 if H2_UNK_BUDGET is None else
                 f"\n     The plan's {H2_UNK_BUDGET} stays primary and is reported above"
                 "\n     whatever it returns; this number is reported beside it, never"
                 "\n     in place of it."))
    print("     The verdict is the interval against the budget, not the p value"
          "\n     (§6.4): a non-significant difference at n=5 an arm is evidence"
          "\n     of five cells, not of a small cost.")

    d = h2_delay(cells)
    print()
    print(f"  completion time (derived, §6.4)  on {fmt(d['on_mean'], 0)} "
          f"off {fmt(d['off_mean'], 0)}  delta {fmt(d['delta'], 0)} s")
    print(f"     95% CI on delta {ci(d['delta_ci'])}  vs budget "
          f"{H2_DELAY_BUDGET:.0f} s -> {d['verdict']}")
    if d["no_completion_on"] or d["no_completion_off"]:
        print(f"     never met the threshold within T: "
              f"{d['no_completion_on']} on, {d['no_completion_off']} off "
              f"(counted, not averaged)")

    m4 = m4_trigger(cells, metric)
    print()
    print(f"  M4 (GT surface recall) required? "
          f"{'YES' if m4['required'] else 'no'}")
    for why in m4["reasons"]:
        print(f"     - {why}")
    if not m4["required"]:
        print(f"     neither §6.4 trigger fired: {metric} delta "
              f"{fmt(m4['primary_delta'])}, M2 delta {fmt(m4['m2_delta'])}")

    print()
    print("=" * 78)
    print(f"H3 DIFFERENCE-IN-DIFFERENCES -- {metric}, targets minus near controls")
    print("=" * 78)
    d = h3_did(cells, PRIMARY_H, metric)
    print(f"  exploit-on  target-control gap {fmt(d['on_gap'])}  95% CI {ci(d['on_gap_ci'])}")
    print(f"  exploit-off target-control gap {fmt(d['off_gap'])}  95% CI {ci(d['off_gap_ci'])}")
    print(f"  difference-in-differences      {fmt(d['did'])}")
    if d["p"] is not None:
        print(f"  one-sided permutation p = {d['p']:.4f} "
              f"(floor {d['p_floor']:.4f})")

    print()
    print("=" * 78)
    print(f"PER-TRUNK TABLE -- {metric} @ t={PRIMARY_H}s (§6.3)")
    print("=" * 78)
    names = set()
    for c in cells:
        names |= set(trunks_at(c, PRIMARY_H).keys())
    def key(n):
        tr = None
        for c in cells:
            v = trunks_at(c, PRIMARY_H).get(n)
            if v:
                tr = v.get("target_id")
                break
        return (0 if tr else 1, tr or 0, n)
    hdr = "  " + f"{'trunk':<16}" + "".join(f"{c['name'][:9]:>10}" for c in cells)
    print(hdr)
    for n in sorted(names, key=key):
        row = f"  {n:<16}"
        for c in cells:
            v = trunks_at(c, PRIMARY_H).get(n)
            row += f"{fmt(v[metric]) if v and metric in v else '-':>10}"
        tid = trunks_at(cells[0], PRIMARY_H).get(n, {}).get("target_id")
        print(row + ("   <- target" if tid else ""))

    if args.json:
        payload = {
            "cells": [{"name": c["name"], "arm": c["arm"], "include": c.get("include"),
                       "manipulation": manipulation_check(c)} for c in cells],
            "gates": {"O1": gate_O1_all(cells), "P1": gate_P1(cells),
                      "P2": p2, "P3": p3},
            "primary_metric": metric,
            "H1": endpoint(cells, PRIMARY_H, metric),
            "curve": [endpoint(cells, h, metric) for h in HORIZON_ORDER],
            "H2": [h2_cost(cells, h,
                            calibrated=calibrated_budget(
                                gate_P3(cells)["sd_unknown_1200"]))
                   for h in HORIZON_ORDER if h != "end"],
            "H2_deciding_horizons": list(H2_DECIDING_H),
            "H2_calibrated_budget": calibrated_budget(
                gate_P3(cells)["sd_unknown_1200"]),
            "H2_delay": h2_delay(cells),
            "M4_trigger": m4_trigger(cells, metric),
            "H3": h3_did(cells, PRIMARY_H, metric),
        }
        with open(args.json, "w") as f:
            json.dump(payload, f, indent=1, default=str)
        print(f"\nwrote {args.json}")


def gate_O1_all(cells):
    out = []
    for c in cells:
        r = gate_O1(c)
        if r:
            r["cell"] = c["name"]
            out.append(r)
    return out


if __name__ == "__main__":
    main()
