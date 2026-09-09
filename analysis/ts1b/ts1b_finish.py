#!/usr/bin/env python3
"""Finish-time tables and arm contrasts for the ts1b team-size series.

WHAT THIS IS
    The reproducible form of the finish-time numbers quoted in
    ../../TS1B_TEAM_SIZE_RESULTS.md. It reads cell directories directly, so it
    works mid-campaign and does not depend on a rebuilt ts1b_cells.csv.

    Stdlib only. No numpy, no scipy -- the box runs campaigns and analysis at
    the same time and this must stay cheap.

THE FOUR RULES IT ENCODES
    1. LAST-event endpoints. Both `exploration_complete` and `mission_complete`
       are taken as the LAST such event per robot, then maxed over robots. Nine
       robots in the base series had two homing episodes; a first-event reader
       records a false censoring for each. Estimator changes have already nearly
       cost this project a real result once (see the separation mediator in
       CAMPAIGN_FINDINGS R1) -- do not swap the rule to "match" another script.

    2. Makespan is a max over robots, so it gains a draw at every rung and must
       rise under a pure team-size null. `per_robot` is therefore reported
       beside it everywhere. Quoting the makespan alone understates the
       team-size effect; quoting per_robot alone ignores that a team is only
       done when its last robot is.

    3. Pooling across rungs is only fair if every arm carries the same rung
       composition. `check_balance` enforces that and refuses rather than
       quietly producing a rung-weighted arm contrast. The same rule binds the
       spread test and the base-vs-top-up session check, which pool too.

    4. Missing endpoints are dropped loudly, never carried. A cell whose
       t_mission is NaN would otherwise walk into perm_test, make every
       `>= obs` comparison False, and come back p = 0.000 -- a maximally
       significant result manufactured out of missing data.

WHAT IT WILL NOT DO
    It never writes ts1b_cells.csv. That file is the frozen 120-cell base
    artifact the results document cites; tools/ts1b_cells.py owns it. This
    script is read-only against the campaign tree.

USAGE
    ts1b_finish.py --selftest              # run this first, always
    ts1b_finish.py table                   # arm x rung, all three endpoints
    ts1b_finish.py contrasts               # each arm vs off, pooled + per rung
    ts1b_finish.py session                 # base half vs top-up half
    ts1b_finish.py table --csv out.csv     # same numbers, machine-readable

    --root      campaign root (default ~/hmr_campaign)
    --seeds     base | topup | all         (default all)
    --endpoint  t_explore | t_mission | per_robot   (contrasts/session only)

EXIT CODES
    0  report printed, nothing suppressed
    2  report printed, but something was refused or suppressed (unbalanced
       pooling, a mismatched session mixture, dropped NaN endpoints)
    1  no data, or --selftest failed
"""
import argparse
import csv
import glob
import itertools
import json
import math
import os
import random
import statistics as st
import sys

sys.dont_write_bytecode = True

ARMS = ("off", "pursuit", "rendezvous", "hybrid")   # off first: it is the reference
RUNGS = ("n2", "n3", "n4")
ENDPOINTS = ("t_explore", "t_mission", "per_robot")
DEFAULT_ROOT = os.path.expanduser("~/hmr_campaign")
BASE_SEED_MAX = 10          # seeds 1-10 are the preregistered base series

# Manifest fields that must not vary across pooled cells. This is the
# comparability group key from ~/hmr_campaign/build_index.py minus `scenario`:
# in a team-size series the scenario file IS the rung (flatforest_dense_2robot
# vs _3robot vs _4robot), so checking it across rungs would fire on every run.
# Scenario is checked WITHIN each rung instead.
KEY_FIELDS = ("git_explo_planner", "tx_power_dbm", "tree_attenuation_db",
              "max_range_m", "done_criterion", "duration_s")


# --------------------------------------------------------------------------
# reading cells
# --------------------------------------------------------------------------

def resolve_endpoints(cell_dir):
    """LAST exploration_complete / LAST mission_complete per robot, max over robots.

    Returns None for a cell with no exploration_complete at all (never finished
    exploring, so it has no finish time to report).

    The string prefilter is a speed hack; the `event` field is the check. It is
    deliberately NOT an if/elif chain: a mission_complete record that happened
    to carry the string "exploration_complete" anywhere in it (a future
    `"reason": "exploration_complete"`, say) would be swallowed by the first
    branch and the mission event lost -- silently, since a robot with no
    mission_complete is simply absent from the max. No such collision exists in
    today's logs; the shape is the hazard.
    """
    latch, mission, censored = [], [], False
    n_files = 0
    for f in sorted(glob.glob(os.path.join(cell_dir, "*.events.jsonl"))):
        n_files += 1
        last_explore = last_mission = None
        with open(f) as fh:
            for line in fh:
                if ('"exploration_complete"' not in line
                        and '"mission_complete"' not in line):
                    continue
                e = json.loads(line)
                ev = e.get("event")
                if ev == "exploration_complete":
                    last_explore = e
                elif ev == "mission_complete":
                    last_mission = e
        if last_explore:
            latch.append(last_explore["t_sim_sec"])
        if last_mission:
            mission.append(last_mission["t_sim_sec"])
            if last_mission.get("result") != "arrived":
                censored = True
    if not latch:
        return None
    return {
        "t_explore": max(latch),
        # NaN, not a silent omission, when no robot reported mission_complete:
        # select() drops it and says so. max() over a partial set would be a
        # makespan over a team that never all reported.
        "t_mission": max(mission) if len(mission) == n_files and mission
                     else float("nan"),
        "per_robot": st.fmean(latch),
        "censored": censored,
        "n_robots": len(latch),
        "n_mission": len(mission),
        "n_logs": n_files,
    }


def load_cells(root=DEFAULT_ROOT, seeds="all", verbose=True):
    """Every completed ts1b cell under `root`. Reports what it skipped.

    Note a deliberate divergence from tools/ts1b_cells.py: that script also
    gates on `cell_metrics(d) is not None` and emits a NaN row for a latch-less
    cell, where this one drops it. The selftest's cell-count check is what holds
    the two in agreement on the base series.
    """
    cells, skipped_incomplete, skipped_noevents = [], 0, 0
    keys = {}
    for rung_dir in sorted(glob.glob(os.path.join(root, "ts1b_n*"))):
        for d in sorted(glob.glob(os.path.join(rung_dir, "ts1b_*_seed*"))):
            if not os.path.isdir(d):
                continue
            man_path = os.path.join(d, "run_manifest.txt")
            if not os.path.exists(man_path):
                skipped_incomplete += 1
                continue
            with open(man_path) as fh:
                man = dict(l.rstrip("\n").split("=", 1) for l in fh if "=" in l)
            # a cell still in flight, or one that hit the duration cap, is not a
            # finish time -- excluding it beats truncating it
            if man.get("run_end_reason") != "all_done":
                skipped_incomplete += 1
                continue
            name = os.path.basename(d)
            parts = name.split("_")
            seed = int(parts[-1].replace("seed", ""))
            half = "base" if seed <= BASE_SEED_MAX else "topup"
            if seeds == "base" and half != "base":
                continue
            if seeds == "topup" and half != "topup":
                continue
            ep = resolve_endpoints(d)
            if ep is None:
                skipped_noevents += 1
                continue
            ep.update(cell=name, rung=parts[1], arm=parts[3], seed=seed,
                      half=half, t_end=float(man["run_end_t_sim"]),
                      stamp_explo_planner=man.get("git_explo_planner", "?"),
                      stamp_hmr_explo=man.get("git_hmr_explo", "?"))
            cells.append(ep)
            keys.setdefault(ep["rung"], set()).add(
                tuple(man.get(k, "?") for k in KEY_FIELDS)
                + (man.get("scenario", "?"),))
    if verbose:
        print(f"# {len(cells)} cells loaded from {root} (seeds={seeds})")
        if skipped_incomplete:
            print(f"#   skipped {skipped_incomplete} not-all_done / in-flight")
        if skipped_noevents:
            print(f"#   skipped {skipped_noevents} with no exploration_complete")
        _warn_comparability(cells, keys)
    return cells


def _warn_comparability(cells, keys):
    """Warn if the loaded cells span more than one comparability block."""
    planner = sorted({c["stamp_explo_planner"] for c in cells})
    if len(planner) > 1:
        print(f"#   WARNING: {len(planner)} explo_planner revisions present "
              f"{planner} -- these are different binary generations and "
              f"MUST NOT be pooled")
    # scenario is rung-bound by design, so compare it within a rung only
    for rung, ks in sorted(keys.items()):
        if len(ks) > 1:
            print(f"#   WARNING: {rung} spans {len(ks)} comparability keys "
                  f"(git/radio/scenario/done/duration) -- not poolable")
    cross = {k[:len(KEY_FIELDS)] for ks in keys.values() for k in ks}
    if len(cross) > 1:
        for i, f in enumerate(KEY_FIELDS):
            vals = sorted({k[i] for k in cross})
            if len(vals) > 1:
                print(f"#   WARNING: {f} varies across the loaded cells: {vals}")


def select(cells, arm=None, rung=None, half=None, endpoint="t_explore"):
    """Endpoint values for matching cells, with non-finite values DROPPED.

    Dropping is not tidying. A single NaN makes perm_test's `obs` NaN, every
    `>= obs - 1e-9` comparison False, and the exact branch return 0/total =
    p 0.000. Missing data must not be able to manufacture significance, so it
    leaves the sample -- and nan_count() below exists so it never leaves quietly.
    """
    return [c[endpoint] for c in _match(cells, arm, rung, half)
            if isinstance(c[endpoint], (int, float)) and math.isfinite(c[endpoint])]


def _match(cells, arm=None, rung=None, half=None):
    return [c for c in cells
            if (arm is None or c["arm"] == arm)
            and (rung is None or c["rung"] == rung)
            and (half is None or c["half"] == half)]


def nan_count(cells, endpoint):
    """How many cells select() would drop for this endpoint."""
    return sum(1 for c in cells
               if not (isinstance(c[endpoint], (int, float))
                       and math.isfinite(c[endpoint])))


# --------------------------------------------------------------------------
# statistics
# --------------------------------------------------------------------------

def perm_test(a, b, iters=200000, seed=0, exact_limit=200000):
    """Two-sided permutation test on the difference in means.

    Exact when the number of splits is <= exact_limit, Monte Carlo otherwise.
    The threshold is its own parameter rather than a reuse of `iters` so the
    branch can be forced either way from a test -- when the two were the same
    knob, the selftest case meant to exercise the Monte Carlo path silently ran
    the exact path instead and still reported PASS.

    The two branches estimate the same quantity. The exact enumeration includes
    the observed split, which always hits, so p >= 1/C(n,na) with no correction;
    the Monte Carlo branch's (hits+1)/(iters+1) adds the observed arrangement
    back as one permutation. The asymmetry is the point, not an oversight.

    Never a bootstrap: the exact permutation test is this project's standard,
    and a hand-rolled sampler has produced a biased p here before, which is why
    selftest() re-derives known answers before any of this is trusted.

    Returns (p, how). Raises on non-finite input rather than returning p = 0.
    """
    a, b = list(a), list(b)
    if not all(math.isfinite(x) for x in a + b):
        raise ValueError("perm_test got a non-finite value; a NaN here returns "
                         "p = 0.000 from the exact branch. Drop it upstream.")
    obs = abs(st.fmean(a) - st.fmean(b))
    pool = a + b
    na, n = len(a), len(a) + len(b)
    if math.comb(n, na) <= exact_limit:
        hits = 0
        total = 0
        for idx in itertools.combinations(range(n), na):
            s = set(idx)
            left = [pool[i] for i in idx]
            right = [pool[i] for i in range(n) if i not in s]
            if abs(st.fmean(left) - st.fmean(right)) >= obs - 1e-9:
                hits += 1
            total += 1
        return hits / total, "exact"
    rng = random.Random(seed)          # Mersenne Twister, not a hand-rolled LCG
    hits = 0
    for _ in range(iters):
        rng.shuffle(pool)
        if abs(st.fmean(pool[:na]) - st.fmean(pool[na:])) >= obs - 1e-9:
            hits += 1
    return (hits + 1) / (iters + 1), f"MC {iters}"


def median_devs(v):
    """|x - median(v)|, the Brown-Forsythe transform.

    The median, not the mean: a mean-centred version is Levene's test, which the
    outlier-heavy right tail of a finish-time distribution drags around.
    """
    m = st.median(v)
    return [abs(x - m) for x in v]


def spread_test(a, b, iters=100000, seed=1, exact_limit=200000):
    """Brown-Forsythe permutation test: is the SPREAD different?

    Deviations from each group's OWN median, so a pure location shift between
    the groups cancels. It does NOT cancel location differences *inside* a
    group: if one arm has a strong rung trend and the other does not, pooling
    rungs before this test charges the trend to spread. Callers pool only when
    check_balance passes, and print per-rung rows beside the pooled one.
    """
    return perm_test(median_devs(list(a)), median_devs(list(b)),
                     iters=iters, seed=seed, exact_limit=exact_limit)


def check_balance(cells, verbose=True):
    """Do all arms carry the same rung composition?

    If they do not, a pooled arm mean is partly a rung mean and the arm
    contrast is confounded with team size. Returns True/False; callers refuse
    to pool on False rather than printing a caveat nobody reads.

    Categories come from the data, not from the ARMS/RUNGS constants, so a new
    arm or a future n5 rung fails the check instead of slipping past a loop that
    never looked at it.
    """
    arms = sorted({c["arm"] for c in cells})
    rungs = sorted({c["rung"] for c in cells})
    unknown = ([f"arm={a}" for a in arms if a not in ARMS]
               + [f"rung={r}" for r in rungs if r not in RUNGS])
    if unknown:
        if verbose:
            print(f"# BALANCE FAIL -- unrecognised categories: {', '.join(unknown)}")
            print("# Nothing here knows how to weight them; add them to "
                  "ARMS/RUNGS deliberately or filter them out.")
        return False
    comp = {a: tuple(len(_match(cells, arm=a, rung=r)) for r in rungs)
            for a in arms}
    ok = len(set(comp.values())) == 1
    if verbose and not ok:
        print("# BALANCE FAIL -- rung composition differs by arm:")
        for a in arms:
            print(f"#   {a:<11} {'/'.join(str(x) for x in comp[a])}")
        print("# Pooling across rungs here would confound arm with team size.")
    return ok


def same_mixture(x, y):
    """Are two rung compositions proportional? (x, y are per-rung counts.)"""
    sx, sy = sum(x), sum(y)
    if not sx or not sy:
        return False
    return all(abs(xi * sy - yi * sx) < 1e-9 for xi, yi in zip(x, y))


# --------------------------------------------------------------------------
# commands
# --------------------------------------------------------------------------

def cmd_table(cells, args):
    balanced = check_balance(cells)
    suppressed = not balanced
    out = []
    for endpoint in ENDPOINTS:
        title = {"t_explore": "t_explore -- exploration makespan (last robot done exploring)",
                 "t_mission": "t_mission -- mission makespan (last robot home)",
                 "per_robot": "per_robot -- per-robot latch (cell mean over robots)"}[endpoint]
        print(f"\n########## {title}\n")
        cols = RUNGS + (None,)
        hdr = f"{'arm':<12}" + "".join(
            f"{('N=' + r[1] if r else 'ALL RUNGS'):>22}" for r in cols)
        print(hdr)
        print("-" * len(hdr))
        for a in ARMS:
            line = f"{a:<12}"
            for r in cols:
                v = select(cells, arm=a, rung=r, endpoint=endpoint)
                line += (f"{st.fmean(v):>8.0f} /{st.median(v):>6.0f} ({len(v):>2})"
                         if v else f"{'-':>22}")
                if v:
                    out.append(dict(endpoint=endpoint, arm=a, rung=r or "all",
                                    n=len(v), mean=round(st.fmean(v), 1),
                                    median=round(st.median(v), 1),
                                    sd=round(st.stdev(v), 1) if len(v) > 1 else ""))
            print(line)
        print("-" * len(hdr))
        line = f"{'ALL ARMS':<12}"
        for r in cols:
            v = select(cells, rung=r, endpoint=endpoint)
            line += (f"{st.fmean(v):>8.1f} /{st.median(v):>6.1f}({len(v):>3})"
                     if v else f"{'-':>22}")
        print(line)
        line = f"{'   sd':<12}"
        for r in cols:
            v = select(cells, rung=r, endpoint=endpoint)
            line += f"{st.stdev(v):>14.1f}{'':>8}" if len(v) > 1 else f"{'-':>22}"
        print(line)
        nan = nan_count(cells, endpoint)
        if nan:
            suppressed = True
            print(f"\n   DROPPED {nan} of {len(cells)} cells with no {endpoint} "
                  f"(n above excludes them)")
        if endpoint == "t_mission":
            # censoring is mission_complete.result != "arrived": it bounds the
            # mission makespan and nothing else. t_explore is untouched by it.
            cens = sum(1 for c in cells if c["censored"])
            print(f"\n   censored cells: {cens} of {len(cells)} "
                  f"({100 * cens / len(cells):.1f}%) -- a robot stopped short of "
                  f"home, so it is stamped where it stopped and this column's "
                  f"affected means are LOWER BOUNDS")
    if not balanced:
        print("\n# The ALL RUNGS column is rung-weighted and NOT a clean arm "
              "contrast -- see the balance warning above.")
    if args.csv:
        if not out:
            print("\n# nothing to write", file=sys.stderr)
        else:
            with open(args.csv, "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(out[0].keys()))
                w.writeheader()
                w.writerows(out)
            print(f"\n# wrote {len(out)} rows -> {args.csv}")
    return 2 if suppressed else 0


def cmd_contrasts(cells, args):
    ep = args.endpoint
    ref = "off"
    suppressed = False
    print(f"\n### {ep}: each arm vs {ref}\n")
    pooled_ok = check_balance(cells)
    if not pooled_ok:
        print("# refusing to report a pooled contrast on unbalanced data; "
              "per-rung rows below are still valid.\n")
        suppressed = True
    nan = nan_count(cells, ep)
    if nan:
        suppressed = True
        print(f"# DROPPED {nan} of {len(cells)} cells with no {ep}\n")

    others = [a for a in ARMS if a != ref]
    print(f"{'contrast':<24} {'rung':<6} {'n/n':>8} {'delta':>10} {'%':>8} {'p':>8}  test")
    for a in others:
        for r in RUNGS + ((None,) if pooled_ok else ()):
            x = select(cells, arm=a, rung=r, endpoint=ep)
            y = select(cells, arm=ref, rung=r, endpoint=ep)
            if len(x) < 2 or len(y) < 2:
                continue
            d = st.fmean(x) - st.fmean(y)
            p, how = perm_test(x, y)
            label = f"{ref} vs {a}"
            print(f"{label:<24} {(r or 'POOLED'):<6} {len(y):>3}/{len(x):<4} "
                  f"{d:>+10.1f} {100 * d / st.fmean(y):>+7.1f}% {p:>8.3f}  {how}")
        print()
    if pooled_ok:
        print(f"# Bonferroni over the {len(others)} pooled contrasts: multiply p by "
              f"{len(others)}.")
        print("# The arm axis is preregistered as a SCREEN, not a test. Pooling "
              "across rungs to reach this n is post-hoc; label it as such.")

    print("\n### spread (Brown-Forsythe on deviations from the median)\n")
    print("# Per rung first. A pooled spread charges an arm's between-rung "
          "location\n# trend to its spread, so the pooled row is the weaker "
          "read, not the summary.")
    for a in others:
        for r in RUNGS + ((None,) if pooled_ok else ()):
            x = select(cells, arm=a, rung=r, endpoint=ep)
            y = select(cells, arm=ref, rung=r, endpoint=ep)
            if len(x) < 2 or len(y) < 2:
                continue
            p, how = spread_test(x, y)
            print(f"  {(r or 'POOLED'):<7} {ref} sd {st.stdev(y):>6.1f}  vs  "
                  f"{a:<11} sd {st.stdev(x):>6.1f}   "
                  f"ratio {st.stdev(x) / st.stdev(y):>4.2f}x   p = {p:.3f}  {how}")
        print()
    if not pooled_ok:
        print("# Pooled spread rows omitted for the same reason as the pooled "
              "contrast.")
    return 2 if suppressed else 0


def cmd_session(cells, args):
    """Base half vs top-up half: did the box or the session drift?

    The two halves are separate run_campaign.sh invocations, so arm and session
    are confounded across them by construction. This does not remove that
    confound -- it measures how big it is before the halves are pooled.
    """
    ep = args.endpoint
    suppressed = False
    print(f"\n### session check, {ep}: base (seeds 1-{BASE_SEED_MAX}) vs top-up\n")
    comp = {h: tuple(len(_match(cells, rung=r, half=h)) for r in RUNGS)
            for h in ("base", "topup")}
    mixture_ok = same_mixture(comp["base"], comp["topup"])
    if not mixture_ok:
        suppressed = True
        print(f"# rung mixtures differ: base {'/'.join(map(str, comp['base']))}  "
              f"top-up {'/'.join(map(str, comp['topup']))}")
        print("# An ALL row over these would report a rung-composition "
              "difference as session drift\n# (rung means differ by hundreds of "
              "seconds), so it is omitted. Per-rung rows below are valid.\n")
    nan = nan_count(cells, ep)
    if nan:
        suppressed = True
        print(f"# DROPPED {nan} of {len(cells)} cells with no {ep}\n")
    print(f"{'rung':<6} {'n base':>7} {'n topup':>8} {'delta':>10} "
          f"{'centre p':>10} {'spread p':>10}   sd base / topup")
    for r in RUNGS + ((None,) if mixture_ok else ()):
        b = select(cells, rung=r, half="base", endpoint=ep)
        t = select(cells, rung=r, half="topup", endpoint=ep)
        if len(b) < 2 or len(t) < 2:
            print(f"{(r or 'ALL'):<6} {len(b):>7} {len(t):>8}   (too few to test)")
            continue
        d = st.fmean(t) - st.fmean(b)
        pc, _ = perm_test(t, b)
        ps, _ = spread_test(t, b)
        print(f"{(r or 'ALL'):<6} {len(b):>7} {len(t):>8} {d:>+10.1f} "
              f"{pc:>10.3f} {ps:>10.3f}   {st.stdev(b):>6.1f} / {st.stdev(t):<6.1f}")
    print("\n# A significant centre p means the halves must not be pooled without "
          "adjustment.\n# A significant spread p flatters every contrast computed "
          "on the combined set.")
    return 2 if suppressed else 0


# --------------------------------------------------------------------------
# selftest -- calibrate the machinery against known answers
# --------------------------------------------------------------------------

def selftest(root=DEFAULT_ROOT):
    """A guard that cannot answer PASS unless it actually checked something.

    Every case here has an answer known independently of this file: hand
    arithmetic, a forced cross-check of one branch against the other, and a
    re-derivation of the frozen ts1b_cells.csv. A check whose expected value is
    computed by the code under test is not a check.

    What case 5 cannot do, stated so nobody mistakes it for total coverage: it
    proves conformance to tools/ts1b_cells.py's pipeline on the base series. A
    bug shared with that reference -- the two read events the same way -- is
    invisible to it. Cases 1-4 and 6-7 exist because of that.
    """
    fails = []

    def ck(name, got, want, tol=0.0):
        ok = abs(got - want) <= tol if isinstance(want, float) else got == want
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: got {got}, want {want}")
        if not ok:
            fails.append(name)

    print("1. permutation test against hand arithmetic")
    # C(6,3) = 20 splits. Only the observed split and its mirror reach the
    # observed |diff| of 9, so p must be exactly 2/20.
    p, how = perm_test([1.0, 2.0, 3.0], [10.0, 11.0, 12.0])
    ck("3v3 fully-separated, exact", round(p, 10), 0.1, 1e-9)
    ck("   ...and it took the exact branch", how, "exact")
    # Identical samples: every split ties the observed 0.0 difference, so p = 1.
    p, _ = perm_test([5.0, 6.0], [5.0, 6.0])
    ck("identical samples", round(p, 10), 1.0, 1e-9)

    print("\n2. Monte Carlo branch agrees with the exact branch")
    a = [float(x) for x in (11, 14, 9, 22, 17, 13, 8, 19, 25, 12)]
    b = [float(x) for x in (16, 21, 13, 28, 19, 24, 11, 30, 17, 20)]
    p_exact, how_e = perm_test(a, b, exact_limit=10 ** 9)   # force exact
    p_mc, how_m = perm_test(a, b, iters=200000, seed=7, exact_limit=0)  # force MC
    ck("   exact branch actually used", how_e, "exact")
    ck("   MC branch actually used", how_m.startswith("MC"), True)
    ck("   MC within 0.01 of exact", abs(p_mc - p_exact) < 0.01, True)
    print(f"         exact p = {p_exact:.4f}, MC p = {p_mc:.4f}")

    print("\n3. a NaN cannot manufacture significance")
    # Before the guard, one NaN made obs NaN, every comparison False, and the
    # exact branch returned p = 0.000 on missing data.
    try:
        perm_test([1.0, 2.0, float("nan")], [10.0, 11.0, 12.0])
        ck("perm_test refuses non-finite input", "returned a p", "raised")
    except ValueError:
        ck("perm_test refuses non-finite input", "raised", "raised")
    probe = [dict(arm="off", rung="n2", half="base", censored=False,
                  t_explore=1.0, t_mission=float("nan"), per_robot=1.0),
             dict(arm="off", rung="n2", half="base", censored=False,
                  t_explore=2.0, t_mission=5.0, per_robot=2.0)]
    ck("   select drops the NaN", len(select(probe, endpoint="t_mission")), 1)
    ck("   and nan_count reports it", nan_count(probe, "t_mission"), 1)

    print("\n4. spread test uses the median, not the mean")
    # Hand arithmetic: median([1,2,3,4,100]) = 3, so deviations are
    # [2,1,0,1,97]. A mean-centred (Levene) version would give [21,20,19,18,78].
    ck("median deviations", median_devs([1.0, 2.0, 3.0, 4.0, 100.0]),
       [2.0, 1.0, 0.0, 1.0, 97.0])
    base = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    p, _ = spread_test(base, [x + 100 for x in base])
    ck("   a shifted copy has identical spread", round(p, 10), 1.0, 1e-9)

    print("\n5. balance check refuses an unbalanced or unrecognised set")
    mk = lambda arm, rung, k: [dict(arm=arm, rung=rung, half="base", censored=False,
                                    t_explore=1.0, t_mission=1.0, per_robot=1.0)
                               for _ in range(k)]
    even = sum((mk(a, r, 3) for a in ARMS for r in RUNGS), [])
    odd = sum((mk(a, r, 3 if a != "hybrid" else 1) for a in ARMS for r in RUNGS), [])
    ck("balanced set accepted", check_balance(even, verbose=False), True)
    ck("unbalanced set rejected", check_balance(odd, verbose=False), False)
    ck("stray rung rejected", check_balance(even + mk("off", "n5", 3),
                                            verbose=False), False)
    ck("stray arm rejected", check_balance(even + mk("greedy", "n2", 3),
                                           verbose=False), False)
    ck("proportional mixtures match", same_mixture((20, 15, 10), (4, 3, 2)), True)
    ck("lopsided mixtures do not", same_mixture((40, 40, 40), (40, 21, 0)), False)

    print("\n6. endpoint resolver reproduces the frozen base artifact")
    frozen = os.path.join(root, "ts1_analysis", "ts1b_cells.csv")
    if not os.path.exists(frozen):
        print(f"  [SKIP] {frozen} not present -- cannot verify the resolver")
        fails.append("resolver-unverified")
    else:
        with open(frozen) as fh:
            want = {r["cell"]: r for r in csv.DictReader(fh)}
        got = {c["cell"]: c for c in load_cells(root, seeds="base", verbose=False)}
        ck("   same cell count as the frozen CSV", len(got), len(want))
        bad = []
        for name, w in want.items():
            g = got.get(name)
            if g is None:
                bad.append(f"{name}: missing")
                continue
            for col in ENDPOINTS:
                # NaN must be compared explicitly. `abs(nan - x) > tol` is False,
                # so a tolerance test alone silently passes a resolver that
                # started returning NaN -- the exact regression case 3 guards.
                a_, b_ = g[col], float(w[col])
                if math.isnan(a_) != math.isnan(b_) or (
                        not math.isnan(a_) and abs(a_ - b_) > 1e-6):
                    bad.append(f"{name}.{col}: {a_} vs {b_}")
            wc = w["censored"] not in ("0", "", "False", "false")
            if g["censored"] != wc:
                bad.append(f"{name}.censored: {g['censored']} vs {wc}")
        # 1e-6, not the 0.51 this used to carry: the frozen CSV stores full
        # precision (730.3, 1079.25, 44.705914...), so the only legitimate
        # divergence is fmean's compensated summation, ~1e-12. A 0.51 window
        # would have passed a resolver that truncated every stamp with int().
        ck("   all endpoints match the frozen values to 1e-6", len(bad), 0)
        for line in bad[:10]:
            print(f"         {line}")

    print()
    if fails:
        print(f"SELFTEST FAILED: {', '.join(fails)}")
        return 1
    print("SELFTEST PASSED -- the machinery answered every known case correctly.")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", nargs="?", choices=("table", "contrasts", "session"),
                    help="which report to print")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--seeds", default="all", choices=("base", "topup", "all"))
    ap.add_argument("--endpoint", default="t_explore", choices=ENDPOINTS)
    ap.add_argument("--csv", help="also write the table as CSV (table only)")
    ap.add_argument("--selftest", action="store_true",
                    help="verify the machinery against known answers, then exit")
    args = ap.parse_args()

    if args.selftest:
        return selftest(args.root)
    if not args.command:
        ap.error("give a command (table | contrasts | session) or --selftest")
    if args.command == "session" and args.seeds != "all":
        ap.error("session compares the base and top-up halves, so it needs "
                 "--seeds all (the default)")
    if args.csv and args.command != "table":
        ap.error("--csv is only implemented for `table`")

    cells = load_cells(args.root, seeds=args.seeds)
    if not cells:
        print("no cells found", file=sys.stderr)
        return 1
    return {"table": cmd_table, "contrasts": cmd_contrasts,
            "session": cmd_session}[args.command](cells, args)


if __name__ == "__main__":
    sys.exit(main())
