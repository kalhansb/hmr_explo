#!/usr/bin/env python3
"""Self-test for aggregate.py's arithmetic.

The campaign's conclusion is a permutation p-value and a pair of confidence
intervals over ten numbers. That arithmetic has no natural error signal: a
wrong test statistic, an off-by-one in the tail count or a silently dropped
cell all produce a plausible number rather than a crash. These check the cases
where the right answer is known independently.

Run: python3 test_aggregate.py
"""

import sys

import aggregate as A

FAILURES = []


def check(name, got, want, tol=1e-9):
    ok = (got is None and want is None) or (
        got is not None and want is not None and abs(got - want) <= tol)
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: got {got!r}, want {want!r}")
    if not ok:
        FAILURES.append(name)


def check_true(name, cond, detail=""):
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILURES.append(name)


print("permutation test")
# Perfect separation: the observed split is the single most extreme of the 252,
# so exactly one labeling is at least as extreme -- itself. That is the floor,
# and an effect cannot beat it however large it is.
p, floor = A.permutation_p([10, 11, 12, 13, 14], [1, 2, 3, 4, 5])
check("separated arms p", p, 1 / 252, 1e-12)
check("floor is 1/252", floor, 1 / 252, 1e-12)

# Identical arms: the observed difference is 0, and by symmetry half the
# labelings are >= 0. Anything far from 0.5 here means the tail is miscounted.
p, _ = A.permutation_p([1, 2, 3, 4, 5], [1, 2, 3, 4, 5])
check_true("identical arms p ~ 0.5", 0.45 <= p <= 0.65, f"(p={p:.4f})")

# Direction matters: the same data tested the other way must be the complement
# of the strict tail, never the same number.
hi, lo = [5, 6, 7], [1, 2, 3]
p_up, _ = A.permutation_p(hi, lo, greater=True)
p_dn, _ = A.permutation_p(hi, lo, greater=False)
check_true("one-sided directions differ", p_up < 0.5 < p_dn,
           f"(up={p_up:.3f}, down={p_dn:.3f})")
check("empty arm returns None", A.permutation_p([], [1, 2])[0], None)

print("bootstrap CI")
xs = [0.40, 0.42, 0.45, 0.47, 0.50]
lo_, hi_ = A.bootstrap_ci(xs)
check_true("CI brackets the mean", lo_ < sum(xs) / len(xs) < hi_,
           f"({lo_:.4f}, {hi_:.4f})")
check_true("CI is narrower than the range", (hi_ - lo_) < (max(xs) - min(xs)),
           f"(width {hi_ - lo_:.4f} vs range {max(xs) - min(xs):.4f})")
check_true("CI is deterministic", A.bootstrap_ci(xs) == (lo_, hi_))
check("CI of a single value is undefined", A.bootstrap_ci([0.4])[0], None)

print("sample SD")
# n-1. On [2,4,4,4,5,5,7,9] the population SD is 2.0 and the sample SD is
# 2.13809; returning 2.0 would mean the n divisor slipped back in.
check("sample SD (n-1)", A.sd([2, 4, 4, 4, 5, 5, 7, 9]), 2.13808993, 1e-6)
check("SD of one value is undefined", A.sd([1.0]), None)

print("completion time")
rows = [{"sim_time_sec": str(t), "unknown_fraction": str(u)} for t, u in
        [(100, 0.90), (200, 0.70), (300, 0.63), (400, 0.65), (500, 0.62),
         (600, 0.61), (700, 0.60), (800, 0.59)]]
# 300 dips below but 400 comes back up, so the streak restarts; the first time
# three consecutive steps are all at or below the threshold ends at 700.
check("needs 3 consecutive steps", A.completion_time(rows, 0.64), 700.0)
check("never crossing returns None", A.completion_time(rows, 0.10), None)
check("single step suffices at consec=1", A.completion_time(rows, 0.64, consec=1), 300.0)

print("series_at")
# t=450 falls between the rows at 400 and 500, so the answer is the row at
# 400 (0.65) -- deliberately a value that went back UP, since a reading that
# quietly returned the running minimum would look right on a monotone series.
check("last value at or before t", A.series_at(rows, 450.0, "unknown_fraction"), 0.65)
check("exact hit", A.series_at(rows, 500.0, "unknown_fraction"), 0.62)
# The guard that stops a dead run contributing to later horizons.
check("past the end of the run is None",
      A.series_at(rows, 2400.0, "unknown_fraction"), None)
check("before the first row is None",
      A.series_at(rows, 50.0, "unknown_fraction"), None)

print("cell-level means and gates")


def cell(name, arm, targets, controls, include=True, m2=None, planner=None):
    """A cell carrying only what the readouts read.

    `m2` gives the targets a second metric that can move independently of M1,
    which is what the M4 trigger is defined on; left None it tracks M1, so the
    two can never disagree and the trigger stays quiet.
    """
    trunks = {}
    for i, v in enumerate(targets, start=1):
        trunks[f"Target_{i}"] = {"M1": v, "M3": v,
                                 "M2": v if m2 is None else m2,
                                 "target_id": i,
                                 "excluded_control": False}
    for i, v in enumerate(controls):
        trunks[f"Ctrl_{i}"] = {"M1": v, "M3": v, "target_id": None,
                               "excluded_control": False}
    # An excluded control must never reach an average, whatever its value.
    trunks["Oak tree_19"] = {"M1": 99.0, "M3": 99.0, "target_id": None,
                             "excluded_control": True}
    return {"name": name, "arm": arm, "include": include,
            "planner": planner or {}, "manifest": {}, "run_end_reason": None,
            "score": {"horizons": {A.PRIMARY_H: {"trunks": trunks}}}}


c = cell("t", "off", [0.2, 0.4, 0.6], [0.1, 0.3])
check("target mean", A.target_means(c, A.PRIMARY_H, "M1"), 0.4, 1e-9)
check("control mean ignores the excluded trunk",
      A.control_means(c, A.PRIMARY_H, "M1"), 0.2, 1e-9)
# A cell missing a target is not scored on a partial average.
partial = cell("p", "off", [0.2, 0.4], [0.1])
check("a missing target makes the cell mean None",
      A.target_means(partial, A.PRIMARY_H, "M1"), None)

cells = ([cell(f"on{i}", "on", [0.9, 0.9, 0.9], [0.5, 0.5]) for i in range(5)] +
         [cell(f"off{i}", "off", [0.4, 0.4, 0.4], [0.5, 0.5]) for i in range(5)])
e = A.endpoint(cells, A.PRIMARY_H, "M1")
check("endpoint delta", e["delta"], 0.5, 1e-9)
check("endpoint p at the floor", e["p"], 1 / 252, 1e-12)

# H3: targets sit 0.4 above controls in the treated arm and 0.1 below them in
# the untreated one, so the difference-in-differences is 0.5. Here it coincides
# with the raw target difference because the controls are identical across arms;
# the arm-specific gaps above are what distinguish the two quantities.
d = A.h3_did(cells, A.PRIMARY_H, "M1")
check("H3 on-arm gap", d["on_gap"], 0.4, 1e-9)
check("H3 off-arm gap", d["off_gap"], -0.1, 1e-9)
check("H3 difference-in-differences", d["did"], 0.5, 1e-9)

# The case that actually distinguishes a difference-in-differences from a raw
# arm difference: let the controls move between arms too. Targets differ by
# 0.5 (0.9 vs 0.4) while controls differ by 0.2 (0.7 vs 0.5), so the excess
# attributable to being targeted is 0.3. A DiD that forgot to subtract the
# controls would report 0.5 here and pass the previous case unnoticed.
cells_c = ([cell(f"on{i}", "on", [0.9, 0.9, 0.9], [0.7, 0.7]) for i in range(5)] +
           [cell(f"off{i}", "off", [0.4, 0.4, 0.4], [0.5, 0.5]) for i in range(5)])
dc = A.h3_did(cells_c, A.PRIMARY_H, "M1")
check("H3 subtracts the control movement", dc["did"], 0.3, 1e-9)
check("raw target delta differs from the DiD",
      A.endpoint(cells_c, A.PRIMARY_H, "M1")["delta"], 0.5, 1e-9)

# An excluded cell is counted in the record but never averaged.
cells2 = list(cells)
cells2.append(cell("on_bad", "on", [0.0, 0.0, 0.0], [0.5, 0.5], include=False))
e2 = A.endpoint(cells2, A.PRIMARY_H, "M1")
check("excluded cell stays out of the mean", e2["on_mean"], 0.9, 1e-9)
check("excluded cell stays out of n", float(e2["n_on"]), 5.0)

print("gates abstain without data")
p2 = A.gate_P2([])
check_true("P2 undecided on no data", p2["pass"] is None and p2["primary"] == "M1",
           f"({p2.get('note', '')})")
p3 = A.gate_P3([])
check_true("P3 undecided on no data", p3["pass"] is None)
# One cell cannot produce an SD, and must not report a passing floor of 0.
p3one = A.gate_P3([cell("off0", "off", [0.4, 0.4, 0.4], [0.5, 0.5])])
check_true("P3 undecided on one cell", p3one["pass"] is None)
p2sat = A.gate_P2([cell(f"off{i}", "off", [0.95, 0.95, 0.95], [0.5, 0.5])
                   for i in range(3)])
check_true("P2 switches to M3 when M1 saturates",
           p2sat["pass"] is False and p2sat["primary"] == "M3")

print()
if FAILURES:
    print(f"SELF-TEST FAIL ({len(FAILURES)}): " + ", ".join(FAILURES))
    sys.exit(1)

# --- §6.4: a cost is decided on the interval, not on a p value -------------

check_true("budget accepted only when the whole interval is under it",
           A.budget_verdict(0.001, 0.02, 0.03).startswith("ACCEPT"))
check_true("budget rejected only when the whole interval is over it",
           A.budget_verdict(0.04, 0.09, 0.03).startswith("REJECT"))
# The case the p-value rule got wrong: a delta that is small on average but
# whose interval reaches well past the budget is NOT a pass.
check_true("a straddling interval is undecided, not a pass",
           A.budget_verdict(-0.01, 0.20, 0.03).startswith("UNDECIDED"))
check_true("no interval is undecided, never a pass",
           A.budget_verdict(None, None, 0.03).startswith("UNDECIDED"))

# The difference CI resamples each arm within itself. With both arms constant
# there is nothing to resample, so the interval collapses onto the difference.
lo, hi = A.bootstrap_ci_diff([0.5] * 5, [0.2] * 5)
check("constant arms give a point interval at the delta", lo, 0.3, 1e-9)
check("constant arms give a point interval at the delta (hi)", hi, 0.3, 1e-9)


def prows(vals):
    """One robot's per-step CSV: (sim_time_sec, unknown_fraction) pairs."""
    return {"r": [{"sim_time_sec": str(t), "unknown_fraction": str(u)}
                  for t, u in vals]}


# Crosses 0.64 for three consecutive steps starting at t=300.
crossed = [(100, 0.90), (200, 0.70), (300, 0.60), (400, 0.60), (500, 0.60)]
# Never crosses: this cell has no completion time and must not be averaged.
never = [(100, 0.90), (200, 0.85), (300, 0.80), (400, 0.80), (500, 0.80)]
late = [(100, 0.90), (200, 0.90), (300, 0.90), (400, 0.60), (500, 0.60),
        (600, 0.60)]
cells_d = ([cell(f"on{i}", "on", [0.9] * 3, [0.5] * 2, planner=prows(late))
            for i in range(4)] +
           [cell("on4", "on", [0.9] * 3, [0.5] * 2, planner=prows(never))] +
           [cell(f"off{i}", "off", [0.4] * 3, [0.5] * 2, planner=prows(crossed))
            for i in range(5)])
for c in cells_d:
    c["manifest"] = {"done_unknown_fraction": "0.64"}
d = A.h2_delay(cells_d)
check("delay averages only the cells that crossed", d["on_mean"], 600.0, 1e-9)
check("delay control arm", d["off_mean"], 500.0, 1e-9)
check("delay delta", d["delta"], 100.0, 1e-9)
check("a cell that never crossed is counted, not averaged",
      d["no_completion_on"], 1)
check("and it is excluded from n, not silently zero", d["n_on"], 4)
check_true("100 s of delay is inside the 900 s budget",
           d["verdict"].startswith("ACCEPT"), d["verdict"])


# --- §6.4: M4 is promoted on a stated condition ----------------------------

# Agreeing metrics: no trigger, however large the effect.
agree = ([cell(f"on{i}", "on", [0.7] * 3, [0.5] * 2) for i in range(5)] +
         [cell(f"off{i}", "off", [0.3] * 3, [0.5] * 2) for i in range(5)])
check_true("agreeing metrics do not promote M4",
           not A.m4_trigger(agree, "M1")["required"])

# M1 up, M2 down, both well past the instrument's 0.083: that is the
# disagreement the trigger exists for.
disagree = ([cell(f"on{i}", "on", [0.7] * 3, [0.5] * 2, m2=0.2) for i in range(5)] +
            [cell(f"off{i}", "off", [0.3] * 3, [0.5] * 2, m2=0.6) for i in range(5)])
check_true("opposite signs promote M4",
           A.m4_trigger(disagree, "M1")["required"])

# The same opposite signs, but both differences inside the instrument error.
# A sign flip below the noise floor is not a disagreement, it is noise.
tiny = ([cell(f"on{i}", "on", [0.50] * 3, [0.5] * 2, m2=0.50) for i in range(5)] +
        [cell(f"off{i}", "off", [0.47] * 3, [0.5] * 2, m2=0.53) for i in range(5)])
check_true("a sign flip smaller than the instrument error does not promote M4",
           not A.m4_trigger(tiny, "M1")["required"])

# Saturation in BOTH arms: M1 cannot express a difference that may still exist.
sat = ([cell(f"on{i}", "on", [0.99] * 3, [0.5] * 2) for i in range(5)] +
       [cell(f"off{i}", "off", [0.95] * 3, [0.5] * 2) for i in range(5)])
check_true("saturation in both arms promotes M4",
           A.m4_trigger(sat, "M1")["required"])
# One arm saturated is the ordinary large-effect case, not a promotion.
one_sat = ([cell(f"on{i}", "on", [0.99] * 3, [0.5] * 2) for i in range(5)] +
           [cell(f"off{i}", "off", [0.40] * 3, [0.5] * 2) for i in range(5)])
check_true("one saturated arm is a result, not a reason to promote M4",
           not A.m4_trigger(one_sat, "M1")["required"])


# --- §3: one variable, enforced on the manifest ----------------------------

def mcell(name, arm, **man):
    c = cell(name, arm, [0.5] * 3, [0.5] * 2)
    c["manifest"] = dict({"done_unknown_fraction": "0.0", "duration_s": "3600",
                          "comms": "0"}, **man)
    return c


ref = mcell("off_rep1", "off")
twin = mcell("off_rep2", "off")
# The treatment itself is the one line a cell is allowed to differ on.
treated = mcell("on_rep1", "on")
treated["manifest"]["exploitation_enabled"] = "true"
# A shakeout at the runner's stock threshold: a valid manifest, the right arm,
# and a second variable. This is the cell the check exists to catch.
stray = mcell("shakeout", "off", done_unknown_fraction="0.64", duration_s="300")

r = {x["cell"]: x for x in A.conformance([ref, twin, treated, stray], ref)}
check_true("an identical cell conforms", r["off_rep2"]["conforms"])
check_true("the treatment line alone does not break conformance",
           r["on_rep1"]["conforms"])
check_true("a cell run at a different threshold does not conform",
           not r["shakeout"]["conforms"])
check_true("and the readout names the line it differs on",
           any(k == "done_unknown_fraction" for k, _, _ in r["shakeout"]["diffs"]),
           str(r["shakeout"]["diffs"]))
# Run-end keys are absent while a cell is still running; that is not a
# configuration difference and must not exclude the live cell.
live = mcell("off_rep3", "off")
del live["manifest"]["comms"]
live["manifest"]["comms"] = "0"
live2 = mcell("off_rep4", "off")
live2["manifest"]["finished_utc"] = "2026-09-21T05:08:44Z"
r2 = {x["cell"]: x for x in A.conformance([ref, live, live2], ref)}
check_true("a timestamp does not break conformance", r2["off_rep4"]["conforms"])


# --- a floor-vs-floor comparison is not evidence of no cost ----------------

def fcell(name, arm, unk):
    c = cell(name, arm, [0.5] * 3, [0.5] * 2,
             planner={"r": [{"sim_time_sec": "1500", "unknown_fraction": str(unk)},
                            {"sim_time_sec": "1600", "unknown_fraction": str(unk)}]})
    c["manifest"] = {"done_unknown_fraction": "0.0"}
    return c


# Both arms pinned at the world's floor: the delta is zero and inside the
# budget, but only because there is nothing left to lose. It must be flagged.
at_floor = ([fcell(f"on{i}", "on", 0.502) for i in range(5)] +
            [fcell(f"off{i}", "off", 0.501) for i in range(5)])
r = A.h2_cost(at_floor, A.PRIMARY_H)
check_true("both arms at the floor is flagged", r["at_floor"])
check_true("...even though the budget verdict itself says accept",
           r["verdict"].startswith("ACCEPT"), r["verdict"])

# The same arms well above the floor: a real comparison, not flagged.
live = ([fcell(f"on{i}", "on", 0.65) for i in range(5)] +
        [fcell(f"off{i}", "off", 0.64) for i in range(5)])
check_true("a horizon above the floor is not flagged",
           not A.h2_cost(live, A.PRIMARY_H)["at_floor"])

# One arm at the floor and one above it is a real difference, and the worse
# arm is what decides: there IS coverage left that the treatment could explain.
mixed = ([fcell(f"on{i}", "on", 0.60) for i in range(5)] +
         [fcell(f"off{i}", "off", 0.50) for i in range(5)])
rm = A.h2_cost(mixed, A.PRIMARY_H)
check_true("one arm above the floor still counts as a live horizon",
           not rm["at_floor"])
check_true("and a 0.10 cost against a 0.03 budget is rejected",
           rm["verdict"].startswith("REJECT"), rm["verdict"])


# --- can the H2 budget be told from zero at this n? ------------------------

# Noise well under the budget: a 5v5 difference can resolve it.
r = A.resolvable(0.010, 0.03)
check_true("small noise makes the budget resolvable", r["resolvable"])
# The pilot's own number: SD ~0.025 on unknown@1200 puts the CI half-width at
# ~0.031, just past a 0.03 budget. This is the case that motivated the check.
r = A.resolvable(0.025, 0.03)
check("CI half-width at n=5", r["ci_half_width"], 1.96 * 0.025 * (2 / 5) ** 0.5, 1e-12)
check_true("noise at the budget's scale makes it unresolvable",
           not r["resolvable"])
check("no SD gives no verdict", A.resolvable(None, 0.03), None)

print("SELF-TEST PASS")
