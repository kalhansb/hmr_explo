#!/usr/bin/env python3
"""Re-derive H2's unknown-fraction budget and the exploration floor, on the 2D
coverage measure, from the exploit-off arm alone.

The rule is the plan's own, unchanged (§5.4, carried forward by §6.6): the
budget is HALF THE HEADROOM between the untreated arm's unknown fraction at the
headline horizon and the floor the world allows. On the retired 3D scale that
read half of 0.55 - 0.4922, giving 0.03. Only the two inputs move to the 2D
scale; the rule does not, because a rule chosen after seeing the numbers it
will be applied to is not a pre-registered rule.

Both inputs come from the EXPLOIT-OFF cells only, and this is run BEFORE any
treated cell is scored. That ordering is the whole point of pre-registration
and it is the reason this script refuses to run if an exploit-on cell has
already been scored.

The floor is read as the arm's best achieved coverage rather than assumed. On
the 2D map it is ROI geometry, not sensing: the residual unknown is the square
ROI's own perimeter and corners, which no amount of exploring removes.
"""
import csv, math, os, statistics, sys

HEADLINE_H = 1800.0
# aggregate.py's H2_DECIDING_H, as floats. H2's verdict is taken at all of
# these, so the budget has to be reported against the noise at all of them.
DECIDING_H = (600.0, 900.0, 1200.0, 1500.0, 1800.0)
RUNS = sys.argv[1] if len(sys.argv) > 1 else "../../runs"


def series(path):
    with open(path) as f:
        return [(float(r["sim_time_sec"]), float(r["unknown_fraction"]))
                for r in csv.DictReader(f)]


def at(rows, t):
    """Last value at or before t -- never interpolated, never the nearest."""
    v = [u for (s, u) in rows if s <= t]
    return v[-1] if v else None


def main():
    cells = {}
    for name in sorted(os.listdir(RUNS)):
        p = os.path.join(RUNS, name, "coverage_2d.csv")
        man = os.path.join(RUNS, name, "run_manifest.txt")
        if not (os.path.exists(p) and os.path.exists(man)):
            continue
        arm = "on" if "exploitation_enabled: true" in open(man).read() else "off"
        cells[name] = (arm, series(p))

    on = [n for n, (a, _) in cells.items() if a == "on"]
    if on:
        sys.exit(f"REFUSED: exploit-on cells are already re-scored ({', '.join(on)}). "
                 "The budget is fixed from the untreated arm BEFORE any treated "
                 "cell is scored; deriving it now would let the effect choose "
                 "its own threshold.")
    off = {n: r for n, (a, r) in cells.items() if a == "off"}
    if len(off) < 2:
        sys.exit(f"REFUSED: {len(off)} re-scored exploit-off cell(s). "
                 "A floor and a headroom read off one cell are that cell's, "
                 "not the arm's.")

    heads, floors = [], []
    print(f"exploit-off cells ({len(off)}):")
    for n, rows in sorted(off.items()):
        h = at(rows, HEADLINE_H)
        fl = min(u for _, u in rows)
        span = (rows[0][0], rows[-1][0])
        heads.append(h)
        floors.append(fl)
        print(f"  {n:<12} sim {span[0]:6.0f}..{span[1]:6.0f}s  "
              f"unknown@{HEADLINE_H:.0f}s = {h:.4f}  best = {fl:.4f}")
        if span[1] < HEADLINE_H:
            sys.exit(f"REFUSED: {n} ends at t={span[1]:.0f}s, before the "
                     f"headline horizon {HEADLINE_H:.0f}s.")

    head = statistics.mean(heads)
    floor = statistics.mean(floors)
    headroom = head - floor
    budget = headroom / 2.0

    # The design's own resolution limit, from the same arm (§6.5). A budget
    # under this can only ever return UNDECIDED, which is worth knowing before
    # five treated cells are spent measuring it.
    sd_u = statistics.stdev(heads) if len(heads) > 1 else None
    half = 1.96 * sd_u * (2.0 / 5) ** 0.5 if sd_u is not None else None

    print(f"\n  headline unknown (off arm, t={HEADLINE_H:.0f}s) = {head:.4f}")
    print(f"  floor (off arm best achieved)            = {floor:.4f}")
    print(f"  headroom                                 = {headroom:.4f}")
    print(f"\n  H2_UNK_BUDGET = {budget:.4f}   (half the headroom, plan §5.4 rule)")
    print(f"  EXPLORE_FLOOR = {floor:.4f}")
    # Resolvability at the HEADLINE horizon alone is the flattering half of the
    # story, and reporting only it would be choosing the horizon that agrees.
    # H2 is decided on five horizons, so the noise is reported at all of them.
    print("\n  budget against replicate noise, at EVERY deciding horizon:")
    worst = None
    for hz in DECIDING_H:
        vs = [at(r, hz) for r in off.values()]
        if any(v is None for v in vs) or len(vs) < 2:
            continue
        sd_h = statistics.stdev(vs)
        half_h = 1.96 * sd_h * math.sqrt(2.0 / 5.0)
        ok = budget >= half_h
        worst = half_h if worst is None else max(worst, half_h)
        print(f"    t={hz:<6.0f} off mean={statistics.mean(vs):.4f} SD={sd_h:.4f} "
              f"CI half={half_h:.4f}  {'resolvable' if ok else 'NOT resolvable'}")
    if worst is not None and budget < worst:
        print("\n  READ THIS BEFORE USING THE BUDGET. It is not resolvable at every"
              "\n  horizon H2 decides on. Where it is not, H2 can only come back"
              "\n  UNDECIDED however the cells fall, and that is a property of the"
              "\n  design read off the pilot -- NOT a finding that exploitation is"
              "\n  free. Report it as a limit on H2.")
        print("  Note also WHY the noise is small at the late horizons: both cells"
              "\n  are near the floor there, so the variance is small because there"
              "\n  is nowhere left to go, not because the measurement got sharper."
              "\n  Resolvability bought by saturation is worth less than it looks.")
    if half is not None:
        print(f"\n  at the headline t={HEADLINE_H:.0f}s: SD={sd_u:.4f}, "
              f"CI half-width at n=5 an arm = {half:.4f}")
        if budget < half:
            print("  NOT RESOLVABLE: the budget sits inside the replicate noise,"
                  "\n  so H2 can only come back UNDECIDED however the cells fall."
                  "\n  That is a limit on H2 to be reported, NOT a reason to widen"
                  "\n  the budget -- a budget moved to clear its own noise floor is"
                  "\n  not a pre-registered budget (§6.5).")
        else:
            print("  RESOLVABLE at n=5 an arm.")
    print(f"\nwrite these into aggregate.py: H2_UNK_BUDGET = {budget:.4f}, "
          f"EXPLORE_FLOOR = {floor:.4f}")


main()
