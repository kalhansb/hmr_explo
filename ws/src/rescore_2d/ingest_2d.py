#!/usr/bin/env python3
"""Turn a cell's offline census into the cell's coverage_2d.csv.

The census stamps each row with the grid message's header stamp, which is WALL
epoch: the offline mapper runs against a bag, and although it is started with
use_sim_time the grids it publishes are stamped from the data, not from the
replayed clock. The campaign's horizons are sim seconds, so the two have to be
reconciled before anything can be read at t=1200.

The reconciliation is in the bag itself. /clock was recorded, and rosbag2
stores each message's wall receive time beside its payload, so the bag carries
an explicit (wall, sim) pair for every clock tick of the ORIGINAL run. That is
a per-cell mapping and it is measured, not assumed -- the runs do not share an
RTF, and off_rep1's 0.4602 would be simply wrong applied to another cell.

Only the un-inflated /global_coverage_map series is kept. The inflated
planning map is what the planner drives on; it reports body-radius inflation as
occupied and would score the ROI as more known than it is (§6.6).
"""
import csv, glob, json, os, struct, sqlite3, sys, bisect

CELL = sys.argv[1]
CENSUS = sys.argv[2]
BAGDIR = sys.argv[3]
OUT = sys.argv[4]
TOPIC = "/dscovox_node/global_coverage_map"


def clock_pairs(bagdir):
    pairs = []
    for db in sorted(glob.glob(os.path.join(bagdir, "*.db3"))):
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        try:
            t = con.execute("select id from topics where name='/clock'").fetchone()
            if not t:
                continue
            for ts, data in con.execute(
                    "select timestamp,data from messages where topic_id=?", (t[0],)):
                # CDR: 4-byte encapsulation header, then int32 sec, uint32 nsec.
                sec, nsec = struct.unpack_from("<iI", data, 4)
                pairs.append((ts / 1e9, sec + nsec * 1e-9))
        finally:
            con.close()
    pairs.sort()
    return pairs


def main():
    pairs = clock_pairs(BAGDIR)
    if len(pairs) < 2:
        sys.exit(f"{CELL}: no /clock in {BAGDIR} -- cannot map wall to sim")
    walls = [p[0] for p in pairs]
    sims = [p[1] for p in pairs]

    def to_sim(w):
        """Linear interpolation between the two bracketing clock ticks.

        Outside the recorded span this returns None rather than extrapolating:
        a census row stamped before the first tick or after the last one has no
        sim time the bag can vouch for, and inventing one would put a reading
        at a horizon the run may never have reached.
        """
        if w < walls[0] or w > walls[-1]:
            return None
        i = bisect.bisect_left(walls, w)
        if i == 0:
            return sims[0]
        w0, w1, s0, s1 = walls[i - 1], walls[i], sims[i - 1], sims[i]
        if w1 == w0:
            return s1
        return s0 + (s1 - s0) * (w - w0) / (w1 - w0)

    rows, dropped = [], 0
    with open(CENSUS) as f:
        for r in csv.DictReader(f):
            if r["topic"] != TOPIC:
                continue
            t = to_sim(float(r["t_sim"]))
            if t is None:
                dropped += 1
                continue
            n_roi = int(r["roi_cells"])
            rows.append({
                "sim_time_sec": f"{t:.3f}",
                "unknown_fraction": f"{int(r['unknown']) / n_roi:.6f}",
                "known_frac": r["known_frac"],
                "frontier": r["frontier"],
                "reach_frontier": r["reach_frontier"],
                "roi_cells": r["roi_cells"],
                "unknown_cells": r["unknown"],
            })
    rows.sort(key=lambda x: float(x["sim_time_sec"]))
    if not rows:
        sys.exit(f"{CELL}: no {TOPIC} rows inside the bag's clock span")
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"{CELL}: {len(rows)} rows -> {OUT} "
          f"(sim {float(rows[0]['sim_time_sec']):.0f}..{float(rows[-1]['sim_time_sec']):.0f}s, "
          f"unknown {rows[0]['unknown_fraction']} -> {rows[-1]['unknown_fraction']}"
          f"{f', {dropped} outside clock span' if dropped else ''})")


main()
