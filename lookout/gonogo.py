#!/usr/bin/env python3
"""Pilot go/no-go checks (brief, "Order of work"), from the pilot logs only.

  gonogo.py --layout L2=/runs/lookout/L2_pilot --layout L3=/runs/lookout/L3_pilot \
            --calib /runs/lookout/calib

1. Every lidar sees the person: at least one pilot scan per lidar with >= 1
   non-static return in the person's box (and the best-case count reached).
   Calibration not far beyond the ~30-38 m real-world range.
2. The lookouts are on their points and hold them: spawned on the calculated
   post, at rest within 0.1 m of it and linked to the mulcher there (walks.py
   "at_post" events), and still at rest during the walks.
3. Walks finish without crashes and every CSV field is filled (a first-
   detection column may be empty only when that detector never fired; a
   lookout's columns are empty in the check walks, which run without them).
4. False alarms do not outnumber true detections (main rule, alarm scans).
"""
import argparse
import csv
import json
import os

NO_DETECT_OK = ("_step", "_t_s", "_x", "_y", "_dist_m", "_warn_s", "_first_by", "linked_at_alarm_")


def read(p):
    return list(csv.DictReader(open(p))) if os.path.exists(p) else []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", action="append", required=True)
    ap.add_argument("--calib", default="/runs/lookout/calib")
    a = ap.parse_args()
    ok_all = True

    def check(name, ok, detail):
        nonlocal ok_all
        ok_all &= bool(ok)
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    cr = json.load(open(os.path.join(a.calib, "calib_ranges.json")))
    far = {k: v["max_m"] for k, v in cr.items() if k.endswith("/best") or k.endswith("/alarm_m5")}
    print("calibration")
    check("calibration not far beyond ~30-38 m", all(v is None or v <= 45 for v in far.values()),
          ", ".join(f"{k} max {v}" for k, v in far.items()))
    for s in a.layout:
        n, d = s.split("=", 1)
        print(n)
        steps = read(os.path.join(d, "steps.csv"))
        walks = read(os.path.join(d, "walks.csv"))
        alarms = read(os.path.join(d, "alarms.csv"))
        ev = os.path.join(d, "events.jsonl")
        at_post = [e for e in map(json.loads, open(ev)) if e["kind"] == "at_post"] if os.path.exists(ev) else []
        lidars = sorted({r["lidar"] for r in steps})
        for lid in lidars:
            rs = [r for r in steps if r["lidar"] == lid and "-chk" not in r["walk"]]
            mx = max((int(r["best_ns"]) for r in rs), default=0)
            check(f"{lid} sees the person", mx >= 1, f"max {mx} non-static returns in the box over {len(rs)} scans")
        check("at_post logged for the lookouts", len(at_post) > 0, f"{len(at_post)} at_post events")
        for r in at_post:
            check(f"{r['robot']} on its post and linked", r["d_post_m"] <= 0.1 and r["linked"] == 1,
                  f"{r['d_post_m']} m from the calculated point; link {r['link_d_m']} m, "
                  f"{r['trees_on_link']} trunks, SNR {r['snr_db']} dB")
        for w in walks:
            if w["lookouts_present"] != "1":
                continue
            hold = {k: float(v) for k, v in w.items() if k.startswith("hold_max_dpos_m_") and v != ""}
            check(f"{w['walk']} lookouts held", all(v < 0.01 for v in hold.values()),
                  ", ".join(f"{k[16:]} {v:.4f} m" for k, v in hold.items()))
        # a check walk runs without the lookouts, so their columns stay empty
        lks = [l for l in lidars if l != "mulcher"]
        absent = lambda w, k: w["lookouts_present"] == "0" and any(
            k.startswith(f"{l}_") or k.endswith(f"_{l}") for l in lks)
        empty = [(w["walk"], k) for w in walks for k, v in w.items()
                 if v == "" and not any(t in k for t in NO_DETECT_OK) and not absent(w, k)]
        check("walks finished, every field filled", len(walks) > 0 and not empty,
              f"{len(walks)} walks in walks.csv; empty fields: {empty[:6]}")
        m5 = [x for x in alarms if x["variant"] == "m5" and "-chk" not in x["walk"]]
        tp, fp = sum(x["hit"] == "1" for x in m5), sum(x["hit"] == "0" for x in m5)
        check("false alarms do not outnumber true detections (main rule)", fp <= tp,
              f"{tp} true / {fp} false alarm scans; by lidar: "
              + ", ".join(f"{l} {sum(x['hit'] == '1' for x in m5 if x['lidar'] == l)}/"
                          f"{sum(x['hit'] == '0' for x in m5 if x['lidar'] == l)}" for l in lidars))
    print("GO" if ok_all else "NO-GO")


if __name__ == "__main__":
    main()
