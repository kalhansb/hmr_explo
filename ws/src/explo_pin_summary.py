#!/usr/bin/env python3
"""Summarise one run_explo_pinned.sh run: per-node CPU/RSS + stack rates.

Usage: explo_pin_summary.py <output_dir>/<run_name>     (no extension)

Artefacts read (all optional except the scovox log):
  <run>.{scovox,dscovox,planner,loc}.cpu.csv   time_sec,cpu_ticks,rss_kb @0.5s
  <run>.scovox.log     one `recv=` line per admitted scan
  <run>.dscovox.log    dscovox_diag: sources= src_voxels= fused_voxels=
  <run>.planner.log    planner-side step/waiting lines
  <run>.planner.csv    MetricsLogger per-step rows (plan_time_ms etc.)

The planner's MEAN cores is a duty-cycle number, not its cost — it is
single-threaded and bursts to ~1 core while planning, idle in between (see
explo_planner_jetson_run.md). Peak/p95 over 0.5 s samples is reported for it.
"""
import csv
import os
import re
import sys


def pct(xs, q):
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))]


def cpu_rows(path):
    if not os.path.exists(path):
        return None
    rows = []
    with open(path) as fh:
        next(fh, None)
        for line in fh:
            f = line.strip().split(",")
            if len(f) == 3 and all(f):
                try:
                    rows.append((float(f[0]), int(f[1]), int(f[2])))
                except ValueError:
                    pass
    return rows if len(rows) >= 2 else None


def cpu_stats(path):
    """-> (mean_cores, inst_cores_series, peak_rss_mb, dt, n) or None."""
    rows = cpu_rows(path)
    if not rows:
        return None
    clk = os.sysconf("SC_CLK_TCK")
    dt = rows[-1][0] - rows[0][0]
    if dt <= 0:
        return None
    mean = (rows[-1][1] - rows[0][1]) / clk / dt
    inst = []
    for (t0, c0, _), (t1, c1, _) in zip(rows, rows[1:]):
        if t1 > t0:
            inst.append((c1 - c0) / clk / (t1 - t0))
    return mean, inst, max(r[2] for r in rows) / 1024.0, dt, len(rows)


LOG_RE = re.compile(
    r"^\[INFO\] \[(?P<t>\d+\.\d+)\] \[[^\]]*\]: recv=(?P<recv>\d+) .*?"
    r"frame_ms=(?P<frame>[\d.]+) tf_ms=(?P<tf>[\d.]+) "
    r"integrate_ms=(?P<integ>[\d.]+) publish_ms=(?P<pub>[\d.]+) "
    r"rss_mb=(?P<rss>[\d.]+)")
TAIL_RE = re.compile(r"gated=(\d+) rearm=(\d+) reject_gated=(\d+).*?tf_fb=(\d+)")
DIAG_RE = re.compile(r"dscovox_diag: sources=(\d+) src_voxels=(\d+) fused_voxels=(\d+)")


def scovox_log(path):
    if not os.path.exists(path):
        return None
    ts, frame, integ, rss = [], [], [], []
    tail = None
    with open(path, errors="replace") as fh:
        for line in fh:
            m = LOG_RE.match(line)
            if not m:
                continue
            ts.append(float(m["t"]))
            frame.append(float(m["frame"]))
            integ.append(float(m["integ"]))
            rss.append(float(m["rss"]))
            t = TAIL_RE.search(line)
            if t:
                tail = tuple(int(x) for x in t.groups())
    return dict(n=len(ts), span=ts[-1] - ts[0], frame=frame,
                integ=integ, rss=rss, tail=tail) if ts else None


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    run = sys.argv[1]
    print(f"=== {os.path.basename(run)} ===")

    s = scovox_log(run + ".scovox.log")
    if s and s["span"] > 0:
        print(f"scovox scans:  {s['n']} admitted in {s['span']:.1f} s "
              f"-> {(s['n'] - 1) / s['span']:.2f} Hz  (bag offers 10 Hz)")
        if s["tail"]:
            g, rearm, rej, tf_fb = s["tail"]
            print(f"               gated={g} reject_gated={rej} rearm={rearm} "
                  f"tf_fallback={tf_fb}"
                  + ("   <-- stale-pose scans, map is smeared" if tf_fb else ""))
        print(f"        frame_ms: p50 {pct(s['frame'], .5):7.1f}  "
              f"p95 {pct(s['frame'], .95):7.1f}  max {max(s['frame']):7.1f}")
        print(f"    integrate_ms: p50 {pct(s['integ'], .5):7.1f}  "
              f"p95 {pct(s['integ'], .95):7.1f}  max {max(s['integ']):7.1f}")
    else:
        print("scovox scans:  NONE -- check TF before trusting any number below.")

    # dscovox diag: last line is the end-of-run fused state.
    diag = None
    if os.path.exists(run + ".dscovox.log"):
        with open(run + ".dscovox.log", errors="replace") as fh:
            for line in fh:
                m = DIAG_RE.search(line)
                if m:
                    diag = tuple(int(x) for x in m.groups())
    if diag:
        print(f"dscovox:       sources={diag[0]} src_voxels={diag[1]} "
              f"fused_voxels={diag[2]}")
    else:
        print("dscovox:       no dscovox_diag line -- merger fused NOTHING;")
        print("               the planner ran without a map. Check scovox_bin wiring.")

    # planner CSV: per-step metrics.
    steps = []
    if os.path.exists(run + ".planner.csv"):
        with open(run + ".planner.csv") as fh:
            steps = list(csv.DictReader(fh))
    if steps:
        pt = [float(r["plan_time_ms"]) for r in steps]
        print(f"planner:       {len(steps)} plan steps, plan_time_ms "
              f"p50 {pct(pt, .5):.0f} max {max(pt):.0f}, "
              f"observed_voxels {int(float(steps[-1]['total_observed_voxels']))}, "
              f"frontier {int(float(steps[-1]['frontier_voxels']))}")
    else:
        print("planner:       0 plan steps -- it never left WAIT_FOR_MAP/POSE.")

    print()
    for label, stem, burst in (("lidar_localization", ".loc", False),
                               ("scovox_mapping_node", ".scovox", False),
                               ("dscovox_mapping_node", ".dscovox", False),
                               ("explo_planner_node", ".planner", True)):
        c = cpu_stats(run + stem + ".cpu.csv")
        if c is None:
            print(f"{label:>21}: no samples")
            continue
        mean, inst, rss, dt, n = c
        extra = (f"  [bursty: p95 {pct(inst, .95):.2f} peak {max(inst):.2f} "
                 f"-- mean is duty cycle, not cost]" if burst else "")
        print(f"{label:>21}: {mean:5.2f} cores avg over {dt:5.1f} s "
              f"({n} samples), peak RSS {rss:7.1f} MB{extra}")
    print(f"\n(box: {os.cpu_count()} cores visible)")


if __name__ == "__main__":
    main()
