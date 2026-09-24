#!/usr/bin/env python3
"""Lookout placement: where each path's lookout stands, calculated from a map.

In the application the robots explore the area first, so the map is known:
trunk positions and path centrelines. Here the map is the world's own trunk
list (a perfect map; lanes cleared, nothing cleared for a post). The mulcher
is at the origin. For each path entry:

  1. Candidates: every STEP m along the path where it is R_MIN..R_SAFE from
     the mulcher, PAD m to either side of it (along the circle through that
     point), facing out along the path towards arriving walkers.
  2. A spot is kept only if
       - it is free: no trunk centre within PAD + FLARE (the pad the world
         builder used to clear round a post; a robot cannot clear trees), and
       - the radio link to the mulcher holds with margin: radio.link (the
         comms model, unchanged) says linked, the spot is at most R_SAFE from
         the mulcher (2 m inside the 30 m horizon), and the nearest trunk is at
         least CLEAR_MARGIN outside the link's Fresnel corridor.
  3. Score: a person walks in along the path centreline in STEP m steps. The
     score is their distance from the mulcher when they are first within
     REACH_MAIN of the spot (the calibrated main-rule range) with no trunk
     (radius TRUNK_R) on the straight line between. Ties: the same with
     REACH_BEST (the calibrated best-case range), then the larger corridor
     clearance.
  4. The highest score wins.

Simplifications: trunks only (no branches or canopy), the person is a point
on the centreline, the lidar is at the spot, and the reaches are the
open-ground calibration (runs/lookout/calib_gpu).

  posts.py        print every entry's winner and runner-up (host python, stdlib)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import radio  # noqa: E402

STEP = 0.5
R_MIN, R_SAFE = 12.0, 28.0
CLEAR_MARGIN = 0.5
TRUNK_R = 0.35                 # trunk seen by the lidar at person height (rough)
REACH_MAIN, REACH_BEST = 12.5, 30.0
WALK_FROM = 100.0              # the scored walk starts this far from the mulcher
ANT_Z = 0.1                    # lookout antenna height for the link (the mulcher's is 0)


def corridor_width(d):
    return radio.TREE_R + 0.5 * math.sqrt(3.0e8 / radio.FREQ * d)


def first_seen(eye, walk, trees, reach):
    """Distance from the mulcher where the walker is first in view, or None."""
    near = [t for t in trees if math.dist(eye, t) <= reach + TRUNK_R]
    for p in walk:
        if math.dist(eye, p) > reach:
            continue
        if all(radio._seg_dist2(eye, p, t) >= TRUNK_R ** 2 for t in near):
            return math.hypot(*p)
    return None


def candidates(bw, P, e, trees, pad, flare):
    """Every candidate spot for entry e on polyline P, scored. `bw` is the
    world builder module (path geometry)."""
    s_far = bw.arc_where_range(P, WALK_FROM, e["s_start"], e["s_end"])
    s_far = e["s_start"] if s_far is None else s_far
    n = int(abs(e["s_end"] - s_far) / STEP)
    walk = [bw.point_at(P, s_far + e["dir"] * STEP * k)[0] for k in range(n + 1)]
    out = []
    r = R_SAFE
    while r >= R_MIN - 1e-9:
        s = bw.arc_where_range(P, r, e["s_start"], e["s_end"])
        if s is not None:
            (cx, cy), (tx, ty) = bw.point_at(P, s)
            tx, ty = tx * e["dir"], ty * e["dir"]           # walking direction
            for side, sgn in (("ccw", 1), ("cw", -1)):
                a = math.atan2(cy, cx) + sgn * pad / r
                x, y = r * math.cos(a), r * math.sin(a)
                c = {"r": r, "side": side, "x": x, "y": y, "yaw": math.atan2(-ty, -tx),
                     "s_post": s, "post_cross": (cx, cy)}
                c["nearest_trunk_m"] = min(math.dist((x, y), t) for t in trees)
                c["free"] = c["nearest_trunk_m"] >= pad + flare
                L = radio.link((x, y, ANT_Z), (0.0, 0.0, 0.0), trees)
                clear = min(math.sqrt(radio._seg_dist2((x, y), (0.0, 0.0), t)) for t in trees)
                c.update(linked=L["linked"], snr_db=L["snr_db"], link_d_m=L["distance_m"],
                         trees_on_link=L["trees_on_link"], corridor_w_m=corridor_width(L["distance_m"]),
                         link_clear_m=clear)
                c["link_ok"] = (L["linked"] and r <= R_SAFE
                                and clear - c["corridor_w_m"] >= CLEAR_MARGIN)
                c["seen_main_m"] = first_seen((x, y), walk, trees, REACH_MAIN)
                c["seen_best_m"] = first_seen((x, y), walk, trees, REACH_BEST)
                out.append(c)
        r = round(r - STEP, 6)
    return out


def rank_key(c):
    return (c["seen_main_m"] or 0.0, c["seen_best_m"] or 0.0, c["link_clear_m"] - c["corridor_w_m"])


def find_post(bw, P, e, trees, pad, flare):
    """The winning spot and the ranked list of kept candidates."""
    cands = candidates(bw, P, e, trees, pad, flare)
    ok = sorted((c for c in cands if c["free"] and c["link_ok"]), key=rank_key, reverse=True)
    if not ok:
        raise RuntimeError(f"entry {e['id']}: no free spot with a safe link")
    return ok[0], ok, cands


def main():
    import build_world as bw
    base = bw.poisson_forest(bw.FOREST_SEED)
    for name in ("L2", "L3"):
        paths, entries, lookouts = bw.layout(name, base)
        trees = bw.clear_forest(base, paths, [])
        pmap = {p["id"]: p["polyline"] for p in paths}
        print(f"== {name}")
        for e in entries:
            best, ok, cands = find_post(bw, pmap[e["path"]], e, trees, bw.PAD, bw.FLARE)
            n_free = sum(c["free"] for c in cands)
            n_link = sum(c["link_ok"] for c in cands)
            print(f"  entry {e['id']}: {len(cands)} candidates, {n_free} free, {n_link} safe link, "
                  f"{len(ok)} both")
            for tag, c in (("winner", best), ("runner-up", ok[1] if len(ok) > 1 else None)):
                if c is None:
                    continue
                print(f"    {tag:9s} {c['r']:.1f} m {c['side']:3s} ({c['x']:.2f}, {c['y']:.2f}): first seen "
                      f"{c['seen_main_m']:.1f} m main, {c['seen_best_m']:.1f} m best; nearest trunk "
                      f"{c['nearest_trunk_m']:.2f} m; link {c['link_d_m']:.1f} m, {c['trees_on_link']} trunks, "
                      f"corridor clearance {c['link_clear_m'] - c['corridor_w_m']:.2f} m")


if __name__ == "__main__":
    main()
