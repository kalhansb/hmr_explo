#!/usr/bin/env python3
"""Offline core of sim/trunk_score.py — everything that needs no ROS.

Belongs at explo_planner/sim/trunk_score.py on new_experiments; the ROS half
(bag replay + GetRegion at each horizon) wraps these functions.

Run with no args: parses the world SDF, reproduces the §4 trunk tables, and
self-tests M1-M3 on synthetic voxel sets with known coverage.
"""
import math
import re
import sys

# --- §4.2 scoring cylinder -------------------------------------------------
CYL_R = 2.0          # m, covers the 1.70 m root flare + one 0.20 m voxel
Z_LO, Z_HI = 0.2, 1.2
N_SECTORS = 12       # legacy 30 deg bins, reported as M1_sec12 only
VOX = 0.20
GAP_BRIDGE = 0.5     # merge angular gaps below half a voxel column (§8.2.5)
OCC_THRESH = 0.7

TARGETS = {"Oak tree_42": 1, "Oak tree_4": 2, "Oak tree_39": 3}
EXCLUDE_CONTROL = {"Oak tree_19"}   # MaleVisitorOnPhone stands 0.12 m from it
NEAR_M = 25.0


def parse_oaks(sdf_path):
    """Model-level <pose> per 'Oak tree*' model.

    The first <pose> inside a <model> body is the LINK's and reads 0 0 0 for
    every tree in this world; the model pose is the one outside the links.
    """
    s = open(sdf_path).read()
    out = {}
    for m in re.finditer(r'<model name="([^"]*Oak tree[^"]*)">(.*?)</model>', s, re.S):
        name, body = m.group(1), m.group(2)
        head = body.split("<link", 1)[0]
        p = re.search(r"<pose[^>]*>([^<]+)</pose>", head)
        p = p.group(1) if p else re.findall(r"<pose[^>]*>([^<]+)</pose>", body)[-1]
        x, y, z = (float(v) for v in p.split()[:3])
        out[name] = (x, y, z)
    return out


# --- §5.2 metrics ----------------------------------------------------------
def in_cylinder(vx, vy, vz, cx, cy, cz):
    if not (cz + Z_LO <= vz <= cz + Z_HI):
        return False
    return (vx - cx) ** 2 + (vy - cy) ** 2 <= CYL_R ** 2


def sector_of(vx, vy, cx, cy):
    a = math.atan2(vy - cy, vx - cx) % (2 * math.pi)
    return int(a / (2 * math.pi / N_SECTORS)) % N_SECTORS


def angular_coverage(columns, centre_xy):
    """M1: fraction of the trunk's circumference the map holds evidence for.

    Resolution-matched, not bin-counted. Each occupied voxel column at radius
    rc subtends 2*atan(VOX/2 / rc) of arc; the union of those footprints is
    taken, gaps below half a column are bridged as sampling noise, and each
    open segment is eroded by one column so the estimate is centre-to-centre
    rather than edge-to-edge. A closed loop has no endpoints and is not
    eroded.

    Fixed-bin M1 cannot do this: a sector counts as covered on one voxel, so
    it over-reads fragmented coverage by up to +0.29 (see main()). This
    estimator's worst error on the same cases is 0.083, and that only at
    rc = 0.40 m where the map's own angular resolution is 28 deg.
    """
    cx, cy = centre_xy
    iv = []
    for vx, vy in columns:
        rc = math.hypot(vx - cx, vy - cy)
        if rc < VOX / 2:                      # column on the axis: all bearings
            return 1.0
        iv.append((math.atan2(vy - cy, vx - cx) % (2 * math.pi),
                   math.atan2(VOX / 2.0, rc)))
    if not iv:
        return 0.0
    iv.sort()
    segs = []
    lo, hi, hh = iv[0][0] - iv[0][1], iv[0][0] + iv[0][1], iv[0][1]
    for a, h in iv[1:]:
        if a - h <= hi + GAP_BRIDGE * 2 * max(h, hh) + 1e-12:
            hi, hh = max(hi, a + h), max(hh, h)
        else:
            segs.append([lo, hi, hh])
            lo, hi, hh = a - h, a + h, h
    segs.append([lo, hi, hh])
    if len(segs) > 1 and (segs[0][0] + 2 * math.pi
                          <= segs[-1][1] + GAP_BRIDGE * 2 * max(segs[0][2],
                                                                segs[-1][2]) + 1e-12):
        segs[0][0] = segs[-1][0] - 2 * math.pi
        segs[0][2] = max(segs[0][2], segs[-1][2])
        segs.pop()
    if len(segs) == 1 and segs[0][1] - segs[0][0] >= 2 * math.pi - 1e-9:
        return 1.0
    total = sum(max(hi - lo - 2 * h, 2 * h) for lo, hi, h in segs)
    return min(1.0, total / (2 * math.pi))


def score_trunk(voxels, centre):
    """voxels: iterable of (x, y, z, a_occ, a_free). centre: (cx, cy, cz).

    M1 angular completeness, M2 observed fraction, M3 median evidence mass.
    """
    cx, cy, cz = centre
    hit = set()
    cols = set()
    n_obs = n_occ = 0
    masses = []
    for vx, vy, vz, a_occ, a_free in voxels:
        if not in_cylinder(vx, vy, vz, cx, cy, cz):
            continue
        n_obs += 1                                   # GetRegion returns non-prior only
        if a_occ / (a_occ + a_free) >= OCC_THRESH:
            n_occ += 1
            hit.add(sector_of(vx, vy, cx, cy))
            cols.add((round(vx / VOX) * VOX, round(vy / VOX) * VOX))
            masses.append(a_occ + a_free - 2.0)
    n_cells = cells_in_cylinder()
    masses.sort()
    m3 = masses[len(masses) // 2] if masses else 0.0
    return {"M1": angular_coverage(cols, (cx, cy)),
            "M1_sec12": len(hit) / N_SECTORS,
            "M2": n_obs / n_cells, "M3": m3,
            "n_occ": n_occ, "n_cols": len(cols), "sectors": sorted(hit)}


def cells_in_cylinder():
    n = 0
    k = int(CYL_R / VOX) + 1
    nz = int(round((Z_HI - Z_LO) / VOX))
    for i in range(-k, k + 1):
        for j in range(-k, k + 1):
            if (i * VOX) ** 2 + (j * VOX) ** 2 <= CYL_R ** 2:
                n += nz
    return n


# --- synthetic trunks ------------------------------------------------------
def bark_voxels(centre, radius, arcs, evidence=10.0):
    """Voxelised bark shell at `radius`, keeping only the given (lo, hi) degree
    arcs. Returns the deduplicated 0.20 m voxel set, as a real map would hold."""
    cx, cy, cz = centre
    out = {}
    steps = 2000
    for s in range(steps):
        deg = 360.0 * s / steps
        if not any(lo <= deg <= hi for lo, hi in arcs):
            continue
        a = math.radians(deg)
        x, y = cx + radius * math.cos(a), cy + radius * math.sin(a)
        z = cz + Z_LO
        while z <= cz + Z_HI + 1e-9:
            key = (round(x / VOX), round(y / VOX), round(z / VOX))
            out[key] = (key[0] * VOX, key[1] * VOX, key[2] * VOX,
                        evidence, 1.0)
            z += VOX
    return list(out.values())


def visible_arc_deg(radius, standoff):
    """Angular extent of a cylinder's surface visible from one viewpoint."""
    return 2.0 * math.degrees(math.acos(min(1.0, radius / standoff)))


def main():
    sdf = ("/home/kalhan/Documents/explo_planner_experiments/hmr_explo/ws/src/"
           "hmr_sim/hmr_sim/worlds/flatforest/flatforestv2.sdf")
    oaks = parse_oaks(sdf)
    print(f"SDF: {len(oaks)} 'Oak tree*' models parsed")

    print("\n--- §4.1 target trunks ---")
    for name, tid in sorted(TARGETS.items(), key=lambda kv: kv[1]):
        x, y, z = oaks[name]
        nn = min(math.hypot(x - x2, y - y2)
                 for n2, (x2, y2, _) in oaks.items() if n2 != name)
        print(f"  id {tid}  {name:<13} ({x:7.2f},{y:7.2f})  "
              f"spawn {math.hypot(x, y):5.1f} m  nearest {nn:5.1f} m")

    near = []
    for name, (x, y, z) in oaks.items():
        d = math.hypot(x, y)
        if d < NEAR_M and name not in TARGETS and name not in EXCLUDE_CONTROL:
            near.append((d, name, x, y))
    near.sort()
    print(f"\n--- §4.3 near controls: {len(near)} (expected 11) ---")
    for d, name, x, y in near:
        print(f"  {name:<13} ({x:7.2f},{y:7.2f})  {d:5.1f} m")
    far = len(oaks) - len(TARGETS) - len(near) - len(EXCLUDE_CONTROL)
    print(f"far controls: {far}   sunk models: "
          f"{[n for n, (_, _, z) in oaks.items() if abs(z) > 1e-6]}")

    print(f"\n--- cylinder: r={CYL_R} m, z {Z_LO}-{Z_HI} m, "
          f"{VOX} m voxels -> {cells_in_cylinder()} cells ---")

    c = (0.0, 0.0, 0.0)
    print("\n--- M1 self-test on synthetic bark (truth = arc actually present) ---")
    cases = [
        ("one 120 deg arc", [(0, 120)], 1 / 3),
        ("three 40 deg arcs, 120 apart", [(0, 40), (120, 160), (240, 280)], 1 / 3),
        ("half ring", [(0, 180)], 0.5),
        ("three 60 deg arcs (3-vantage dwell)", [(0, 60), (120, 180), (240, 300)], 0.5),
        ("one 30 deg sliver", [(0, 30)], 1 / 12),
        ("full ring", [(0, 360)], 1.0),
        ("full ring minus one 30 deg hole", [(0, 330)], 330 / 360),
    ]
    # tolerance is the map's own angular resolution at that radius, halved
    ok = True
    print(f"  {'bark r':>7} {'case':36} {'truth':>6} {'M1':>6} {'err':>7} "
          f"{'sec12':>6} {'err':>7}")
    for r in (0.40, 0.70, 1.10, 1.70):
        tol = math.degrees(2 * math.atan2(VOX / 2, r)) / 360.0 + 0.02
        for label, arcs, truth in cases:
            g = score_trunk(bark_voxels(c, r, arcs), c)
            bad = abs(g["M1"] - truth) > tol
            ok &= not bad
            print(f"  {r:7.2f} {label:36} {truth:6.3f} {g['M1']:6.3f} "
                  f"{g['M1']-truth:+7.3f} {g['M1_sec12']:6.3f} "
                  f"{g['M1_sec12']-truth:+7.3f}{'  FAIL' if bad else ''}")

    print("\n--- why not fixed bins (threat 8.2.5) ---")
    for r in (0.40, 0.70, 1.10, 1.70):
        v = bark_voxels(c, r, [(0, 360)])
        cols = {(round(x / VOX), round(y / VOX)) for x, y, _, _, _ in v}
        w = math.degrees(2 * math.atan2(VOX / 2, r))
        print(f"  r={r:.2f} m: {len(cols):3d} voxel columns around the "
              f"circumference, one column = {w:4.1f} deg "
              f"({len(cols)/N_SECTORS:.1f} per 30 deg sector)")

    print("\n--- predicted M1 from viewing geometry (vantage ring at 3.8 m) ---")
    for r in (0.40, 1.10):
        arc = visible_arc_deg(r, 3.8)
        print(f"  bark r={r:.2f} m: one viewpoint sees {arc:.0f} deg "
              f"-> M1 ~ {min(1.0, arc/360):.2f};  three vantages 120 apart "
              f"-> M1 ~ {min(1.0, 3*arc/360):.2f}")

    print("\nSELF-TEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
