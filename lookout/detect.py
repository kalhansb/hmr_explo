"""Person detection from one lidar (Part 4 of the brief). Pure numpy/scipy, no ROS.

Per lidar, per step:
  1. background: per-beam (16 x 1800) median range over 10 scans with no person
     present (a beam whose median is no-return has background "none").
  2. new point: a return at least 0.3 m closer than its beam's background, or
     any return on a beam whose background is none.
  3. clusters: new points linked at 0.5 m (single linkage, 3D).
  4. candidate: a cluster with >= 5 points on >= 2 rings. Alarm: a candidate
     at this step with a candidate of the same lidar at the previous step
     whose centroid is within 1 m.
  5. detection: an alarm whose centroid is within 1 m (horizontal) of the true
     person position; every other alarm is a false alarm.
Best-case rule (comparison only): >= 5 returns inside the person's box in one
scan, not counting static returns (a return within 0.1 m of its beam's
background: trunk, branch, ground).

The person position is a point on the ground and a person's centroid sits
~0.9 m above it, so the hit test is horizontal distance; the persistence
test is 3D (the brief says "centroids within 1 m").

Linkage is 3D (main rule, the brief's "within 0.5 m of each other"). VLP-16
rings are 2 deg apart, so returns on neighbouring rings are 0.035 d apart
vertically at range d: past ~14.3 m no 3D cluster can span two rings, and
the >= 2-ring rule cannot fire. `xy=True` links on horizontal distance
instead (declared before the pilot as a labelled sensitivity variant, not a
replacement; see PLAN.md, Deviations).
"""
from dataclasses import dataclass, field, asdict

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class Rule:
    closer: float = 0.3        # new point: at least this much closer than background
    link: float = 0.5          # cluster linkage distance
    min_pts: int = 5           # candidate: points ...
    min_rings: int = 2         # ... on this many rings
    persist: float = 1.0       # alarm: centroid within this of a previous-step candidate
    hit: float = 1.0           # detection: centroid within this (horizontal) of the person
    box_min: int = 5           # best case: returns in the person box
    static_tol: float = 0.1    # best case: a return this close to background is static
    log_closer: float = 0.1    # logging: keep new points at this looser threshold


RULE = Rule()

# Person box in the model frame (the mesh faces -y; model yaw = heading + pi/2):
# rendered mesh extents x +-0.272, y -0.405..0.477 (stride axis) times the
# scale, z from the ground to the measured 1.739 m top; 5 cm margin all round.
PERSON_SCALE = 0.91748
BOX_X = 0.272 * PERSON_SCALE + 0.05
BOX_Y = (-0.405 * PERSON_SCALE - 0.05, 0.477 * PERSON_SCALE + 0.05)
BOX_Z = (0.05, 1.739 + 0.05)


def learn_background(ranges):
    """ranges: sequence of (H, W) range arrays (no-return = non-finite).
    Returns the per-beam median; +inf where the median is no-return."""
    R = np.stack([np.where(np.isfinite(r), r, np.inf) for r in ranges]).astype(np.float64)
    bg = np.median(R, axis=0)
    bg[~np.isfinite(bg)] = np.inf
    return bg


def new_mask(r, bg, closer):
    fin = np.isfinite(r)
    with np.errstate(invalid="ignore"):
        return fin & (~np.isfinite(bg) | (r <= bg - closer))


def cluster(P, link):
    """Single-linkage labels for points P (N, 3) at distance `link`."""
    n = len(P)
    if n == 0:
        return np.zeros(0, dtype=int)
    pairs = cKDTree(P).query_pairs(link, output_type="ndarray")
    if len(pairs) == 0:
        return np.arange(n)
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    return connected_components(g, directed=False)[1]


def clusters(P, rings, link=RULE.link, xy=False):
    """Every cluster of new points: list of dicts (centroid, n, rings).
    xy: link on horizontal distance only (the centroid stays 3D)."""
    out = []
    if len(P) == 0:
        return out
    lab = cluster(P[:, :2] if xy else P, link)
    for l in np.unique(lab):
        k = lab == l
        out.append({"centroid": P[k].mean(0), "n": int(k.sum()),
                    "rings": [int(x) for x in np.unique(rings[k])]})
    return out


def passes(c, rule=RULE, min_pts=None):
    return c["n"] >= (rule.min_pts if min_pts is None else min_pts) and len(c["rings"]) >= rule.min_rings


def candidates(P, rings, rule=RULE, min_pts=None):
    """Clusters of new points that pass the size rule. P: (N, 3) world points,
    rings: (N,). Returns a list of dicts (centroid, n, rings)."""
    return [c for c in clusters(P, rings, rule.link) if passes(c, rule, min_pts)]


def in_person_box(Pw, px, py, model_yaw):
    c, s = np.cos(-model_yaw), np.sin(-model_yaw)
    dx, dy = Pw[..., 0] - px, Pw[..., 1] - py
    lx, ly = c * dx - s * dy, s * dx + c * dy
    z = Pw[..., 2]
    with np.errstate(invalid="ignore"):
        return (np.isfinite(z) & (np.abs(lx) <= BOX_X) & (ly >= BOX_Y[0]) & (ly <= BOX_Y[1])
                & (z >= BOX_Z[0]) & (z <= BOX_Z[1]))


@dataclass
class Detector:
    """State for one lidar across the steps of one walk (persistence)."""
    name: str
    rule: Rule = RULE
    min_pts: int = None                   # override for the sensitivity runs
    prev: list = field(default_factory=list)

    def reset(self):
        self.prev = []

    def step(self, Pw, rings, r, bg, person_xy):
        """Pw: (H, W, 3) world points, rings: (H, W), r: (H, W) ranges, bg:
        (H, W) background; person_xy: true (x, y) or None. Returns the alarms
        of this step, each tagged hit True/False."""
        m = new_mask(r, bg, self.rule.closer)
        alarms, cands = self.step_clusters(clusters(Pw[m], rings[m], self.rule.link), person_xy)
        return alarms, cands, int(m.sum())

    def step_clusters(self, cl, person_xy):
        """Same as step() from precomputed clusters (shared by the
        sensitivity settings, which differ only in min_pts)."""
        cands = [c for c in cl if passes(c, self.rule, self.min_pts)]
        alarms = []
        for c in cands:
            if any(np.linalg.norm(c["centroid"] - p["centroid"]) <= self.rule.persist for p in self.prev):
                hit = person_xy is not None and \
                    float(np.hypot(c["centroid"][0] - person_xy[0], c["centroid"][1] - person_xy[1])) <= self.rule.hit
                alarms.append({**c, "hit": bool(hit)})
        self.prev = cands
        return alarms, cands


def best_case_count(Pw, r, bg, px, py, model_yaw, rule=RULE, rings=None):
    """(returns in the person box, of which not static, box mask[, rings
    covered by the non-static box returns if `rings` is given])."""
    box = in_person_box(Pw, px, py, model_yaw)
    with np.errstate(invalid="ignore"):
        static = np.isfinite(bg) & (np.abs(r - bg) <= rule.static_tol)
    ns = box & ~static
    if rings is None:
        return int(box.sum()), int(ns.sum()), box
    return int(box.sum()), int(ns.sum()), box, sorted(int(x) for x in np.unique(rings[ns]))


def nearest(cl, person_xy):
    """The cluster whose centroid is horizontally nearest the person:
    (n, number of rings, horizontal distance) or (0, 0, None)."""
    if not cl or person_xy is None:
        return 0, 0, None
    d = [float(np.hypot(c["centroid"][0] - person_xy[0], c["centroid"][1] - person_xy[1])) for c in cl]
    i = int(np.argmin(d))
    return cl[i]["n"], len(cl[i]["rings"]), d[i]


def log_arrays(Pw, rings, r, bg, rule=RULE):
    """Per-scan record for offline recomputation: every return at least
    `log_closer` closer than background (or on a no-background beam)."""
    m = new_mask(r, bg, rule.log_closer)
    rows, cols = np.nonzero(m)
    return {"row": rows.astype(np.uint8), "col": cols.astype(np.uint16),
            "xyz": Pw[m].astype(np.float32), "ring": rings[m].astype(np.uint8),
            "range": r[m].astype(np.float32), "bg": bg[m].astype(np.float32)}


def rule_dict(rule=RULE):
    return asdict(rule)
