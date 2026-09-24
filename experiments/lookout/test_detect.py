"""Synthetic tests for detect.py (run in the container: python3 -m pytest lookout/test_detect.py)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detect as D

H, W = 16, 1800


def scene(bg_range=20.0):
    r = np.full((H, W), bg_range)
    r[12:, :] = np.inf                       # upward rings: no return
    return r


def test_background_median_and_none():
    scans = [scene() for _ in range(10)]
    scans[3][0, 0] = 5.0                     # one outlier: median ignores it
    for s in scans[:6]:
        s[1, 1] = np.inf                     # majority no-return -> none
    bg = D.learn_background(scans)
    assert bg[0, 0] == 20.0
    assert np.isinf(bg[1, 1])
    assert np.isinf(bg[12:]).all()


def test_new_mask_threshold_and_none_background():
    bg = scene()
    r = bg.copy()
    r[0, 0] = 19.71                          # 0.29 closer: not new
    r[0, 1] = 19.70                          # 0.30 closer: new
    r[0, 2] = 25.0                           # further: not new
    r[13, 5] = 40.0                          # return where background had none: new
    m = D.new_mask(r, bg, 0.3)
    assert not m[0, 0] and m[0, 1] and not m[0, 2] and m[13, 5]
    assert m.sum() == 2


def blob(center, n, rings, spread=0.1, seed=0):
    rng = np.random.default_rng(seed)
    P = np.asarray(center) + rng.uniform(-spread, spread, (n, 3))
    R = np.array([rings[i % len(rings)] for i in range(n)])
    return P, R


def test_cluster_and_candidate_rule():
    P1, R1 = blob((10, 0, 1), 6, [5, 6])           # passes
    P2, R2 = blob((20, 0, 1), 6, [5])              # one ring only
    P3, R3 = blob((30, 0, 1), 4, [5, 6])           # 4 points
    P = np.vstack([P1, P2, P3]); R = np.concatenate([R1, R2, R3])
    c = D.candidates(P, R)
    assert len(c) == 1 and abs(c[0]["centroid"][0] - 10) < 0.2 and c[0]["n"] == 6
    # the 3-point sensitivity setting takes the 4-point blob too
    assert len(D.candidates(P, R, min_pts=3)) == 2


def test_single_linkage_chains():
    # points 0.45 m apart in a line: one cluster although the ends are 2.7 m apart
    P = np.array([[i * 0.45, 0, 1.0] for i in range(7)])
    assert len(set(D.cluster(P, 0.5))) == 1
    P = np.array([[i * 0.55, 0, 1.0] for i in range(7)])
    assert len(set(D.cluster(P, 0.5))) == 7


def grid_from(P, R):
    """Pack points into (H, W) grids at unique cells (row = ring)."""
    Pw = np.full((H, W, 3), np.nan); rings = np.repeat(np.arange(H)[:, None], W, 1)
    r = np.full((H, W), np.inf)
    col = np.zeros(H, dtype=int)
    for p, ring in zip(P, R):
        Pw[ring, col[ring]] = p; r[ring, col[ring]] = np.linalg.norm(p); col[ring] += 1
    return Pw, rings, r


def test_persistence_and_hit():
    bg = np.full((H, W), np.inf)                  # everything is new
    d = D.Detector("x")
    P, R = blob((10, 0, 1), 8, [5, 6, 7])
    a, c, n = d.step(*grid_from(P, R), bg, (10.0, 0.0))
    assert len(c) == 1 and a == []                # first sighting: no alarm yet
    P, R = blob((10.5, 0, 1), 8, [5, 6, 7], seed=1)
    a, c, n = d.step(*grid_from(P, R), bg, (10.5, 0.0))
    assert len(a) == 1 and a[0]["hit"]            # moved 0.5 m: persists, hit
    P, R = blob((13, 0, 1), 8, [5, 6, 7], seed=2)
    a, c, n = d.step(*grid_from(P, R), bg, (13.0, 0.0))
    assert a == []                                # jumped 2.5 m: no persistence
    P, R = blob((13.2, 0, 1), 8, [5, 6, 7], seed=3)
    a, c, n = d.step(*grid_from(P, R), bg, (20.0, 0.0))
    assert len(a) == 1 and not a[0]["hit"]        # persists but 6.8 m from the person: false alarm


def test_person_box_orientation():
    # model yaw = heading + pi/2; walking along +x, the stride axis (model y) lies along x
    yaw = 0.0 + math.pi / 2
    # 0.40 m ahead is inside (front face at 0.42 m); 0.45 m to the side is not (0.30 m)
    Pw = np.array([[[0.0, 0.0, 1.0], [0.40, 0.0, 1.0], [0.0, 0.45, 1.0], [0.0, 0.0, 1.9]]])
    b = D.in_person_box(Pw, 0.0, 0.0, yaw)[0]
    assert b[0] and b[1] and not b[2] and not b[3]


def test_best_case_ignores_static():
    Pw = np.array([[[0.0, 0.0, 1.0]] * 6])
    r = np.full((1, 6), 10.0)
    bg = np.array([[10.05, 10.05, np.inf, 20.0, 20.0, 20.0]])
    n_all, n_ns, _ = D.best_case_count(Pw, r, bg, 0.0, 0.0, 0.0)
    assert n_all == 6 and n_ns == 4


def test_xy_linkage_spans_rings():
    # two rings 0.7 m apart vertically at the same spot (a person at ~20 m):
    # 3D linkage splits them, horizontal linkage joins them
    P = np.array([[20.0, 0.0 + 0.05 * i, 0.5] for i in range(4)] + [[20.0, 0.0 + 0.05 * i, 1.2] for i in range(4)])
    R = np.array([7] * 4 + [8] * 4)
    assert len(D.clusters(P, R)) == 2
    c = D.clusters(P, R, xy=True)
    assert len(c) == 1 and c[0]["rings"] == [7, 8] and D.passes(c[0])
    assert abs(c[0]["centroid"][2] - 0.85) < 1e-9
