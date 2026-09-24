"""Port of the link model in hmr_sim/src/hmr_comms_sim_node.cpp (ComputePair +
NextBandwidth), parameter defaults unchanged:

  path loss = p0 + 20 log10(d) + 70 dB per trunk in the Fresnel corridor
              + AR(1) shadow fade (stationary sigma 4.8 dB, alpha 0.9)
              (+200 dB past the 30 m radio horizon; d is 3D)
  SNR = tx power - path loss - noise floor
  rate tier {72, 28.9, 7.2, 0} Mbps from the last 8 SNR samples (3-of-8 down,
  8-of-8 up; the first 7 samples map straight to a tier); connected = tier > 0.

`link()` is the static budget without the fade (a single look).
`Link` is the stateful model sampled at link_rate_hz (5 Hz) by radio_node.py.
The fade uses Python's RNG, not the node's std::mt19937: same process and
parameters, not the same sample sequence.
"""
import math
import random

P0_DB, TX_DBM, NOISE_DBM = 49.17, 30.0, -101.0
TREE_DB, TREE_R, FREQ, MAX_RANGE, MIN_SNR = 70.0, 0.3, 2.4e9, 30.0, 2.0
FADE_SIGMA_DB, FADE_ALPHA, LINK_RATE_HZ = 4.8, 0.9, 5.0


def _seg_dist2(a, b, p):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    ex, ey = ax + t * dx - px, ay + t * dy - py
    return ex * ex + ey * ey


def budget(a, b, trees, fade_db=0.0):
    """(distance, trees on link, path loss, snr) for antennas a, b (x, y, z)."""
    d = max(math.dist(a, b), 0.1)
    width = TREE_R + 0.5 * math.sqrt(3.0e8 / FREQ * d)
    n = sum(1 for t in trees if _seg_dist2(a[:2], b[:2], t) < width * width)
    pl = P0_DB + 20.0 * math.log10(d) + n * TREE_DB + fade_db + (200.0 if d > MAX_RANGE else 0.0)
    return d, n, pl, TX_DBM - pl - NOISE_DBM


def link(a, b, trees):
    """Static budget, no fade. Returns dict(linked, snr_db, distance_m, trees_on_link)."""
    d, n, _, snr = budget(a, b, trees)
    return {"linked": snr > MIN_SNR, "snr_db": snr, "distance_m": d, "trees_on_link": n}


def next_bandwidth(history, cur):
    """NextBandwidth: `history` already holds the newest sample (<= 8 kept)."""
    snr = history[-1]
    if len(history) < 8:
        return 72.0 if snr > 25.0 else 28.9 if snr > 11.0 else 7.2 if snr > 2.0 else 0.0
    thr = 25.0 if cur >= 72.0 else 11.0 if cur >= 28.9 else 2.0
    high = sum(1 for s in history if s > thr)
    low = sum(1 for s in history if s < thr)
    if cur >= 72.0:
        return 28.9 if low >= 3 else 72.0
    if cur >= 28.9:
        return 72.0 if high == 8 else 7.2 if low >= 3 else 28.9
    if cur >= 7.2:
        return 28.9 if high == 8 else 0.0 if low >= 3 else 7.2
    return 7.2 if high == 8 else 0.0


class Link:
    """One robot pair, sampled at LINK_RATE_HZ (ComputePair)."""

    def __init__(self, seed=42):
        self.rng = random.Random(seed)
        self.fade_db = 0.0
        self.history = []
        self.bandwidth = 0.0

    def sample(self, a, b, trees):
        self.fade_db = FADE_ALPHA * self.fade_db + math.sqrt(1.0 - FADE_ALPHA ** 2) * self.rng.gauss(0.0, FADE_SIGMA_DB)
        d, n, pl, snr = budget(a, b, trees, self.fade_db)
        self.history.append(snr)
        if len(self.history) > 8:
            self.history.pop(0)
        self.bandwidth = next_bandwidth(self.history, self.bandwidth)
        return {"connected": self.bandwidth > 0.0, "bandwidth_mbps": self.bandwidth, "snr_db": snr,
                "distance_m": d, "trees_on_link": n, "fade_db": self.fade_db}


def load_trees(path):
    with open(path) as f:
        next(f)
        return [tuple(map(float, l.split(",")[:2])) for l in f if l.strip()]
