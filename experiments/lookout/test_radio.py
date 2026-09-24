"""Checks of the radio port against the node's equations (pytest)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import radio as R


def test_static_budget():
    assert abs(R.link((0, 0, 0), (20, 0, 0), [])["snr_db"] - 55.81) < 0.01
    assert R.link((0, 0, 0), (20, 0, 0), [])["linked"]
    one = R.link((0, 0, 0), (20, 0, 0), [(10, 0.2)])
    assert one["trees_on_link"] == 1 and not one["linked"]
    assert not R.link((0, 0, 0), (30.5, 0, 0), [])["linked"]        # past the horizon


def test_hysteresis():
    h = []
    bw = 0.0
    for s in [30.0] * 7:                  # first 7 samples: straight to a tier
        h.append(s); bw = R.next_bandwidth(h, bw)
    assert bw == 72.0
    for s in [0.0, 0.0]:                  # two lows: stays up
        h.append(s); h[:] = h[-8:]; bw = R.next_bandwidth(h, bw)
    assert bw == 72.0
    h.append(0.0); h[:] = h[-8:]; bw = R.next_bandwidth(h, bw)
    assert bw == 28.9                     # third low: one tier down
    # from 0 back up needs 8 of 8 above 2 dB: 1.6 s at 5 Hz
    h, bw = [-50.0] * 8, 0.0
    ups = []
    for i in range(10):
        h.append(40.0); h[:] = h[-8:]; bw = R.next_bandwidth(h, bw); ups.append(bw > 0)
    assert ups.index(True) == 7


def test_link_comes_up_inside_horizon():
    L = R.Link(seed=1)
    up = [L.sample((0, 0, 0), (15, 0, 0), [])["connected"] for _ in range(50)]
    assert all(up)                        # 55 dB margin: the fade never matters
    L = R.Link(seed=1)
    up = [L.sample((0, 0, 0), (40, 0, 0), [])["connected"] for _ in range(50)]
    assert not any(up)
