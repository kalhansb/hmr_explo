#!/usr/bin/env python3
"""Smoke probe (inside the container, sim + bridge already up).

Checks, each printed as a labelled block and saved to <out>/smoke.json:
  layout    cloud shape / fields / frame of every lidar
  stepping  paused world + multi_step: is the first scan after a move stamped
            after it, and how long does it take (wall)
  selfret   Husky and mulcher: returns under 1 m (the near plane is 0.5 m)
  head      mulcher head: per channel, blocked (< 3 m) fraction by azimuth
  height    person height from the vertical 'gauge' lidar
  person    returns inside the person's box from each lidar at a few spots
  (mulcher set_pose: see setpose3.py)
"""
import argparse, json, math, os, sys, time
import numpy as np
import rclpy

sys.path.insert(0, "/lookout")
import gz
from scans import ScanNode

W = "lookout_smoke"
LIDARS = {"lk0": "/lk0/velodyne_points", "mulcher": "/mulcher/velodyne_points",
          "gauge": "/gauge/velodyne_points"}


class Probe(ScanNode):
    def __init__(self):
        super().__init__(W, LIDARS, odoms=["lk0"], name="smoke_probe")

    def fresh(self, keys):
        return self.step_fresh(keys)[0]


def husky_sensor_pose(odom):
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    R = gz.quat_to_R(q.x, q.y, q.z, q.w)
    t = np.array([p.x, p.y, p.z]) + R @ np.array([0.0012, 0.0, 0.716])
    return R, t


def to_world(d, R, t):
    P = np.stack([d["x"], d["y"], d["z"]], -1).astype(np.float64)
    return P @ R.T + t


def person_box_count(Pw, px, py, yaw, rings):
    # person frame: faces -y of the mesh; box from the scaled mesh extents + 5 cm
    s = 0.91748
    c, sn = math.cos(-yaw), math.sin(-yaw)
    dx, dy = Pw[..., 0] - px, Pw[..., 1] - py
    lx, ly = c * dx - sn * dy, sn * dx + c * dy
    z = Pw[..., 2]
    fin = np.isfinite(z)
    inside = fin & (np.abs(lx) <= 0.28 * s + 0.05) & (ly >= -0.41 * s - 0.05) & \
        (ly <= 0.48 * s + 0.05) & (z >= 0.05) & (z <= 1.80)
    rr = sorted(set(int(r) for r in rings[inside])) if inside.any() else []
    return int(inside.sum()), rr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rclpy.init()
    n = Probe()
    rep = {}
    assert n.wait_all(odoms=["lk0"]), f"missing topics: {[k for k in LIDARS if k not in n.last]}"
    # --- layout
    rep["layout"] = {}
    for k, m in n.last.items():
        rep["layout"][k] = {"h": m.height, "w": m.width, "frame": m.header.frame_id,
                            "fields": [(f.name, f.offset, f.datatype) for f in m.fields],
                            "point_step": m.point_step}
    print("LAYOUT", json.dumps(rep["layout"], indent=1))

    # let the Husky settle, then pause
    time.sleep(3.0)
    gz.pause(W, True)
    n.drain(0.5)

    # --- stepping
    gz.set_pose(W, "person", 1000, 1000, 0, 0)
    sc, info = n.step_fresh(list(LIDARS))
    rep["stepping"] = info
    print("STEPPING", rep["stepping"])
    base = {k: gz.decode(m) for k, m in sc.items()}

    # --- self-returns
    rep["selfret"] = {}
    for k in ("lk0", "mulcher"):
        r = base[k]["range"]
        fin = np.isfinite(r)
        rep["selfret"][k] = {"finite": int(fin.sum()), "min_range": float(np.nanmin(np.where(fin, r, np.nan))),
                             "n_below_1m": int((fin & (r < 1.0)).sum()),
                             "n_below_0.6m": int((fin & (r < 0.6)).sum()),
                             "rings_below_1m": sorted(set(int(x) for x in base[k]["ring"][fin & (r < 1.0)]))
                             if "ring" in base[k] else None}
    print("SELFRET", rep["selfret"])

    # --- head: per ring, azimuth of each column from the finite points
    d = base["mulcher"]
    az = np.degrees(np.arctan2(d["y"], d["x"]))
    col_az = np.nanmedian(np.where(np.isfinite(az), az, np.nan), axis=0)
    ring_el = np.degrees(np.arctan2(d["z"], np.hypot(d["x"], d["y"])))
    ring_el = np.nanmedian(np.where(np.isfinite(ring_el), ring_el, np.nan), axis=1)
    head = {}
    for i in range(d["range"].shape[0]):
        blk = np.isfinite(d["range"][i]) & (d["range"][i] < 3.0)
        # widest contiguous blocked span containing azimuth 0
        cols = np.where(blk)[0]
        span = [float(np.nanmin(col_az[cols])), float(np.nanmax(col_az[cols]))] if len(cols) else None
        within30 = (np.abs(col_az) <= 30.0)
        head[i] = {"el_deg": float(ring_el[i]) if np.isfinite(ring_el[i]) else None,
                   "blocked_frac_within_30": float(blk[within30].mean()),
                   "blocked_frac_within_10": float(blk[np.abs(col_az) <= 10.0].mean()),
                   "near_hits_az_range": span,
                   "near_range_median": float(np.nanmedian(np.where(blk, d["range"][i], np.nan))) if blk.any() else None}
    rep["head"] = head
    rep["col_az_first_last"] = [float(col_az[0]), float(col_az[-1])]
    print("HEAD")
    for i, h in head.items():
        print(f"  ring {i:2d} el {h['el_deg']}: blocked within +-30: {h['blocked_frac_within_30']:.3f}, "
              f"+-10: {h['blocked_frac_within_10']:.3f}, near-hit az span {h['near_hits_az_range']}, "
              f"median near range {h['near_range_median']}")

    # --- height gauge: person 3 m in +x of the gauge (0, 30)
    gz.set_pose(W, "person", 3.0, 30.0, 0.0, math.pi / 2)
    g = gz.decode(n.fresh(["gauge"])["gauge"])
    # gauge sensor at (0, 30, 1.0) rolled +90 deg: sensor (x, y, z) -> world (x, -z, y)
    gx, gzz = g["x"].ravel(), g["y"].ravel() + 1.0
    sel = np.isfinite(gx) & (gx > 2.5) & (gx < 3.6)
    rep["height"] = {"n": int(sel.sum()), "z_max": float(gzz[sel].max()) if sel.any() else None,
                     "z_min": float(gzz[sel].min()) if sel.any() else None}
    print("HEIGHT", rep["height"])

    # --- person visibility from lk0 and the mulcher
    R_h, t_h = husky_sensor_pose(n.odom["lk0"])
    rep["husky_sensor_pose"] = t_h.tolist()
    spots = [(0.0, -30.0), (0.0, -45.0), (-20.0, 0.0), (-30.0, 5.0), (15.0, 1.0), (0.0, 12.0)]
    rep["person"] = []
    for (px, py) in spots:
        yaw = math.atan2(py, px) + math.pi / 2       # walking towards the mulcher... faces -y rotated
        yaw = math.atan2(-py, -px) + math.pi / 2
        gz.set_pose(W, "person", px, py, 0.0, yaw)
        sc = n.fresh(["lk0", "mulcher"])
        row = {"spot": [px, py]}
        for k, (R, t) in {"lk0": (R_h, t_h), "mulcher": (np.eye(3), np.array([0, 0, 2.1]))}.items():
            dd = gz.decode(sc[k])
            Pw = to_world(dd, R, t)
            cnt, rings = person_box_count(Pw, px, py, yaw, dd["ring"])
            row[k] = {"dist": float(math.hypot(px - t[0], py - t[1])), "n_box": cnt, "rings": rings}
        rep["person"].append(row)
        print("PERSON", row)

    json.dump(rep, open(os.path.join(a.out, "smoke.json"), "w"), indent=1, default=str)
    np.savez_compressed(os.path.join(a.out, "base_clouds.npz"),
                        **{f"{k}_{f}": v[f] for k, v in base.items() for f in ("x", "y", "z", "range")})
    rep["retries"] = getattr(n, "retries", 0)
    print("RETRIES", rep["retries"])
    print("SMOKE DONE")


if __name__ == "__main__":
    main()
