#!/usr/bin/env python3
"""Register real lidar frames to the site map (yaw grid + point-to-point ICP).

Usage: register_frames.py <map.ply> <frames.npz> <out.json> [frame indices...]
frames.npz as written by grab_frames.py (ts, f0, f1, ... in the sensor frame).
Each frame is voxel-filtered (0.3 m, 1.5-60 m range); starts are the origin
with 18 yaws; best by inlier share (< 0.3 m), then refined.
Writes the sensor pose (x y z roll pitch yaw) and fit per frame.
"""
import json, sys
import numpy as np
from scipy.spatial import cKDTree


def load_ply(p):
    with open(p, 'rb') as f:
        h = b''
        while not h.endswith(b'end_header\n'):
            h += f.readline()
        return np.frombuffer(f.read(), dtype='<f4').reshape(-1, 4)[:, :3].astype(np.float64)


def vox(p, r):
    _, i = np.unique(np.floor(p / r).astype(np.int64), axis=0, return_index=True)
    return p[i]


def rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def icp(T, M, P, R, t, iters, dmax=(3, 1.5, 0.8, 0.5)):
    for it in range(iters):
        d = dmax[min(it // 10, len(dmax) - 1)]
        Q = P @ R.T + t
        dist, idx = T.query(Q, distance_upper_bound=d)
        ok = np.isfinite(dist)
        A, B = Q[ok], M[idx[ok]]
        ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
        dR = Vt.T @ D @ U.T
        R = dR @ R; t = dR @ (t - ca) + cb
    dist, _ = T.query(P @ R.T + t)
    return R, t, float(np.median(dist)), float(np.mean(dist < 0.3))


def main():
    mp, fp, out = sys.argv[1:4]
    idxs = [int(i) for i in sys.argv[4:]] or [0]
    M = vox(load_ply(mp), 0.3); T = cKDTree(M)
    z = np.load(fp); res = []
    for fi in idxs:
        P = z[f'f{fi}'].astype(np.float64)
        r = np.linalg.norm(P, axis=1); P = vox(P[(r > 1.5) & (r < 60)], 0.3)
        best = max((icp(T, M, P, rotz(y), np.zeros(3), 20) for y in np.deg2rad(np.arange(0, 360, 20))),
                   key=lambda b: b[3])
        R, t, med, inl = icp(T, M, P, best[0], best[1], 40)
        rpy = [float(np.arctan2(R[2, 1], R[2, 2])), float(-np.arcsin(R[2, 0])), float(np.arctan2(R[1, 0], R[0, 0]))]
        res.append(dict(frame=fi, stamp=int(z['ts'][fi]), xyz=t.tolist(), rpy=rpy, R=R.tolist(),
                        median_nn=med, inlier_03=inl))
        print(res[-1]['frame'], np.round(t, 2), np.round(np.degrees(rpy), 1), round(med, 3), round(inl, 3), flush=True)
    json.dump(res, open(out, 'w'), indent=1)


if __name__ == '__main__':
    main()
