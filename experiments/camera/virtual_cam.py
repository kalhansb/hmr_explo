#!/usr/bin/env python3
"""Virtual NDVI camera: the camera-experiment camera, modelled geometrically on a run's
ground-truth poses, so runs without a rendered camera can be scored too.

Container python (numpy). Usage:
  virtual_cam.py <run_dir> <lesions.json> <world.sdf> [--hz 2] [--out virtual_cam.csv]
Needs <run>/gt_poses.npz (gt_poses.py). Writes <run>/<out>: t,robot,label,npix
(one row per frame per lesion with npix >= 1), same shape as the rendered
cam/counts.csv, so the same scorer reads both.

Camera (PLAN.md): base_link + (0.30, 0, 0.55), forward, 640 x 480,
fx = fy = 732.5, clip 0.1-40 m. Full robot roll/pitch/yaw from the odometry.
Lesion pixel area = its projected quad clipped to the image, times the
fraction of 9 sample points on its face with a clear line of sight.
Back-facing lesions (camera behind the plate's plane) count 0.
Occluders (vertical cylinders, lesion heights only): every oak trunk
r 0.40 m about its axis (the lesion's own tree excluded: its self-occlusion is
the facing test), the pines' foliage r 1.5 m, stones r 1.5 m.
"""
import json, math, os, re, sys
import numpy as np

W, H, F, CX, CY = 640, 480, 732.5, 320.0, 240.0
MOUNT = np.array([0.30, 0.0, 0.55])
NEAR, FAR = 0.1, 40.0


def rotm(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def clip_poly(poly):
    """Sutherland-Hodgman against the image rectangle."""
    def clip(pts, inside, inter):
        out = []
        for i in range(len(pts)):
            a, b = pts[i - 1], pts[i]
            ia, ib = inside(a), inside(b)
            if ib:
                if not ia: out.append(inter(a, b))
                out.append(b)
            elif ia:
                out.append(inter(a, b))
        return out

    def ix(x0):
        return lambda a, b: (x0, a[1] + (b[1] - a[1]) * (x0 - a[0]) / (b[0] - a[0]))

    def iy(y0):
        return lambda a, b: (a[0] + (b[0] - a[0]) * (y0 - a[1]) / (b[1] - a[1]), y0)
    pts = [tuple(p) for p in poly]
    for inside, inter in ((lambda p: p[0] >= 0, ix(0.0)), (lambda p: p[0] <= W, ix(float(W))),
                          (lambda p: p[1] >= 0, iy(0.0)), (lambda p: p[1] <= H, iy(float(H)))):
        pts = clip(pts, inside, inter)
        if not pts:
            return 0.0
    a = 0.0
    for i in range(len(pts)):
        a += pts[i - 1][0] * pts[i][1] - pts[i][0] * pts[i - 1][1]
    return abs(a) / 2


def occluders(world, axis_off):
    s = open(world).read()
    occ = []   # (x, y, r, oak_name or None)
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', s, re.S):
        name, body = m.groups()
        if not (name.startswith("Oak tree") or name.startswith("Stone")):
            continue
        mp = re.search(r'</link>\s*<pose>([^<]+)</pose>', body) or re.search(r'<pose>([^<]+)</pose>', body)
        x, y = map(float, mp.group(1).split()[:2])
        if name.startswith("Oak tree"):
            occ.append((x + axis_off[0], y + axis_off[1], 0.40, name))
        elif name.startswith("Stone"):
            occ.append((x, y, 1.5, None))
    for n, x, y in re.findall(r'<include><name>([^<]+)</name><uri>model://cmu_pine_tree</uri><pose>([-0-9.]+) ([-0-9.]+)', s):
        occ.append((float(x), float(y), 1.5, None))
    return occ


def main():
    run, lj, world = sys.argv[1:4]
    hz = float(sys.argv[sys.argv.index("--hz") + 1]) if "--hz" in sys.argv else 2.0
    outname = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else "virtual_cam.csv"
    L = json.load(open(lj))
    les = L["lesions"]
    occ = occluders(world, L["axis_model"])
    oxy = np.array([(o[0], o[1]) for o in occ]); orad = np.array([o[2] for o in occ])
    oname = [o[3] for o in occ]
    corners = np.array([l["corners_world"] for l in les])          # (N,4,3)
    face = np.array([l["face_world"] for l in les]); nrm = np.array([l["normal"] for l in les])
    # 9 samples on each face: centre, 4 corners pulled in 20 %, 4 edge midpoints pulled in 20 %
    c4 = corners
    mids = (c4 + np.roll(c4, -1, axis=1)) / 2
    samp = np.concatenate([face[:, None], face[:, None] + 0.8 * (c4 - face[:, None]),
                           face[:, None] + 0.8 * (mids - face[:, None])], axis=1)   # (N,9,3)
    own = [[i for i, n in enumerate(oname) if n == l["oak"]] for l in les]
    mask_own = np.ones((len(les), len(occ)), bool)
    for k, ids in enumerate(own):
        mask_own[k, ids] = False
    P = np.load(os.path.join(run, "gt_poses.npz"))
    # --frames <frames.csv>: evaluate at the rendered frame times (poses
    # linearly interpolated, yaw unwrapped) instead of every 1/hz s.
    fr = sys.argv[sys.argv.index("--frames") + 1] if "--frames" in sys.argv else None
    rows = 0
    with open(os.path.join(run, outname), "w") as f:
        f.write("t,robot,label,npix,vis\n")
        for rb in ("atlas", "bestla"):
            t, xyz, rpy = P[f"{rb}_t"], P[f"{rb}_xyz"], P[f"{rb}_rpy"]
            if fr:
                ft = np.array(sorted({float(l.split(",")[0]) for l in open(fr).read().splitlines()[1:]
                                      if l.split(",")[1] == rb}))
                ft = ft[(ft >= t[0]) & (ft <= t[-1])]
                ang = np.unwrap(rpy, axis=0)
                xyz = np.stack([np.interp(ft, t, xyz[:, c]) for c in range(3)], 1)
                rpy = np.stack([np.interp(ft, t, ang[:, c]) for c in range(3)], 1)
                t = ft
            last = -1e9
            for i in range(len(t)):
                if not fr and t[i] - last < 1.0 / hz - 1e-6:
                    continue
                last = t[i]
                R = rotm(*rpy[i])
                cam = xyz[i] + R @ MOUNT
                d = face - cam
                dist = np.linalg.norm(d, axis=1)
                facing = (np.einsum("ij,ij->i", -d, nrm) > 0)
                pc = (d @ R)                                       # camera frame (fwd, left, up)
                cand = np.nonzero(facing & (dist < FAR) & (pc[:, 0] > NEAR))[0]
                if len(cand) == 0:
                    continue
                # cheap FOV prefilter on the centre (with margin)
                u = CX - F * pc[cand, 1] / pc[cand, 0]; v = CY - F * pc[cand, 2] / pc[cand, 0]
                cand = cand[(u > -60) & (u < W + 60) & (v > -60) & (v < H + 60)]
                for k in cand:
                    q = (corners[k] - cam) @ R
                    if (q[:, 0] <= NEAR).any():
                        continue
                    poly = np.c_[CX - F * q[:, 1] / q[:, 0], CY - F * q[:, 2] / q[:, 0]]
                    area = clip_poly(poly)
                    if area <= 0:
                        continue
                    # occlusion: 2D segment (cam -> sample) vs occluder circles
                    s2 = samp[k][:, :2]; c2 = cam[:2]
                    seg = s2 - c2                                   # (9,2)
                    rel = oxy[mask_own[k]] - c2                     # (M,2)
                    rr = orad[mask_own[k]]
                    L2 = (seg ** 2).sum(1)                          # (9,)
                    tt = np.clip((rel @ seg.T) / L2, 0, 1)          # (M,9)
                    closest = tt[..., None] * seg[None]             # (M,9,2)
                    dd = np.linalg.norm(rel[:, None, :] - closest, axis=2)
                    blocked = ((dd < rr[:, None]) & (tt > 0) & (tt < 1)).any(0)
                    vis = 1.0 - blocked.mean()
                    n = area * vis
                    if n >= 1:
                        f.write(f"{t[i]:.3f},{rb},{les[k]['label']},{n:.1f},{vis:.2f}\n")
                        rows += 1
    print(os.path.basename(run.rstrip("/")), "rows", rows)


if __name__ == "__main__":
    main()
