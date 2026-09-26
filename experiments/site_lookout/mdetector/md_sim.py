#!/usr/bin/env python3
"""M-detector on the simulated walks (exploratory, after the results).

For every walk saved with LOOKOUT_FULLSCANS=1 (<run>/fullscans/*.npz) and every
lidar in it, the whole scans go through M-detector (md_offline, parameters
md_vlp16.yaml) in walk order, one frame per walk step, time = walk time. Each
point comes back labelled moving or not; the frame-out labels (after
M-detector's own clustering) are used.

Alarm rule for M-detector (set before any of its output was seen; the parts
that are not M-detector's are the lookout experiment's):
  moving points of one scan are grouped with 0.5 m linkage in the horizontal
  plane; a group of >= MIN_PTS points is a candidate; a candidate within 1 m
  of one in the previous step is an alarm (2 consecutive steps); an alarm
  within 1 m of the walker (horizontal) is a hit, otherwise a false alarm.
Output: <out>/md_first.csv (walk x lidar: first hit), md_alarms.csv, md_steps.csv.
Runs in the mdetector image: md_sim.py <run dir> <out dir> [config]"""
import csv, glob, math, os, subprocess, sys, time
import numpy as np
from scipy.sparse.csgraph import connected_components
from scipy.sparse import coo_matrix
from scipy.spatial import cKDTree

LINK, MIN_PTS, PERSIST, HIT = 0.5, 3, 1.0, 1.0
MD = '/catkin_ws/devel/lib/m_detector/md_offline'
HERE = os.path.dirname(os.path.abspath(__file__))


def groups(P):
    if len(P) == 0:
        return []
    pr = cKDTree(P[:, :2]).query_pairs(LINK, output_type='ndarray')
    n = len(P)
    g = coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(n, n)) if len(pr) else coo_matrix((n, n))
    k, lab = connected_components(g, directed=False)
    return [P[lab == i] for i in range(k)]


def run_md(frames, work):
    """frames: list of (t, R, tr, xyz sensor). Returns per frame bool arrays (frame-out, point-out)."""
    os.makedirs(work, exist_ok=True)
    for f in glob.glob(f'{work}/*.label'):
        os.remove(f)
    with open(f'{work}/frames.bin', 'wb') as fh:
        for t, R, tr, xyz in frames:
            fh.write(np.float64(t).tobytes()); fh.write(np.asarray(R, np.float64).tobytes())
            fh.write(np.asarray(tr, np.float64).tobytes()); fh.write(np.int32(len(xyz)).tobytes())
            fh.write(np.asarray(xyz, np.float32).tobytes())
    subprocess.run([MD, f'{work}/frames.bin', work], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    out = []
    for k in range(len(frames)):
        a = np.fromfile(f'{work}/{k:06d}.label', np.int32) == 251
        b = np.fromfile(f'{work}/{k:06d}_o.label', np.int32) == 251
        out.append((a, b))
    return out


def main():
    run, out = sys.argv[1:3]
    cfg = sys.argv[3] if len(sys.argv) > 3 else f'{HERE}/md_vlp16.yaml'
    os.makedirs(out, exist_ok=True)
    os.environ.update(ROS_HOSTNAME='localhost', ROS_MASTER_URI='http://localhost:11311')
    rc = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(60):
        time.sleep(1)
        if subprocess.run(['rosparam', 'load', cfg]).returncode == 0:
            break
    else:
        raise RuntimeError('no ROS master')
    work = f'{out}/work'
    ff = open(f'{out}/md_first.csv', 'w', newline=''); wf = csv.writer(ff)
    wf.writerow(['walk', 'lidar', 'labels', 'step', 't_s', 'x', 'y', 'dist_m', 'n_false'])
    fa = open(f'{out}/md_alarms.csv', 'w', newline=''); wa = csv.writer(fa)
    wa.writerow(['walk', 'lidar', 'labels', 'k', 't_s', 'hit', 'cx', 'cy', 'n', 'err_xy_m'])
    fs = open(f'{out}/md_steps.csv', 'w', newline=''); ws = csv.writer(fs)
    ws.writerow(['walk', 'lidar', 'k', 'n_pts', 'n_moving_frame', 'n_moving_point', 'moving_near_walker'])
    files = sorted(glob.glob(f'{run}/fullscans/*.npz'))
    for i, f in enumerate(files):
        walk = os.path.basename(f).split('__')[0]
        d = np.load(f)
        st, sxy = d['steps_t'], d['steps_xy']
        for n in d['lidars']:
            r, R, T, dirs = d[f'r_{n}'], d[f'R_{n}'], d[f't_{n}'], d[f'dirs_{n}']
            frames, world = [], []
            for k in range(len(st)):
                ok = r[k] > 0
                xyz = dirs[ok] * (r[k][ok, None] / 100.0)
                frames.append((st[k], R[k], T[k], xyz))
                world.append(xyz @ R[k].T + T[k])
            labs = run_md(frames, work)
            for li, lname in enumerate(('frame', 'point')):
                prev, first, nfalse = [], None, 0
                for k in range(len(st)):
                    mv = world[k][labs[k][li]]
                    cands = [g.mean(0) for g in groups(mv) if len(g) >= MIN_PTS]
                    px, py = sxy[k]
                    for c in cands:
                        if any(math.hypot(c[0] - p[0], c[1] - p[1]) <= PERSIST for p in prev):
                            e = math.hypot(c[0] - px, c[1] - py)
                            hit = e <= HIT
                            wa.writerow([walk, n, lname, k, f'{st[k]:.3f}', int(hit), f'{c[0]:.2f}', f'{c[1]:.2f}',
                                         '', f'{e:.2f}'])
                            if hit and first is None:
                                first = k
                            if not hit:
                                nfalse += 1
                    prev = cands
                    if li == 0:
                        near = int((np.hypot(world[k][:, 0] - px, world[k][:, 1] - py) <= HIT)[labs[k][0]].sum())
                        ws.writerow([walk, n, k, len(world[k]), int(labs[k][0].sum()), int(labs[k][1].sum()), near])
                if first is None:
                    wf.writerow([walk, n, lname, '', '', '', '', '', nfalse])
                else:
                    x, y = sxy[first]
                    wf.writerow([walk, n, lname, first, f'{st[first]:.3f}', f'{x:.3f}', f'{y:.3f}',
                                 f'{math.hypot(x, y):.3f}', nfalse])
            ff.flush(); fa.flush(); fs.flush()
        print(f'[md_sim] {i + 1}/{len(files)} {walk}', flush=True)
    rc.terminate()


if __name__ == '__main__':
    main()
