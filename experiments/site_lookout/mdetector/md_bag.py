#!/usr/bin/env python3
"""M-detector on real lidar frames from a site bag, with the robot standing
still (bag_frames.py segments, identity pose). For each segment: per-point
moving labels (frame-out), grouped with 0.5 m linkage in 3-D; groups of
>= MIN_PTS points that persist (within 1 m of one in the previous frame) are
events. Events are chained into tracks (within 1.5 m and 0.5 s).
Usage (mdetector image): md_bag.py <segment dir> <config> <out dir>
Writes <out>/events.csv and tracks.csv."""
import csv, glob, math, os, subprocess, sys, time
import numpy as np
from scipy.sparse.csgraph import connected_components
from scipy.sparse import coo_matrix
from scipy.spatial import cKDTree

MIN_PTS, LINK, PERSIST = 10, 0.5, 1.0
MD = '/catkin_ws/devel/lib/m_detector/md_offline'


def groups(P):
    if len(P) == 0:
        return []
    pr = cKDTree(P).query_pairs(LINK, output_type='ndarray')
    n = len(P)
    g = coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(n, n)) if len(pr) else coo_matrix((n, n))
    k, lab = connected_components(g, directed=False)
    return [P[lab == i] for i in range(k)]


def read_frames(path):
    with open(path, 'rb') as fh:
        while True:
            h = fh.read(8 + 72 + 24 + 4)
            if len(h) < 108:
                return
            n = int(np.frombuffer(h[104:108], np.int32)[0])
            yield np.frombuffer(h[:8], np.float64)[0], np.frombuffer(fh.read(12 * n), np.float32).reshape(-1, 3)


def main():
    seg, cfg, out = sys.argv[1:4]
    os.makedirs(out, exist_ok=True)
    os.environ.update(ROS_HOSTNAME='localhost', ROS_MASTER_URI='http://localhost:11311')
    rc = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(60):
        time.sleep(1)
        if subprocess.run(['rosparam', 'load', cfg]).returncode == 0:
            break
    ev = open(f'{out}/events.csv', 'w', newline=''); we = csv.writer(ev)
    we.writerow(['seg', 'k', 't', 'cx', 'cy', 'cz', 'n', 'dx', 'dy', 'dz', 'range'])
    events = []
    for b in sorted(glob.glob(f'{seg}/seg_*.bin')):
        name = os.path.basename(b)[:-4]
        work = f'{out}/work_{name}'
        os.makedirs(work, exist_ok=True)
        subprocess.run([MD, b, work], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        prev = []
        for k, (t, P) in enumerate(read_frames(b)):
            lab = np.fromfile(f'{work}/{k:06d}.label', np.int32) == 251
            cands = [g for g in groups(P[lab]) if len(g) >= MIN_PTS]
            cur = []
            for g in cands:
                c = g.mean(0)
                cur.append(c)
                if any(np.linalg.norm(c[:2] - p[:2]) <= PERSIST for p in prev):
                    ext = g.max(0) - g.min(0)
                    row = [name, k, round(t, 2), *np.round(c, 2), len(g), *np.round(ext, 2), round(float(np.linalg.norm(c)), 1)]
                    we.writerow(row); events.append(row)
            prev = cur
        for f in glob.glob(f'{work}/*.label'):
            os.remove(f)
        os.rmdir(work)
        ev.flush()
        print(f'[md_bag] {name} events so far {len(events)}', flush=True)
    # tracks
    tracks = []
    for e in events:
        for tr in tracks:
            l = tr[-1]
            if l[0] == e[0] and 0 < e[2] - l[2] <= 0.5 and math.hypot(e[3] - l[3], e[4] - l[4]) <= 1.5:
                tr.append(e); break
        else:
            tracks.append([e])
    with open(f'{out}/tracks.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['track', 'seg', 't0', 't1', 'n_events', 'x0', 'y0', 'x1', 'y1', 'path_m', 'range_min', 'range_max',
                    'n_pts_med', 'height_med', 'width_med', 'z_med'])
        for i, tr in enumerate(tracks):
            a = np.array([e[2:] for e in tr], float)
            path = float(np.sum(np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2]))))
            w.writerow([i, tr[0][0], a[0, 0], a[-1, 0], len(tr), a[0, 1], a[0, 2], a[-1, 1], a[-1, 2], round(path, 1),
                        a[:, 8].min(), a[:, 8].max(), np.median(a[:, 4]), np.median(a[:, 7]),
                        round(float(np.median(np.hypot(a[:, 5], a[:, 6]))), 2), np.median(a[:, 3])])
    rc.terminate()


if __name__ == '__main__':
    main()
