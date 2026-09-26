#!/usr/bin/env python3
"""Check (after the results, exploratory): layout S1G = S1 with the oak models
replaced by the gt_map's own points. Everything else identical (mulcher,
path, walks, seed, lidar, rules).

Map points inside the ROI that stand more than HMIN above the ground model
(dem.npz of build_site_world.py) are put on the flat plane at their height
above the ground, voxelised at VOX (fine: the map's point spacing is about
5 cm) and drawn as one visual-only mesh, which the gpu lidar sees. Points in
the mulcher's own footprint (radius CLEAR_R, below CLEAR_Z) are removed: the
mulcher cannot stand inside them.
Usage: gtmap_world.py <gt_map.ply> <world dir (dem.npz)> <worlds dir> [vox]
(writes <worlds>/gtmap_S1G/objects.obj, lookout_S1G.sdf, forest_S1G.csv,
config/S1G.yaml)"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build_site_world as b

MULCHER_MAP = (-2.0, 0.6)
HMIN = b.HMIN            # 0.35 m, as for the site world
CLEAR_R, CLEAR_Z = 2.3, 2.4
MOUNT = '/runs/lookout/worlds'   # where docker_run.sh mounts <worlds dir>


def main():
    ply, wdir, worlds = sys.argv[1:4]
    b.VOX = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05
    tag = 'S1G'
    P = b.load_ply(ply)
    P = P[b.in_roi(P[:, 0], P[:, 1], 0.0)]
    D = np.load(os.path.join(wdir, 'dem.npz'))
    dem, org, cell = D['dem'], D['org'], float(D['cell'])
    ix = ((P[:, 0] - org[0]) / cell).astype(int); iy = ((P[:, 1] - org[1]) / cell).astype(int)
    h = P[:, 2] - dem[ix, iy]
    Q = np.c_[P[:, 0] - MULCHER_MAP[0], P[:, 1] - MULCHER_MAP[1], h][h > HMIN]
    near = (np.hypot(Q[:, 0], Q[:, 1]) < CLEAR_R) & (Q[:, 2] < CLEAR_Z)
    print(f'ROI points {len(P)}, above {HMIN} m {len(Q)}, removed at the mulcher {near.sum()}')
    Q = Q[~near]
    K = np.unique(np.floor(Q / b.VOX).astype(np.int64), axis=0)
    V, F = b.voxel_mesh(K, np.zeros(3))
    mdir = os.path.join(worlds, f'gtmap_{tag}'); os.makedirs(mdir, exist_ok=True)
    b.write_obj(os.path.join(mdir, 'objects.obj'), V, F)
    print(f'voxels {len(K)} at {b.VOX} m, triangles {len(F)}')
    model = (f'\n    <model name="gtmap"><static>true</static><link name="l"><visual name="v"><geometry><mesh>'
             f'<uri>{MOUNT}/gtmap_{tag}/objects.obj</uri></mesh></geometry><material><ambient>0.3 0.4 0.25 1</ambient>'
             f'<diffuse>0.35 0.5 0.3 1</diffuse></material></visual></link></model>')
    w = open(f'{worlds}/lookout_S1N.sdf').read().replace('<world name="lookout_S1N">', f'<world name="lookout_{tag}">')
    w = w.replace('\n  </world>', model + '\n  </world>', 1)
    assert 'name="gtmap"' in w
    open(f'{worlds}/lookout_{tag}.sdf', 'w').write(w)
    open(f'{worlds}/forest_{tag}.csv', 'w').write('x,y\n')
    c = open(f'{HERE}/config/S1N.yaml').read().split('\n', 1)[1]
    c = c.replace('layout: S1N\n', f'layout: {tag}\n').replace('world: lookout_S1N\n', f'world: lookout_{tag}\n') \
         .replace('csv: forest_S1N.csv', f'csv: forest_{tag}.csv') \
         .replace('source: site_lookout/trees.py (all oaks)', f'source: gt_map points (gtmap_world.py, {b.VOX} m voxels)')
    open(f'{HERE}/config/{tag}.yaml', 'w').write(f'# S1 with the gt_map points in place of the oaks (gtmap_world.py)\n' + c)
    # points on the walked lane (+-1.5 m of path S), for the record
    a = np.array([-57.228, -42.481]) - MULCHER_MAP; e = np.array([68.87, 18.342]) - MULCHER_MAP
    u = (e - a) / np.linalg.norm(e - a)
    rel = Q[:, :2] - a; off = np.abs(rel[:, 0] * u[1] - rel[:, 1] * u[0])
    print(f'map points on the lane (within 1.5 m of the path line): {(off < 1.5).sum()}, '
          f'within 3 m: {(off < 3.0).sum()}')


if __name__ == '__main__':
    main()
