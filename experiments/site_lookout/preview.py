#!/usr/bin/env python3
"""Top view and oblique view of the built ROI world (from dem.npz).
Usage: preview.py <world dir> <out.png>"""
import sys
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

d = np.load(f'{sys.argv[1]}/dem.npz'); out = sys.argv[2]
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from build_site_world import in_roi, dem_at, ROI_ORIGIN, ROI_THETA, ROI_SIZE
C = d['vorg'] + (d['K'] + 0.5) * d['vox']
C = C[in_roi(C[:, 0], C[:, 1])]
h = C[:, 2] - dem_at(d['dem'], d['org'], C[:, 0], C[:, 1])
c, s = np.cos(np.radians(ROI_THETA)), np.sin(np.radians(ROI_THETA))
corn = np.array([[0, 0], [ROI_SIZE[0], 0], [ROI_SIZE[0], ROI_SIZE[1]], [0, ROI_SIZE[1]], [0, 0]])
corn = np.c_[ROI_ORIGIN[0] + c * corn[:, 0] - s * corn[:, 1], ROI_ORIGIN[1] + s * corn[:, 0] + c * corn[:, 1]]
fig = plt.figure(figsize=(16, 10))
ax = fig.add_subplot(2, 1, 1)
o = np.argsort(h)
sc = ax.scatter(C[o, 0], C[o, 1], c=np.clip(h[o], 0, 15), s=0.4, cmap='viridis')
ax.plot(corn[:, 0], corn[:, 1], 'r-', lw=1); ax.plot(0, 0, 'k+', ms=12)
ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
ax.set_title('Simulated world, top view (height above ground, m; + = map origin)')
fig.colorbar(sc, ax=ax, shrink=0.8)
ax3 = fig.add_subplot(2, 1, 2, projection='3d')
k = np.random.default_rng(0).choice(len(C), min(len(C), 60000), replace=False)
ax3.scatter(C[k, 0], C[k, 1], C[k, 2], c=np.clip(h[k], 0, 15), s=0.3, cmap='viridis')
xs = d['org'][0] + (np.arange(d['dem'].shape[0]) + 0.5); ys = d['org'][1] + (np.arange(d['dem'].shape[1]) + 0.5)
X, Y = np.meshgrid(xs, ys, indexing='ij'); m = in_roi(X, Y)
ax3.scatter(X[m], Y[m], d['dem'][m], c='tan', s=1)
ax3.view_init(elev=30, azim=-120); ax3.set_box_aspect((100, 60, 20))
ax3.set_title('Oblique view from the south-west (ground in tan)')
fig.tight_layout(); fig.savefig(out, dpi=110)
