#!/usr/bin/env python3
"""Find the trees in the ROI of the site map and place our oak and pine
models at them on a flat plane.

Trees: canopy height model (CHM) = highest map point above the ground per
CELL cell; tree tops = local maxima of the smoothed CHM over a TOP_W window,
at least H_MIN high; every canopy cell (CHM > H_CANOPY) within R_MAX goes to
its nearest top. Per tree: height = top, crown radius = sqrt(crown area / pi).
Crowns more than ELONG_MAX times longer than wide are dropped (walls).
Type: pine when crown radius / height < PINE_RATIO (narrow), else oak.
Model scale: horizontal = crown radius / model crown radius, vertical =
height / model height (the horizontal scale clamped to XY_CLAMP).

Writes <world dir>/trees.csv, trees.sdf (flat world) and trees.png.
Usage: trees.py <map.ply> <world dir>   (world dir holds dem.npz from build_site_world.py)
"""
import os, sys
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_site_world import load_ply, dem_at, in_roi, ROI_ORIGIN, ROI_THETA, ROI_SIZE

CELL = 0.5
TOP_W = 7            # cells (3.5 m)
H_MIN = 3.0
H_CANOPY = 2.0
R_MAX = 8.0
PINE_RATIO = 0.22
XY_CLAMP = (0.4, 2.5)
ELONG_MAX = 3.0     # crowns longer than this times their width are walls or hedges, not trees
# model sizes (meshes at 0.025 scale; hmr_sim cmu_oak_tree / cmu_pine_tree)
MODELS = {'oak': dict(h=6.4, r=5.0), 'pine': dict(h=5.0, r=1.56)}


def main():
    ply, wdir = sys.argv[1:3]
    d = np.load(f'{wdir}/dem.npz')
    P = load_ply(ply); P = P[in_roi(P[:, 0], P[:, 1], 3.0)]
    h = P[:, 2] - dem_at(d['dem'], d['org'], P[:, 0], P[:, 1])
    x0, y0 = P[:, 0].min(), P[:, 1].min()
    ix = ((P[:, 0] - x0) / CELL).astype(int); iy = ((P[:, 1] - y0) / CELL).astype(int)
    chm = np.zeros((ix.max() + 1, iy.max() + 1)); np.maximum.at(chm, (ix, iy), np.clip(h, 0, 40))
    chs = ndimage.gaussian_filter(chm, 1.0)
    tops = (chs == ndimage.maximum_filter(chs, size=TOP_W)) & (chs >= H_MIN)
    ti, tj = np.nonzero(tops)
    T = np.c_[x0 + (ti + 0.5) * CELL, y0 + (tj + 0.5) * CELL]
    keep = in_roi(T[:, 0], T[:, 1]); ti, tj, T = ti[keep], tj[keep], T[keep]
    ci, cj = np.nonzero(chm > H_CANOPY)
    C = np.c_[x0 + (ci + 0.5) * CELL, y0 + (cj + 0.5) * CELL]
    dist, owner = cKDTree(T).query(C, distance_upper_bound=R_MAX)
    trees = []
    for k in range(len(T)):
        m = owner == k
        area = m.sum() * CELL * CELL
        r = np.sqrt(area / np.pi); H = chm[ti[k], tj[k]]
        # crown centre (mean of owned cells) is a better stem estimate than the top for leaning crowns
        cx, cy = C[m].mean(0) if m.any() else T[k]
        if m.sum() >= 3:
            ev = np.linalg.eigvalsh(np.cov(C[m].T))
            if np.sqrt(ev[1] / max(ev[0], 1e-6)) > ELONG_MAX:
                continue
        kind = 'pine' if r / H < PINE_RATIO else 'oak'
        M = MODELS[kind]
        sxy = float(np.clip(r / M['r'], *XY_CLAMP)); sz = H / M['h']
        trees.append((kind, cx, cy, H, r, sxy, sz))
    trees.sort(key=lambda t: (t[1], t[2]))
    with open(f'{wdir}/trees.csv', 'w') as f:
        f.write('name,type,x,y,height_m,crown_r_m,scale_xy,scale_z\n')
        for n, t in enumerate(trees):
            f.write(f'{t[0]}_{n},{t[0]},{t[1]:.2f},{t[2]:.2f},{t[3]:.2f},{t[4]:.2f},{t[5]:.3f},{t[6]:.3f}\n')
    write_world(f'{wdir}/trees.sdf', trees)
    n_oak = sum(t[0] == 'oak' for t in trees)
    print(f'trees {len(trees)}: oak {n_oak}, pine {len(trees) - n_oak}; heights {np.percentile([t[3] for t in trees], [0, 50, 100]).round(1)}, crown r {np.percentile([t[4] for t in trees], [0, 50, 100]).round(1)}')
    figure(f'{wdir}/trees.png', chm, x0, y0, trees)


def tree_model(name, kind, x, y, sxy, sz):
    mdl = f'cmu_{kind}_tree'; mesh = f'model://{mdl}/meshes/{kind}_tree.dae'
    tex = f'model://{mdl}/materials/textures'
    leaf = 'branch_diffuse.png' if kind == 'oak' else 'branch_2_diffuse.png'
    sc = f'<scale>{sxy:.3f} {sxy:.3f} {sz:.3f}</scale>'
    def vis(sub, png, ds):
        return (f'<visual name="{sub.lower()}"><geometry><mesh><uri>{mesh}</uri><submesh><name>{sub}</name></submesh>{sc}</mesh></geometry>'
                f'<material>{"<double_sided>true</double_sided>" if ds else ""}<diffuse>1 1 1 1</diffuse>'
                f'<pbr><metal><albedo_map>{tex}/{png}</albedo_map></metal></pbr></material></visual>')
    return (f'    <model name="{name}"><static>true</static><pose>{x:.2f} {y:.2f} 0 0 0 0</pose><link name="link">'
            f'<collision name="collision"><geometry><mesh><uri>{mesh}</uri>{sc}</mesh></geometry></collision>'
            f'{vis("Branch", leaf, True)}{vis("Bark", "bark_diffuse.png", False)}</link></model>\n')


def write_world(path, trees):
    c, s = np.cos(np.radians(ROI_THETA)), np.sin(np.radians(ROI_THETA))
    mx = ROI_ORIGIN[0] + c * ROI_SIZE[0] / 2 - s * ROI_SIZE[1] / 2
    my = ROI_ORIGIN[1] + s * ROI_SIZE[0] / 2 + c * ROI_SIZE[1] / 2
    body = ''.join(tree_model(f'{t[0]}_{n}', t[0], t[1], t[2], t[5], t[6]) for n, t in enumerate(trees))
    open(path, 'w').write(f'''<?xml version="1.0"?>
<!-- Site ROI (hmr_localisation gt_map, explo_planner ROI) as a flat world:
     trees found in the map (trees.py), replaced by the cmu oak and pine models.
     Map frame: bag poses and the map line up with this world in x, y. -->
<sdf version="1.9">
<world name="site_trees">
  <scene><grid>false</grid><ambient>0.9 0.9 0.9 1</ambient><background>0.6 0.8 1.0 1</background></scene>
  <physics name="4ms" type="dart"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
  <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
  <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
  <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 50 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.4 0.2 -0.9</direction></light>
  <include><uri>model://cmu_grass_plane</uri><name>ground</name><pose>{mx:.2f} {my:.2f} 0 0 0 {np.radians(ROI_THETA):.4f}</pose></include>
{body}</world>
</sdf>
''')


def figure(path, chm, x0, y0, trees):
    c, s = np.cos(np.radians(ROI_THETA)), np.sin(np.radians(ROI_THETA))
    corn = np.array([[0, 0], [ROI_SIZE[0], 0], [ROI_SIZE[0], ROI_SIZE[1]], [0, ROI_SIZE[1]], [0, 0]])
    corn = np.c_[ROI_ORIGIN[0] + c * corn[:, 0] - s * corn[:, 1], ROI_ORIGIN[1] + s * corn[:, 0] + c * corn[:, 1]]
    fig, ax = plt.subplots(figsize=(15, 7))
    im = ax.imshow(chm.T, origin='lower', extent=(x0, x0 + chm.shape[0] * CELL, y0, y0 + chm.shape[1] * CELL),
                   cmap='Greys', vmin=0, vmax=15)
    for t in trees:
        col = 'tab:orange' if t[0] == 'oak' else 'tab:blue'
        ax.add_patch(plt.Circle((t[1], t[2]), t[5] * MODELS[t[0]]['r'], fill=False, color=col, lw=1.2))
        ax.plot(t[1], t[2], '+', color=col, ms=6)
    ax.plot(corn[:, 0], corn[:, 1], 'r-', lw=1); ax.plot(0, 0, 'kx')
    ax.plot([], [], 'o', mfc='none', color='tab:orange', label='oak (model crown)')
    ax.plot([], [], 'o', mfc='none', color='tab:blue', label='pine (model crown)')
    ax.legend(loc='upper left'); ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title(f'{len(trees)} trees found in the ROI; grey = map height above ground (0-15 m)')
    fig.colorbar(im, ax=ax, shrink=0.7); fig.tight_layout(); fig.savefig(path, dpi=110)


if __name__ == '__main__':
    main()
