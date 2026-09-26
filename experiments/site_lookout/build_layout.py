#!/usr/bin/env python3
"""Site lookout layout S1, in the lookout experiment's format, so that its
walk driver, detector and analysis (experiments/lookout) run unchanged.

Frame: the mulcher at the origin. World (x, y) = map (x, y) - MULCHER_MAP,
map = hmr_localisation gt_map frame.

  * Trees: every tree trees.py found in the ROI (runs/site_lookout/world/
    trees.csv), all as the cmu oak (the pines replaced by oaks): horizontal
    scale crown radius / 5.0 m (clamped 0.4-2.5), vertical scale height /
    6.4 m. The oak's horizontal scale is <= 0.9 here, so the unscaled root
    flare (1.7 m) and trunk radius (0.35 m) used by posts.py are conservative.
  * Path: the straight lane south of the tree band (layout.py path S, ROI
    offset v = -2.5 m, clearance to every trunk >= 3.2 m), extended to
    PATH_HALF m either side of its closest approach to the mulcher. Two
    entries, one from each end, walking to the closest approach (as L2).
  * Lookouts: posts.py, unchanged (free spot 12-28 m out beside the path with
    a safe radio link, earliest view of a walker).
  * Mulcher, person, ground plane (400 m), physics, lidar: lookout/build_world.py.
  * Walks: as L2 (12 headings x 2 entries x 2 repeats, 45 m +- 5 m before the
    post line, 0.5 m steps at 1.3 m/s), seed 13.

Writes experiments/site_lookout/config/S1.yaml, <worlds>/lookout_S1.sdf,
<worlds>/forest_S1.csv. Usage: build_layout.py <trees.csv> <worlds dir>
"""
import csv, math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', 'lookout'))
from build_site_world import ROI_ORIGIN, ROI_THETA
import build_world as bw
import posts

NAME = 'S1'
MULCHER_MAP = (-2.0, 0.6)
PATH_V = -2.5            # ROI-frame offset of path S (layout.py)
PATH_HALF = 110.0        # >= posts.WALK_FROM from the mulcher at both ends
PATH_STEP = 2.5
OAK_R, OAK_H = 5.0, 6.4
XY_CLAMP = (0.4, 2.5)
C, S = math.cos(math.radians(ROI_THETA)), math.sin(math.radians(ROI_THETA))


def roi_to_world(u, v):
    return (ROI_ORIGIN[0] + C * u - S * v - MULCHER_MAP[0], ROI_ORIGIN[1] + S * u + C * v - MULCHER_MAP[1])


def oak(name, x, y, sxy, sz, yaw):
    mesh = 'model://cmu_oak_tree/meshes/oak_tree.dae'; tex = 'model://cmu_oak_tree/materials/textures'
    sc = f'<scale>{sxy:.3f} {sxy:.3f} {sz:.3f}</scale>'
    vis = lambda sub, png, ds: (
        f'<visual name="{sub.lower()}"><geometry><mesh><uri>{mesh}</uri><submesh><name>{sub}</name></submesh>{sc}</mesh></geometry>'
        f'<material>{"<double_sided>true</double_sided>" if ds else ""}<diffuse>1 1 1 1</diffuse>'
        f'<pbr><metal><albedo_map>{tex}/{png}</albedo_map></metal></pbr></material></visual>')
    return (f'\n    <model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}</pose><link name="link">'
            f'<collision name="collision"><geometry><mesh><uri>{mesh}</uri>{sc}</mesh></geometry></collision>'
            f'{vis("Branch", "branch_diffuse.png", True)}{vis("Bark", "bark_diffuse.png", False)}</link></model>')


def strip(P):
    (x0, y0), (x1, y1) = P[0], P[-1]
    L = math.hypot(x1 - x0, y1 - y0)
    return (f'\n    <model name="path_S"><static>true</static><pose>{(x0 + x1) / 2:.3f} {(y0 + y1) / 2:.3f} 0.005 0 0 '
            f'{math.atan2(y1 - y0, x1 - x0):.5f}</pose><link name="l"><visual name="v"><geometry><box>'
            f'<size>{L:.2f} {2 * bw.LANE_HALF} 0.01</size></box></geometry><material><ambient>0.45 0.35 0.22 1</ambient>'
            f'<diffuse>0.55 0.43 0.28 1</diffuse></material></visual></link></model>')


def main():
    tcsv, worlds = sys.argv[1:3]
    rows = list(csv.DictReader(open(tcsv)))
    trees = []
    for r in rows:
        x, y = float(r['x']) - MULCHER_MAP[0], float(r['y']) - MULCHER_MAP[1]
        sxy = min(max(float(r['crown_r_m']) / OAK_R, XY_CLAMP[0]), XY_CLAMP[1])
        trees.append(dict(src=r['name'], x=x, y=y, sxy=sxy, sz=float(r['height_m']) / OAK_H, h=float(r['height_m'])))
    tp = [(t['x'], t['y']) for t in trees]
    # path S: the line v = PATH_V; closest approach to the mulcher (origin)
    a, b = roi_to_world(0, PATH_V), roi_to_world(100, PATH_V)
    d = ((b[0] - a[0]) / 100, (b[1] - a[1]) / 100)
    t0 = -(a[0] * d[0] + a[1] * d[1]); q = (a[0] + t0 * d[0], a[1] + t0 * d[1])
    miss = math.hypot(*q)
    n = int(PATH_HALF / PATH_STEP)
    P = [(round(q[0] + k * PATH_STEP * d[0], 3), round(q[1] + k * PATH_STEP * d[1], 3)) for k in range(-n, n + 1)]
    L = bw.polyline_len(P); mid = L / 2
    lane_min = min(bw.dist_to_polyline(t, P) for t in tp)
    entries = [{'id': 'A', 'path': 'P1', 's_start': 0.0, 'dir': 1, 's_end': mid},
               {'id': 'B', 'path': 'P1', 's_start': L, 'dir': -1, 's_end': mid}]
    lookouts = []
    for e, rname in zip(entries, ['lksa', 'lksb']):
        c, ok, cands = posts.find_post(bw, P, e, tp, bw.PAD, bw.FLARE)
        e['r_post'] = c['r']; e['s_post'] = round(c['s_post'], 3); e['post_cross'] = [round(v, 3) for v in c['post_cross']]
        s_sp = e['s_end'] - e['dir'] * 8.0
        (sx, sy), _ = bw.point_at(P, s_sp)
        lookouts.append({'name': rname, 'entry': e['id'], 'x': round(c['x'], 3), 'y': round(c['y'], 3),
                         'yaw': round(c['yaw'], 5), 'r': c['r'], 'side': c['side'],
                         'placement': {k: round(c[k], 3) if isinstance(c[k], float) else c[k] for k in
                                       ('seen_main_m', 'seen_best_m', 'nearest_trunk_m', 'link_d_m', 'snr_db',
                                        'trees_on_link', 'corridor_w_m', 'link_clear_m')},
                         'spawn': {'x': round(sx, 3), 'y': round(sy, 3), 'z': 0.3,
                                   'yaw': round(math.atan2(c['y'] - sy, c['x'] - sx), 5)}})
        print(f"entry {e['id']}: {len(cands)} candidates, {sum(x['free'] for x in cands)} free, "
              f"{sum(x['link_ok'] for x in cands)} safe link, {len(ok)} both; post {c['r']} m {c['side']} "
              f"({c['x']:.2f}, {c['y']:.2f}); first seen {c['seen_main_m']} m main, {c['seen_best_m']} m best; "
              f"nearest trunk {c['nearest_trunk_m']:.2f} m; link {c['link_d_m']:.1f} m, {c['trees_on_link']} trunks")
    wname = f'lookout_{NAME}'
    extra = ''.join(oak(f"oak_{i}", t['x'], t['y'], t['sxy'], t['sz'], 0.0) for i, t in enumerate(trees)) + strip(P)
    open(os.path.join(worlds, f'{wname}.sdf'), 'w').write(
        bw.world_sdf(wname, [], bw.husky_lidar_block(), extra=extra).replace(
            'Generated by lookout/build_world.py', 'Generated by site_lookout/build_layout.py'))
    with open(os.path.join(worlds, f'forest_{NAME}.csv'), 'w') as f:
        f.write('x,y\n'); [f.write(f"{t['x']:.3f},{t['y']:.3f}\n") for t in trees]
    cfg = {
        'layout': NAME, 'world': wname,
        'site': {'map': 'ws/src/hmr_localisation/gt_map/gt_map.ply', 'mulcher_map_xy': list(MULCHER_MAP),
                 'world_to_map': 'map = world + mulcher_map_xy', 'roi': 'build_site_world.py ROI_*',
                 'path_roi_v': PATH_V, 'path_miss_m': round(miss, 3), 'lane_min_trunk_m': round(lane_min, 3)},
        'mulcher': {'x': 0.0, 'y': 0.0, 'body_size': list(bw.BODY), 'head_size': list(bw.HEAD),
                    'head_center': list(bw.HEAD_C), 'lidar_z': bw.MULCHER_LIDAR_Z},
        'person': {'height': bw.PERSON_H, 'scale': round(bw.PERSON_SCALE, 5), 'yaw_offset': round(math.pi / 2, 6),
                   'park': list(bw.PERSON_PARK)},
        'forest': {'source': 'site_lookout/trees.py (all oaks)', 'n_trees': len(trees),
                   'model': 'cmu_oak_tree scaled', 'csv': f'forest_{NAME}.csv'},
        'end_radius': round(miss, 3),
        'placement': {'method': 'lookout/posts.py', 'step': posts.STEP, 'r_min': posts.R_MIN, 'r_safe': posts.R_SAFE,
                      'clear_margin': posts.CLEAR_MARGIN, 'trunk_r': posts.TRUNK_R, 'reach_main': posts.REACH_MAIN,
                      'reach_best': posts.REACH_BEST, 'pad_free': bw.PAD + bw.FLARE},
        'walk': {'step': 0.5, 'speed': 1.3, 'start_before_post': 45.0, 'jitter_along': 5.0, 'jitter_across': 0.5,
                 'seed': 13, 'headings_deg': [30 * k for k in range(12)], 'repeats': 2},
        'paths': [{'id': 'P1', 'polyline': [list(v) for v in P]}],
        'entries': [{k: (round(v, 3) if isinstance(v, float) else v) for k, v in e.items()} for e in entries],
        'lookouts': lookouts,
    }
    open(os.path.join(HERE, 'config', f'{NAME}.yaml'), 'w').write(
        f'# Generated by site_lookout/build_layout.py; do not edit.\n{bw.yaml_dump(cfg)}\n')
    print(f'{NAME}: {len(trees)} oaks; path passes {miss:.2f} m from the mulcher, nearest trunk {lane_min:.2f} m '
          f'from its centreline; length {L:.0f} m')


if __name__ == '__main__':
    main()
