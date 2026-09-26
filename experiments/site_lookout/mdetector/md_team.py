#!/usr/bin/env python3
"""Mulcher alone vs mulcher and lookouts, with M-detector (frame-out labels)
as every lidar's detector, on the main walks. Before the post line = first
alarm while the walker is still further from the mulcher than its lookout's
post (r_post of its entry in the layout config). Warning = time from first
alarm to crossing that line.
Usage: md_team.py <run dir (with mdet/)> <layout config yaml> [labels]"""
import csv, glob, math, os, statistics as S, sys
import numpy as np
import yaml
run, cfgp = sys.argv[1:3]; lab = sys.argv[3] if len(sys.argv) > 3 else 'frame'
RP = {e['id']: float(e['r_post']) for e in yaml.safe_load(open(cfgp))['entries']}
first = {}
for r in csv.DictReader(open(f'{run}/mdet/md_first.csv')):
    if r['labels'] == lab:
        first.setdefault(r['walk'], {})[r['lidar']] = float(r['t_s']) if r['t_s'] else None
rows = []
d_lidars = lambda f: np.load(f)['lidars']
for f in sorted(glob.glob(f'{run}/fullscans/*.npz')):
    w = os.path.basename(f).split('__')[0]
    if 'chk' in w or w not in first or len(first[w]) < len(d_lidars(f)):
        continue
    d = np.load(f); t, xy = d['steps_t'], d['steps_xy']
    dist = np.hypot(xy[:, 0], xy[:, 1]); rp = RP[w.split('-')[2]]
    t_post = float(t[np.argmax(dist <= rp)]) if (dist <= rp).any() else float(t[-1])
    m = first[w].get('mulcher'); tm = min([v for v in first[w].values() if v is not None], default=None)
    rows.append((w, m, tm, t_post))
n = len(rows)
bp = lambda v, tp: v is not None and v < tp
mb = sum(bp(m, tp) for _, m, _, tp in rows); tb = sum(bp(tm, tp) for _, _, tm, tp in rows)
wm = [tp - m if m is not None else None for _, m, _, tp in rows]; wt = [tp - tm for _, _, tm, tp in rows if tm is not None]
gain = [m - tm for _, m, tm, _ in rows if m is not None and tm is not None]
print(f'{run}: {n} main walks ({lab})')
print(f'  detected at all: mulcher {sum(m is not None for _, m, _, _ in rows)}/{n}, team {sum(tm is not None for _, _, tm, _ in rows)}/{n}')
print(f'  before the post line: mulcher {mb}/{n}, team {tb}/{n}')
print(f'  warning before the post line, median: mulcher {S.median([x for x in wm if x is not None]):.1f} s, team {S.median(wt):.1f} s')
print(f'  team earlier than mulcher, same walk: median {S.median(gain):.1f} s, least {min(gain):.1f} s, most {max(gain):.1f} s')
print(f'  walks where the lookouts were not earlier: {sum(g <= 0 for g in gain)}')
