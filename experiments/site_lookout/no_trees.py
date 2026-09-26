#!/usr/bin/env python3
"""Check (after the results, exploratory): layout S1N = S1 with every tree
removed; everything else identical (mulcher, path, walks, seed, lidar, rules).
Run with NO_LOOKOUTS=1 in check mode, it repeats S1's 12 check walks (mulcher
only) on open ground, to compare with S1's check walks among the trees.
Usage: no_trees.py <worlds dir>   (writes config/S1N.yaml, lookout_S1N.sdf, forest_S1N.csv)"""
import os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__))
wdir = sys.argv[1]
w = open(f'{wdir}/lookout_S1.sdf').read()
w, n = re.subn(r'\n    <model name="oak_\d+">.*?</model>', '', w, flags=re.S)
open(f'{wdir}/lookout_S1N.sdf', 'w').write(w.replace('<world name="lookout_S1">', '<world name="lookout_S1N">'))
open(f'{wdir}/forest_S1N.csv', 'w').write('x,y\n')
c = open(f'{HERE}/config/S1.yaml').read()
c = c.replace('layout: S1\n', 'layout: S1N\n').replace('world: lookout_S1\n', 'world: lookout_S1N\n') \
     .replace('csv: forest_S1.csv', 'csv: forest_S1N.csv').replace('n_trees: 38', 'n_trees: 0')
open(f'{HERE}/config/S1N.yaml', 'w').write('# S1 without trees (no_trees.py)\n' + c)
print('removed', n, 'trees')
