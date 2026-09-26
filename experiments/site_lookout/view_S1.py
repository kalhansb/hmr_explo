#!/usr/bin/env python3
"""A viewing copy of world S1 (or another layout): the lookout Huskies on
their posts (static) and the walker, for the Gazebo window. Not used by the
experiment.
Usage: view_S1.py <worlds dir> <config S1.yaml> [layout, default S1]   -> <worlds dir>/view_<layout>.sdf"""
import sys, yaml
wdir, cfgp = sys.argv[1:3]
T = sys.argv[3] if len(sys.argv) > 3 else 'S1'
cfg = yaml.safe_load(open(cfgp))
w = open(f'{wdir}/lookout_{T}.sdf').read().replace(f'<world name="lookout_{T}">', f'<world name="view_{T}">')
extra = ''.join(
    f'\n    <include><uri>model://COSTAR_HUSKY_SENSOR_CONFIG_LIDAR</uri><name>{l["name"]}</name><static>true</static>'
    f'<pose>{l["x"]} {l["y"]} 0.1322 0 0 {l["yaw"]}</pose></include>' for l in cfg['lookouts'])
open(f'{wdir}/view_{T}.sdf', 'w').write(w.replace('</world>', extra + '\n  </world>'))
