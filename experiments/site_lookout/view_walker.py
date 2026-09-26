#!/usr/bin/env python3
"""Move the walker along the path in the Gazebo window, entry A then entry
B, over and over, at walking speed (viewing only; in the container).
Usage: view_walker.py <config S1.yaml> <world name>"""
import math, subprocess, sys, time
import yaml
sys.path.insert(0, '/lookout')
import walks

cfg = yaml.safe_load(open(sys.argv[1])); W = sys.argv[2]
yaw_off = cfg['person']['yaw_offset']


def set_pose(x, y, yaw):
    q = (math.cos(yaw / 2), math.sin(yaw / 2))
    subprocess.run(['ign', 'service', '-s', f'/world/{W}/set_pose', '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean', '--timeout', '2000', '--req',
                    f'name: "person", position: {{x: {x:.3f}, y: {y:.3f}, z: 0}}, '
                    f'orientation: {{w: {q[0]:.5f}, z: {q[1]:.5f}}}'], capture_output=True)


while True:
    for e in cfg['entries']:
        pts = walks.walk_points(cfg, {'entry': e['id'], 'along': 0.0, 'across': 0.0})
        for p in pts:
            t0 = time.time()
            set_pose(p['x'], p['y'], p['heading'] + yaw_off)
            time.sleep(max(0.0, 0.5 / 1.3 - (time.time() - t0)))
        time.sleep(2)
