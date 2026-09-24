#!/usr/bin/env python3
"""Spawn a lookout Husky as a STATIC model on its post (in the container).
  spawn_static.py <world> <name> <x> <y> <yaw>

Why: in DART with the bullet collision detector a parked Husky does not stay
still. Its cylinder wheels on the ground plane rock it now and then: 0.52 deg in
one 0.1 s step (lk2b, L2 pilot 2026-09-24), and on open flat ground with
nothing near (smoke/wobble.sh, 10 Huskies, 540 s: one 0.53 deg jump, two
~0.05 deg creeps; with the ode detector none of them moved). Every tilt past
0.01 deg makes walks.py relearn that lidar's background, and at grazing
incidence a 0.5 deg tilt turns thousands of ground returns "new" (false alarms).
The lookouts do not move in this experiment, so they are made static, as the
mulcher, the people and the trees already are.

The model is the same one spawn_robot.launch.py spawns (its namespacing is
reused, hmr_sim is not changed), with <static> set, placed where a settled
Husky's base sits on flat ground (z 0.1322 m, level: smoke_wobble_bullet),
so the lidar is at the height it had in the calibration and the pilots."""
import importlib.util
import math
import os
import shutil
import subprocess
import sys

from ament_index_python import get_package_share_directory

Z_SETTLED = 0.1322   # base_link z of a settled Husky on the ground plane (m)

world, name, x, y, yaw = sys.argv[1], sys.argv[2], *map(float, sys.argv[3:6])
share = get_package_share_directory("hmr_sim")
spec = importlib.util.spec_from_file_location("spawn_robot", os.path.join(share, "launch", "spawn_robot.launch.py"))
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)

# same directory as model.sdf: its mesh uris are relative
src = os.path.join(share, "models", "COSTAR_HUSKY_SENSOR_CONFIG_LIDAR", "model.sdf")
dst = src.replace(".sdf", f"_{name}_static.sdf")
shutil.copy(src, dst)
launch.namespace_sdf_file(dst, {"robot_name": name, "use_imu": True})
s = open(dst).read()
assert s.count("<static>0</static>") == 1
open(dst, "w").write(s.replace("<static>0</static>", "<static>1</static>"))

qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
req = (f'sdf_filename: "{dst}", name: "{name}", pose: {{ position: {{ x: {x}, y: {y}, z: {Z_SETTLED} }}, '
       f'orientation: {{ x: 0, y: 0, z: {qz}, w: {qw} }}}}')
out = subprocess.run(["ign", "service", "-s", f"/world/{world}/create", "--reqtype", "ignition.msgs.EntityFactory",
                      "--reptype", "ignition.msgs.Boolean", "--timeout", "10000", "--req", req],
                     capture_output=True, text=True)
print(out.stdout.strip(), out.stderr.strip())
sys.exit(0 if "data: true" in out.stdout else 1)
