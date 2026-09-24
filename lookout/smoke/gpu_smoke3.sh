#!/bin/bash
# Renderer smoke, walk setup (in the container): layout world, lookouts on
# their posts (sim_up.sh AT_POST=1), every lidar bridged; times stepped scans
# from every lidar and samples per-process CPU while stepping.
#   gpu_smoke3.sh <layout> <out>
set -u
LAYOUT=$1; OUT=$2; AT_POST=1
source /lookout/sim_up.sh
sleep 5
( for i in 1 2 3 4 5 6; do sleep 8; top -b -n 1 -w 200 | sed -n 7,14p; echo ---; done ) > $OUT/top.txt 2>&1 &
timeout 600 python3 - "$W" $LOOKOUTS <<'PY'
import sys, time
import numpy as np
sys.path.insert(0, "/lookout")
import rclpy, gz
from scans import ScanNode
W, lks = sys.argv[1], sys.argv[2:]
rclpy.init()
topics = {"mulcher": "/mulcher/velodyne_points", **{n: f"/{n}/velodyne_points" for n in lks}}
node = ScanNode(W, topics, odoms=lks, name="gpu_smoke3")
print("clouds:", node.wait_all(odoms=lks), flush=True)
gz.pause(W, True); node.drain(0.5)
node.step_fresh()
ts = []
for i in range(60):
    t0 = time.time(); msgs, info = node.step_fresh(); ts.append(time.time() - t0)
print(f"{len(topics)} lidars, step wall s: median {np.median(ts):.3f} min {min(ts):.3f} max {max(ts):.3f}", flush=True)
PY
cat $OUT/top.txt
