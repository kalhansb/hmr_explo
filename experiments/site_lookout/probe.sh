#!/bin/bash
# In-container (via ../lookout/cell.sh): render the probe world once and save
# each probe's scan. Usage: probe.sh <sensor> <poses.json> <out.npz>
set -e
sensor=$1; poses=$2; out=$3
W=/runs/site_lookout/world
export IGN_GAZEBO_RESOURCE_PATH=$W/models:$IGN_GAZEBO_RESOURCE_PATH
python3 /site/probe_world.py $W $sensor $poses $W/probe_$sensor.sdf
ign gazebo -s -r -v 3 ${GZ_HEADLESS:-} $W/probe_$sensor.sdf > ${out%.npz}_gz.log 2>&1 &
G=$!
PROBES=$(python3 -c "import json;print(' '.join('probe_%d'%j['frame'] for j in json.load(open('$poses'))))")
ARGS=(); TOP=()
for p in $PROBES; do ARGS+=("/$p/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked"); TOP+=("/$p/scan/points"); done
ros2 run ros_gz_bridge parameter_bridge "${ARGS[@]}" > ${out%.npz}_bridge.log 2>&1 &
B=$!
python3 /site/capture.py $out "${TOP[@]}"
kill $B $G; wait 2>/dev/null || true
