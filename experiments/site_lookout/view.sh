#!/bin/bash
# Host: open the site world in the Gazebo GUI on this screen (NVIDIA GPU).
# Usage: experiments/site_lookout/view.sh [world.sdf under runs/site_lookout/world]
set -e
L="$(cd "$(dirname "$0")/../.." && pwd)"
world=${1:-site.sdf}
xhost +SI:localuser:$(id -un) >/dev/null
exec docker run --rm --name site_view --network none --cap-add=NET_ADMIN --shm-size=2g \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $L/ws:/ws -v $L/runs:/runs -v $L/runs/site_lookout/gzhome:/gzhome \
  --entrypoint bash hmrexplo:humble -c "
    ip link set lo multicast on
    exec gosu kalhan bash -lc '
      source /opt/ros/humble/setup.bash
      export HOME=/gzhome IGN_IP=127.0.0.1
      export IGN_GAZEBO_RESOURCE_PATH=/runs/site_lookout/world/models:/ws/install/hmr_sim/share/hmr_sim/models
      exec ign gazebo -v 3 /runs/site_lookout/world/$world'"
