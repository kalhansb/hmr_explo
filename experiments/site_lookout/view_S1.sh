#!/bin/bash
# Host: open world S1 (or view_S1.sh <layout>, after view_S1.py makes view_<layout>.sdf) (lookouts on their posts, a walker going along the path)
# in the Gazebo window. Viewing only. Close the window or: docker stop site_view
set -e
L="$(cd "$(dirname "$0")/../.." && pwd)"
LK=${LK_ENV:-$(cd "$L/../lookout_experiment" && pwd)}
T=${1:-S1}
xhost +SI:localuser:$(id -un) >/dev/null
exec docker run --rm --name site_view --network none --cap-add=NET_ADMIN --shm-size=2g \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $LK/ws:/ws -v $L/experiments/lookout:/lookout -v $L/experiments/site_lookout:/site \
  -v $LK/runs/lookout/models:/models:ro -v $L/runs/site_lookout/worlds:/worlds:ro -v $L/runs/site_lookout/worlds:/runs/lookout/worlds:ro -v $LK/runs/lookout/gzhome:/gzhome \
  --entrypoint bash hmrexplo:humble -c "
    ip link set lo multicast on
    exec gosu kalhan bash -lc '
      source /opt/ros/humble/setup.bash; source /ws/install/setup.bash
      export HOME=/gzhome IGN_IP=127.0.0.1
      export IGN_GAZEBO_RESOURCE_PATH=/models:/ws/install/hmr_sim/share/hmr_sim/models
      ign gazebo -r -v 2 /worlds/view_$T.sdf &
      G=\$!
      for i in \$(seq 1 600); do ign service -l 2>/dev/null | grep -q /world/view_$T/set_pose && break; sleep 1; done
      sleep 5
      python3 /site/view_walker.py /site/config/$T.yaml view_$T &
      wait \$G'"
