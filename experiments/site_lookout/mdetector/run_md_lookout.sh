#!/bin/bash
# Host: the lookout experiment's layouts L2 and L3 re-run exactly as in
# experiments/lookout (its code and configs, and the lookout experiment's
# environment: ../lookout_experiment ws build, worlds, calibration), with every
# lidar's whole scan saved (LOOKOUT_FULLSCANS=1), then M-detector on the walks
# (md_sim.py) and the mulcher-vs-team comparison (md_team.py).
# Output: runs/lookout_md/<L>_md. One layout at a time; M-detector on a finished
# layout runs while the next one is simulated.
set -u
L="$(cd "$(dirname "$0")/../../.." && pwd)"
LK=${LK_ENV:-$(cd "$L/../lookout_experiment" && pwd)}
for T in ${@:-L2 L3}; do
  docker run --rm --name lkmd_$T --network none --cap-add=NET_ADMIN --shm-size=2g \
    --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e LOOKOUT_GPU=1 -e LOOKOUT_FULLSCANS=1 \
    -e ROS_DOMAIN_ID=88 -e IGN_PARTITION=lkmd_$T \
    -v $LK/ws:/ws -v $L/experiments/lookout:/lookout -v $LK/runs/lookout:/runs/lookout \
    -v $L/runs/lookout_md:/runs/lookout_md -v $LK/runs/lookout/gzhome:/gzhome \
    hmrexplo:humble /lookout/cell.sh /lookout/run_layout.sh $T /runs/lookout_md/${T}_md full \
    > "$L/runs/lookout_md/${T}_md.console.log" 2>&1
  ( docker run --rm --name md_$T --network none -u $(id -u):$(id -g) -e HOME=/tmp -e ROS_HOME=/tmp \
      -v $L:/p -w /p mdetector:noetic bash -c "source /catkin_ws/devel/setup.bash; \
      python3 experiments/site_lookout/mdetector/md_sim.py runs/lookout_md/${T}_md runs/lookout_md/${T}_md/mdet" \
      > "$L/runs/lookout_md/${T}_mdet.log" 2>&1
    docker run --rm --network none -v $L:/p -w /p hmrexplo:humble python3 experiments/site_lookout/mdetector/md_team.py \
      runs/lookout_md/${T}_md experiments/lookout/config/$T.yaml > "$L/runs/lookout_md/${T}_team.txt" 2>&1 ) &
done
wait
