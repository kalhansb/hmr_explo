#!/bin/bash
# Host wrapper: one site-lookout container. Usage:
#   experiments/site_lookout/docker_run.sh <container-name> <domain-id> <command...>
# The environment of the lookout experiment, unchanged: its workspace build
# (hmr_sim d528357: VLP-16 0.5-100 m, 2 cm noise), its run folder (service
# client bin/gzsvc, calibration calib_gpu, person model, Gazebo home) and its
# code (experiments/lookout, mounted at /lookout). Overlaid for this site:
#   /lookout/config       <- experiments/site_lookout/config (layout S1)
#   /runs/lookout/worlds  <- runs/site_lookout/worlds        (world S1)
# LK_ENV: the lookout experiment's checkout (default ../lookout_experiment).
set -e
L="$(cd "$(dirname "$0")/../.." && pwd)"
LK=${LK_ENV:-$(cd "$L/../lookout_experiment" && pwd)}
name=$1; dom=$2; shift 2
GPU_ARGS=()
[ "${LOOKOUT_GPU:-0}" = "1" ] && GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e LOOKOUT_GPU=1)
exec docker run --rm --name "$name" --network none --cap-add=NET_ADMIN --shm-size=2g "${GPU_ARGS[@]}" \
  -e ROS_DOMAIN_ID=$dom -e LOOKOUT_FULLSCANS=${LOOKOUT_FULLSCANS:-0} -e IGN_PARTITION=$name \
  -v $LK/ws:/ws -v $L/experiments/lookout:/lookout -v $L/experiments/site_lookout/config:/lookout/config:ro \
  -v $L/experiments/site_lookout:/site \
  -v $LK/runs/lookout:/runs/lookout -v $L/runs/site_lookout/worlds:/runs/lookout/worlds \
  -v $L/runs/site_lookout:/runs/site_lookout -v $LK/runs/lookout/gzhome:/gzhome \
  hmrexplo:humble /lookout/cell.sh "$@"
