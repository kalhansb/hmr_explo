#!/bin/bash
# Host wrapper: one lookout container. Usage:
#   lookout/docker_run.sh <container-name> <domain-id> <command...>
# --network none keeps concurrent containers from hearing each other's DDS and
# ign-transport traffic; ROS_DOMAIN_ID / IGN_PARTITION are set per container
# as a second guard. LOOKOUT_GPU=1 renders on the NVIDIA GPU (EGL, headless)
# instead of llvmpipe; see cell.sh.
set -e
L="$(cd "$(dirname "$0")/.." && pwd)"
name=$1; dom=$2; shift 2
GPU_ARGS=()
[ "${LOOKOUT_GPU:-0}" = "1" ] && GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e LOOKOUT_GPU=1)
exec docker run --rm --name "$name" --network none --cap-add=NET_ADMIN --shm-size=2g "${GPU_ARGS[@]}" \
  -e ROS_DOMAIN_ID=$dom -e IGN_PARTITION=$name \
  -v $L/ws:/ws -v $L/lookout:/lookout -v $L/runs:/runs \
  -v $L/runs/lookout/gzhome:/gzhome \
  hmrexplo:humble /lookout/cell.sh "$@"
