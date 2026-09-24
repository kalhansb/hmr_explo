#!/bin/bash
# Camera ("NDVI") experiment cell. Usage: run_cam_cell.sh <name> <rep 1-6> [duration_s]
# Explore-then-exploit (RELEASE_ON_DONE=1), clean, targets from
# targets/targets_rep<rep>.yaml, segmentation camera on both robots, lesion
# plates on the 19 usable in-ROI oaks (PLAN.md). Fine band off (campaign default).
# Same harness as runs/pilot_fine (campaign binaries + overlay sources), with
# the world, the Husky model and the scenario swapped by read-only mounts.
# Outputs: runs/cam_experiment/<name>/ (camera counts in <name>/cam/).
set -eu
name=$1; rep=$2; dur=${3:-2400}
P=/home/kalhan/Documents/exploitation_experiments
O=$P/runs/pilot_fine/overlay
C=$P/runs/cam_experiment
S=${CAM_SCRATCH:-$P/runs/cam_experiment/scratch}   # writable scratch; holds the Gazebo home (gzhome/)
HS=/ws/src/hmr_sim/hmr_sim
[ -e "$C/$name" ] && { echo "refusing: $C/$name exists"; exit 1; }
docker run --rm --name "cam_$name" --cap-add=NET_ADMIN --shm-size=2g \
  -e EXPLOIT=1 -e RELEASE_ON_DONE=1 -e DURATION_S=$dur -e RECORD=2 -e RVIZ=0 -e GZ_GUI=0 \
  -e RECONNECT_MODE=off -e STOP_ON_DONE=1 -e COMMS=0 -e LINK_GATE=0 \
  -e ROI_HALF=25 -e DONE_UNKNOWN=0.10 -e DONE_CRITERION=streak \
  -e PLANNER_EXTRA="done_min_consecutive_steps:=1" \
  -e TARGETS=/cam/targets/targets_rep$rep.yaml \
  -e FINE_BAND=0 \
  -e CAM_COUNTER=/cam/cam_counter.py \
  -e OUTDIR=/runs/cam_experiment/$name \
  -e GIT_CONFIG_COUNT=1 -e GIT_CONFIG_KEY_0=safe.directory -e GIT_CONFIG_VALUE_0='*' \
  -v $P/ws:/ws -v $P/.git:/.git:ro -v $P/runs:/runs -v $S:/sp -v $S/gzhome:/gzhome \
  -v $O/explo_planner:/ws/src/explo_planner:ro \
  -v $O/simple_nav_3d:/ws/src/simple_nav_3d:ro \
  -v $O/scovox:/ws/src/scovox:ro \
  -v $C:/cam:ro \
  -v $C/overlay/flatforestv2_cam.sdf:$HS/worlds/flatforest/flatforestv2.sdf:ro \
  -v $C/overlay/husky_cam.sdf:$HS/models/COSTAR_HUSKY_SENSOR_CONFIG_LIDAR/model.sdf:ro \
  -v $C/overlay/scenario_cam.yaml:$HS/config/scenarios/flatforest_2robot_lidar.yaml:ro \
  -v $P/runs/pilot_fine/cell.sh:/cell.sh:ro \
  hmrexplo:humble /cell.sh
