#!/bin/bash
# Phase A (PLAN.md): virtual camera on runs that had no rendered camera.
# Usage: phaseA.sh <run dir relative to runs/>...   e.g. on_rep1 pilot_fine/off
# Per run (container, --network none, 1 CPU so a concurrent sim keeps its RTF):
#   gt_poses.npz + gt_path_cum.json (skipped if present), virtual_cam.csv.
# The bag's .zstd files are symlinked into the container's /tmp, so the run's
# rosbag2/ is only read (zband replays decompress in place there).
set -u
C=/home/kalhan/Documents/exploitation_experiments/runs/cam_experiment
P=/home/kalhan/Documents/exploitation_experiments
for r in "$@"; do
  d=$P/runs/$r
  if [ -f "$d/virtual_cam.csv" ]; then echo "[skip] $r $(date -Is)"; continue; fi
  echo "[start] $r $(date -Is)"
  docker run --rm --network none --cpus 1 --name "camA_${r//\//_}" \
    -v $C:/cam:ro -v $d:/run -v $P/ws/src/hmr_sim:/hs:ro hmrexplo:humble bash -lc "
    source /opt/ros/humble/setup.bash
    if [ ! -f /run/gt_poses.npz ]; then
      mkdir -p /tmp/bag && cp /run/rosbag2/metadata.yaml /tmp/bag/ && ln -s /run/rosbag2/*.zstd /tmp/bag/ &&
      python3 /cam/gt_poses.py /run /tmp/bag --hz 10 || exit 1
    fi
    python3 /cam/virtual_cam.py /run /cam/lesions.json /hs/hmr_sim/worlds/flatforest/flatforestv2.sdf || exit 1
    chown $(id -u):$(id -g) /run/gt_poses.npz /run/gt_path_cum.json /run/virtual_cam.csv" > "$C/phaseA_${r//\//_}.log" 2>&1
  rc=$?
  echo "[done ] $r rc=$rc $(date -Is)"
done
