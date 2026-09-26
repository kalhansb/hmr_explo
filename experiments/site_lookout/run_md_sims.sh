#!/bin/bash
# Host: layouts S1 (oak models) and S1G (map points), full walks and check walks,
# with every lidar's whole scan saved (LOOKOUT_FULLSCANS=1) for M-detector
# (mdetector/). Detection rules and walks as in S1_full; one layout at a time.
set -u
L="$(cd "$(dirname "$0")/../.." && pwd)"
for T in S1 S1G; do
  LOOKOUT_GPU=1 LOOKOUT_FULLSCANS=1 "$L/experiments/site_lookout/docker_run.sh" site_md_$T 87 \
    /lookout/run_layout.sh $T /runs/site_lookout/${T}_md full > "$L/runs/site_lookout/${T}_md.console.log" 2>&1
done
