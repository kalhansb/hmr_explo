#!/usr/bin/env bash
# Host-side orchestrator for a SPLIT GLIM + SCovox experiment.
#
# Architecture (one container per repo, talking over ROS 2 DDS):
#   glim_loc container (glim_localisation/) : GLIM SLAM + bag play + GLIM recorders
#                                             + SCovox-map salvage/capture (DDS clients) + RViz
#   scovox   container (scovox/)            : the SCovox mapping node(s)
# Both run host-net + ipc host + ROS_DOMAIN_ID=0, so they share one DDS graph; no
# repo is mounted into the other and neither builds the other's code.
#
# This script (runs on the HOST -- needs docker, NOT ROS):
#   1. brings up both containers
#   2. starts the SCovox node(s) for <mode> in the scovox container (detached)
#   3. runs the GLIM-side driver in glim_loc -- it plays the bag, records GLIM
#      outputs, and salvages the SCovox map over DDS (blocks until done)
#   4. stops the scovox node via `docker compose stop scovox`
#
# Usage:   ./run_glim_experiment.sh <mode> [driver-args...]
#   modes: map | odom | viz
#   e.g.   ./run_glim_experiment.sh odom 0.5        # duration unset, rate 0.5
#          ./run_glim_experiment.sh map "" 1.0      # full bag, rate 1.0
# RViz modes (viz) need `xhost +local:` on the host first; the driver blocks until
# you Ctrl-C it, then the scovox node is stopped.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# This script lives in ws/src/ alongside glim_localisation/ and scovox/.
GLIM_DIR="$HERE/glim_localisation"
SCOVOX_DIR="$HERE/scovox"

MODE="${1:?usage: run_glim_experiment.sh <map|odom|viz> [driver-args...]}"
shift || true

case "$MODE" in
  map)    DRIVER=run_glim_scovox.sh ;;
  odom)   DRIVER=run_glim_scovox_odom.sh ;;
  viz)    DRIVER=run_glim_scovox_viz.sh ;;
  # raw: same GLIM driver as `map` (plays /ouster/points + /imu/data, captures the
  # scovox map), but scovox runs on the RAW cloud with native gyro deskew.
  raw)    DRIVER=run_glim_scovox.sh ;;
  # rawoff: deskew-OFF control (same raw cloud, deskew_mode:off) — baseline for
  # the deskew A/B comparison at identical rate/range.
  rawoff) DRIVER=run_glim_scovox.sh ;;
  # rawctl: placement control — raw cloud with OLD TF behavior (0.2s timeout +
  # Time(0) fallback) — baseline for the exact-stamp-placement A/B.
  rawctl) DRIVER=run_glim_scovox.sh ;;
  *) echo "unknown mode '$MODE' (map|odom|viz|raw|rawoff|rawctl)"; exit 1 ;;
esac

dc_glim()   { docker compose -f "$GLIM_DIR/compose.yaml"   "$@"; }
dc_scovox() { docker compose -f "$SCOVOX_DIR/compose.yaml" "$@"; }

cleanup() {
  echo "[orch] stopping scovox node (docker compose stop scovox)..."
  dc_scovox stop scovox 2>/dev/null || true
}
trap cleanup EXIT

echo "[orch] mode=$MODE  driver=$DRIVER"
echo "[orch] bringing up both containers..."
dc_glim   up -d glim
dc_scovox up -d scovox

echo "[orch] launching SCovox node(s) for mode=$MODE in the scovox container (detached)..."
dc_scovox exec -d scovox bash /scovox/scripts/glim/launch_scovox.sh "$MODE"

echo "[orch] giving SCovox a few seconds to subscribe to /glim_ros/points + TF..."
sleep 5

echo "[orch] running GLIM driver in glim_loc: bash /ws/scripts/glim/$DRIVER $*"
dc_glim exec glim bash "/ws/scripts/glim/$DRIVER" "$@"

echo "[orch] driver finished. GLIM + SCovox outputs are in $GLIM_DIR/output/."
# trap cleanup() stops the scovox node on exit.
