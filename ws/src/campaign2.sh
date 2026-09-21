#!/bin/bash
# Exploitation map-gain campaign: 10 cells, arms alternated, exploit-off first
# so the §7 pilot gates (P1/P2/P3) can be read off the first three off cells.
# One container per cell; cells are strictly sequential (the box has 8 cores and
# one cell already saturates them at RTF ~0.6).
set -u
P=/home/kalhan/Documents/exploitation_experiments
S="$(cd "$(dirname "$0")" && pwd)"
IDX="$P/runs/index.csv"
[ -f "$IDX" ] || echo "cell,arm,exploit,rep,started,ended,rc,dir" > "$IDX"

# DONE_UNKNOWN=0 disables the coverage stop latch (explo_planner_node.cpp
# returns false at <=0), so a cell runs the full DURATION_S instead of ending
# the moment the ROI unknown fraction crosses the threshold.
#
# Why this is necessary, not a convenience. The exploitation plan's horizons
# start at 600 s, its primary endpoint is at 1500 s, and its third target does
# not release until ~t_sim 750. A cell that latches DONE before those moments
# loses target 3 outright and fails the manipulation check, and no horizon past
# the first would exist. Running to T and reading the crossing afterwards keeps
# every horizon available in both arms.
#
# max_steps (500) is then the only stop. It does not bind: ~27 s/step means
# 3600 s is ~133 steps.
#
# The cost is that no run ever DECLARES done, which is one of H2's cost
# endpoints. It is recovered offline instead, identically for both arms.
#
# C4 (2026-09-21): the ORIGINAL version of this note justified the choice in 3D
# terms -- a latch at 0.64 firing near t_sim 485, and an "achievable floor of
# ~0.486 unknown" said to be a property of this world. Both numbers came from
# the 3D column measure, which has since been retired (plan section 6.6): it
# counts unobservable z-column volume, so it saturates near 0.50 and stops
# responding to exploration entirely from ~1500 s while the 2D planning map
# over the SAME sensor data keeps falling to ~0.06. The 0.486 "floor" was that
# artifact, not the world.
#
# What this does NOT change is the decision above. Disabling the latch is still
# right, and for the same reason: the run has to outlive the plan's horizons.
# It is if anything more clearly right on the 2D measure, where 90% known is
# reached near t_sim 1300 -- still before the 1500 s primary endpoint.
#
# What it DOES change is the offline recovery. The per-step unknown_fraction in
# planner_<robot>.csv is the retired 3D column series and is NOT the quantity
# the crossing is read on any more. Each cell is re-scored from its own bagged
# scovox_bin streams into <cell>/coverage_2d.csv, and aggregate.py reads that
# and leaves the cell blank when it is missing rather than falling back.

run_cell () {
  local arm="$1" rep="$2" exploit="$3"
  local name="${arm}_rep${rep}"
  local dir="$P/runs/$name"
  # Resume-safe, on the COMPLETION marker rather than the manifest.
  #
  # The manifest is written at run START, so guarding on it treats a cell that
  # died at t=900s as finished. That is not hypothetical: it is how the
  # truncated off_rep3 got banked into index.csv and had to be pulled back out
  # (runs/_truncated_off_rep3/WHY_TRUNCATED.txt). The runner logs "teardown
  # complete" only after every process has been stopped in order, so that line
  # is the one thing that means the cell actually ran to its end.
  if grep -q "teardown complete" "$P/runs/$name.console.log" 2>/dev/null; then
    echo "[skip] $name (complete)"; return 0
  fi
  if [ -d "$dir" ]; then
    echo "[redo] $name — present but never reached teardown; re-running"
  fi
  rm -rf "$dir"; mkdir -p "$dir"
  echo "[start] $name $(date -Is)"
  local t0; t0=$(date -Is)
  # Host-side provenance. The in-container manifest can report the four
  # SUBMODULES correctly (their .git files resolve once /.git is mounted), but
  # NOT the superproject: its worktree inside the container is /, where every
  # path outside /ws reads as deleted, so it would always say -dirty. Capture
  # the real superproject state here, on the host, where the worktree is whole.
  git -C "$P" describe --always --dirty > "$dir/git_superproject.txt" 2>&1
  git -C "$P" status --porcelain >> "$dir/git_superproject.txt" 2>&1
  docker rm -f cell >/dev/null 2>&1
  docker run --rm --name cell --cap-add=NET_ADMIN --shm-size=2g \
    -e EXPLOIT="$exploit" -e DURATION_S=3600 -e RECORD=2 -e RVIZ=0 -e GZ_GUI=0 \
    -e RECONNECT_MODE=off -e STOP_ON_DONE=1 -e COMMS=0 -e LINK_GATE=0 \
    -e DONE_UNKNOWN=0 \
    -e OUTDIR="/runs/$name" \
    -e GIT_CONFIG_COUNT=1 -e GIT_CONFIG_KEY_0=safe.directory -e GIT_CONFIG_VALUE_0='*' \
    -v "$P/ws:/ws" -v "$P/.git:/.git:ro" -v "$P/runs:/runs" -v "$S:/sp" -v "$S/gzhome:/gzhome" \
    hmrexplo:humble /sp/cell.sh > "$P/runs/$name.console.log" 2>&1
  local rc=$?
  # Only a cell that reached teardown goes into the index. index.csv is what
  # aggregate.py treats as the campaign, so a truncated cell listed there is a
  # truncated cell silently averaged into an arm.
  if grep -q "teardown complete" "$P/runs/$name.console.log" 2>/dev/null; then
    echo "$name,$arm,$exploit,$rep,$t0,$(date -Is),$rc,$dir" >> "$IDX"
    echo "[done ] $name rc=$rc $(date -Is) $(du -sh "$dir" 2>/dev/null | cut -f1)"
  else
    echo "[TRUNCATED] $name rc=$rc — never reached teardown, NOT indexed"
  fi
}

# Order is doc 6.1, not a plain alternation. The three exploit-off PILOT cells
# run back-to-back first because gates P1-P3 are read off them, and one of those
# gates (P2) chooses the primary metric. Deciding that with treated cells already
# in hand would be choosing the endpoint after seeing the effect, which is the
# thing the plan explicitly forbids. After the pilot the arms alternate so that
# any drift over the campaign's many hours -- thermal, a filling disk, another
# process -- cannot line up with the arm.
SCHEDULE="off:1 off:2 off:3 on:1 off:4 on:2 on:3 off:5 on:4 on:5"
for spec in $SCHEDULE; do
  arm="${spec%%:*}"; rep="${spec##*:}"
  if [ "$arm" = "on" ]; then run_cell on "$rep" 1; else run_cell off "$rep" 0; fi
done
echo "[campaign complete] $(date -Is)"
