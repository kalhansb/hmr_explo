#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# explo_planner + SCovox over the CURTMINI bag (2026_07_31_11_24_12__kalhan_2_).
#
# WHY THIS IS NOT run_explo_experiment.sh:
#
#  1. NO LOCALIZER. That script runs EKF + NDT against gt_map_us050.pcd to
#     synthesise map->odom->base_link, and publishes three static extrinsics,
#     because the map-test-2 bag carries raw sensors only. THIS bag was recorded
#     during a live autonomous run and already contains the whole tree:
#         map_global -> map        (identity)
#         map_global -> map_curt -> odom_curt -> base_link_curt -> os_sensor -> os_lidar
#     so we simply replay /tf + /tf_static. No NDT, no gt map, no extrinsics.
#     (Verified: base_link_curt->os_sensor is (0.1105, 0, 0.404) yaw 180 — the
#     exact extrinsic run_explo_experiment.sh hardcodes. Same rig, same site.)
#
#  2. NO COMPOSE. This host has docker.io only — no `docker compose` plugin —
#     so the compose.yaml flags are inlined as `docker run` below.
#
#  3. VIA THE DSCOVOX MERGER, NOT THE DIRECT NODE. explo_planner subscribes to
#     the map with KeepLast(1).reliable().TRANSIENT_LOCAL
#     (explo_planner_node.cpp:1118). dscovox_node (the merger) publishes with
#     exactly that (dscovox_node.cpp:209, whose comment warns "Any subscriber
#     MUST match this QoS or it receives nothing"), but scovox_node — the direct
#     mapper — publishes ~/scovox via create_publisher(sm_t, 10), i.e. RELIABLE
#     but VOLATILE (scovox_node.cpp:759). Volatile pub + transient_local sub do
#     NOT match, so get_subscription_count() stays 0 and publishScovoxMap()
#     returns at its first line every tick. Verified live: `topic hz` silent,
#     planner pinned at "Waiting to start: map=0 pose=1" for a whole run.
#     => the planner can only ever be fed by the merger. This is why BOTH
#     committed runbooks (dscovox_exploration_run.md / _exploitation_run.md)
#     override dscovox_topic to /robot1/dscovox_node/scovox. NOTE this makes the
#     committed exploration_fused_bag.yaml default (/scovox_node/scovox, used by
#     run_explo_experiment.sh) unable to deliver a map — a latent repo bug,
#     unrelated to this bag.
#
#  4. ONE CONTAINER. No hmr_loc (nothing to localize) and no hmr_seg: this bag
#     recorded depth ONLY as .../image_raw/compressed, and the seg node wants
#     raw depth. The planner reads Beta occupancy only, so semantics are not
#     needed to exercise it — this is the LIDAR_ONLY=1 path.
#
# FRAME / TOPIC DELTAS vs the committed param files (applied as -p overrides):
#     base_frame:  base_link  ->  base_link_curt      (planner + scovox)
#     imu_topic:   /imu/data  ->  /curt/imu/data      (deskew gyro source)
#   unchanged and correct already: map_frame/integration_frame = map,
#   lidar_base_frame = os_lidar, input_pointcloud_topic = /ouster/points.
#
# NOTE the bag also contains /goal_pose, /plan and /dscovox_node/scovox from the
# ORIGINAL live run. They are deliberately NOT replayed (explicit --topics list)
# so they cannot collide with the goals our planner publishes.
#
# Usage (HOST; needs docker, NOT ROS):
#   ./run_explo_curtmini.sh [playback_duration_s]   # empty = full bag (~416 s)
#   RVIZ=0 ./run_explo_curtmini.sh 120              # headless (CSV/log only —
#                                                   # marker publishing is
#                                                   # subscriber-gated!)
#   RATE=0.5 ./run_explo_curtmini.sh                # faster/slower replay
#   MAX_RANGE=20.0 ./run_explo_curtmini.sh          # committed-config range (slow!)
#   REBUILD=1 ./run_explo_curtmini.sh               # force workspace rebuilds
#   SOFTGL=1 ./run_explo_curtmini.sh                # llvmpipe if GL misbehaves
# ---------------------------------------------------------------------------
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCOVOX_DIR="$HERE/scovox"
BAGS_DIR="$(cd "$HERE/../.." && pwd)/bags"
BAG="/scovox/bags/2026_07_31_11_24_12__kalhan_2_CURTMINI"
CTR="scovox_curt"

DUR="${1:-}"; DUR_ARG=""; [ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"
# PERFORMANCE — both of these matter on this host (15 GB RAM, no swap):
#   max_range 20 m (the committed config) with carve_band -1.0 at 0.1 m marches
#   ~200 voxels/ray and costs frame_ms~1300 against a ~104 ms budget at rate 1.0.
#   The SingleThreadedExecutor then never drains, so the 1 Hz sm_timer_ that
#   publishes ~/scovox is starved and the planner sits in WAIT_FOR_MAP forever
#   (map=0) — even though integration itself is fine (TF_FAILED=0).
#   15 m is the same limit dscovox_exploitation_run.md calls "not optional".
RATE="${RATE:-0.2}"
MAX_RANGE="${MAX_RANGE:-15.0}"
RVIZ="${RVIZ:-1}"; [ "$RVIZ" = "0" ] && RVIZ=""
REBUILD="${REBUILD:-}"
KERNEL_L="${KERNEL_L:-0.4}"
CARVE_BAND="${CARVE_BAND:--1.0}"   # full-ray: frontier + EIG raycasts need carved free space

dex() { docker exec "$CTR" bash -lc "$1"; }

# --- container up (compose.yaml flags inlined) -----------------------------
if ! docker ps --format '{{.Names}}' | grep -qx "$CTR"; then
  docker rm -f "$CTR" >/dev/null 2>&1 || true
  echo "[orch] starting $CTR…"
  docker run -d --name "$CTR" \
    --network host --ipc host --runtime nvidia \
    -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -e RCUTILS_COLORIZED_OUTPUT=1 \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    -v "$SCOVOX_DIR":/scovox \
    -v "$BAGS_DIR":/scovox/bags:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -w /scovox scovox:jazzy sleep infinity >/dev/null
else
  echo "[orch] $CTR already running."
fi
[ -n "$RVIZ" ] && { xhost +local:root >/dev/null 2>&1 || true; }

dex "test -e '$BAG/metadata.yaml'" || { echo "[orch] BAG NOT FOUND: $BAG"; exit 1; }

# --- 0a. scovox workspace build (install/ is gitignored → empty on a fresh clone)
if [ -n "$REBUILD" ] || ! dex 'test -x /scovox/install/scovox_mapping/lib/scovox_mapping/scovox_mapping_node'; then
  echo "[orch] building scovox workspace (first run; several minutes)…"
  dex 'source /opt/ros/jazzy/setup.bash && cd /scovox &&
       colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'
else
  echo "[orch] scovox_mapping_node present — skipping build (REBUILD=1 to force)."
fi

# --- 0b. explo_planner overlay ---------------------------------------------
# nav2_msgs is a hard dep and is NOT in the scovox:jazzy image; --packages-up-to
# (not --packages-select) so the sibling explo_planner_msgs builds too.
PLANNER_BIN=/tmp/ovl/install/explo_planner/lib/explo_planner/explo_planner_node
OVL_SHARE=/tmp/ovl/install/explo_planner/share/explo_planner
if [ -n "$REBUILD" ] || ! dex "test -x $PLANNER_BIN"; then
  echo "[orch] installing nav2_msgs + building explo_planner overlay…"
  dex 'dpkg -s ros-jazzy-nav2-msgs >/dev/null 2>&1 ||
       { apt-get update && apt-get install -y ros-jazzy-nav2-msgs; }'
  tar --exclude=.git --exclude=build --exclude=install --exclude=log \
      -C "$HERE" -cf - explo_planner | \
    docker exec -i "$CTR" bash -c \
      'mkdir -p /tmp/ovl/src && rm -rf /tmp/ovl/src/explo_planner && tar -C /tmp/ovl/src -xf -'
  dex 'source /opt/ros/jazzy/setup.bash && source /scovox/install/setup.bash &&
       cd /tmp/ovl && colcon build --packages-up-to explo_planner \
         --cmake-args -DCMAKE_BUILD_TYPE=Release'
else
  echo "[orch] explo_planner overlay present — skipping build (REBUILD=1 to force)."
fi

# Params + RViz layouts live in the explo_planner repo and reach the container
# only via install(DIRECTORY config launch rviz ...) in its CMakeLists. The build
# above is SKIPPED whenever the binary already exists, so an edited or newly
# added config would otherwise never arrive. Re-sync unconditionally: a few kB
# of tar, and it turns parameter tuning into edit-and-rerun with no rebuild.
tar -C "$HERE/explo_planner/explo_planner" -cf - config rviz | \
  docker exec -i "$CTR" bash -c "mkdir -p $OVL_SHARE && tar -C $OVL_SHARE -xf -"

# NOT `pkill -f`: the [b]racket trick fails when one pattern's literal text
# contains another's target ("[d]scovox_node" on this very command line matches
# the regex [s]covox_mapping_node's sibling patterns), so pkill SIGKILLs the loop
# before it reaches rviz2 and every run leaks its window. pgrep + self/parent skip.
dex 'for p in "ros2 bag play" "ros2 launch" "scovox_mapping_node" "dscovox_node" \
             "explo_planner_node" "rviz2"; do
       for pid in $(pgrep -f "$p"); do
         [ "$pid" = "$$" ] || [ "$pid" = "$PPID" ] || kill -9 "$pid" 2>/dev/null
       done
     done; sleep 2; true' || true

# --- 1. dscovox single-robot stack: rolling mapper + merger ----------------
# The launch pins mode:=rolling (so scovox_bin exists at all) and starts the
# merger, whose transient_local ~/scovox is the only thing the planner can hear.
# base_frame here is the LiDAR RAY-ORIGIN frame: with fuse_lidar_rgbd=false the
# node uses base_frame_ as the observation frame, so os_lidar is correct and
# base_link_curt is NOT wanted here (the planner still needs it, step 2).
echo "[orch] launching dscovox mapper + merger (max_range=$MAX_RANGE)…"
docker exec -d "$CTR" bash -lc "
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  exec ros2 launch scovox_mapping dscovox_single_robot.launch.py \
    robot:=robot1 \
    cloud_topic:=/ouster/points \
    imu_topic:=/curt/imu/data \
    base_frame:=os_lidar \
    use_sim_time:=true \
    max_range:=$MAX_RANGE \
    > /tmp/scovox.log 2>&1
"

# --- 2. explo_planner ------------------------------------------------------
# roi_min_z: the committed yaml ships -5.0, but terrain_relative_z needs
# roi_min_z <= -5.125 to fit the ground-search window plus the 1.125 re-band
# hysteresis. At -5.0 the planner warns that groundZAt can return NaN outside
# the ingested slab on slopes and vantages get rejected for want of a ground
# height. -5.5 clears it; roi_max_z 4.0 is already past the 2.125 it needs.
echo "[orch] launching explo_planner…"
docker exec -d "$CTR" bash -lc '
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  source /tmp/ovl/install/setup.bash
  exec ros2 run explo_planner explo_planner_node --ros-args \
    --params-file /tmp/ovl/install/explo_planner/share/explo_planner/config/exploration_fused_bag.yaml \
    -p dscovox_topic:=/robot1/dscovox_node/scovox \
    -p roi_min_z:=-5.5 \
    -p base_frame:=base_link_curt \
    -p output_csv:=/tmp/exploration_curtmini.csv \
    > /tmp/explo.log 2>&1
'

# --- 3. RViz (REQUIRED to see markers: publishCandidateViz is subscriber-gated)
if [ -n "$RVIZ" ]; then
  echo "[orch] launching RViz…"
  GLENV='export __NV_PRIME_RENDER_OFFLOAD=1; export __GLX_VENDOR_LIBRARY_NAME=nvidia'
  [ "${SOFTGL:-0}" = "1" ] && GLENV='export LIBGL_ALWAYS_SOFTWARE=1; unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD'
  docker exec -d "$CTR" bash -lc "
    source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
    $GLENV
    exec rviz2 -d /tmp/ovl/install/explo_planner/share/explo_planner/rviz/explo_experiment_dscovox.rviz --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1
  "
fi

echo "[orch] letting nodes subscribe (6 s)…"
sleep 6

# --- 4. play the bag LAST (starts /clock) ----------------------------------
echo "[orch] playing bag ${DUR:+(first ${DUR}s) }at rate $RATE…"
docker exec "$CTR" bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 bag play $BAG --clock --rate $RATE $DUR_ARG \
    --topics /ouster/points /curt/imu/data /tf /tf_static
"

# --- 5. diagnostics --------------------------------------------------------
echo "[orch] bag finished — diagnostics:"
echo "----- scovox mapper (frame_ms must stay under the budget) -----"
dex 'grep -a "frame_ms" /tmp/scovox.log | tail -3
     echo "TF_FAILED_count=$(grep -acE "TF.FAILED|DROPPING" /tmp/scovox.log 2>/dev/null)"' || true
echo "----- dscovox merger (want sources=1 fused_voxels>0) -----"
dex 'grep -a "dscovox_diag" /tmp/scovox.log | tail -3' || true
echo "----- explo_planner -----"
dex 'grep -aE "planner ready|Waiting|selected goal|candidates rejected|Navigation|DONE" /tmp/explo.log | tail -15
     echo "plan_steps=$(grep -ac "selected goal" /tmp/explo.log 2>/dev/null)"
     echo "all_rejected_ticks=$(grep -ac "candidates rejected" /tmp/explo.log 2>/dev/null)"' || true
echo "[orch] per-step CSV: /tmp/exploration_curtmini.csv inside $CTR"
echo "[orch] nodes + RViz left running. Tear down with:  docker rm -f $CTR"
