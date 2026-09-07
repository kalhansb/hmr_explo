#!/usr/bin/env bash
# =======================================================================
# TWO-ROBOT (bunker + curt) DISTRIBUTED DSCovox: per-robot mapper, per-robot
# merger fed by BOTH robots' deltas, per-robot explo_planner. 2 of everything.
# =======================================================================
# Companion to run_explo_curtmini.sh (single robot). What differs, and why:
#
# 1. TWO LIVE MAPPERS, NOT THE RECORDED BINS. Both bags DO contain their
#    robot's ~/scovox_bin delta stream, and replaying those straight into the
#    merger would skip all carve cost — but it does not work: the recorded
#    blobs are codec revision 5 and this tree's BinarySerializer is at
#    FORMAT_VERSION=7 (5->6 block-run coords + u16 quantization, 6->7 fine-TSDF
#    band). deserialize rejects the VERSION byte by design, so the merger sits
#    at sources=0 logging "deserialize failed for 'map': bad VERSION". There is
#    no v5 decode path in the tree. So we re-map from raw LiDAR instead, which
#    also produces v7 bins: bunker from /hesai/points, curt from
#    /ouster/points. That is why RATE defaults to 0.2 here — the full-ray
#    carve is running twice over.
#
# 2. DISTINCT SOURCE KEYS COME FROM integration_frame. The merger keys sources
#    by the bin's header.frame_id, which scovox_node sets to integration_frame
#    ("unique per robot in a fleet", per the launch file's own arg doc). Both
#    robots' maps are geometrically in `map`, so pointing both mappers at
#    integration_frame:=map would collapse them to ONE source. Each therefore
#    gets its own identity-bridged alias (map_int_bunker / map_int_curt); the
#    launch auto-publishes the identity static map->alias when
#    integration_frame != map_frame, so the geometry is untouched and the
#    merger sees a genuine sources=2.
#
# 3. THE 27-MINUTE GAP DOES NOT MATTER. bunker ran 10:57:06-11:04:14 and curt
#    11:24:13-11:31:09 — sequential sorties, zero overlap. Only ONE bag may
#    drive /clock (two --clock publishers fight over sim time), so curt gets it
#    and bunker rides a clock ~27 min ahead of its own stamps. That is safe
#    because the two robots' DYNAMIC TF trees are disjoint:
#      bunker  map -> odom -> base_link
#      curt    map_curt -> odom_curt -> base_link_curt   (map->map_curt static)
#    tf2 prunes per child-frame cache, so curt's newer stamps cannot evict
#    bunker's older ones, and every lookup is at an explicit stamp (the
#    mappers, tf_require_exact:=true) or TimePointZero (the planners,
#    explo_planner_node.cpp:3919) — never against now().
#
# 4. TWO OF EVERYTHING — NO SHARED MAP OBJECT. Each robot runs its own merger
#    subscribing BOTH bin streams, and its planner reads only that merger
#    (`dscovox_topic:=/<robot>/dscovox_node/scovox`). Nothing in the loop
#    depends on a central node: the only thing crossing between robots is the
#    bin delta stream, exactly as it would over a real radio link. The two
#    fused copies are independent state and may differ transiently. Planners
#    differ otherwise only in robot_name / base_frame / goal_topic; node names
#    are remapped so their ~/candidates markers do not collide, and
#    coordination_enabled:=true wires both to the shared /exploration/intents
#    topic so each sees the other's claimed goal instead of racing it.
#    FUSION=central falls back to one shared /fused merger as an A/B baseline.
#
# Usage:
#   ./run_explo_dual.sh            # full bags (~420 s each) at rate 0.2
#   ./run_explo_dual.sh 120        # first 120 s only
#   RATE=0.1 ./run_explo_dual.sh   # slower (if frames are being dropped)
#   RVIZ=0 ./run_explo_dual.sh 90  # headless
#   FUSION=central ./run_explo_dual.sh 200   # one shared merger instead of two
# =======================================================================
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCOVOX_DIR="$HERE/scovox"
BAGS_DIR="$(cd "$HERE/../.." && pwd)/bags"
CTR="scovox_curt"

BAG_BUNKER="/scovox/bags/2026_07_31_10_57_06__kalhan_2_"
BAG_CURT="/scovox/bags/2026_07_31_11_24_12__kalhan_2_CURTMINI"
BIN_BUNKER="/bunker/scovox_node/scovox_bin"
BIN_CURT="/curt/scovox_node/scovox_bin"

DUR="${1:-}"
FUSION="${FUSION:-distributed}"   # distributed = one merger PER ROBOT; central = one shared
RATE="${RATE:-0.2}"
MAX_RANGE="${MAX_RANGE:-15.0}"
RVIZ="${RVIZ:-1}"; [ "$RVIZ" = "0" ] && RVIZ=""
DUR_ARG=""; [ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"

dex() { docker exec "$CTR" bash -lc "$1"; }

# --- container up ----------------------------------------------------------
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

for b in "$BAG_BUNKER" "$BAG_CURT"; do
  dex "test -e '$b/metadata.yaml'" || { echo "[orch] BAG NOT FOUND: $b"; exit 1; }
done

# --- builds (same as the single-robot script) ------------------------------
if [ -n "${REBUILD:-}" ] || ! dex 'test -x /scovox/install/scovox_mapping/lib/scovox_mapping/dscovox_mapping_node'; then
  echo "[orch] building scovox workspace (several minutes)…"
  dex 'source /opt/ros/jazzy/setup.bash && cd /scovox &&
       colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'
else
  echo "[orch] dscovox_mapping_node present — skipping build (REBUILD=1 to force)."
fi

PLANNER_BIN=/tmp/ovl/install/explo_planner/lib/explo_planner/explo_planner_node
OVL_SHARE=/tmp/ovl/install/explo_planner/share/explo_planner
if [ -n "${REBUILD:-}" ] || ! dex "test -x $PLANNER_BIN"; then
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

# --- clean slate -----------------------------------------------------------
# NOT `pkill -f`: the [b]racket trick fails here because one pattern's literal
# text contains another's target — the shell's own command line carries
# "[d]scovox_mapping_node", which the regex [s]covox_mapping_node matches. pkill
# then SIGKILLs the loop itself (rc=137) before it reaches the later patterns,
# which is why every earlier run leaked its rviz2. pgrep + an explicit self/parent
# skip is immune to that.
# "ros2 launch" and "static_transform_publisher" matter as much as the node
# names: killing scovox_mapping_node leaves its launch supervisor and the
# identity map->map_int_* publishers the launch spawned still running.
dex 'for p in "ros2 bag play" "ros2 launch" "explo_planner_node" \
              "dscovox_mapping_node" "scovox_mapping_node" \
              "static_transform_publisher" "rviz2"; do
       for pid in $(pgrep -f "$p"); do
         [ "$pid" = "$$" ] || [ "$pid" = "$PPID" ] || kill -9 "$pid" 2>/dev/null
       done
     done; true' >/dev/null 2>&1
sleep 2

# --- 1. one live mapper per robot ------------------------------------------
# with_merger:=false — the launch's built-in merger subscribes ONLY its own
# robot's bin (input_topics: [bin_topic]), which is precisely what a distributed
# run must not do. We start the mergers ourselves in step 2 with both inputs.
# base_frame is the LiDAR
# RAY-ORIGIN frame (fuse_lidar_rgbd=false makes base_frame the observation
# frame), so it is the sensor frame, not the robot body: hesai_lidar for
# bunker, os_lidar for curt. integration_frame is the per-robot alias that
# becomes the merger's source key (see header note 2).
launch_mapper() {  # $1=robot $2=cloud_topic $3=imu_topic $4=lidar_frame $5=int_frame
  echo "[orch] launching $1 mapper ($2 @ $4, max_range=$MAX_RANGE)…"
  docker exec -d "$CTR" bash -lc "
    source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
    exec ros2 launch scovox_mapping dscovox_single_robot.launch.py \
      robot:=$1 \
      cloud_topic:=$2 \
      imu_topic:=$3 \
      base_frame:=$4 \
      integration_frame:=$5 \
      map_frame:=map \
      with_merger:=false \
      use_sim_time:=true \
      max_range:=$MAX_RANGE \
      > /tmp/scovox_$1.log 2>&1
  "
}
launch_mapper bunker /hesai/points  /imu/data      hesai_lidar map_int_bunker
launch_mapper curt   /ouster/points /curt/imu/data os_lidar    map_int_curt
sleep 4

# --- 2. the mergers --------------------------------------------------------
# DISTRIBUTED (default): one dscovox_mapping_node PER ROBOT, each subscribing
# BOTH bin streams. That is the real fleet topology — every robot maintains its
# own copy of the shared world model from the deltas its peers broadcast, and
# nothing in the loop depends on a central node staying up. The two copies are
# independent state: they can disagree transiently (different arrival order,
# different first-frame pinning) and each planner sees only its own.
# CENTRAL (FUSION=central): a single /fused merger both planners read, kept as
# an A/B baseline — same inputs, one shared copy.
# map_frame=map either way: both mappers integrate into an identity-bridged
# alias of map, so each source's carried map_from_source is identity and every
# fold lands in the world frame the planners and RViz use.
launch_merger() {  # $1=namespace  $2=logname
  echo "[orch] launching merger /$1/dscovox_node (inputs: bunker + curt)…"
  docker exec -d "$CTR" bash -lc "
    source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
    exec ros2 run scovox_mapping dscovox_mapping_node --ros-args \
      -r __ns:=/$1 -r __node:=dscovox_node \
      -p use_sim_time:=true \
      -p 'input_topics:=[$BIN_BUNKER, $BIN_CURT]' \
      -p map_frame:=map \
      -p pointcloud_min_interval_s:=1.0 \
      > /tmp/merger_$2.log 2>&1
  "
}
if [ "$FUSION" = "distributed" ]; then
  launch_merger bunker bunker
  launch_merger curt   curt
  MERGERS="bunker curt"
else
  launch_merger fused fused
  MERGERS="fused"
fi
sleep 4

# --- 3. one planner per robot, each over ITS OWN fused map ------------------
# In distributed mode dscovox_topic is the robot's own merger, so a planner
# never reads a peer's map object — only the bin deltas that reached its own
# merger. __node remap keeps ~/candidates distinct. roi_min_z -5.5 for
# terrain_relative_z (the shipped -5.0 is too tight; see run_explo_curtmini.sh).
# Per-robot survey pace (m/s) feeding the nav budget. Bunker Mini is tracked
# and CURT Mini is wheeled, so they need not share a number — but both are
# skid-steer and neither can strafe. Values are the MEASURED mean speed of
# each platform over its own bag (bunker 0.223 m/s from its coherent 10 Hz
# 3D odom source, curt 0.203 m/s), not an assumption.
SPEED_BUNKER="${SPEED_BUNKER:-0.22}"
SPEED_CURT="${SPEED_CURT:-0.20}"

launch_planner() {  # $1=robot  $2=base_frame  $3=node_name  $4=nav_speed_mps
  [ "$FUSION" = "distributed" ] && MAPNS="$1" || MAPNS="fused"
  echo "[orch] launching explo_planner for $1 (base=$2, map=/$MAPNS/dscovox_node/scovox, speed=$4 m/s)…"
  docker exec -d "$CTR" bash -lc "
    source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
    source /tmp/ovl/install/setup.bash
    exec ros2 run explo_planner explo_planner_node --ros-args \
      -r __node:=$3 \
      --params-file /tmp/ovl/install/explo_planner/share/explo_planner/config/exploration_fused_bag.yaml \
      -p use_sim_time:=true \
      -p nav_speed_estimate_mps:=$4 \
      -p robot_name:=$1 \
      -p base_frame:=$2 \
      -p map_frame:=map \
      -p dscovox_topic:=/$MAPNS/dscovox_node/scovox \
      -p goal_topic:=/$1/goal_pose \
      -p coordination_enabled:=true \
      -p roi_min_z:=-5.5 \
      -p output_csv:=/tmp/explo_$1.csv \
      > /tmp/explo_$1.log 2>&1
  "
}
launch_planner bunker base_link      explo_planner_bunker $SPEED_BUNKER
launch_planner curt   base_link_curt explo_planner_curt $SPEED_CURT
# --- 4. RViz ---------------------------------------------------------------
if [ -n "$RVIZ" ]; then
  echo "[orch] launching RViz (dual layout)…"
  GLENV='export __NV_PRIME_RENDER_OFFLOAD=1; export __GLX_VENDOR_LIBRARY_NAME=nvidia'
  [ "${SOFTGL:-0}" = "1" ] && GLENV='export LIBGL_ALWAYS_SOFTWARE=1; unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD'
  docker exec -d "$CTR" bash -lc "
    source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
    $GLENV
    exec rviz2 -d /tmp/ovl/install/explo_planner/share/explo_planner/rviz/explo_experiment_dual.rviz --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1
  "
fi

echo "[orch] letting subscribers join (8 s)…"
sleep 8

# --- 5. play BOTH bags -----------------------------------------------------
# curt drives /clock; bunker must NOT (two clock publishers fight) — see header
# note 3 for why the 27-minute stamp gap is harmless. Replayed topics are the
# raw LiDAR + IMU + TF each mapper needs; the recorded scovox_bin streams are
# deliberately NOT replayed (header note 1: codec v5, this build wants v7).
echo "[orch] playing BOTH bags ${DUR:+(first ${DUR}s) }at rate $RATE…"
docker exec -d "$CTR" bash -lc "
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  ros2 bag play $BAG_BUNKER --rate $RATE $DUR_ARG \
    --topics /hesai/points /imu/data /tf /tf_static > /tmp/play_bunker.log 2>&1
"
docker exec "$CTR" bash -lc "
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  ros2 bag play $BAG_CURT --clock --rate $RATE $DUR_ARG \
    --topics /ouster/points /curt/imu/data /tf /tf_static
"

# --- 6. diagnostics --------------------------------------------------------
echo "[orch] bags finished — diagnostics:"
for m in $MERGERS; do
  echo "----- merger /$m (WANT sources=2) -----"
  dex "grep -aE 'dscovox_diag|pinned num_classes|prior mismatch|dropping|deserialize failed' /tmp/merger_$m.log | tail -4" || true
done
for r in bunker curt; do
  echo "----- $r mapper -----"
  dex "grep -aE 'frame_ms|TF_FAILED|DROPPING|gated' /tmp/scovox_$r.log | tail -3" || true
done
for r in bunker curt; do
  echo "----- explo_planner $r -----"
  dex "grep -aE 'planner ready|Waiting|selected goal|candidates rejected' /tmp/explo_$r.log | tail -6
       echo \"plan_steps=\$(grep -ac 'selected goal' /tmp/explo_$r.log 2>/dev/null)\"" || true
done
echo "[orch] CSVs: /tmp/explo_bunker.csv /tmp/explo_curt.csv inside $CTR"
echo "[orch] nodes + RViz left running. Tear down with:  docker rm -f $CTR"
