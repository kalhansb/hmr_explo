#!/usr/bin/env bash
# Host-side orchestrator for the EXPLORATION-PLANNER visualization experiment:
# the full fused LiDAR+RGB-D pipeline of run_fused_experiment.sh (NDT+EKF
# localization, seg node, ONE fused scovox node) PLUS the explo_planner EIG
# planner running against the LIVE fused map, with its candidate waypoints and
# selected waypoint drawn in RViz as the bag plays.
#
# What you see per PLAN cycle (~every 15-40 s):
#   * /explo_planner/candidates (MarkerArray): one arrow per candidate viewpoint
#     (polar grid around the live pose + frontier centroids from the fused map),
#     colored red->green by EIG utility, + a cyan sphere over the selected goal.
#   * /goal_pose (PoseStamped): the selected NBV waypoint. NOTHING navigates to
#     it — the "robot" replays the bag trajectory — so each NAVIGATE leg ends by
#     timeout/no-progress watchdog and the planner replans from wherever the bag
#     robot is by then. That is the point of the experiment: live candidate
#     generation + selection over a real sensor-built map.
#
# TERRAIN (3D mode): the planner runs terrain-relative (terrain_relative_z in
# the planner yaml): the map z-band rides with the robot, candidates snap to
# local ground + clearance (true 3D waypoints on /goal_pose — nav2 would use
# only x/y/yaw, the z rides along). There is NO 2D planning_map in this run:
# require_planning_map: false puts the planner in its straight-line fallback
# (Euclidean candidate costs, no 2D reachability/free-cell filter) — obstacle
# rejection happens against the 3D map instead (candidate_occ_thresh + ground
# snapping). See scovox/config/exploration_fused_bag.yaml header.
#
# Per-node param sets are versioned in the submodules (this script carries only
# the wiring, extrinsics, and env-knob overrides):
#   hmr_localisation/config/gt_ouster_ndt_tree_fused.yaml       NDT (launch args + extrinsics doc'd in its header)
#   scovox/config/scovox_fused_lidar_rgbd.yaml                  fused mapping node (KERNEL_L/CARVE_BAND override it)
#   scovox/src/seg_pipeline/config/seg_fused_experiment.yaml    seg node (MODEL overrides it)
#   scovox/config/exploration_fused_bag.yaml                    EIG planner (read live via the /scovox mount)
#   scovox/config/explo_experiment.rviz                         RViz layout (read live via the /scovox mount)
#
# Containers / roles (one DDS graph: host net + ipc host + ROS_DOMAIN_ID=0):
#   hmr_loc (svc ros)   : EKF (odom->base_link) + NDT (map->odom, loads gt_map_us050)
#                         + 3 static extrinsic publishers + the bag play
#   hmr_seg (svc seg)   : seg_node -> /scovox/segmentation/colored + reframed depth+info
#   scovox  (svc scovox): fused scovox_mapping_node + explo_planner (built into a
#                         /tmp/ovl overlay ws on first run; EXPLO_REBUILD=1 forces
#                         a re-copy + rebuild after host edits to explo_planner)
#                         + RViz (scovox/config/explo_experiment.rviz)
#
# explo_planner is NOT mounted into any container: it is tar-streamed into the
# scovox container and colcon-built there as an overlay of /scovox/install
# (needs scovox_core + scovox_msgs). The binary persists in the container's
# writable layer across stop/start, so the build runs once (~20 s).
#
# Usage (HOST; needs docker, NOT ROS):
#   ./run_explo_experiment.sh [playback_duration_s]     # empty = full bag (~500 s)
#   RVIZ=0 ./run_explo_experiment.sh 120                # headless (logs/CSV only;
#                                                       # marker publishing is gated
#                                                       # on subscribers, so no RViz
#                                                       # means no marker traffic)
#   LIDAR_ONLY=1 ./run_explo_experiment.sh 180          # skip RGB-D/semantics; the
#                                                       # planner only reads Beta
#                                                       # occupancy, so this is the
#                                                       # cheap planner-focused run
# RVIZ defaults ON here (visualization IS the experiment) and leaves the nodes +
# RViz up after the bag so you can inspect the final candidate set; the script
# prints the docker-compose-stop commands to tear down.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOC_DIR="$HERE/hmr_localisation"
SEG_DIR="$HERE/scovox/src/seg_pipeline"
SCOVOX_DIR="$HERE/scovox"
BAG="/ws/bags/2026_06_19_18_19_06__kalhan-map-test-2_"
DUR="${1:-}"
DUR_ARG=""; [ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"
RVIZ="${RVIZ:-1}"; [ "$RVIZ" = "0" ] && RVIZ=""   # default ON; RVIZ=0 disables
MODEL="${MODEL:-}"                          # MODEL=<hf-id> -> override seg model_name
KERNEL_L="${KERNEL_L:-0.4}"                 # RGB-D->LiDAR BKI spread radius (m)
CARVE_BAND="${CARVE_BAND:--1.0}"            # keep full-ray (-1.0): frontier
                                            # extraction + the EIG raycasts read
                                            # carved free space in the 3D map; a
                                            # bounded band starves them of free voxels
LIDAR_ONLY="${LIDAR_ONLY:-}"                # LIDAR_ONLY=1 -> no seg node / camera
                                            # topics; planner is unaffected (it
                                            # reads Beta occupancy only)
EXPLO_REBUILD="${EXPLO_REBUILD:-}"          # EXPLO_REBUILD=1 -> re-copy + rebuild
                                            # the explo_planner overlay even if the
                                            # binary already exists

dc_loc()    { docker compose -f "$LOC_DIR/compose.yaml"    "$@"; }
dc_seg()    { docker compose -f "$SEG_DIR/compose.yaml"    "$@"; }
dc_scovox() { docker compose -f "$SCOVOX_DIR/compose.yaml" "$@"; }

LEAVE_UP=""   # set at the end in RVIZ mode so cleanup() leaves everything running
stop_cmds() {
  echo "    docker compose -f $SCOVOX_DIR/compose.yaml stop scovox"   # also kills planner + RViz
  echo "    docker compose -f $SEG_DIR/compose.yaml stop seg"
  echo "    docker compose -f $LOC_DIR/compose.yaml stop ros"
}
cleanup() {
  if [ -n "$LEAVE_UP" ]; then
    echo "[orch] RVIZ mode: leaving nodes + RViz running so you can inspect the final map + candidates."
    echo "[orch] stop everything when done with:"; stop_cmds
    return 0
  fi
  echo "[orch] stopping nodes (docker compose stop)…"
  dc_scovox stop scovox 2>/dev/null || true   # stopping scovox also kills planner + RViz
  dc_seg    stop seg    2>/dev/null || true
  dc_loc    stop ros    2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[orch] bringing up containers…"
dc_loc    up -d
if [ -z "$LIDAR_ONLY" ]; then dc_seg up -d; fi   # LIDAR_ONLY skips the seg container entirely
dc_scovox up -d
if [ -n "$RVIZ" ]; then
  echo "[orch] RVIZ on → RViz will run inside the scovox container (GPU/hardware GL); allowing local X11…"
  xhost +local:root >/dev/null 2>&1 || true
fi

# 0) explo_planner overlay build in the scovox container (once; ~20 s).
#    The repo is tar-streamed in (docker-cp-into-existing-dir nests silently)
#    and built against /scovox/install (scovox_core + scovox_msgs underlay).
PLANNER_BIN=/tmp/ovl/install/explo_planner/lib/explo_planner/explo_planner_node
if [ -n "$EXPLO_REBUILD" ] || ! dc_scovox exec -T scovox bash -c "test -x $PLANNER_BIN"; then
  echo "[orch] building explo_planner overlay in scovox (first run or EXPLO_REBUILD=1; ~20 s)…"
  tar --exclude=.git --exclude=build --exclude=install --exclude=log \
      -C "$HERE" -cf - explo_planner | \
    dc_scovox exec -T scovox bash -c \
      'mkdir -p /tmp/ovl/src && rm -rf /tmp/ovl/src/explo_planner && tar -C /tmp/ovl/src -xf -'
  dc_scovox exec -T scovox bash -c '
    source /opt/ros/jazzy/setup.bash && source /scovox/install/setup.bash &&
    cd /tmp/ovl && colcon build --packages-select explo_planner \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  '
else
  echo "[orch] explo_planner overlay binary present — skipping build (EXPLO_REBUILD=1 to force)."
fi

# kill any stray bag play left over from manual probing (the [r] trick avoids the
# shell SIGTERMing itself — see run_seg_experiment.sh).
dc_loc exec -T ros bash -lc 'pkill -f "[r]os2 bag play" 2>/dev/null; sleep 1; true' || true

# 1) localizer stack in hmr_loc: EKF + NDT (map->odom ONLY) + 3 static extrinsics.
#    Identical to run_fused_experiment.sh — see that script + the NDT yaml header
#    for why /tf_static is NOT played and where these extrinsics come from.
echo "[orch] launching EKF + NDT (map->odom) + static extrinsics in hmr_loc…"
dc_loc exec -d ros bash -lc '
  source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml   # SHM for big clouds
  ros2 launch /ws/launch/ekf_odom.launch.py use_sim_time:=true > /tmp/ekf.log 2>&1 &
  ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
    localization_param_dir:=/ws/config/gt_ouster_ndt_tree_fused.yaml \
    cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
    global_frame_id:=map odom_frame_id:=odom base_frame_id:=base_link \
    use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
    publish_lidar_tf:=false publish_imu_tf:=false \
    > /tmp/ndt.log 2>&1 &
  ros2 run tf2_ros static_transform_publisher \
    --x 0.1105 --y 0.0 --z 0.404 --qx 0.0 --qy 0.0 --qz 1.0 --qw 0.0 \
    --frame-id base_link --child-frame-id os_lidar > /tmp/lidartf.log 2>&1 &
  ros2 run tf2_ros static_transform_publisher \
    --x 0.062 --y 0.0 --z 0.015 --qx 0.0 --qy 0.0 --qz 0.7071068 --qw 0.7071068 \
    --frame-id base_link --child-frame-id imu > /tmp/imutf.log 2>&1 &
  ros2 run tf2_ros static_transform_publisher \
    --x 0.270676 --y 0.049297 --z 0.279109 \
    --qx -0.013514 --qy 0.000986 --qz 0.000780 --qw 0.999908 \
    --frame-id base_link --child-frame-id camera_color_frame > /tmp/camtf.log 2>&1 &
  wait
'

echo "[orch] waiting for NDT map load + activation…"
dc_loc exec -T ros bash -lc 'for i in $(seq 1 60); do grep -aq "Activating end" /tmp/ndt.log 2>/dev/null && { echo NDT_ACTIVE; exit 0; }; sleep 1; done; echo NDT_TIMEOUT; tail -5 /tmp/ndt.log'

# 2) online seg node in hmr_seg (default output_frame=camera_color_frame).
if [ -z "$LIDAR_ONLY" ]; then
  echo "[orch] launching seg node in hmr_seg${MODEL:+ (model=$MODEL)}…"
  dc_seg exec -d seg bash -lc "
    source /opt/ros/jazzy/setup.bash
    cd /seg && exec python3 -m seg_pipeline.seg_node --ros-args --params-file /seg/config/seg_fused_experiment.yaml ${MODEL:+-p model_name:=$MODEL} > /root/seg.log 2>&1
  "
else
  echo "[orch] LIDAR_ONLY=1 → skipping seg node (no RGB-D/semantics; scovox maps LiDAR only)."
fi

# 3) SCovox FUSED node in scovox — run_fused_experiment.sh params file. The 2D
#    planning_map stays OFF: the planner runs require_planning_map: false and
#    scores candidates by straight-line distance (live/GT 2D maps both proved
#    too sparse on this site — see the planner yaml header), so nothing
#    consumes the projection and publishing it would only burn CPU.
echo "[orch] launching SCovox FUSED node in scovox…"
dc_scovox exec -d scovox bash -lc '
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  exec ros2 run scovox_mapping scovox_mapping_node --ros-args -r __node:=scovox_node \
    --params-file /scovox/config/scovox_fused_lidar_rgbd.yaml \
    -p rgbd_kernel_radius:='"$KERNEL_L"' \
    -p carve_band:='"$CARVE_BAND"' \
    -p publish_planning_map:=false \
    > /tmp/scovox.log 2>&1
'

# 3b) explo_planner in scovox, against the LIVE fused map. The params file
#     lives in the scovox repo (config/exploration_fused_bag.yaml), which is
#     bind-mounted at /scovox — host edits apply on the next run with no
#     rebuild or copy (the overlay install's own copy would go stale). Node
#     name is fixed "explo_planner" (constructor), matching the yaml key. It
#     idles in WAIT_FOR_MAP (sim-time clock isn't even running) until the bag
#     brings up /clock, TF and the map.
echo "[orch] launching explo_planner in scovox…"
dc_scovox exec -d scovox bash -lc '
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  source /tmp/ovl/install/setup.bash
  exec ros2 run explo_planner explo_planner_node --ros-args \
    --params-file /scovox/config/exploration_fused_bag.yaml > /tmp/explo.log 2>&1
'

# 3c) RViz inside the scovox container (candidates + selected goal + semantic
#     cloud; config scovox/config/explo_experiment.rviz, read live via the
#     /scovox mount). NOTE: the planner gates marker publishing on subscriber
#     count, so without RViz (RVIZ=0) /explo_planner/candidates stays silent —
#     use the log + CSV instead.
if [ -n "$RVIZ" ]; then
  echo "[orch] launching RViz inside the scovox container (config: scovox/config/explo_experiment.rviz)…"
  GLENV='export __NV_PRIME_RENDER_OFFLOAD=1; export __GLX_VENDOR_LIBRARY_NAME=nvidia'
  [ "${SOFTGL:-0}" = "1" ] && GLENV='export LIBGL_ALWAYS_SOFTWARE=1; unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD'
  dc_scovox exec -d scovox bash -lc "
    source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
    export DISPLAY=\"\${DISPLAY:-:1}\"
    $GLENV
    exec rviz2 -d /scovox/config/explo_experiment.rviz --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1
  "
fi

echo "[orch] letting seg + scovox + planner subscribe (6 s)…"
sleep 6

# 4) play the bag: feeds NDT (/ouster reliable via SHM, /imu), scovox LiDAR
#    (/ouster, /imu for deskew) AND seg (camera depth+color). NOT /tf, NOT
#    /tf_static (see run_fused_experiment.sh header).
CAM_TOPICS="/camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info /camera/color/image_raw/compressed"
[ -n "$LIDAR_ONLY" ] && CAM_TOPICS=""
echo "[orch] playing bag ${DUR:+(first ${DUR}s)}${LIDAR_ONLY:+ [LIDAR_ONLY]}…"
dc_loc exec -T ros bash -lc "
  source /opt/ros/jazzy/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml
  ros2 bag play $BAG --clock --rate 1.0 $DUR_ARG \
    --topics /ouster/points /imu/data $CAM_TOPICS \
    --qos-profile-overrides-path /ws/config/ouster_reliable_qos.yaml
"

echo "[orch] bag finished — diagnostics:"
echo "----- NDT good scans -----"
dc_loc    exec -T ros    bash -lc 'echo "good_scans=$(grep -acE fitness.score: /tmp/ndt.log 2>/dev/null)"; grep -aE "TF FAILED" /tmp/ndt.log | tail -2 || true' || true
echo "----- scovox fused integration -----"
dc_scovox exec -T scovox bash -lc 'grep -aiE "fuse|PointCloud2 input|deskew=|TF FAILED|integrated|frames" /tmp/scovox.log | tail -8; echo "TF_FAILED_count=$(grep -acE TF.FAILED /tmp/scovox.log 2>/dev/null)"' || true
echo "----- explo_planner: PLAN cycles -----"
dc_scovox exec -T scovox bash -lc '
  grep -aE "planner ready|Waiting to start|selected goal|candidates rejected|Navigation (succeeded|failed)|DONE" /tmp/explo.log | tail -15
  echo "plan_steps=$(grep -ac "selected goal" /tmp/explo.log 2>/dev/null)"
  echo "all_rejected_ticks=$(grep -ac "candidates rejected" /tmp/explo.log 2>/dev/null)"
  echo "per-step metrics CSV: /tmp/exploration_fused_bag.csv (in scovox container)"
' || true

if [ -n "$RVIZ" ]; then LEAVE_UP=1; fi   # (plain && here would make the script exit 1)
# cleanup() runs on EXIT
