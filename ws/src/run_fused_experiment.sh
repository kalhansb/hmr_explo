#!/usr/bin/env bash
# Host-side orchestrator for the FUSED LiDAR-occupancy + RGB-D-semantics pipeline
# against a bag — LiDAR and RGB-D integrate into ONE SCovox SemSplitMap (the split
# Beta/Dirichlet substrate), not two separate maps. Derived from run_seg_experiment.sh;
# the ONLY additions are (a) the LiDAR stream into the same scovox node and (b) a
# cleaner TF tree sourced from the bag's /tf_static (see "Frames" below).
#
# Fusion policy (pure LiDAR authority — behind the fuse_lidar_rgbd master switch):
#   * LiDAR owns Beta occupancy: lidar_w_occ=8, lidar_w_free=4, carve bounded to
#     the last CARVE_BAND m before each hit (perf; see CARVE_BAND note below).
#   * RGB-D is semantics-ONLY: rgbd_w_occ=0, rgbd_w_free=0, rgbd_geometry_off=true —
#     it deposits ZERO occupancy/carve/TSDF; a bad RGB-D pixel can only misplace a
#     decayable Dirichlet label, never move geometry.
#   * Stream-B gate (rgbd_dirichlet_min_p_occ=0.55) reads the LiDAR-only p_occ, so a
#     semantic label commits ONLY where LiDAR says occupied. >0.5 is mandatory: at 0.5
#     an RGB-D hit on a voxel LiDAR never touched (prior p_occ=0.5) would commit
#     semantics on prior-only geometry and defeat LiDAR authority.
#
# Per-node param sets are versioned in the submodules (this script carries only
# the wiring, extrinsics, and env-knob overrides):
#   hmr_localisation/config/gt_ouster_ndt_tree_fused.yaml     NDT (launch args + extrinsics doc'd in its header)
#   scovox/config/scovox_fused_lidar_rgbd.yaml                fused mapping node (KERNEL_L/CARVE_BAND override it)
#   scovox/src/seg_pipeline/config/seg_fused_experiment.yaml  seg node (MODEL overrides it)
#
# Containers / roles (one DDS graph: host net + ipc host + ROS_DOMAIN_ID=0):
#   hmr_loc (svc ros)   : EKF (odom->base_link) + NDT (map->odom, loads gt_map_us050)
#                         + 3 static extrinsic publishers (base_link->{os_lidar,imu,
#                         camera_color_frame}) + the bag play (drives /clock + sensors)
#   hmr_seg (svc seg)   : seg_node -> /scovox/segmentation/colored + reframed depth+info
#   scovox  (svc scovox): scovox_mapping_node in FUSED mode -> /scovox_node/pointcloud
#                         (map-frame colored semantic-occupancy cloud) + /scovox_node/scovox
#
# Frames (why this differs from run_seg_experiment.sh):
#   The tree is  map --NDT--> odom --EKF--> base_link --STATIC--> {os_lidar, imu, camera}.
#   NDT does map->odom ONLY (publish_lidar_tf:=false publish_imu_tf:=false); it still
#   CONSUMES base_link<-os_lidar / base_link<-imu from TF (lidar_localization_component
#   .cpp:1786 / :1395 lookupTransform), so the standalone statics below feed it.
#   The three extrinsics are READ FROM THE BAG's /tf_static and re-published manually
#   (we do NOT play /tf_static — its base_link->camera_link is an UNCALIBRATED identity,
#   and playing it would also double-parent os_lidar against the live tree):
#     base_link->os_lidar : from base_link->os_sensor (0.1105,0,0.404 yaw pi) o os_sensor
#                           ->os_lidar (identity)                 = 0.1105,0,0.404 quat(0,0,1,0)
#     base_link->imu      : /imu/data frame_id is "imu" (NOT the bag's "imu_link"); use the
#                           bag's latched base_link->imu_link value = 0.062,0,0.015 yaw +pi/2
#     base_link->camera_color_frame : the CALIBRATED camera pose lives ONLY on the bag's
#                           os_lidar->camera_color_optical_frame leg; collapsed to a
#                           base_link->camera_color_frame BODY static (SCovox applies its
#                           own optical->body kR to the depth frame_id) = the value below.
#   SCovox integrates into map (integration_frame:=map) so LiDAR + semantics land in the
#   same global frame as the NDT/gt map; per-stream ray origins are lidar_base_frame:=
#   os_lidar and rgbd_base_frame:=camera_color_frame.
#
# Usage (HOST; needs docker, NOT ROS):
#   ./run_fused_experiment.sh [playback_duration_s]     # empty = full bag (~500 s)
#   RVIZ=1 ./run_fused_experiment.sh 120                # first 120 s + live RViz
# RVIZ mode leaves the nodes + RViz up after the bag so you can inspect the fused map;
# the script prints the docker-compose-stop commands to tear down.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOC_DIR="$HERE/hmr_localisation"
SEG_DIR="$HERE/scovox/src/seg_pipeline"
SCOVOX_DIR="$HERE/scovox"
GLIM_DIR="$HERE/glim_localisation"          # (unused; RViz now runs inside the scovox container)
BAG="/ws/bags/2026_06_19_18_19_06__kalhan-map-test-2_"
DUR="${1:-}"
DUR_ARG=""; [ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"
RVIZ="${RVIZ:-}"                            # RVIZ=1 -> open RViz + leave nodes up at end
RVIZ_CFG="$HERE/seg_experiment.rviz"
MODEL="${MODEL:-}"                          # MODEL=<hf-id> -> override seg model_name
KERNEL_L="${KERNEL_L:-0.4}"                 # RGB-D->LiDAR BKI spread radius (m); 0 = exact-voxel gate
CARVE_BAND="${CARVE_BAND:--1.0}"            # LiDAR free-space carve length (m) before each hit; <=0 = full-ray.
                                            # Full-ray (-1.0) is the default — the planner needs complete
                                            # free-space. A positive value bounds the fused walk (~carve_band/res
                                            # voxels/ray) and is far cheaper, but only carves free-space within
                                            # that band of each surface. Keep full-ray unless profiling says
                                            # otherwise. See the perf notes on making full-ray carving fast.
LIDAR_ONLY="${LIDAR_ONLY:-}"                # LIDAR_ONLY=1 -> skip RGB-D/semantics entirely: no seg node, no
                                            # camera topics played. scovox STAYS in fused mode, so the LiDAR
                                            # occupancy/carve path is byte-identical to the full fused run —
                                            # only the RGB-D stream is starved of data. Use to judge the LiDAR
                                            # voxel map (occupancy + free-space carve) in isolation, as a clean
                                            # A/B against the fused map.

dc_loc()    { docker compose -f "$LOC_DIR/compose.yaml"    "$@"; }
dc_seg()    { docker compose -f "$SEG_DIR/compose.yaml"    "$@"; }
dc_scovox() { docker compose -f "$SCOVOX_DIR/compose.yaml" "$@"; }
dc_glim()   { docker compose -f "$GLIM_DIR/compose.yaml"   "$@"; }

LEAVE_UP=""   # set at the end in RVIZ mode so cleanup() leaves everything running
stop_cmds() {
  echo "    docker compose -f $SCOVOX_DIR/compose.yaml stop scovox"   # also kills RViz (runs inside scovox)
  echo "    docker compose -f $SEG_DIR/compose.yaml stop seg"
  echo "    docker compose -f $LOC_DIR/compose.yaml stop ros"
}
cleanup() {
  if [ -n "$LEAVE_UP" ]; then
    echo "[orch] RVIZ mode: leaving nodes + RViz running so you can inspect the fused map."
    echo "[orch] stop everything when done with:"; stop_cmds
    return 0
  fi
  echo "[orch] stopping nodes (docker compose stop)…"
  dc_scovox stop scovox 2>/dev/null || true   # stopping scovox also kills the in-container RViz
  dc_seg    stop seg    2>/dev/null || true
  dc_loc    stop ros    2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[orch] bringing up containers…"
dc_loc    up -d
if [ -z "$LIDAR_ONLY" ]; then dc_seg up -d; fi   # LIDAR_ONLY skips the seg container entirely
dc_scovox up -d
if [ -n "$RVIZ" ]; then
  echo "[orch] RVIZ=1 → RViz will run inside the scovox container (GPU/hardware GL); allowing local X11…"
  xhost +local:root >/dev/null 2>&1 || true
fi

# kill any stray bag play left over from manual probing (the [r] trick avoids the
# shell SIGTERMing itself — see run_seg_experiment.sh).
dc_loc exec -T ros bash -lc 'pkill -f "[r]os2 bag play" 2>/dev/null; sleep 1; true' || true

# 1) localizer stack in hmr_loc: EKF + NDT (map->odom ONLY) + 3 static extrinsics.
#    publish_lidar_tf:=false publish_imu_tf:=false → NDT does NOT broadcast the sensor
#    legs; the standalone statics below own them (values read from the bag's /tf_static).
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
  # --- extrinsics READ FROM THE BAG /tf_static, published manually (see header) ---
  # base_link -> os_lidar  (0.1105,0,0.404, yaw pi = quat 0,0,1,0)
  ros2 run tf2_ros static_transform_publisher \
    --x 0.1105 --y 0.0 --z 0.404 --qx 0.0 --qy 0.0 --qz 1.0 --qw 0.0 \
    --frame-id base_link --child-frame-id os_lidar > /tmp/lidartf.log 2>&1 &
  # base_link -> imu  (/imu/data frame_id="imu"; bag imu_link value, yaw +pi/2)
  ros2 run tf2_ros static_transform_publisher \
    --x 0.062 --y 0.0 --z 0.015 --qx 0.0 --qy 0.0 --qz 0.7071068 --qw 0.7071068 \
    --frame-id base_link --child-frame-id imu > /tmp/imutf.log 2>&1 &
  # base_link -> camera_color_frame  (CALIBRATED, from os_lidar->camera chain; body frame)
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
  echo "[orch] LIDAR_ONLY=1 → skipping seg node (no RGB-D/semantics; scovox will map LiDAR only)."
fi

# 3) SCovox FUSED node in scovox — ONE node, BOTH streams into ONE SemSplitMap.
#    Full param set lives in scovox/config/scovox_fused_lidar_rgbd.yaml (see its
#    header for the fusion policy + per-stream range gating); the two env knobs
#    are appended AFTER the file so they win (later assignment overrides).
echo "[orch] launching SCovox FUSED node in scovox…"
dc_scovox exec -d scovox bash -lc '
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  exec ros2 run scovox_mapping scovox_mapping_node --ros-args -r __node:=scovox_node \
    --params-file /scovox/config/scovox_fused_lidar_rgbd.yaml \
    -p rgbd_kernel_radius:='"$KERNEL_L"' \
    -p carve_band:='"$CARVE_BAND"' \
    > /tmp/scovox.log 2>&1
'

# 3b) RViz viewer in glim_loc (opt-in) — subscribes only, no localization/mapping.
if [ -n "$RVIZ" ]; then
  echo "[orch] launching RViz inside the scovox container (config: seg_experiment.rviz)…"
  docker cp "$RVIZ_CFG" scovox:/tmp/seg_experiment.rviz
  # scovox now has NVIDIA GPU passthrough (see compose.yaml) → hardware GL via
  # PRIME offload by default. SOFTGL=1 forces Mesa/llvmpipe (no-GPU fallback).
  GLENV='export __NV_PRIME_RENDER_OFFLOAD=1; export __GLX_VENDOR_LIBRARY_NAME=nvidia'
  [ "${SOFTGL:-0}" = "1" ] && GLENV='export LIBGL_ALWAYS_SOFTWARE=1; unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD'
  dc_scovox exec -d scovox bash -lc "
    source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
    export DISPLAY=\"\${DISPLAY:-:1}\"
    $GLENV
    exec rviz2 -d /tmp/seg_experiment.rviz --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1
  "
fi

echo "[orch] letting seg + scovox subscribe (6 s)…"
sleep 6

# 4) play the bag: feeds NDT (/ouster reliable via SHM, /imu), scovox LiDAR (/ouster,
#    /imu for deskew) AND seg (camera depth+color). NOT /tf (fights live tree), NOT
#    /tf_static (we publish the 3 statics above; its camera leg is uncalibrated identity).
# LIDAR_ONLY drops the 3 camera topics so RGB-D gets no data (LiDAR-only map).
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
echo "----- seg heartbeat -----"
dc_seg    exec -T seg    bash -lc 'grep "seg:" /root/seg.log | tail -3 || true' || true
echo "----- scovox fused integration (LiDAR + RGB-D) -----"
dc_scovox exec -T scovox bash -lc 'grep -aiE "fuse|PointCloud2 input|deskew=|recv=|TF FAILED|waiting for CameraInfo|integrated|frames" /tmp/scovox.log | tail -12; echo "TF_FAILED_count=$(grep -acE TF.FAILED /tmp/scovox.log 2>/dev/null)"' || true

if [ -n "$RVIZ" ]; then LEAVE_UP=1; fi   # (plain && here would make the script exit 1)
# cleanup() runs on EXIT
