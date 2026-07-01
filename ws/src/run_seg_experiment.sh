#!/usr/bin/env bash
# Host-side orchestrator for the full RGB-D SEMANTIC pipeline against a bag:
#   NDT localizer (hmr_localisation) + online segmentation (seg_pipeline) +
#   SCovox RGB-D semantic mapping (scovox) -- one DDS graph (host net + ipc host +
#   ROS_DOMAIN_ID=0), one container per repo, no repo mounted into another.
#
# Containers / roles:
#   hmr_loc (svc ros)   : EKF (odom->base_link) + NDT (map->odom, loads gt_map_us050)
#                         + a static base_link->camera_color_frame (CALIBRATED camera
#                         extrinsic) + the bag play (drives /clock + all sensors)
#   hmr_seg (svc seg)   : seg_node -> /scovox/segmentation/colored + reframed depth+info
#   scovox  (svc scovox): scovox_mapping_node (RGB-D path) -> /scovox_node/pointcloud
#                         (map-frame colored semantic-occupancy cloud) + /scovox_node/scovox
#
# Frames (why this is wired the way it is):
#   * NDT publishes map->odom; EKF publishes odom->base_link; NDT's own static
#     publishers give base_link->os_lidar (cloud frame) and base_link->imu (imu frame),
#     which is why the bag's /tf and /tf_static are NOT played (they would fight the
#     live tree / create os_lidar double-parents).
#   * The camera link is supplied by ONE static base_link->camera_color_frame computed
#     from the bag's CALIBRATED os_lidar->camera_color_optical_frame extrinsic (the bag's
#     own base_link->camera_link is an uncalibrated identity = camera at the robot origin).
#     camera_color_frame is a BODY frame, which is exactly what SCovox's hardcoded
#     optical->body kR rotation expects on the depth frame_id (seg_node re-frames depth
#     to camera_color_frame by default).
#   * SCovox integrates into the map frame (integration_frame:=map) so the semantic map
#     lands in the same global frame as the NDT/gt map.
#
# Usage (HOST; needs docker, NOT ROS):
#   ./run_seg_experiment.sh [playback_duration_s]     # empty = full bag (~500 s)
#   e.g. ./run_seg_experiment.sh 70
#
# RViz (opt-in): prefix with RVIZ=1 to ALSO open an RViz window showing the live
# semantic map (/scovox_node/pointcloud, RGB8 = semantic class colors) as it builds:
#   RVIZ=1 ./run_seg_experiment.sh            # full bag, live RViz
#   RVIZ=1 ./run_seg_experiment.sh 120        # first 120 s, live RViz
# RViz runs in the glim_loc container (it already ships rviz2 + X11; it shares the
# same DDS graph, so it just subscribes — it does no localization/mapping). In RVIZ
# mode the nodes + RViz are LEFT RUNNING after the bag ends so you can pan around the
# finished map; the script prints the docker-compose-stop commands to tear down.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOC_DIR="$HERE/hmr_localisation"
SEG_DIR="$HERE/scovox/src/seg_pipeline"
SCOVOX_DIR="$HERE/scovox"
GLIM_DIR="$HERE/glim_localisation"          # used ONLY as the RViz viewer when RVIZ=1
BAG="/ws/bags/2026_06_19_18_19_06__kalhan-map-test-2_"
DUR="${1:-}"
DUR_ARG=""; [ -n "$DUR" ] && DUR_ARG="--playback-duration $DUR"
RVIZ="${RVIZ:-}"                            # RVIZ=1 -> open RViz + leave nodes up at end
RVIZ_CFG="$HERE/seg_experiment.rviz"
MODEL="${MODEL:-}"                          # MODEL=<hf-id> -> override seg model_name
                                            # (empty = seg_node default: Mapillary Mask2Former).
                                            # e.g. MODEL=facebook/mask2former-swin-large-ade-semantic

dc_loc()    { docker compose -f "$LOC_DIR/compose.yaml"    "$@"; }
dc_seg()    { docker compose -f "$SEG_DIR/compose.yaml"    "$@"; }
dc_scovox() { docker compose -f "$SCOVOX_DIR/compose.yaml" "$@"; }
dc_glim()   { docker compose -f "$GLIM_DIR/compose.yaml"   "$@"; }

LEAVE_UP=""   # set at the end in RVIZ mode so cleanup() leaves everything running
stop_cmds() {
  echo "    docker compose -f $SCOVOX_DIR/compose.yaml stop scovox"
  echo "    docker compose -f $SEG_DIR/compose.yaml stop seg"
  echo "    docker compose -f $LOC_DIR/compose.yaml stop ros"
  echo "    docker compose -f $GLIM_DIR/compose.yaml stop glim"
}
cleanup() {
  if [ -n "$LEAVE_UP" ]; then
    echo "[orch] RVIZ mode: leaving nodes + RViz running so you can inspect the map."
    echo "[orch] stop everything when done with:"; stop_cmds
    return 0
  fi
  echo "[orch] stopping nodes (docker compose stop)…"
  dc_scovox stop scovox 2>/dev/null || true
  dc_seg    stop seg    2>/dev/null || true
  dc_loc    stop ros    2>/dev/null || true
  [ -n "$RVIZ" ] && dc_glim stop glim 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[orch] bringing up containers…"
dc_loc    up -d
dc_seg    up -d
dc_scovox up -d
if [ -n "$RVIZ" ]; then
  echo "[orch] RVIZ=1 → bringing up glim_loc as the RViz viewer + allowing local X11…"
  xhost +local:root >/dev/null 2>&1 || true   # let the (root) container reach the host X server
  dc_glim up -d glim
fi

# kill any stray bag play left over from manual probing (not an experiment).
# NOTE the [r] trick: a plain `pkill -f "ros2 bag play"` also matches THIS shell's
# own command line (it contains that string) and SIGTERMs itself (exit 143).
dc_loc exec -T ros bash -lc 'pkill -f "[r]os2 bag play" 2>/dev/null; sleep 1; true' || true

# 1) localizer stack in hmr_loc: EKF + NDT + calibrated camera static, all detached.
echo "[orch] launching EKF + NDT + camera static in hmr_loc…"
dc_loc exec -d ros bash -lc '
  source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml   # SHM for big clouds
  ros2 launch /ws/launch/ekf_odom.launch.py use_sim_time:=true > /tmp/ekf.log 2>&1 &
  ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
    localization_param_dir:=/ws/config/gt_ouster_ndt_tree_realtime.yaml \
    cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
    global_frame_id:=map odom_frame_id:=odom base_frame_id:=base_link \
    use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
    publish_lidar_tf:=true lidar_frame_id:=os_lidar \
    lidar_tf_x:=0.1105 lidar_tf_y:=0.0 lidar_tf_z:=0.404 lidar_tf_yaw:=3.14159265 \
    publish_imu_tf:=true imu_frame_id:=imu \
    imu_tf_x:=0.062 imu_tf_y:=0.0 imu_tf_z:=0.015 imu_tf_yaw:=1.5707963 \
    > /tmp/ndt.log 2>&1 &
  # CALIBRATED base_link->camera_color_frame (from os_lidar->camera extrinsic; body frame)
  ros2 run tf2_ros static_transform_publisher \
    --x 0.270676 --y 0.049297 --z 0.279109 \
    --qx -0.013514 --qy 0.000986 --qz 0.000780 --qw 0.999908 \
    --frame-id base_link --child-frame-id camera_color_frame > /tmp/camtf.log 2>&1 &
  wait
'

echo "[orch] waiting for NDT map load + activation…"
dc_loc exec -T ros bash -lc 'for i in $(seq 1 60); do grep -aq "Activating end" /tmp/ndt.log 2>/dev/null && { echo NDT_ACTIVE; exit 0; }; sleep 1; done; echo NDT_TIMEOUT; tail -5 /tmp/ndt.log'

# 2) online seg node in hmr_seg (default output_frame=camera_color_frame).
#    MODEL (if set) overrides the seg model_name (default = Mapillary Mask2Former).
echo "[orch] launching seg node in hmr_seg${MODEL:+ (model=$MODEL)}…"
dc_seg exec -d seg bash -lc "
  source /opt/ros/jazzy/setup.bash
  cd /seg && exec python3 -m seg_pipeline.seg_node --ros-args -p use_sim_time:=true ${MODEL:+-p model_name:=$MODEL} > /root/seg.log 2>&1
"

# 3) SCovox RGB-D semantic node in scovox. The RGB-D (depth+seg) path is selected by
#    leaving input_pointcloud_topic at its default "" (empty) — do NOT pass it on the
#    CLI as :="" (the shell strips the quotes and rcl rejects the valueless override).
#    Palette (keys/classes) + num/max classes come from outdoor_palette.
echo "[orch] launching SCovox RGB-D semantic node in scovox…"
dc_scovox exec -d scovox bash -lc '
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  exec ros2 run scovox_mapping scovox_mapping_node --ros-args -r __node:=scovox_node \
    -p use_sim_time:=true \
    -p depth_topic:=/scovox/depth/image_raw \
    -p depth_info_topic:=/scovox/depth/camera_info \
    -p seg_topic:=/scovox/segmentation/colored \
    -p dataset_mode:=false \
    -p integration_frame:=map -p map_frame:=map -p base_frame:=camera_color_frame \
    -p num_classes:=14 -p max_semantic_classes:=14 -p semantic_top_k:=2 \
    -p semantic_mode:=dirichlet -p dirichlet_prior:=0.01 \
    -p mode:=persistent -p enable_tsdf:=false \
    -p min_depth:=0.3 -p max_depth:=6.0 -p min_range:=0.3 -p max_range:=6.0 \
    -p startup_tf_stable_sec:=0.0 -p startup_tf_jump_threshold:=10.0 -p runtime_tf_gate:=false \
    -p "semantic_color_map_keys:=[8405120,15999976,10025880,4605510,7048739,10066329,16427550,14423100,16711680,142,4620980,12491161,6710940,0]" \
    -p "semantic_color_map_classes:=[1,2,3,4,5,6,7,8,9,10,11,12,13,0]" \
    > /tmp/scovox.log 2>&1
'

# 3b) RViz viewer in glim_loc (opt-in). It only subscribes (/scovox_node/pointcloud
#     + /ouster/points + TF) over the shared DDS graph — no localization/mapping here.
#     use_sim_time so RViz reads /clock and TF stamps line up with the bag.
if [ -n "$RVIZ" ]; then
  echo "[orch] launching RViz in glim_loc (config: seg_experiment.rviz)…"
  docker cp "$RVIZ_CFG" glim_loc:/tmp/seg_experiment.rviz
  # GL on this laptop is HYBRID: the X server (:1) is driven by the iGPU, so RViz
  # must PRIME-offload onto the discrete NVIDIA GPU — otherwise MESA tries the Intel
  # `iris` driver and GLX context creation fails (black/no window). Pass SOFTGL=1 to
  # force llvmpipe software rendering instead (slower, but needs no GPU/DRI).
  GLENV='export __NV_PRIME_RENDER_OFFLOAD=1; export __GLX_VENDOR_LIBRARY_NAME=nvidia'
  [ "${SOFTGL:-0}" = "1" ] && GLENV='export LIBGL_ALWAYS_SOFTWARE=1; unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD'
  dc_glim exec -d glim bash -lc "
    source /opt/ros/jazzy/setup.bash
    export DISPLAY=\"\${DISPLAY:-:1}\"
    $GLENV
    exec rviz2 -d /tmp/seg_experiment.rviz --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1
  "
fi

echo "[orch] letting seg + scovox subscribe (6 s)…"
sleep 6

# 4) play the bag: feeds NDT (/ouster reliable via SHM, /imu) AND seg (camera depth+color).
#    NOT /tf (fights live tree), NOT /tf_static (NDT owns os_lidar+imu; camera is the static above).
echo "[orch] playing bag ${DUR:+(first ${DUR}s)}…"
dc_loc exec -T ros bash -lc "
  source /opt/ros/jazzy/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml
  ros2 bag play $BAG --clock --rate 1.0 $DUR_ARG \
    --topics /ouster/points /imu/data \
             /camera/aligned_depth_to_color/image_raw \
             /camera/aligned_depth_to_color/camera_info \
             /camera/color/image_raw/compressed \
    --qos-profile-overrides-path /ws/config/ouster_reliable_qos.yaml
"

echo "[orch] bag finished — diagnostics:"
echo "----- NDT good scans -----"
# NB: bareword -E patterns ('.' matches the space) — do NOT wrap the pattern in
# escaped \"...\" inside this single-quoted bash -lc: the backslash-quotes don't
# group, the space splits the pattern, and grep treats the words as filenames
# (prints '/tmp/ndt.log:0' instead of the real count).
dc_loc    exec -T ros    bash -lc 'echo "good_scans=$(grep -acE fitness.score: /tmp/ndt.log 2>/dev/null)"; grep -aE "TF FAILED" /tmp/ndt.log | tail -2 || true' || true
echo "----- seg heartbeat -----"
dc_seg    exec -T seg    bash -lc 'grep "seg:" /root/seg.log | tail -3 || true' || true
echo "----- scovox integration -----"
dc_scovox exec -T scovox bash -lc 'grep -aiE "recv=|TF FAILED|waiting for CameraInfo|integrated|frames" /tmp/scovox.log | tail -8; echo "TF_FAILED_count=$(grep -acE TF.FAILED /tmp/scovox.log 2>/dev/null)"' || true

# In RVIZ mode, keep the nodes + RViz alive so the finished semantic map stays on
# screen (the cloud republishes off the sim clock, so it's frozen-but-visible once
# the bag stops). cleanup() then only prints the stop commands instead of tearing
# down. Without RVIZ, cleanup() tears everything down on EXIT as before.
[ -n "$RVIZ" ] && LEAVE_UP=1
# cleanup() runs on EXIT
