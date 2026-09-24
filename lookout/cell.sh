#!/bin/bash
# In-container entry for every lookout step (same environment as the
# exploitation cells: runs/pilot_fine/cell.sh). Usage: cell.sh <command...>
#   * lo needs MULTICAST for ign-transport and FastDDS discovery.
#   * the workload runs as the NAMED user kalhan (ign-transport derives its
#     partition from a username lookup; a bare uid silently partitions).
#   * ogre2 needs a real X display, or the gpu_lidar produces no scans
#     (llvmpipe); with LOOKOUT_GPU=1 it renders headless on the NVIDIA GPU
#     through EGL and ign gazebo takes $GZ_HEADLESS (--headless-rendering).
set -e
ip link set lo multicast on
Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp &
for i in $(seq 1 50); do [ -e /tmp/.X11-unix/X99 ] && break; sleep 0.2; done
chmod 1777 /tmp/.X11-unix 2>/dev/null || true
exec gosu kalhan bash -lc '
  source /opt/ros/humble/setup.bash
  source /ws/install/setup.bash
  export HOME=/gzhome DISPLAY=:99 IGN_IP=127.0.0.1
  export IGN_FUEL_CACHE_PATH=/gzhome/.ignition/fuel
  if [ "${LOOKOUT_GPU:-0}" = "1" ]; then
    # NVIDIA via EGL; the toolkit injects libEGL_nvidia but not its glvnd entry
    export __EGL_VENDOR_LIBRARY_FILENAMES=/lookout/gpu/10_nvidia.json GZ_HEADLESS=--headless-rendering
  else
    export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe GZ_HEADLESS=
  fi
  export IGN_GAZEBO_RESOURCE_PATH=/runs/lookout/models:/ws/install/hmr_sim/share/hmr_sim/models:/ws/install/hmr_sim/share/hmr_sim/worlds
  export PYTHONUNBUFFERED=1
  exec "$@"
' bash "$@"
