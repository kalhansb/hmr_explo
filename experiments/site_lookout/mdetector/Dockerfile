# M-detector (hku-mars/M-detector, Wu et al., Nature Communications 2024),
# unchanged except its build file, plus an offline driver (md_offline.cpp):
# frames and poses from a file, per-point labels to files. ROS 1 Noetic.
FROM ros:noetic-perception
RUN apt-get update && apt-get install -y --no-install-recommends git libtbb-dev python3-dev \
      ros-noetic-eigen-conversions ros-noetic-tf && rm -rf /var/lib/apt/lists/*
ARG MD_COMMIT=0645dabe307a99f3b59a5a3d3007acfdcad1a39f
RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src && git clone https://github.com/hku-mars/M-detector m_detector \
    && cd m_detector && git checkout $MD_COMMIT
COPY md_offline.cpp /catkin_ws/src/m_detector/src/md_offline.cpp
COPY patch_build.py /tmp/patch_build.py
RUN python3 /tmp/patch_build.py /catkin_ws/src/m_detector \
    && . /opt/ros/noetic/setup.sh && cd /catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 md_offline
RUN apt-get update && apt-get install -y --no-install-recommends python3-numpy python3-scipy python3-yaml \
    && rm -rf /var/lib/apt/lists/*
