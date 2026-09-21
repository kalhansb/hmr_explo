#!/bin/bash
# Re-score a banked cell under the 2D coverage definition, and at the same time
# verify the new un-inflated coverage topic against the inflated planning map.
# ONE mapper in the PRODUCTION configuration (inflation 1.5) publishing both
# grids, so the two series differ only by the inflation this change removes.
set -e
ip link set lo multicast on
exec gosu kalhan bash -lc '
source /opt/ros/humble/setup.bash; source /ws/install/setup.bash
export HOME=/gzhome
ros2 run scovox_mapping dscovox_mapping_node --ros-args \
  -r __node:=dscovox_node \
  -p use_sim_time:=true -p map_frame:=map -p publish_rate_hz:=1.0 \
  -p scovox_bin_qos_depth:=4000 \
  -p "input_topics:=['"'"'/atlas/scovox_node/scovox_bin'"'"','"'"'/bestla/scovox_node/scovox_bin'"'"']" \
  -p publish_global_planning_map:=true \
  -p global_planning_map_topic:=~/global_planning_map \
  -p global_coverage_map_topic:=~/global_coverage_map \
  -p global_planning_map_size_m:=150.0 \
  -p global_planning_map_origin_x:=-75.0 -p global_planning_map_origin_y:=-75.0 \
  -p global_planning_map_resolution:=0.40 \
  -p global_planning_map_period_sec:=1.0 \
  -p global_planning_map_inflation_m:=1.5 \
  -p global_planning_map_min_z:=0.05 -p global_planning_map_max_z:=1.0 \
  > /sp/gridcheck/rescore_$CELL.mapper.log 2>&1 &
sleep 8
CELL="$CELL" python3 /sp/gridcheck/grid_census.py \
  /dscovox_node/global_planning_map,/dscovox_node/global_coverage_map \
  /atlas/odom_ground_truth /sp/gridcheck/rescore_$CELL.csv \
  > /sp/gridcheck/rescore_$CELL.census.log 2>&1 &
CENSUS=$!
ros2 bag play /runs/$CELL/rosbag2 --clock 200 --rate "$RATE" \
  --topics /atlas/scovox_node/scovox_bin /bestla/scovox_node/scovox_bin \
           /atlas/odom_ground_truth
sleep 20
kill -INT $CENSUS 2>/dev/null || true
wait $CENSUS 2>/dev/null || true
echo "rescore rows: $(wc -l < /sp/gridcheck/rescore_$CELL.csv)"
'
