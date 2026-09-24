#!/usr/bin/env python3
"""Smoke-test world: the calibration world (open ground, mulcher, person) plus
a 'height gauge' -- a gpu_lidar rolled 90 deg so its 1800-sample sweep is
vertical (0.2 deg, ~1 cm at 3 m) -- to measure the rendered person height."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import build_world as bw

gauge = f"""
    <model name="gauge">
      <static>true</static>
      <pose>0 30 0 0 0 0</pose>
      <link name="link">
        <sensor name="gauge_lidar" type="gpu_lidar">
          <topic>gauge/laser_scan</topic>
          <update_rate>10</update_rate>
          <always_on>1</always_on>
          <pose>0 0 1.0 1.5707963 0 0</pose>
          <lidar>
            <scan><horizontal><samples>1800</samples><resolution>1</resolution>
              <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
              <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical></scan>
            <range><min>0.5</min><max>100</max><resolution>0.01</resolution></range>
          </lidar>
        </sensor>
      </link>
    </model>"""
out = sys.argv[1]
open(out, "w").write(bw.world_sdf("lookout_smoke", [], bw.husky_lidar_block(), extra=gauge))
print("wrote", out)
