#!/usr/bin/env python3
"""World for the sim-versus-real check: the site models plus one static
gpu_lidar per registered real frame, at the frame's registered pose, with the
real sensor's beam layout.

Usage: probe_world.py <world dir> <sensor: hesai|ouster> <poses.json> <out.sdf>
"""
import json, sys

SENSORS = {
    # Hesai frame: elevation -52.4..52.8 deg, ranges 0.3..61 m (JT128-like);
    # Ouster frame: elevation -21.2..21.2 deg, ~128 rings, ranges 0.7..194 m (OS1-128-like).
    'hesai': dict(h=1024, v=128, vmin=-52.6, vmax=52.6, rmin=0.3, rmax=60.0),
    'ouster': dict(h=1024, v=128, vmin=-21.2, vmax=21.2, rmin=0.7, rmax=120.0),
}


def lidar(name, s, pose):
    import math
    x, y, z, r, p, yw = pose
    return f'''
  <model name="{name}"><static>true</static><pose>{x} {y} {z} {r} {p} {yw}</pose>
    <link name="l"><sensor name="lidar" type="gpu_lidar">
      <topic>{name}/scan</topic><ignition_frame_id>{name}</ignition_frame_id>
      <update_rate>2</update_rate><always_on>1</always_on>
      <lidar><scan>
        <horizontal><samples>{s["h"]}</samples><resolution>1</resolution><min_angle>{-math.pi}</min_angle><max_angle>{math.pi}</max_angle></horizontal>
        <vertical><samples>{s["v"]}</samples><resolution>1</resolution><min_angle>{math.radians(s["vmin"])}</min_angle><max_angle>{math.radians(s["vmax"])}</max_angle></vertical>
      </scan><range><min>{s["rmin"]}</min><max>{s["rmax"]}</max><resolution>0.01</resolution></range>
      <noise><type>gaussian</type><mean>0</mean><stddev>0.02</stddev></noise></lidar>
    </sensor></link></model>'''


def main():
    wdir, sensor, pj, out = sys.argv[1:5]
    s = SENSORS[sensor]
    probes = ''.join(lidar(f'probe_{j["frame"]}', s, j['xyz'] + j['rpy']) for j in json.load(open(pj)))
    w = open(f'{wdir}/site.sdf').read().replace('</world>', probes + '\n</world>')
    open(out, 'w').write(w)


if __name__ == '__main__':
    main()
