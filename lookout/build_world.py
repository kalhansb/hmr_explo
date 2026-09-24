#!/usr/bin/env python3
"""Build the path-lookout worlds and layout configs (deterministic, fixed seeds).

Writes
  lookout/config/L2.yaml, L3.yaml   paths (polylines), entries, lookout points,
                                    spawn poses, mulcher, forest and walk settings
  <worlds>/lookout_L2.sdf, lookout_L3.sdf, lookout_calib.sdf
  <worlds>/forest_L2.csv, forest_L3.csv   tree positions (x, y)

Geometry (all in the world frame, mulcher at the origin):
  * Forest: Fuel "oak tree" (the flatforest v2 oak), Poisson-disc, density
    0.0086 /m^2 and minimum spacing 6.5 m (flatforest v2), disc of radius 160 m.
  * Lanes: 3 m wide. No trunk centre within 1.5 m + 1.7 m (the oak's root-flare
    radius) of a path centreline. Paths bend gently (lateral sine, amplitude
    5 m, wavelength 100-120 m, minimum radius of curvature ~50 m).
    L2: one crossing path whose closest approach is 10 m from the mulcher; its
        two ends are the two entries.
    L3: three paths from entries about 120 deg apart, each ending 10 m from
        the mulcher.
  * Lookout points: calculated from the map (the lane-cleared forest) by
    posts.py: the free spot beside each path, at most 28 m out, with a safe
    radio link to the mulcher, that sees a walker coming in the earliest;
    facing out along the path, towards arriving walkers. Nothing is cleared
    for a post: a spot needs 2 m free (plus root flare) already. (Before
    2026-09-24: on the 90 m circle, 2 m counter-clockwise of the path, with a
    pad cleared round each.)
  * No trunk within 8 m of the mulcher.
  * Mulcher: body box 2.59 x 1.68 x 1.94 m on the ground, head box
    0.80 x 1.70 x 0.80 m centred 1.5 m ahead at 2.0 m height, both rendered and
    colliding; the Husky's VLP-16 lidar block on the roof centre at 2.1 m.
  * Person: Fuel "Walking person" (OpenRobotics, Marina Kollmitz, MakeHuman;
    CC0 1.0), visual only, static, scaled to 1.75 m, parked out of range.

Usage: build_world.py --worlds <dir>   (host python, stdlib only)
"""
import argparse
import math
import os
import random
import re
import sys

import posts

HERE = os.path.dirname(os.path.abspath(__file__))
HUSKY_SDF = os.path.join(HERE, "..", "ws", "src", "hmr_sim", "hmr_sim", "models",
                         "COSTAR_HUSKY_SENSOR_CONFIG_LIDAR", "model.sdf")

R_FOREST = 160.0
DENSITY = 0.0086
MIN_SPACING = 6.5
FLARE = 1.7
LANE_HALF = 1.5
PAD = 2.0
MULCHER_CLEAR = 8.0
R_END = 10.0
R_OUTER = 155.0
FOREST_SEED = 20260923

BODY = (2.59, 1.68, 1.94)
HEAD = (0.80, 1.70, 0.80)
HEAD_C = (1.5, 0.0, 2.0)
MULCHER_LIDAR_Z = 2.1

# Person mesh (walking.dae) as RENDERED (measured with a vertical gpu_lidar,
# 1 cm resolution, lookout/smoke): z -0.029 .. 1.878 in mesh units (1.907 m);
# the model faces -y. Scaled to 1.75 m and lifted so the soles sit on the ground.
PERSON_H = 1.75
PERSON_MESH_ZMIN, PERSON_MESH_ZMAX = -0.0294, 1.878
PERSON_SCALE = PERSON_H / (PERSON_MESH_ZMAX - PERSON_MESH_ZMIN)
PERSON_PARK = (1000.0, 1000.0)


# ---------------------------------------------------------------- geometry --
def polyline_len(P):
    return sum(math.dist(P[i], P[i + 1]) for i in range(len(P) - 1))


def point_at(P, s):
    """Point and unit tangent at arc length s along polyline P (clamped)."""
    acc = 0.0
    for i in range(len(P) - 1):
        seg = math.dist(P[i], P[i + 1])
        if acc + seg >= s or i == len(P) - 2:
            t = 0.0 if seg == 0 else min(max((s - acc) / seg, 0.0), 1.0)
            x = P[i][0] + t * (P[i + 1][0] - P[i][0])
            y = P[i][1] + t * (P[i + 1][1] - P[i][1])
            return (x, y), ((P[i + 1][0] - P[i][0]) / seg, (P[i + 1][1] - P[i][1]) / seg)
        acc += seg
    raise ValueError


def dist_to_polyline(q, P):
    best = float("inf")
    for i in range(len(P) - 1):
        ax, ay = P[i]; bx, by = P[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((q[0] - ax) * dx + (q[1] - ay) * dy) / L2))
        best = min(best, math.hypot(q[0] - ax - t * dx, q[1] - ay - t * dy))
    return best


def arc_where_range(P, r, s_from, s_to, step=0.01):
    """First arc length between s_from and s_to (either direction) where the
    distance to the origin crosses r, refined by bisection."""
    sgn = 1 if s_to > s_from else -1
    prev_s = s_from
    prev_d = math.hypot(*point_at(P, s_from)[0]) - r
    s = s_from
    while (s - s_to) * sgn < 0:
        s = s + sgn * 0.5
        if (s - s_to) * sgn > 0:
            s = s_to
        d = math.hypot(*point_at(P, s)[0]) - r
        if prev_d * d <= 0:
            a, b = prev_s, s
            for _ in range(60):
                m = 0.5 * (a + b)
                dm = math.hypot(*point_at(P, m)[0]) - r
                if (math.hypot(*point_at(P, a)[0]) - r) * dm <= 0:
                    b = m
                else:
                    a = m
            return 0.5 * (a + b)
        prev_s, prev_d = s, d
    return None


def ux(a):
    return (math.cos(a), math.sin(a))


def radial_path(bearing_deg, amp, wavelength, phase):
    """Outer end (r = R_OUTER) to inner end (r = R_END), lateral sine bend,
    tapered to zero over the last 15 m so the inner end is exactly R_END."""
    b = math.radians(bearing_deg)
    u, n = ux(b), ux(b + math.pi / 2)
    P = []
    r = R_OUTER
    while r >= R_END - 1e-9:
        taper = min(1.0, (r - R_END) / 15.0)
        off = amp * math.sin(2 * math.pi * r / wavelength + phase) * taper
        P.append((r * u[0] + off * n[0], r * u[1] + off * n[1]))
        r -= 2.5
    if math.dist(P[-1], (R_END * u[0], R_END * u[1])) > 1e-6:
        P.append((R_END * u[0], R_END * u[1]))
    return [(round(x, 3), round(y, 3)) for x, y in P]


def crossing_path(dir_deg, miss, amp, wavelength):
    """Straight-ish chord whose closest approach to the origin is `miss` at
    s = 0; lateral bend tapered to zero within 20 m of that point."""
    d = math.radians(dir_deg)
    u, n = ux(d), ux(d + math.pi / 2)
    # half-length so both ends reach R_OUTER
    half = math.sqrt(R_OUTER ** 2 - miss ** 2)
    P = []
    s = -half
    while s <= half + 1e-9:
        taper = min(1.0, abs(s) / 20.0)
        off = miss + amp * math.sin(2 * math.pi * s / wavelength) * taper
        P.append((s * u[0] + off * n[0], s * u[1] + off * n[1]))
        s += 2.5
    return [(round(x, 3), round(y, 3)) for x, y in P]


# ------------------------------------------------------------------ layouts --
def layout(name, base=None):
    """Paths, entries and lookout points. The points come from the map: the
    forest `base` (default: the fixed-seed one) with the lanes cleared."""
    if name == "L2":
        P = crossing_path(dir_deg=0.0, miss=R_END, amp=5.0, wavelength=110.0)
        paths = [{"id": "P1", "polyline": P}]
        L = polyline_len(P)
        mid = L / 2.0
        # entries: walk from either end towards the closest approach
        entries = [{"id": "A", "path": "P1", "s_start": 0.0, "dir": 1},
                   {"id": "B", "path": "P1", "s_start": L, "dir": -1}]
        # the closest approach (10 m) is at the middle vertex
        for e in entries:
            e["s_end"] = mid
        names = ["lk2a", "lk2b"]
    elif name == "L3":
        spec = [(90.0, 0.0), (210.0, 2.1), (330.0, 4.2)]
        paths, entries = [], []
        for k, (brg, ph) in enumerate(spec):
            P = radial_path(brg, amp=5.0, wavelength=120.0, phase=ph)
            pid = f"P{k + 1}"
            paths.append({"id": pid, "polyline": P})
            entries.append({"id": "ABC"[k], "path": pid, "s_start": 0.0, "dir": 1,
                            "s_end": polyline_len(P)})
        names = ["lk3a", "lk3b", "lk3c"]
    else:
        raise ValueError(name)

    pmap = {p["id"]: p["polyline"] for p in paths}
    trees = clear_forest(poisson_forest(FOREST_SEED) if base is None else base, paths, [])
    lookouts = []
    for e, rname in zip(entries, names):
        P = pmap[e["path"]]
        c, _, _ = posts.find_post(sys.modules[__name__], P, e, trees, PAD, FLARE)
        e["r_post"] = c["r"]                             # the post line: where the path crosses r_post
        e["s_post"] = round(c["s_post"], 3)
        e["post_cross"] = [round(v, 3) for v in c["post_cross"]]
        lx, ly, yaw = c["x"], c["y"], c["yaw"]           # yaw faces arriving walkers
        # spawn on the lane 16 m (L3) / 12.8 m (L2) from the mulcher, facing out
        if name == "L3":
            s_sp = arc_where_range(P, 16.0, e["s_end"], e["s_start"])
        else:
            s_sp = e["s_end"] - e["dir"] * 8.0
        (sx, sy), _ = point_at(P, s_sp)
        lookouts.append({"name": rname, "entry": e["id"],
                         "x": round(lx, 3), "y": round(ly, 3), "yaw": round(yaw, 5),
                         "r": c["r"], "side": c["side"],
                         "placement": {k: round(c[k], 3) if isinstance(c[k], float) else c[k] for k in
                                       ("seen_main_m", "seen_best_m", "nearest_trunk_m", "link_d_m", "snr_db",
                                        "trees_on_link", "corridor_w_m", "link_clear_m")},
                         # beside the mulcher: the drive-out start (trips.py); the experiment spawns on the post
                         "spawn": {"x": round(sx, 3), "y": round(sy, 3), "z": 0.3,
                                   "yaw": round(math.atan2(ly - sy, lx - sx), 5)}})
    return paths, entries, lookouts


# ------------------------------------------------------------------- forest --
def poisson_forest(seed):
    """Dart throwing at the target count with minimum spacing, over the disc."""
    rng = random.Random(seed)
    target = int(round(DENSITY * math.pi * R_FOREST ** 2))
    cell = MIN_SPACING / math.sqrt(2)
    grid = {}
    pts = []
    tries = 0
    while len(pts) < target and tries < 2_000_000:
        tries += 1
        r = R_FOREST * math.sqrt(rng.random())
        a = 2 * math.pi * rng.random()
        x, y = r * math.cos(a), r * math.sin(a)
        gx, gy = int(math.floor(x / cell)), int(math.floor(y / cell))
        ok = True
        for i in range(gx - 2, gx + 3):
            for j in range(gy - 2, gy + 3):
                q = grid.get((i, j))
                if q is not None and math.dist(q, (x, y)) < MIN_SPACING:
                    ok = False
                    break
            if not ok:
                break
        if ok:
            grid[(gx, gy)] = (x, y)
            pts.append((x, y))
    if len(pts) < target:
        raise RuntimeError(f"forest: placed {len(pts)} of {target}")
    return pts


def clear_forest(pts, paths, lookouts):
    keep = []
    for p in pts:
        if math.hypot(*p) < MULCHER_CLEAR:
            continue
        if any(dist_to_polyline(p, q["polyline"]) < LANE_HALF + FLARE for q in paths):
            continue
        if any(math.dist(p, (lk["x"], lk["y"])) < PAD + FLARE for lk in lookouts):
            continue
        keep.append(p)
    return keep


# --------------------------------------------------------------------- SDF --
def husky_lidar_block():
    """The Husky's front_laser <lidar> element, verbatim, so the mulcher carries
    exactly the same sensor model."""
    s = open(HUSKY_SDF).read()
    m = re.search(r"<lidar>.*?</lidar>", s, re.S)
    return re.sub(r"<!--.*?-->\s*", "", m.group(0), flags=re.S)


OAK_URI = "https://fuel.gazebosim.org/1.0/openrobotics/models/oak tree/7/files"


def oak_model(i, x, y, yaw):
    mat = lambda sub: f"""
            <material>
              {'<double_sided>true</double_sided>' if sub == 'Branch' else ''}
              <diffuse>1.0 1.0 1.0</diffuse>
              <pbr><metal><albedo_map>{OAK_URI}/materials/textures/{sub.lower()}_diffuse.png</albedo_map></metal></pbr>
            </material>"""
    return f"""
    <model name="oak_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><mesh><uri>{OAK_URI}/meshes/oak_tree.dae</uri></mesh></geometry>
        </collision>
        <visual name="branch">
          <geometry><mesh><uri>{OAK_URI}/meshes/oak_tree.dae</uri><submesh><name>Branch</name></submesh></mesh></geometry>{mat('Branch')}
        </visual>
        <visual name="bark">
          <geometry><mesh><uri>{OAK_URI}/meshes/oak_tree.dae</uri><submesh><name>Bark</name></submesh></mesh></geometry>{mat('Bark')}
        </visual>
      </link>
    </model>"""


def box(name, size, pose, rgb):
    geo = f"<geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>"
    return f"""
        <collision name="{name}"><pose>{pose[0]} {pose[1]} {pose[2]} 0 0 0</pose>{geo}</collision>
        <visual name="{name}"><pose>{pose[0]} {pose[1]} {pose[2]} 0 0 0</pose>{geo}
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>
        </visual>"""


# Static: set_pose turns body, head and roof lidar together (checked at eight
# poses by lookout/smoke/setpose3.py), and no physics can nudge it mid-walk.
def mulcher_model(lidar, x=0.0, y=0.0, yaw=0.0):
    return f"""
    <model name="mulcher">
      <static>true</static>
      <pose>{x} {y} 0 0 0 {yaw}</pose>
      <link name="base">{box('body', BODY, (0, 0, BODY[2] / 2), '0.85 0.55 0.1')}{box('head', HEAD, HEAD_C, '0.3 0.3 0.3')}
        <sensor name="roof_lidar" type="gpu_lidar">
          <topic>mulcher/laser_scan</topic>
          <ignition_frame_id>mulcher/velodyne</ignition_frame_id>
          <visualize>0</visualize>
          <update_rate>10</update_rate>
          <always_on>1</always_on>
          <pose>0 0 {MULCHER_LIDAR_Z} 0 0 0</pose>
          {lidar}
        </sensor>
      </link>
    </model>"""


def person_model():
    s = PERSON_SCALE
    z = -PERSON_MESH_ZMIN * s
    return f"""
    <model name="person">
      <static>true</static>
      <pose>{PERSON_PARK[0]} {PERSON_PARK[1]} 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <pose>0 0 {z:.4f} 0 0 0</pose>
          <geometry><mesh><uri>model://person_walking/meshes/walking.dae</uri><scale>{s:.5f} {s:.5f} {s:.5f}</scale></mesh></geometry>
        </visual>
      </link>
    </model>"""


def world_sdf(wname, trees, lidar, with_mulcher=True, extra=""):
    ground = 400
    parts = [f"""<?xml version="1.0" ?>
<!-- Generated by lookout/build_world.py; do not edit. -->
<sdf version="1.9">
  <world name="{wname}">
    <scene><grid>false</grid><ambient>1.0 1.0 1.0 1</ambient>
      <background>0.6 0.8 1.0 1</background><shadows>false</shadows></scene>
    <physics name="20ms" type="dart">
      <max_step_size>0.02</max_step_size>
      <real_time_factor>2.0</real_time_factor>
      <dart><collision_detector>bullet</collision_detector><solver><solver_type>dantzig</solver_type></solver></dart>
    </physics>
    <plugin name="ignition::gazebo::systems::Physics" filename="ignition-gazebo-physics-system" />
    <plugin name="ignition::gazebo::systems::Sensors" filename="ignition-gazebo-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="ignition::gazebo::systems::SceneBroadcaster" filename="ignition-gazebo-scene-broadcaster-system" />
    <plugin name="ignition::gazebo::systems::UserCommands" filename="ignition-gazebo-user-commands-system" />
    <plugin name="ignition::gazebo::systems::Imu" filename="libignition-gazebo-imu-system.so" />
    <gravity>0 0 -9.8</gravity>
    <atmosphere type="adiabatic" />
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{ground} {ground}</size></plane></geometry>
          <surface><friction><ode><mu>50</mu></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{ground} {ground}</size></plane></geometry>
          <material><ambient>0.4 0.26 0.13 1</ambient><diffuse>0.4 0.26 0.13 1</diffuse></material>
        </visual>
      </link>
    </model>"""]
    if with_mulcher:
        parts.append(mulcher_model(lidar))
    parts.append(person_model())
    rng = random.Random(FOREST_SEED + 1)
    for i, (x, y) in enumerate(trees):
        parts.append(oak_model(i, x, y, rng.uniform(-math.pi, math.pi)))
    parts.append(extra)
    parts.append("\n  </world>\n</sdf>\n")
    return "".join(parts)


# -------------------------------------------------------------------- YAML --
def yaml_dump(d, ind=0):
    """Tiny YAML writer (stdlib only): dicts, lists, scalars; polylines inline."""
    sp = "  " * ind
    out = []
    if isinstance(d, dict):
        for k, v in d.items():
            if isinstance(v, (dict, list)) and v and not _inline(v):
                out.append(f"{sp}{k}:")
                out.append(yaml_dump(v, ind + 1))
            else:
                out.append(f"{sp}{k}: {_scalar(v)}")
    else:
        for v in d:
            if isinstance(v, dict):
                sub = yaml_dump(v, ind + 1).lstrip()
                out.append(f"{sp}- {sub}")
            else:
                out.append(f"{sp}- {_scalar(v)}")
    return "\n".join(out)


def _inline(v):
    return isinstance(v, list) and all(not isinstance(x, dict) for x in v)


def _scalar(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return repr(v)
    if isinstance(v, list):
        return "[" + ", ".join(_scalar(x) for x in v) + "]"
    if isinstance(v, tuple):
        return _scalar(list(v))
    if isinstance(v, dict):
        return "{" + ", ".join(f"{k}: {_scalar(x)}" for k, x in v.items()) + "}"
    return str(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--worlds", required=True)
    a = ap.parse_args()
    os.makedirs(a.worlds, exist_ok=True)
    os.makedirs(os.path.join(HERE, "config"), exist_ok=True)
    lidar = husky_lidar_block()
    base = poisson_forest(FOREST_SEED)
    for name in ("L2", "L3"):
        paths, entries, lookouts = layout(name, base)
        trees = clear_forest(base, paths, lookouts)
        if len(trees) != len(clear_forest(base, paths, [])):
            raise RuntimeError(f"{name}: a post's pad is not free")   # posts.py only picks free spots
        wname = f"lookout_{name}"
        open(os.path.join(a.worlds, f"{wname}.sdf"), "w").write(world_sdf(wname, trees, lidar))
        with open(os.path.join(a.worlds, f"forest_{name}.csv"), "w") as f:
            f.write("x,y\n")
            for x, y in trees:
                f.write(f"{x:.3f},{y:.3f}\n")
        cfg = {
            "layout": name,
            "world": wname,
            "mulcher": {"x": 0.0, "y": 0.0, "body_size": list(BODY), "head_size": list(HEAD),
                        "head_center": list(HEAD_C), "lidar_z": MULCHER_LIDAR_Z},
            "person": {"height": PERSON_H, "scale": round(PERSON_SCALE, 5),
                       "yaw_offset": round(math.pi / 2, 6),
                       "park": list(PERSON_PARK)},
            "forest": {"seed": FOREST_SEED, "density": DENSITY, "min_spacing": MIN_SPACING,
                       "radius": R_FOREST, "n_base": len(base), "n_trees": len(trees),
                       "lane_clear": LANE_HALF + FLARE, "pad_free": PAD + FLARE,
                       "mulcher_clear": MULCHER_CLEAR, "csv": f"forest_{name}.csv"},
            "end_radius": R_END,
            "placement": {"method": "lookout/posts.py", "step": posts.STEP, "r_min": posts.R_MIN,
                          "r_safe": posts.R_SAFE, "clear_margin": posts.CLEAR_MARGIN, "trunk_r": posts.TRUNK_R,
                          "reach_main": posts.REACH_MAIN, "reach_best": posts.REACH_BEST,
                          "pad_free": PAD + FLARE},
            "walk": {"step": 0.5, "speed": 1.3, "start_before_post": 45.0,
                     "jitter_along": 5.0, "jitter_across": 0.5, "seed": 7 if name == "L2" else 11,
                     "headings_deg": [30 * k for k in range(12)], "repeats": 2},
            "paths": [{"id": p["id"], "polyline": [list(v) for v in p["polyline"]]} for p in paths],
            "entries": [{k: (round(v, 3) if isinstance(v, float) else v) for k, v in e.items()}
                        for e in entries],
            "lookouts": lookouts,
        }
        open(os.path.join(HERE, "config", f"{name}.yaml"), "w").write(
            f"# Generated by lookout/build_world.py; do not edit.\n{yaml_dump(cfg)}\n")
        print(f"{name}: {len(trees)} oaks (of {len(base)}), "
              + ", ".join(f"{e['id']}: post line {e['r_post']} m at s={e['s_post']:.1f} of {e['s_end']:.1f}"
                          for e in entries))
        for lk in lookouts:
            pl = lk["placement"]
            print(f"   {lk['name']} -> ({lk['x']:.2f}, {lk['y']:.2f}) yaw {math.degrees(lk['yaw']):.1f}, "
                  f"{lk['r']} m {lk['side']}: first seen {pl['seen_main_m']} m (main), {pl['seen_best_m']} m "
                  f"(best); nearest trunk {pl['nearest_trunk_m']} m; link clearance "
                  f"{pl['link_clear_m'] - pl['corridor_w_m']:.2f} m")
    # Calibration: open ground, the mulcher at the origin (heading 0) and a
    # person; the Husky is spawned by the calibration runner.
    open(os.path.join(a.worlds, "lookout_calib.sdf"), "w").write(
        world_sdf("lookout_calib", [], lidar))
    print("calib world written")


if __name__ == "__main__":
    main()
