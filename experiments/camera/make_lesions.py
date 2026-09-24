#!/usr/bin/env python3
"""Lesion plates on the 19 usable in-ROI oaks, and the camera world overlay.

Container python (numpy). Usage:
  make_lesions.py <world.sdf> <oak_tree.dae> <all_oaks_trees.txt> <out_dir>
Writes <out_dir>/lesions.json and <out_dir>/overlay/flatforestv2_cam.sdf.

Layout (PLAN.md): 6 per tree, sizes 2 x 4/8/16 cm, height U(0.65, 1.25) m,
one per 60 deg sector about the trunk axis with uniform jitter, sizes shuffled
over sectors, seed 2026. Each plate is a 3 mm box whose back face sits 2 mm
outside the largest bark radius under its footprint, facing radially out.
Labels: lesion 10 + 6k + j, bark of oak k 130 + k (k = order in the trees file).
"""
import json, math, os, re, sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pilot_fine"))
from fine_dbh import load_mesh, slice_mesh, robust_circle  # noqa: E402

SIZES = [0.04, 0.04, 0.08, 0.08, 0.16, 0.16]
Z_RANGE = (0.65, 1.25)
THICK = 0.003
GAP = 0.002
SEED = 2026


def main():
    world, dae, trees_file, out = sys.argv[1:5]
    trees = []
    for line in open(trees_file):
        if line.strip():
            tid, name, x, y = line.strip().split(":")
            trees.append((int(tid), name.replace("_", " ", 1), float(x), float(y)))
    tris = load_mesh(dae)
    # Trunk axis in the model frame: same robust fit fine_dbh.py uses (z 0.6-1.2).
    levels = np.round(np.arange(0.60, 1.2001, 0.01), 3)
    pts = []
    for z in levels:
        for x0, y0, x1, y1 in slice_mesh(tris, z):
            k = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.005) + 1)
            t = np.linspace(0, 1, k)
            pts.append(np.c_[x0 + t * (x1 - x0), y0 + t * (y1 - y0)])
    c0, r0, _ = robust_circle(np.concatenate(pts))
    print("trunk axis (model frame) %.4f %.4f  r %.4f" % (c0[0], c0[1], r0))

    # Dense bark samples over the lesion band (+ margin), in polar coords about the axis.
    zs = np.round(np.arange(Z_RANGE[0] - 0.09, Z_RANGE[1] + 0.0901, 0.01), 3)
    polar = []
    for z in zs:
        for x0, y0, x1, y1 in slice_mesh(tris, z):
            k = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.003) + 1)
            t = np.linspace(0, 1, k)
            xy = np.c_[x0 + t * (x1 - x0), y0 + t * (y1 - y0)] - c0
            polar.append(np.c_[np.full(k, z), np.arctan2(xy[:, 1], xy[:, 0]), np.hypot(xy[:, 0], xy[:, 1])])
    polar = np.concatenate(polar)
    polar = polar[polar[:, 2] < 1.0]       # trunk only (branches start above the band)

    rng = np.random.default_rng(SEED)
    lesions, bark = [], {}
    for k, (tid, name, ox, oy) in enumerate(trees):
        bark[name] = 130 + k
        sizes = list(rng.permutation(SIZES))
        for j in range(6):
            phi = math.radians(60 * j + rng.uniform(0, 60))
            s = float(sizes[j])
            z = float(rng.uniform(*Z_RANGE))
            half_ang = (s / 2) / r0 + 0.02
            dphi = np.angle(np.exp(1j * (polar[:, 1] - phi)))
            m = (np.abs(dphi) <= half_ang) & (np.abs(polar[:, 0] - z) <= s / 2 + 0.005)
            rmax = float(polar[m, 2].max())
            R = rmax + GAP + THICK / 2
            n = np.array([math.cos(phi), math.sin(phi), 0.0])
            u = np.array([-math.sin(phi), math.cos(phi), 0.0])   # tangent (horizontal)
            cm = np.array([c0[0] + R * n[0], c0[1] + R * n[1], z])   # model frame
            cw = cm + [ox, oy, 0.0]
            front = cw + n * THICK / 2                            # visible face centre
            corners = [(front + a * u * s / 2 + np.array([0, 0, b * s / 2])).tolist()
                       for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            lesions.append({
                "label": 10 + 6 * k + j, "tree_k": k, "tree_id": tid, "oak": name,
                "size": s, "z": z, "phi": phi, "r_bark_max": rmax,
                "center_model": cm.tolist(), "center_world": cw.tolist(),
                "face_world": front.tolist(), "normal": n.tolist(), "corners_world": corners})

    # --- world overlay ------------------------------------------------------
    s = open(world).read()
    for name, lab in bark.items():
        blk = re.search(r'<model name="%s">.*?</model>' % re.escape(name), s, re.S)
        body = blk.group(0)
        new = re.sub(r'(<visual name="bark">.*?<label>)2(</label>)',
                     lambda mm: mm.group(1) + str(lab) + mm.group(2), body, count=1, flags=re.S)
        assert new != body, name
        s = s[:blk.start()] + new + s[blk.end():]
    models = []
    for k, (tid, name, ox, oy) in enumerate(trees):
        vis = []
        for L in [l for l in lesions if l["tree_k"] == k]:
            x, y, z = L["center_model"]
            vis.append(f"""        <visual name="lesion_{L['label']}">
          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {L['phi']:.5f}</pose>
          <geometry><box><size>{THICK} {L['size']} {L['size']}</size></box></geometry>
          <material><ambient>0.55 0.1 0.1 1</ambient><diffuse>0.55 0.1 0.1 1</diffuse></material>
          <cast_shadows>false</cast_shadows>
          <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
            <label>{L['label']}</label>
          </plugin>
        </visual>""")
        models.append(f"""    <model name="lesions_{tid}">
      <static>true</static>
      <pose>{ox} {oy} 0 0 0 0</pose>
      <link name="link">
{chr(10).join(vis)}
      </link>
    </model>""")
    note = ("    <!-- cam_experiment overlay (runs/cam_experiment/make_lesions.py): lesion plates on the 19\n"
            "         usable in-ROI oaks, visual only, labels 10-123; their bark relabelled 130-148. -->\n")
    i = s.rindex("</world>")
    s = s[:i] + note + "\n".join(models) + "\n  " + s[i:]
    os.makedirs(os.path.join(out, "overlay"), exist_ok=True)
    open(os.path.join(out, "overlay", "flatforestv2_cam.sdf"), "w").write(s)
    json.dump({"seed": SEED, "axis_model": c0.tolist(), "r_fit": r0, "thick": THICK,
               "bark_labels": bark, "trees": [t[:2] for t in trees], "lesions": lesions},
              open(os.path.join(out, "lesions.json"), "w"), indent=1)
    print("lesions", len(lesions), "labels", lesions[0]["label"], "-", lesions[-1]["label"])
    print("r_bark_max range %.3f-%.3f" % (min(l["r_bark_max"] for l in lesions),
                                         max(l["r_bark_max"] for l in lesions)))


if __name__ == "__main__":
    main()
