#!/usr/bin/env python3
"""Stepped person walks for one layout (brief Part 6). Runs in the layout's
container after the lookouts hold their posts and their stacks are stopped.

  walks.py --layout L2 --out /runs/lookout/L2_full --mode full|pilot|check

Per layout: backgrounds for every lidar are learned once with the person
parked (lookouts; re-learned only if a lookout's lidar moves), then for each of
the 12 mulcher headings: turn the mulcher, re-learn its background, run that
heading's walks. A walk starts 45 m (+-5 m, fixed seed) before the path's post
line (where it crosses the radius of its lookout's post, posts.py), +-0.5 m
across the path, and steps 0.5 m at a time, facing the
walking direction, until 10 m from the mulcher; walk time = distance / 1.3 m/s.
After each move the world steps once (0.1 s) and every lidar's first scan
rendered after the move is run through detect.py.

--mode check: the 12 check walks (one per heading, entries in turn, repeat
0, the matching walk's offsets) without the lookouts, two ways (--check-how):
  away    the lookouts spawned, then every one moved 1 km away (run_layout.sh: in
          the sim of the main walks)
  absent  in a fresh sim where the lookouts were never spawned
Removing a lookout (`ign service .../remove`) crashes the server in Fortress
(ogre2 assert when a model with a gpu_lidar goes; 2026-09-24 pilots).
--mode pilot: 6 walks, one per heading 0, 60, ..., 300, entries in turn, with
at least one in the mulcher's blind wedge.

Lidar poses come from Gazebo: each lookout from its ground-truth odometry and
its sensor's pose in the model (`ign model`), the mulcher from its pose read
back after every turn. Nothing is hard-coded.

Outputs (all rows carry the session id; a walk interrupted by a crash is
re-run on restart and only the session in walks.csv counts):
  walks.csv        one row per walk (brief Part 7 fields, main rules)
  first_detect.csv one row per walk x detector x rule variant (sensitivity)
  steps.csv        one row per walk x step x lidar
  alarms.csv       every alarm, every variant (false alarms: hit = 0)
  events.jsonl     learning, turns, re-learns, removals, timing
  bg/*.npz         every background learned, with the lidar pose
  scans/<walk>.npz per step and lidar: returns >= 0.1 m closer than background
                   (or on a no-background beam) and the returns in the
                   person's box, world frame, with ring/col/range/background
"""
import argparse
import csv
import json
import math
import os
import random
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, "/lookout")
import build_world as bw  # noqa: E402  (stdlib-only; path geometry)
import detect as D  # noqa: E402
import gz  # noqa: E402
import radio  # noqa: E402

# Alarm-rule variants: name -> (min points, horizontal linkage). m5 is the
# main rule; m3/m10/m20 are the brief's sensitivity settings; xy5 is the main
# rule with horizontal linkage (sensitivity, declared before the pilot: 3D
# linkage cannot span two rings past ~14 m, see detect.py).
VARIANTS = {"m3": (3, False), "m5": (5, False), "m10": (10, False), "m20": (20, False), "xy5": (5, True)}
MAIN = "m5"
# A lookout lidar that moved more than this, or turned more than this, since
# its background gets a new one at the next walk boundary. Tight, because at
# grazing incidence 0.02 deg of tilt already turns hundreds of ground returns
# "new" (walk-driver test, a Husky still settling after spawn).
MOVE_TOL_M = 0.005
MOVE_TOL_DEG = 0.01
# Before the first background the lookouts must be at rest: pose change over
# SETTLE_WINDOW_S of sim time below these (a spawned Husky takes ~20 s of sim
# time to settle on its suspension; lookout/smoke/settle.py). The lookouts are
# spawned static now (spawn_static.py), so they pass at once; kept as a guard.
SETTLE_WINDOW_S = 2.0
SETTLE_TOL_M = 0.0002
SETTLE_TOL_DEG = 0.002
SETTLE_MAX_S = 120.0
N_BG = 10                           # scans per background
WEDGE_DEG = 30.0                    # blind wedge half-width (brief: +-30 deg at 20 m)
FAR = (1000.0, 1000.0)              # check walks "away": where the lookouts are moved
MAX_STEPS = None                    # testing only (--max-steps)


def wrap(a):
    return math.remainder(a, 2 * math.pi)


# ------------------------------------------------------------------ plan --
def load_cfg(layout):
    with open(f"/lookout/config/{layout}.yaml") as f:
        return yaml.safe_load(f)


def plan_walks(cfg):
    """Every main walk in run order, offsets drawn in that order from the
    layout's seed (so a walk's offsets never depend on the mode)."""
    w = cfg["walk"]
    rng = random.Random(w["seed"])
    out = []
    for h in w["headings_deg"]:
        for e in cfg["entries"]:
            for rep in range(w["repeats"]):
                along = rng.uniform(-w["jitter_along"], w["jitter_along"])
                across = rng.uniform(-w["jitter_across"], w["jitter_across"])
                out.append({"id": f"{cfg['layout']}-h{h:03d}-{e['id']}-r{rep}", "heading": h,
                            "entry": e["id"], "rep": rep, "along": along, "across": across})
    return out


def walk_points(cfg, walk):
    w = cfg["walk"]
    e = next(x for x in cfg["entries"] if x["id"] == walk["entry"])
    P = [tuple(v) for v in next(p for p in cfg["paths"] if p["id"] == e["path"])["polyline"]]
    d = e["dir"]
    mx, my = cfg["mulcher"]["x"], cfg["mulcher"]["y"]
    s0 = e["s_post"] - d * (w["start_before_post"] + walk["along"])
    n = int(math.floor(d * (e["s_end"] - s0) / w["step"] + 1e-9))
    pts = []
    for k in range(n + 1):
        s = s0 + d * w["step"] * k
        (x, y), (tx, ty) = bw.point_at(P, s)
        px, py = x - walk["across"] * ty, y + walk["across"] * tx
        pts.append({"k": k, "s": s, "x": px, "y": py, "heading": math.atan2(d * ty, d * tx),
                    "t": w["step"] * k / w["speed"], "dist": math.hypot(px - mx, py - my),
                    # path distance still to go to the post line (negative once past it)
                    "to_post": d * (e["s_post"] - s)})
    return pts


def bearing20(cfg, walk, pts):
    """Person bearing relative to the mulcher heading where the walk first
    comes within 20 m (degrees, wrapped)."""
    mx, my = cfg["mulcher"]["x"], cfg["mulcher"]["y"]
    p = next((q for q in pts if q["dist"] <= 20.0), pts[-1])
    return math.degrees(wrap(math.atan2(p["y"] - my, p["x"] - mx) - math.radians(walk["heading"])))


def pick_pilot(cfg, walks):
    ents = [e["id"] for e in cfg["entries"]]
    pick = []
    for i, h in enumerate((0, 60, 120, 180, 240, 300)):
        pick.append(next(w for w in walks if w["heading"] == h and w["entry"] == ents[i % len(ents)] and w["rep"] == 0))
    if not any(abs(bearing20(cfg, w, walk_points(cfg, w))) <= WEDGE_DEG for w in pick):
        wedge = next(w for w in walks if w["rep"] == 0 and w not in pick
                     and abs(bearing20(cfg, w, walk_points(cfg, w))) <= WEDGE_DEG)
        pick[-1] = wedge
    return sorted(pick, key=lambda w: walks.index(w))


def pick_check(cfg, walks, how):
    ents = [e["id"] for e in cfg["entries"]]
    out = []
    for i, h in enumerate(cfg["walk"]["headings_deg"]):
        w = next(x for x in walks if x["heading"] == h and x["entry"] == ents[i % len(ents)] and x["rep"] == 0)
        out.append({**w, "id": f"{w['id']}-chk-{how}", "matches": w["id"]})
    return out


# --------------------------------------------------------------- logging --
class Csv:
    def __init__(self, path, fields):
        new = not os.path.exists(path)
        self.f = open(path, "a", newline="")
        self.w = csv.DictWriter(self.f, fieldnames=fields, extrasaction="raise")
        if new:
            self.w.writeheader()
        self.fields = fields

    def row(self, **kw):
        self.w.writerow({k: ("" if kw.get(k) is None else kw.get(k)) for k in self.fields})

    def flush(self):
        self.f.flush()


def fmt(v, nd=3):
    return None if v is None else round(float(v), nd)


# --------------------------------------------------------------- session --
class Session:
    def __init__(self, cfg, out, mode, lookouts_present):
        import rclpy
        from scans import ScanNode
        self.cfg, self.out, self.mode = cfg, out, mode
        self.W = cfg["world"]
        self.sid = time.strftime("%Y%m%dT%H%M%S")
        self.lk = [l["name"] for l in cfg["lookouts"]] if lookouts_present else []
        self.names = ["mulcher"] + self.lk
        os.makedirs(os.path.join(out, "bg"), exist_ok=True)
        os.makedirs(os.path.join(out, "scans"), exist_ok=True)
        self.ev = open(os.path.join(out, "events.jsonl"), "a")
        rclpy.init()
        self.node = ScanNode(self.W, {n: f"/{n}/velodyne_points" for n in self.names}, odoms=self.lk,
                             name=f"walks_{cfg['layout']}")
        if not self.node.wait_all(odoms=self.lk):
            raise RuntimeError(f"no clouds/odom from {[n for n in self.names if n not in self.node.last]}")
        self.sensor = {n: gz.model_info(n, "front_laser")[1] for n in self.lk}
        self.sensor["mulcher"] = gz.model_info("mulcher", "roof_lidar")[1]
        self.bg, self.bg_tag, self.bg_pose = {}, {}, {}
        self.relearn = set()
        self.mulcher_pose = None
        self.heading = None
        self.trees = [tuple(map(float, l.split(","))) for l in
                      open(f"/runs/lookout/worlds/{cfg['forest']['csv']}").read().split("\n")[1:] if l]
        self.yaw_off = cfg["person"]["yaw_offset"]
        self.park = cfg["person"]["park"]
        self.log("session", mode=mode, sid=self.sid, lidars=self.names,
                 sensor_poses={k: v for k, v in self.sensor.items()}, rule=D.rule_dict())
        fields_w = (["session", "layout", "walk", "mode", "entry", "heading_deg", "repeat", "along_m", "across_m",
                     "lookouts_present", "matches", "n_steps", "s_start", "bearing20_deg", "in_wedge"]
                    + [f"{d}_{r}_{f}" for d in ["mulcher"] + [l["name"] for l in cfg["lookouts"]] + ["team"]
                       for r in ("alarm", "best")
                       for f in ("step", "t_s", "x", "y", "dist_m", "warn_s", "before_post", "before50")]
                    + ["team_alarm_first_by", "team_best_first_by"]
                    + [f"fa_{n}" for n in ["mulcher"] + [l["name"] for l in cfg["lookouts"]]]
                    + [f"linked_at_alarm_{l['name']}" for l in cfg["lookouts"]]
                    + [f"hold_max_dpos_m_{l['name']}" for l in cfg["lookouts"]]
                    + [f"hold_max_dang_deg_{l['name']}" for l in cfg["lookouts"]]
                    + ["retries", "wall_s", "sim_steps"])
        self.cw = Csv(os.path.join(out, "walks.csv"), fields_w)
        self.cf = Csv(os.path.join(out, "first_detect.csv"),
                      ["session", "walk", "detector", "variant", "step", "t_s", "x", "y", "dist_m", "warn_s",
                       "before_post", "before50"])
        self.cs = Csv(os.path.join(out, "steps.csv"),
                      ["session", "walk", "k", "t_s", "s", "px", "py", "dist_m", "to_post_m", "stamp", "retries",
                       "wall_s", "lidar", "bg_tag", "lx", "ly", "lz", "lyaw_deg", "dpos_m", "dang_deg",
                       "n_new", "n_clusters"]
                      + [f"{f}_{v}" for v in VARIANTS for f in ("n_cand", "n_alarm", "hit", "n_false")]
                      + ["best_all", "best_ns", "best_rings", "best_ring_list",
                         "near_n", "near_rings", "near_d_m", "nearxy_n", "nearxy_rings", "nearxy_d_m"])
        self.ca = Csv(os.path.join(out, "alarms.csv"),
                      ["session", "walk", "k", "t_s", "stamp", "lidar", "variant", "hit", "cx", "cy", "cz",
                       "n", "rings", "px", "py", "err_xy_m"])

    def log(self, kind, **kw):
        self.ev.write(json.dumps({"t_wall": time.time(), "kind": kind, **kw}, default=str) + "\n")
        self.ev.flush()
        print(f"[walks] {kind} {json.dumps(kw, default=str)[:300]}", flush=True)

    # --- poses
    def lidar_pose(self, n):
        if n == "mulcher":
            return self.mulcher_pose
        return gz.odom_sensor(self.node.odom[n], self.sensor[n])

    def set_person(self, x, y, heading):
        gz.set_pose(self.W, "person", x, y, 0.0, heading + self.yaw_off)

    # --- backgrounds
    def settle(self):
        """Step the paused world until every lookout is at rest (see
        SETTLE_*). Logs the history; raises if one never settles."""
        if not self.lk:
            return
        hist, t_sim = [], 0.0
        n_win = int(round(SETTLE_WINDOW_S / 0.1))
        while True:
            self.node.step_fresh()
            t_sim += 0.1
            hist.append({n: self.lidar_pose(n) for n in self.lk})
            if len(hist) > n_win:
                worst = {}
                for n in self.lk:
                    R0, t0 = hist[-1 - n_win][n]
                    R, t = hist[-1][n]
                    c = (np.trace(R0.T @ R) - 1.0) / 2.0
                    worst[n] = (float(np.linalg.norm(t - t0)), math.degrees(math.acos(max(-1.0, min(1.0, c)))))
                if all(d <= SETTLE_TOL_M and a <= SETTLE_TOL_DEG for d, a in worst.values()):
                    self.log("settled", sim_s=round(t_sim, 1), last_window={n: [round(d, 6), round(a, 5)]
                                                                            for n, (d, a) in worst.items()})
                    return
                if t_sim >= SETTLE_MAX_S:
                    raise RuntimeError(f"lookouts not at rest after {SETTLE_MAX_S} s sim: {worst}")
                if int(round(t_sim * 10)) % 50 == 0:
                    self.log("settling", sim_s=round(t_sim, 1), window={n: [round(d, 6), round(a, 5)]
                                                                        for n, (d, a) in worst.items()})

    def learn(self, names, tag):
        self.set_person(self.park[0], self.park[1], 0.0)
        self.node.step_fresh()                              # the parking move
        acc = {n: [] for n in names}
        for _ in range(N_BG):
            msgs, _ = self.node.step_fresh()
            for n in names:
                acc[n].append(gz.decode(msgs[n])["range"])
        for n in names:
            R, t = self.lidar_pose(n)
            self.bg[n] = D.learn_background(acc[n])
            self.bg_tag[n] = f"{n}__{tag}"
            self.bg_pose[n] = (R.copy(), t.copy())
            np.savez_compressed(os.path.join(self.out, "bg", f"{n}__{tag}__{self.sid}.npz"),
                                bg=self.bg[n].astype(np.float32), R=R, t=t)
            fin = np.isfinite(self.bg[n])
            self.log("background", lidar=n, tag=tag, finite=int(fin.sum()), t=t.tolist())

    def moved(self, n, R, t):
        R0, t0 = self.bg_pose[n]
        dpos = float(np.linalg.norm(t - t0))
        c = (np.trace(R0.T @ R) - 1.0) / 2.0
        dang = math.degrees(math.acos(max(-1.0, min(1.0, c))))
        return dpos, dang

    def set_heading(self, h):
        yaw = math.radians(h)
        gz.set_pose(self.W, "mulcher", self.cfg["mulcher"]["x"], self.cfg["mulcher"]["y"], 0.0, yaw)
        self.node.step_fresh()
        mp, _ = gz.model_info("mulcher")
        if abs(wrap(mp[1][2] - yaw)) > 1e-3 or math.hypot(mp[0][0] - self.cfg["mulcher"]["x"],
                                                           mp[0][1] - self.cfg["mulcher"]["y"]) > 1e-3:
            raise RuntimeError(f"mulcher readback {mp} != heading {h}")
        self.mulcher_pose = gz.compose(mp, self.sensor["mulcher"])
        self.heading = h
        self.log("heading", heading=h, readback=mp, lidar_t=self.mulcher_pose[1].tolist())

    # --- one walk
    def run_walk(self, walk):
        cfg = self.cfg
        pts = walk_points(cfg, walk)
        b20 = bearing20(cfg, walk, pts)
        if MAX_STEPS:
            pts = pts[:MAX_STEPS]
        det = {n: {v: D.Detector(n, min_pts=mp) for v, (mp, _) in VARIANTS.items()} for n in self.names}
        first = {n: {f"alarm_{v}": None for v in VARIANTS} | {"best": None} for n in self.names}
        fa = {n: 0 for n in self.names}
        hold = {n: [0.0, 0.0] for n in self.lk}
        linked = {}
        rec = {k: [] for k in ("step", "lidar", "row", "col", "xyz", "ring", "range", "bg", "box")}
        retries, w0 = 0, time.time()
        tm = {"set_pose": 0.0, "step_fresh": 0.0, "process": 0.0}
        self.log("walk_start", walk=walk["id"], heading=walk["heading"], entry=walk["entry"], n_steps=len(pts),
                 bearing20=b20)
        for p in pts:
            t0 = time.time()
            self.set_person(p["x"], p["y"], p["heading"])
            t1 = time.time()
            msgs, info = self.node.step_fresh()
            t2 = time.time()
            tm["set_pose"] += t1 - t0
            tm["step_fresh"] += t2 - t1
            retries += info["retries"]
            model_yaw = p["heading"] + self.yaw_off
            stamp = info["stamps"][self.names[0]]
            for li, n in enumerate(self.names):
                d = gz.decode(msgs[n])
                R, t = self.lidar_pose(n)
                Pw = gz.to_world(d, R, t)
                r, rings, bg = d["range"], d["ring"].astype(np.int64), self.bg[n]
                m = D.new_mask(r, bg, D.RULE.closer)
                cls = {False: D.clusters(Pw[m], rings[m]), True: D.clusters(Pw[m], rings[m], xy=True)}
                cl = cls[False]
                row = {}
                for v, (mp, xy) in VARIANTS.items():
                    alarms, cands = det[n][v].step_clusters(cls[xy], (p["x"], p["y"]))
                    hits = [a for a in alarms if a["hit"]]
                    row.update({f"n_cand_{v}": len(cands), f"n_alarm_{v}": len(alarms),
                                f"hit_{v}": int(bool(hits)), f"n_false_{v}": len(alarms) - len(hits)})
                    if hits and first[n][f"alarm_{v}"] is None:
                        first[n][f"alarm_{v}"] = p
                    if v == MAIN:
                        fa[n] += len(alarms) - len(hits)
                        if hits and n in self.lk and n not in linked:
                            mx, my = cfg["mulcher"]["x"], cfg["mulcher"]["y"]
                            o = self.node.odom[n].pose.pose.position
                            linked[n] = radio.link((o.x, o.y, o.z), (mx, my, 0.0), self.trees)["linked"]
                    for a in alarms:
                        c = a["centroid"]
                        self.ca.row(session=self.sid, walk=walk["id"], k=p["k"], t_s=fmt(p["t"]), stamp=stamp,
                                    lidar=n, variant=v, hit=int(a["hit"]), cx=fmt(c[0]), cy=fmt(c[1]),
                                    cz=fmt(c[2]), n=a["n"], rings=" ".join(map(str, a["rings"])),
                                    px=fmt(p["x"]), py=fmt(p["y"]),
                                    err_xy_m=fmt(math.hypot(c[0] - p["x"], c[1] - p["y"])))
                b_all, b_ns, box, b_rings = D.best_case_count(Pw, r, bg, p["x"], p["y"], model_yaw, rings=rings)
                near = D.nearest(cls[False], (p["x"], p["y"]))
                nearxy = D.nearest(cls[True], (p["x"], p["y"]))
                if b_ns >= D.RULE.box_min and first[n]["best"] is None:
                    first[n]["best"] = p
                dpos = dang = 0.0
                if n in self.lk:
                    dpos, dang = self.moved(n, R, t)
                    hold[n] = [max(hold[n][0], dpos), max(hold[n][1], dang)]
                    if dpos > MOVE_TOL_M or dang > MOVE_TOL_DEG:
                        self.relearn.add(n)
                yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
                self.cs.row(session=self.sid, walk=walk["id"], k=p["k"], t_s=fmt(p["t"]), s=fmt(p["s"]),
                            px=fmt(p["x"]), py=fmt(p["y"]), dist_m=fmt(p["dist"]), to_post_m=fmt(p["to_post"]),
                            stamp=stamp, retries=info["retries"], wall_s=fmt(info["wall_s"]), lidar=n,
                            bg_tag=self.bg_tag[n], lx=fmt(t[0]), ly=fmt(t[1]), lz=fmt(t[2]), lyaw_deg=fmt(yaw),
                            dpos_m=fmt(dpos, 4), dang_deg=fmt(dang, 4), n_new=int(m.sum()), n_clusters=len(cl),
                            best_all=b_all, best_ns=b_ns, best_rings=len(b_rings),
                            best_ring_list=" ".join(map(str, b_rings)),
                            near_n=near[0], near_rings=near[1], near_d_m=fmt(near[2]),
                            nearxy_n=nearxy[0], nearxy_rings=nearxy[1], nearxy_d_m=fmt(nearxy[2]), **row)
                lg = D.log_arrays(Pw, rings, r, bg)
                lb = box[lg["row"].astype(np.int64), lg["col"].astype(np.int64)]
                # plus the returns in the person's box that the loose set leaves out
                extra = box & ~D.new_mask(r, bg, D.RULE.log_closer)
                er, ec = np.nonzero(extra)
                for k2, v in (("row", er.astype(np.uint8)), ("col", ec.astype(np.uint16)),
                              ("xyz", Pw[extra].astype(np.float32)), ("ring", rings[extra].astype(np.uint8)),
                              ("range", r[extra].astype(np.float32)), ("bg", bg[extra].astype(np.float32))):
                    lg[k2] = np.concatenate([lg[k2], v])
                lb = np.concatenate([lb, np.ones(len(er), dtype=bool)])
                nrec = len(lg["row"])
                rec["step"].append(np.full(nrec, p["k"], dtype=np.uint16))
                rec["lidar"].append(np.full(nrec, li, dtype=np.uint8))
                rec["box"].append(lb)
                for k2 in ("row", "col", "xyz", "ring", "range", "bg"):
                    rec[k2].append(lg[k2])
            tm["process"] += time.time() - t2
        self.cs.flush(); self.ca.flush()
        np.savez_compressed(os.path.join(self.out, "scans", f"{walk['id']}__{self.sid}.npz"),
                            lidars=np.array(self.names), steps_t=np.array([q["t"] for q in pts]),
                            steps_xy=np.array([[q["x"], q["y"]] for q in pts]),
                            **{k: (np.concatenate(v) if v else np.zeros(0)) for k, v in rec.items()})
        self.log("walk_timing", walk=walk["id"], svc_retries_total=len(gz.SVC_RETRIES),
                 **{k: round(v, 1) for k, v in tm.items()})
        self.summarise(walk, pts, b20, first, fa, linked, hold, retries, time.time() - w0)

    def summarise(self, walk, pts, b20, first, fa, linked, hold, retries, wall):
        cfg = self.cfg
        r_post = next(e["r_post"] for e in cfg["entries"] if e["id"] == walk["entry"])
        row = {"session": self.sid, "layout": cfg["layout"], "walk": walk["id"], "mode": self.mode,
               "entry": walk["entry"], "heading_deg": walk["heading"], "repeat": walk["rep"],
               "along_m": fmt(walk["along"]), "across_m": fmt(walk["across"]),
               "lookouts_present": int(bool(self.lk)), "matches": walk.get("matches", walk["id"]),
               "n_steps": len(pts), "s_start": fmt(pts[0]["s"]), "bearing20_deg": fmt(b20, 2),
               "in_wedge": int(abs(b20) <= WEDGE_DEG), "retries": retries, "wall_s": fmt(wall, 1),
               "sim_steps": len(pts)}

        def cols(p):
            if p is None:
                return {"step": None, "t_s": None, "x": None, "y": None, "dist_m": None, "warn_s": None,
                        "before_post": 0, "before50": 0}
            return {"step": p["k"], "t_s": fmt(p["t"]), "x": fmt(p["x"]), "y": fmt(p["y"]),
                    "dist_m": fmt(p["dist"]), "warn_s": fmt(p["to_post"] / cfg["walk"]["speed"]),
                    "before_post": int(p["dist"] > r_post), "before50": int(p["dist"] > 50.0)}

        variants = [f"alarm_{v}" for v in VARIANTS] + ["best"]
        team = {}
        for v in variants:
            ks = {n: first[n][v]["k"] for n in self.names if first[n][v] is not None}
            if ks:
                k0 = min(ks.values())
                by = sorted(n for n, k in ks.items() if k == k0)      # ties: all, joined by '+'
                team[v] = ("+".join(by), first[by[0]][v])
            else:
                team[v] = (None, None)
        for n in self.names + ["team"]:
            for v in variants:
                p = team[v][1] if n == "team" else first[n][v]
                c = cols(p)
                self.cf.row(session=self.sid, walk=walk["id"], detector=n, variant=v, **c)
                if v in (f"alarm_{MAIN}", "best"):
                    rn = "alarm" if v.startswith("alarm") else "best"
                    row.update({f"{n}_{rn}_{f}": x for f, x in c.items()})
        row["team_alarm_first_by"] = team[f"alarm_{MAIN}"][0]
        row["team_best_first_by"] = team["best"][0]
        for n in self.names:
            row[f"fa_{n}"] = fa[n]
        for n in self.lk:
            row[f"linked_at_alarm_{n}"] = None if n not in linked else int(linked[n])
            row[f"hold_max_dpos_m_{n}"] = fmt(hold[n][0], 4)
            row[f"hold_max_dang_deg_{n}"] = fmt(hold[n][1], 4)
        self.cw.row(**row)
        self.cw.flush(); self.cf.flush()
        self.log("walk_done", walk=walk["id"], wall_s=round(wall, 1), retries=retries,
                 team_alarm=row.get("team_alarm_dist_m"), mulcher_alarm=row.get("mulcher_alarm_dist_m"),
                 fa={n: fa[n] for n in self.names})


def done_walks(out):
    p = os.path.join(out, "walks.csv")
    if not os.path.exists(p):
        return set()
    with open(p) as f:
        return {r["walk"] for r in csv.DictReader(f)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--mode", choices=("full", "pilot", "check", "pilot_check"), default="full")
    ap.add_argument("--check-how", choices=("away", "absent"), help="check modes: see the docstring")
    ap.add_argument("--only", nargs="*", help="(testing) run only these walk ids")
    ap.add_argument("--max-steps", type=int, help="(testing) cut every walk to this many steps")
    a = ap.parse_args()
    cfg = load_cfg(a.layout)
    walks = plan_walks(cfg)
    check = a.mode in ("check", "pilot_check")
    if check and not a.check_how:
        ap.error("--check-how is required with --mode check|pilot_check")
    if a.mode == "pilot":
        walks = pick_pilot(cfg, walks)
    elif a.mode == "check":
        walks = pick_check(cfg, walks, a.check_how)
    elif a.mode == "pilot_check":
        walks = [w for w in pick_check(cfg, walks, a.check_how)
                 if w["matches"] in {x["id"] for x in pick_pilot(cfg, plan_walks(cfg))}]
    if a.only:
        walks = [w for w in walks if w["id"] in a.only]
    global MAX_STEPS
    MAX_STEPS = a.max_steps
    os.makedirs(a.out, exist_ok=True)
    done = done_walks(a.out)
    todo = [w for w in walks if w["id"] not in done]
    print(f"[walks] {a.layout} {a.mode}: {len(walks)} walks, {len(done & {w['id'] for w in walks})} done, "
          f"{len(todo)} to run", flush=True)
    if not todo:
        return
    if check and a.check_how == "away":
        for i, l in enumerate(cfg["lookouts"]):
            gz.set_pose(cfg["world"], l["name"], FAR[0] + 10.0 * i, FAR[1], 0.2, 0.0)
    # An earlier session in this sim (the main walks, before the check walks)
    # left the world paused; a paused world publishes no clouds to wait for.
    gz.pause(cfg["world"], False)
    s = Session(cfg, a.out, a.mode, lookouts_present=not check)
    gz.pause(s.W, True)
    s.node.drain(0.5)
    s.settle()
    s.log(f"lookouts_{a.check_how}" if check else "lookouts_present", lookouts=[l["name"] for l in cfg["lookouts"]])
    for l in cfg["lookouts"] if not check else []:
        # at rest on its post: how far from the calculated point, and the static radio budget there
        o = s.node.odom[l["name"]].pose.pose.position
        L = radio.link((o.x, o.y, o.z), (cfg["mulcher"]["x"], cfg["mulcher"]["y"], 0.0), s.trees)
        s.log("at_post", robot=l["name"], x=round(o.x, 3), y=round(o.y, 3), z=round(o.z, 4),
              d_post_m=round(math.hypot(o.x - l["x"], o.y - l["y"]), 3), linked=int(L["linked"]),
              snr_db=round(L["snr_db"], 1), link_d_m=round(L["distance_m"], 2),
              trees_on_link=L["trees_on_link"])
    first_heading = True
    for h in cfg["walk"]["headings_deg"]:
        hw = [w for w in todo if w["heading"] == h]
        if not hw:
            continue
        s.set_heading(h)
        if first_heading:
            s.learn(s.names, f"h{h:03d}")        # lookouts once per session, mulcher for this heading
            first_heading = False
        else:
            s.learn(["mulcher"], f"h{h:03d}")
        for w in hw:
            if s.relearn:
                moved = sorted(s.relearn)
                s.log("relearn", lidars=moved, reason="lookout lidar moved beyond "
                      f"{MOVE_TOL_M} m / {MOVE_TOL_DEG} deg since its background")
                s.learn(moved, f"relearn_{w['id']}")
                s.relearn.clear()
            s.run_walk(w)
    s.log("session_done", retries=s.node.retries, svc_retries=len(gz.SVC_RETRIES))


if __name__ == "__main__":
    main()
