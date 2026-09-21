#!/usr/bin/env python3
"""Offline oracle-merge scorer for one experiment cell (doc §5.1-§5.3).

Replays a cell's bagged `scovox_bin` through a FRESH `dscovox_mapping_node` and
reads the fused map back over `GetRegion` (§5.1).

COMMS=0 is PERFECT comms -- one broadcast domain, not a severed link -- so each
robot's own merger already held both robots' bins and the team-observed and
robot-known maps coincide up to the merger's publish latency. The oracle is
still built offline rather than read off a runtime map, for two reasons that
survive that: it is one map scored by one rule at every horizon, free of
whatever each merger happened to have integrated at the instant a horizon fell,
and it is reachable by `GetRegion` boxes instead of the `~/scovox` full-map dump
(tens of MB per message at 0.20 m over this world, which a replay at speed drops).

Run inside the container (needs the ROS graph and scovox_msgs):
    ros2 run --prefix 'python3' ... or simply:
    python3 /ws/src/score_cell.py --cell /runs/off_rep1 --out /runs/off_rep1/score.json
"""
import argparse, json, math, os, re, subprocess, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trunk_score_core as core

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from scovox_msgs.srv import GetRegion
from rosbag2_interfaces.srv import Pause, Resume

HORIZONS = [600, 900, 1200, 1500, 1800, 2400]   # §5.3; "end" is appended live
PLAY_RATE = 5.0
SETTLE_S = 3.0          # wall seconds after a pause before querying the oracle
M5_RADIUS = 5.0         # §5.2 presence covariate
ROBOTS = ("atlas", "bestla")
ROI_Z = (-5.5, 4.0)     # §5.1 gate O1 uses the planner's own z clip


# --- inputs off the cell directory -----------------------------------------
def find_bag(cell):
    for root, _dirs, files in os.walk(cell):
        if any(f.endswith((".db3", ".db3.zstd", ".mcap")) for f in files):
            return root
    sys.exit(f"no rosbag2 under {cell}")


def read_t0(cell, override=None):
    """T0 in SIM seconds.

    The runner prints `sim t0=<n>` to stdout and does NOT put it in
    run_manifest.txt, so the authoritative record is the per-cell console log
    the campaign driver captures beside the cell dir. There is no reconstructing
    it from the manifest: `run_end_wall_sec_since_t0` is wall, not sim. The
    scheduler's "origin latched at t=" line is close but not equal (the runner
    sleeps ~8 s between the latch and reading T0), so it is not used as a
    silent fallback -- an 8 s slip would quietly mis-register every horizon.
    Cells without a console log must pass --t0.
    """
    if override is not None:
        return float(override), "--t0"
    name = os.path.basename(os.path.normpath(cell))
    for cand in (os.path.join(cell, "sim.log"),
                 os.path.join(os.path.dirname(os.path.normpath(cell)), name + ".console.log")):
        if not os.path.exists(cand):
            continue
        m = re.search(r"sim t0=(\d+(?:\.\d+)?)", open(cand, errors="replace").read())
        if m:
            return float(m.group(1)), cand
    sys.exit("could not determine sim t0 (no `sim t0=` line found); pass --t0")


# --- the oracle ------------------------------------------------------------
class Oracle(Node):
    def __init__(self, centres):
        super().__init__("cell_scorer")
        self.sim = 0.0
        self.centres = centres
        # M5 accumulates off THIS replay rather than a separate offline read of
        # the bag: Humble's plain SequentialReader does not decompress, and the
        # cells are recorded zstd, so an offline pass would mean inflating ~4 GB
        # per cell onto a disk that is already near full. Playing every topic
        # and listening costs one small subscription instead.
        self.presence = {n: 0.0 for n in centres}
        self._near = {}
        self._prev_t = None
        self.create_subscription(
            Clock, "/clock", self._on_clock,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        for r in ROBOTS:
            self.create_subscription(
                Odometry, f"/{r}/odom_ground_truth",
                (lambda rb: lambda m: self._on_odom(rb, m))(r),
                QoSProfile(depth=2000, reliability=QoSReliabilityPolicy.RELIABLE))
        self.region = self.create_client(GetRegion, "/dscovox_node/get_region")
        self.pause = self.create_client(Pause, "/rosbag2_player/pause")
        self.resume = self.create_client(Resume, "/rosbag2_player/resume")

    def _on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_odom(self, robot, msg):
        """§5.2 M5: seconds ANY robot was within 5.0 m of the trunk. That is a
        union over time, not a sum over robots -- with both robots parked at one
        trunk, which is exactly the rendezvous this experiment creates, a
        per-robot sum would report 2x elapsed and inflate the covariate
        precisely on the treated trunks. One global clock, each interval
        credited once, to the union of the robots' near-sets."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._prev_t is not None:
            dt = t - self._prev_t
            if 0.0 < dt <= 1.0:
                for n in set().union(*self._near.values()) if self._near else ():
                    self.presence[n] += dt
        self._prev_t = t
        p = msg.pose.pose.position
        self._near[robot] = {n for n, (cx, cy, _cz) in self.centres.items()
                             if math.hypot(p.x - cx, p.y - cy) <= M5_RADIUS}

    def call(self, client, req, timeout=60.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            raise RuntimeError("service call timed out")
        return fut.result()

    def spin_until(self, pred, timeout):
        t_end = time.time() + timeout
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if pred():
                return True
        return False

    def get_box(self, lo, hi):
        rq = GetRegion.Request()
        rq.min_corner = Point(x=float(lo[0]), y=float(lo[1]), z=float(lo[2]))
        rq.max_corner = Point(x=float(hi[0]), y=float(hi[1]), z=float(hi[2]))
        rs = self.call(self.region, rq)
        # GetRegion reports each voxel's MIN CORNER (dscovox_node.cpp: position =
        # coordToPos(c)), but every §5.2 metric is defined on voxel CENTRES: the
        # cylinder test and, worse, the per-column bearing used by M1. Half a
        # voxel is 0.10 m against a 2.0 m radius, and the bias is systematic --
        # it pushes every column the same way, so it does not average out.
        h = core.VOX / 2.0
        return [(v.position.x + h, v.position.y + h, v.position.z + h,
                 v.a_occ, v.a_free) for v in rs.map.voxels]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cell", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--sdf", default=core.DEFAULT_SDF)
    ap.add_argument("--roi", default=None, help="xmin,ymin,xmax,ymax for gate O1")
    ap.add_argument("--rate", type=float, default=PLAY_RATE)
    ap.add_argument("--t0", type=float, default=None,
                    help="sim T0 override when no console log was captured")
    args = ap.parse_args()
    cell = os.path.abspath(args.cell)
    out = args.out or os.path.join(cell, "score.json")

    centres = core.parse_oaks(args.sdf)
    t0, t0_src = read_t0(cell, args.t0)
    bagdir = find_bag(cell)
    print(f"[scorer] cell={cell}\n[scorer] bag={bagdir}\n[scorer] t0={t0} (from {t0_src})"
          f"\n[scorer] {len(centres)} oak models", flush=True)

    horizons = list(HORIZONS) + [("end", None)]

    bins = [f"/{r}/scovox_node/scovox_bin" for r in ("atlas", "bestla")]
    env = dict(os.environ)
    oracle_proc = subprocess.Popen(
        ["ros2", "run", "scovox_mapping", "dscovox_mapping_node", "--ros-args",
         "-p", "input_topics:=[" + ",".join(bins) + "]",
         "-p", "publish_rate_hz:=0.0"],
        stdout=open(os.path.join(cell, "oracle.log"), "w"), stderr=subprocess.STDOUT, env=env)
    player = None
    rclpy.init()
    node = Oracle(centres)
    try:
        if not node.region.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("dscovox_node/get_region never appeared")
        print("[scorer] oracle up", flush=True)
        player = subprocess.Popen(
            # Every topic is played, not just the bins: odom_ground_truth feeds
            # the M5 covariate in this same pass (see Oracle._on_odom).
            #
            # NO --clock. The bag already CONTAINS /clock, and those messages
            # carry the simulation time the horizons are defined in. --clock
            # would make the player publish a second /clock derived from the
            # bag's record timestamps, which are wall-epoch -- both land on the
            # same topic, epoch wins, and every horizon fires at once against an
            # empty map. Replaying the recorded /clock is the sim clock.
            ["ros2", "bag", "play", bagdir, "--start-paused",
             "--rate", str(args.rate)],
            stdout=open(os.path.join(cell, "player.log"), "w"), stderr=subprocess.STDOUT, env=env)
        if not node.pause.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("rosbag2 player services never appeared")
        node.resume.wait_for_service(timeout_sec=10.0)

        results = {}
        for h in horizons:
            label, target = ("end", None) if isinstance(h, tuple) else (str(h), t0 + h)
            node.call(node.resume, Resume.Request())
            if target is None:
                node.spin_until(lambda: player.poll() is not None, timeout=7200.0)
            else:
                ok = node.spin_until(lambda: node.sim >= target or player.poll() is not None,
                                     timeout=7200.0)
                if not ok:
                    raise RuntimeError(f"never reached horizon {label}")
                if player.poll() is not None:
                    # The bag ran out before this horizon (a short or truncated
                    # cell). There is no more data, so this snapshot IS the final
                    # map; record it as "end" and stop rather than emitting
                    # identical rows for every remaining horizon.
                    label, target = "end", None
                else:
                    node.call(node.pause, Pause.Request())
            # Let the oracle drain the bins the player already put on the wire.
            node.spin_until(lambda: False, timeout=SETTLE_S)
            print(f"[scorer] horizon {label}: t_sim={node.sim:.1f}, querying", flush=True)

            per_trunk = {}
            for name, (cx, cy, cz) in centres.items():
                vox = node.get_box((cx - core.CYL_R, cy - core.CYL_R, cz + core.Z_LO),
                                   (cx + core.CYL_R, cy + core.CYL_R, cz + core.Z_HI))
                s = core.score_trunk(vox, (cx, cy, cz), lattice_off=core.VOX / 2.0)
                s["M5"] = node.presence.get(name, 0.0)
                s["target_id"] = core.TARGETS.get(name)
                s["excluded_control"] = name in core.EXCLUDE_CONTROL
                per_trunk[name] = s
            entry = {"t_rel": node.sim - t0, "t_sim": node.sim, "trunks": per_trunk}
            if args.roi:
                x0, y0, x1, y1 = (float(v) for v in args.roi.split(","))
                roi = node.get_box((x0, y0, ROI_Z[0]), (x1, y1, ROI_Z[1]))
                entry["roi_observed_voxels"] = len(roi)
            results[label] = entry
            print(f"[scorer]   targets M1: " + ", ".join(
                f"{core.TARGETS[n]}={per_trunk[n]['M1']:.3f}" for n in core.TARGETS
                if n in per_trunk), flush=True)
            if target is None:
                break

        payload = {"cell": os.path.basename(cell), "t0": t0, "end_sim": node.sim,
                   "sdf": args.sdf, "horizons": results}
        with open(out, "w") as f:
            json.dump(payload, f, indent=1)
        print(f"[scorer] wrote {out}", flush=True)
    finally:
        for p in (player, oracle_proc):
            if p and p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    p.kill()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
