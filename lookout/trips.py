#!/usr/bin/env python3
"""Drive-out and messenger trips (in the container, sim time, world running).

  trips.py --layout L2 --out <dir> --trips 3

With the lookouts' planners (exploit_mode lookout, messenger on), their nav
stacks and radio_node.py running:

1. Drive-out: waits until every lookout reports /<r>/lookout/in_position.
   driveout.csv, per lookout: when it first moved, when it reported in
   position, path length driven (ground-truth odom), average speed, where it
   stopped and how far that is from its point, and where the link was last up
   on the way out (from /<r>/lookout/link_up).
2. Trips: one lookout at a time, N trips each. A trip is one alarm published
   on /<r>/lookout/alarm while the lookout is in position (a point 20 m out
   along its heading; the planner does not use the position). Measured from
   ground-truth odom and the sim clock:
     linked_at_alarm      link_up at the alarm
     t_link_s             alarm -> warning on /<r>/lookout/warning
     dist_out_m           path driven alarm -> warning
     speed_out_mps        dist_out_m / t_link_s
     link_x, link_y       where the warning went (link came back)
     link_d_mulcher_m, link_d_post_m
     t_back_s             warning -> in_position again (settled)
     dist_back_m, speed_back_mps
     unwatched_s          alarm -> in_position again (0 if it never left)
   trips.csv, one row per trip; a leg that exceeds --timeout is logged with
   timeout=1 and the trip is abandoned (and so is the lookout's remaining trips).
"""
import argparse
import csv
import json
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool

MOVED_M = 0.10          # first movement: this far from the spawn position
HOLD_BEFORE_S = 5.0     # in position this long before an alarm is sent
ALARM_AHEAD_M = 20.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Robot:
    def __init__(self):
        self.pos = None          # (x, y, yaw)
        self.spawn = None
        self.odo = 0.0           # path length since start (m)
        self.t_moved = None
        self.in_pos = False
        self.t_in_pos = None     # sim time of the latest in_position change
        self.link = None
        self.last_link = None    # (x, y, t) of the latest link_up=True sample
        self.warnings = []       # (t, x, y, odo)


class Trips(Node):
    def __init__(self, cfg):
        super().__init__("lookout_trips", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.cfg = cfg
        self.lk = {l["name"]: l for l in cfg["lookouts"]}
        self.r = {n: Robot() for n in self.lk}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.alarm_pub = {}
        for n in self.lk:
            self.create_subscription(Odometry, f"/{n}/odom_ground_truth", lambda m, n=n: self.on_odom(n, m),
                                     qos_profile_sensor_data)
            self.create_subscription(Bool, f"/{n}/lookout/in_position", lambda m, n=n: self.on_inpos(n, m), latched)
            self.create_subscription(Bool, f"/{n}/lookout/link_up", lambda m, n=n: self.on_link(n, m), latched)
            self.create_subscription(PointStamped, f"/{n}/lookout/warning", lambda m, n=n: self.on_warn(n, m),
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
            self.alarm_pub[n] = self.create_publisher(PointStamped, f"/{n}/lookout/alarm",
                                                      QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, n, m):
        r = self.r[n]
        p = m.pose.pose.position
        pos = (p.x, p.y, yaw_of(m.pose.pose.orientation))
        if r.pos is not None:
            r.odo += math.hypot(pos[0] - r.pos[0], pos[1] - r.pos[1])
        else:
            r.spawn = pos
        r.pos = pos
        if r.t_moved is None and math.hypot(pos[0] - r.spawn[0], pos[1] - r.spawn[1]) > MOVED_M:
            r.t_moved = self.now()

    def on_inpos(self, n, m):
        r = self.r[n]
        if bool(m.data) != r.in_pos:
            r.in_pos, r.t_in_pos = bool(m.data), self.now()

    def on_link(self, n, m):
        r = self.r[n]
        r.link = bool(m.data)
        if r.link and r.pos is not None:
            r.last_link = (r.pos[0], r.pos[1], self.now())

    def on_warn(self, n, m):
        r = self.r[n]
        r.warnings.append((self.now(), r.pos[0], r.pos[1], r.odo))

    def spin_until(self, cond, timeout_s):
        """Spin until cond() or `timeout_s` of sim time; True if cond held."""
        t0 = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
            t = self.now()
            if t <= 0:
                continue
            t0 = t if t0 is None else t0
            if t - t0 > timeout_s:
                return False
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--trips", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=1800.0, help="per leg, sim s")
    a, ros_args = ap.parse_known_args()
    cfg = yaml.safe_load(open(f"/lookout/config/{a.layout}.yaml"))
    mx, my = cfg["mulcher"]["x"], cfg["mulcher"]["y"]
    os.makedirs(a.out, exist_ok=True)
    ev = open(os.path.join(a.out, "trips_events.jsonl"), "a")
    rclpy.init(args=ros_args)
    node = Trips(cfg)

    def log(kind, **kw):
        ev.write(json.dumps({"t_wall": time.time(), "t_sim": node.now(), "kind": kind, **kw}, default=str) + "\n")
        ev.flush()
        print(f"[trips] {kind} {json.dumps(kw, default=str)[:300]}", flush=True)

    # 1. drive-out
    ok = node.spin_until(lambda: all(r.in_pos for r in node.r.values()), a.timeout)
    rows = []
    for n, r in node.r.items():
        l = node.lk[n]
        t_drive = None if r.t_in_pos is None or r.t_moved is None else r.t_in_pos - r.t_moved
        rows.append({
            "layout": a.layout, "robot": n, "in_position": int(r.in_pos),
            "t_moved": r.t_moved and round(r.t_moved, 2), "t_in_position": r.t_in_pos and round(r.t_in_pos, 2),
            "drive_s": t_drive and round(t_drive, 2), "path_m": round(r.odo, 2),
            "speed_mps": round(r.odo / t_drive, 3) if t_drive else None,
            "x": r.pos and round(r.pos[0], 3), "y": r.pos and round(r.pos[1], 3),
            "yaw_deg": r.pos and round(math.degrees(r.pos[2]), 2),
            "d_point_m": r.pos and round(math.hypot(r.pos[0] - l["x"], r.pos[1] - l["y"]), 3),
            "dyaw_deg": r.pos and round(math.degrees(math.remainder(r.pos[2] - l["yaw"], 2 * math.pi)), 2),
            "straight_m": round(math.hypot(l["x"] - l["spawn"]["x"], l["y"] - l["spawn"]["y"]), 2),
            "last_link_x": r.last_link and round(r.last_link[0], 2), "last_link_y": r.last_link and round(r.last_link[1], 2),
            "last_link_d_mulcher_m": r.last_link and round(math.hypot(r.last_link[0] - mx, r.last_link[1] - my), 2),
        })
    with open(os.path.join(a.out, "driveout.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    log("driveout", ok=ok, rows=rows)
    if not ok:
        log("abort", reason="not every lookout reached its point")
        raise SystemExit(2)

    # 2. trips
    fields = ["layout", "robot", "trip", "t_alarm", "linked_at_alarm", "t_link_s", "dist_out_m", "speed_out_mps",
              "link_x", "link_y", "link_d_mulcher_m", "link_d_post_m", "t_back_s", "dist_back_m", "speed_back_mps",
              "unwatched_s", "left_post", "timeout"]
    path = os.path.join(a.out, "trips.csv")
    new = not os.path.exists(path)
    ft = open(path, "a", newline="")
    wt = csv.DictWriter(ft, fieldnames=fields)
    if new:
        wt.writeheader()
    for n, r in node.r.items():
        l = node.lk[n]
        for k in range(a.trips):
            row = {"layout": a.layout, "robot": n, "trip": k, "timeout": 0}
            if not node.spin_until(lambda: r.in_pos and node.now() - r.t_in_pos >= HOLD_BEFORE_S, a.timeout):
                row["timeout"] = 1
                wt.writerow(row); ft.flush()
                log("trip_timeout", robot=n, trip=k, leg="hold")
                break
            nw, odo0, left = len(r.warnings), r.odo, False
            msg = PointStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.point.x = l["x"] + ALARM_AHEAD_M * math.cos(l["yaw"])
            msg.point.y = l["y"] + ALARM_AHEAD_M * math.sin(l["yaw"])
            t0 = node.now()
            row.update(t_alarm=round(t0, 2), linked_at_alarm=int(bool(r.link)))
            node.alarm_pub[n].publish(msg)
            log("alarm", robot=n, trip=k, link=r.link)
            if not node.spin_until(lambda: len(r.warnings) > nw, a.timeout):
                row["timeout"] = 1
                wt.writerow(row); ft.flush()
                log("trip_timeout", robot=n, trip=k, leg="out")
                break
            tw, wx, wy, odow = r.warnings[nw]
            row.update(t_link_s=round(tw - t0, 2), dist_out_m=round(odow - odo0, 2),
                       speed_out_mps=round((odow - odo0) / (tw - t0), 3) if tw > t0 and odow > odo0 else None,
                       link_x=round(wx, 2), link_y=round(wy, 2),
                       link_d_mulcher_m=round(math.hypot(wx - mx, wy - my), 2),
                       link_d_post_m=round(math.hypot(wx - l["x"], wy - l["y"]), 2))
            # back: in position again after the warning; it left if in_position dropped after the alarm
            node.spin_until(lambda: False, 1.0)
            left = (not r.in_pos) or (r.t_in_pos is not None and r.t_in_pos > t0)
            if left:
                if not node.spin_until(lambda: r.in_pos and r.t_in_pos > tw, a.timeout):
                    row["timeout"] = 1
                    wt.writerow(row); ft.flush()
                    log("trip_timeout", robot=n, trip=k, leg="back")
                    break
                tb = r.t_in_pos
                row.update(t_back_s=round(tb - tw, 2), dist_back_m=round(r.odo - odow, 2),
                           speed_back_mps=round((r.odo - odow) / (tb - tw), 3) if tb > tw else None,
                           unwatched_s=round(tb - t0, 2), left_post=1)
            else:
                row.update(t_back_s=0.0, dist_back_m=0.0, unwatched_s=0.0, left_post=0)
            wt.writerow(row); ft.flush()
            log("trip", **row)
    log("done")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
