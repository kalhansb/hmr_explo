#!/usr/bin/env python3
"""Lookout <-> mulcher radio link (in the container, sim time).

  radio_node.py --layout L2 --out <dir>

Runs radio.Link (the comms emulator's model, unchanged) at 5 Hz of sim time
between each lookout (ground-truth odom position) and the static mulcher
(its base at the layout origin) over the layout's trunk list. Publishes
/<r>/lookout/link_up (std_msgs/Bool) every sample and logs every sample to
<out>/radio.csv. Only the model's link state is emulated; no traffic is
relayed. A lookout whose odom is older than 2 s (the node's pose_timeout_s)
is reported down.
"""
import argparse
import csv
import os
import sys

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool

sys.path.insert(0, "/lookout")
import radio  # noqa: E402

POSE_TIMEOUT_S = 2.0


class RadioNode(Node):
    def __init__(self, cfg, out):
        super().__init__("lookout_radio", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.names = [l["name"] for l in cfg["lookouts"]]
        self.mulcher = (cfg["mulcher"]["x"], cfg["mulcher"]["y"], 0.0)
        self.trees = radio.load_trees(f"/runs/lookout/worlds/{cfg['forest']['csv']}")
        self.links = {n: radio.Link(seed=42 + i) for i, n in enumerate(self.names)}
        self.odom = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = {n: self.create_publisher(Bool, f"/{n}/lookout/link_up", latched) for n in self.names}
        for n in self.names:
            self.create_subscription(Odometry, f"/{n}/odom_ground_truth",
                                     lambda m, n=n: self.odom.__setitem__(n, m), qos_profile_sensor_data)
        self.f = open(os.path.join(out, "radio.csv"), "a", newline="")
        self.w = csv.writer(self.f)
        if self.f.tell() == 0:
            self.w.writerow(["t_sim", "robot", "x", "y", "z", "distance_m", "trees_on_link", "fade_db", "snr_db",
                             "bandwidth_mbps", "connected", "pose_age_s"])
        self.create_timer(1.0 / radio.LINK_RATE_HZ, self.tick)
        self.get_logger().info(f"radio: {self.names} <-> mulcher {self.mulcher}, {len(self.trees)} trunks")

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        for n in self.names:
            m = self.odom.get(n)
            if m is None:
                continue
            age = now - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            p = m.pose.pose.position
            if age > POSE_TIMEOUT_S:
                up, s = False, {"distance_m": "", "trees_on_link": "", "fade_db": "", "snr_db": "",
                                "bandwidth_mbps": 0.0}
            else:
                s = self.links[n].sample((p.x, p.y, p.z), self.mulcher, self.trees)
                up = s["connected"]
            self.pub[n].publish(Bool(data=bool(up)))
            self.w.writerow([f"{now:.2f}", n, f"{p.x:.3f}", f"{p.y:.3f}", f"{p.z:.3f}", s["distance_m"] and
                             f"{s['distance_m']:.2f}", s["trees_on_link"], s["fade_db"] and f"{s['fade_db']:.2f}",
                             s["snr_db"] and f"{s['snr_db']:.1f}", s["bandwidth_mbps"], int(up), f"{age:.2f}"])
        self.f.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--out", required=True)
    a, ros_args = ap.parse_known_args()
    cfg = yaml.safe_load(open(f"/lookout/config/{a.layout}.yaml"))
    os.makedirs(a.out, exist_ok=True)
    rclpy.init(args=ros_args)
    rclpy.spin(RadioNode(cfg, a.out))


if __name__ == "__main__":
    main()
