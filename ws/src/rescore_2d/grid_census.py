#!/usr/bin/env python3
"""Census of the 2D planning map over the ROI, as the candidate DONE criterion
would see it.

Subscribes to one or more `nav_msgs/OccupancyGrid` planning maps rebuilt
offline from a cell's bagged `scovox_bin` streams, and to a robot odometry for
the reachability seed. Writes one CSV row per grid message per topic.

The point of the exercise is a single question that the PLAN cannot answer and
the recorded data can: does the reachable-frontier count ever reach zero in
this world, or does it plateau above zero forever? A criterion that can never
fire is worse than the threshold it replaces, because its failure is silent.

Nothing here decides anything. It measures what the criterion WOULD have said,
on runs that have already happened.
"""
import csv
import math
import os
import sys
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# The campaign's ROI: explo_planner clips every candidate to this box, so it is
# the box the criterion must be computed over -- NOT the 150 m grid envelope,
# which is 2.25x the area and mostly margin.
ROI_HALF = float(os.environ.get("ROI_HALF", "50.0"))
# nav_msgs/OccupancyGrid: -1 unknown, 0..100 probability of occupancy.
# simple_nav_3d treats >= 50 as blocked (occupancy_grid_utils.hpp), and
# dscovox writes exactly 0 or 100, so the cut is only a formality here.
OCC_CUT = 50


def census(grid, seed_xy):
    """known / free / frontier / reachable-frontier over the ROI.

    `frontier` is a FREE cell with at least one unknown 8-neighbour: somewhere
    a robot could stand and see something new. The neighbour may lie outside
    the ROI -- unknown space just past the boundary is still a reason to go to
    the boundary -- so the neighbour lookup is over the whole grid and only the
    frontier cell itself is required to be in the ROI.

    `reachable` floods from the robot's own cell treating unknown as passable,
    which is what CostGrid does (cost_grid.hpp: unknown is traversable). That
    makes the reachable set an OVER-estimate of where the robot can really get,
    so the reachable-frontier count is an upper bound and the criterion errs
    towards firing LATE. That is the safe direction: a criterion that stops too
    early ends a mission with work left undone.
    """
    info = grid.info
    w, h, res = info.width, info.height, info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    g = np.asarray(grid.data, dtype=np.int16).reshape(h, w)

    unknown = g < 0
    free = (g >= 0) & (g < OCC_CUT)
    occ = g >= OCC_CUT

    # ROI mask on cell CENTRES, the same convention the metrics use.
    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h) + 0.5) * res
    in_x = (xs >= -ROI_HALF) & (xs <= ROI_HALF)
    in_y = (ys >= -ROI_HALF) & (ys <= ROI_HALF)
    roi = np.outer(in_y, in_x)

    n_roi = int(roi.sum())
    if n_roi == 0:
        return None
    n_unknown = int((unknown & roi).sum())
    n_free = int((free & roi).sum())
    n_occ = int((occ & roi).sum())

    # Unknown 8-neighbourhood by shifting the mask, padding with False so the
    # grid edge is not itself treated as a frontier.
    pad = np.zeros((h + 2, w + 2), dtype=bool)
    pad[1:-1, 1:-1] = unknown
    nbr_unknown = np.zeros((h, w), dtype=bool)
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            if dy == 1 and dx == 1:
                continue
            nbr_unknown |= pad[dy:dy + h, dx:dx + w]
    frontier = free & nbr_unknown
    n_front = int((frontier & roi).sum())

    n_rfront = -1
    n_reach = -1
    if seed_xy is not None:
        sx = int(math.floor((seed_xy[0] - ox) / res))
        sy = int(math.floor((seed_xy[1] - oy) / res))
        if 0 <= sx < w and 0 <= sy < h and not occ[sy, sx]:
            reach = np.zeros((h, w), dtype=bool)
            reach[sy, sx] = True
            q = deque([(sy, sx)])
            passable = ~occ
            while q:
                cy, cx = q.popleft()
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < h and 0 <= nx < w and passable[ny, nx] \
                                and not reach[ny, nx]:
                            reach[ny, nx] = True
                            q.append((ny, nx))
            n_reach = int((reach & roi).sum())
            n_rfront = int((frontier & reach & roi).sum())

    return {"roi_cells": n_roi, "unknown": n_unknown, "free": n_free,
            "occ": n_occ, "known_frac": (n_roi - n_unknown) / n_roi,
            "frontier": n_front, "reach_cells": n_reach,
            "reach_frontier": n_rfront}


class Census(Node):
    def __init__(self, topics, odom_topic, out_path):
        super().__init__("grid_census")
        self.seed = None
        self.rows = 0
        self.last = {}
        self.out_dir = os.path.dirname(out_path)
        self.fh = open(out_path, "w", newline="")
        self.csv = csv.writer(self.fh)
        self.csv.writerow(["t_sim", "topic", "roi_cells", "unknown", "free",
                           "occ", "known_frac", "frontier", "reach_cells",
                           "reach_frontier"])
        # The planning map is transient_local: a late subscriber still gets the
        # last grid. Match it or nothing is received at all.
        q = QoSProfile(depth=5)
        q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        for t in topics:
            self.create_subscription(
                OccupancyGrid, t, lambda m, tt=t: self.on_grid(m, tt), q)
        self.get_logger().info(f"census -> {out_path}; grids={topics}")

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.seed = (p.x, p.y)

    def on_grid(self, msg, topic):
        c = census(msg, self.seed)
        if c is None:
            return
        # Keep the latest raw grid per topic. The census is a handful of
        # integers; a criterion that fires for the wrong reason (a hole shaped
        # like the lidar's blind cone, a wall of inflation) still produces
        # plausible integers, so the final map has to be looked at.
        self.last[topic] = (msg, c, self.seed)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv.writerow([f"{t:.3f}", topic, c["roi_cells"], c["unknown"],
                           c["free"], c["occ"], f"{c['known_frac']:.6f}",
                           c["frontier"], c["reach_cells"], c["reach_frontier"]])
        self.fh.flush()
        self.rows += 1
        if self.rows % 20 == 0:
            self.get_logger().info(
                f"t={t:.0f} {topic.split('/')[1]} known={c['known_frac']:.4f} "
                f"front={c['frontier']} rfront={c['reach_frontier']}")


def dump_final_impl(self):
    """Write the last grid of each merger as .npy plus a viewable PGM.

    The PGM is written by hand rather than through an imaging library because
    the container has numpy and nothing else, and a criterion this cheap to
    check should not need a dependency to look at.
    """
    for topic, (msg, c, seed) in self.last.items():
        tag = topic.strip("/").split("/")[0]
        info = msg.info
        g = np.asarray(msg.data, dtype=np.int16).reshape(
            info.height, info.width)
        base = os.path.join(self.out_dir, f"final_{self.cell}_{tag}")
        np.save(base + ".npy", g)
        # Rows are flipped so the PGM reads y-up, like the world.
        img = np.full(g.shape, 128, dtype=np.uint8)   # unknown -> mid grey
        img[(g >= 0) & (g < OCC_CUT)] = 255           # free -> white
        img[g >= OCC_CUT] = 0                         # occupied -> black
        img = np.flipud(img)
        with open(base + ".pgm", "wb") as f:
            f.write(b"P5\n%d %d\n255\n" % (g.shape[1], g.shape[0]))
            f.write(img.tobytes())
        with open(base + ".txt", "w") as f:
            f.write(f"topic={topic}\nseed={seed}\nstamp={msg.header.stamp.sec}\n")
            for k, v in c.items():
                f.write(f"{k}={v}\n")
        print(f"final map -> {base}.pgm ({c['known_frac']:.4f} known, "
              f"frontier={c['frontier']}, reach_frontier={c['reach_frontier']})")


Census.dump_final = dump_final_impl


def main():
    topics = sys.argv[1].split(",")
    odom = sys.argv[2]
    out = sys.argv[3]
    rclpy.init()
    n = Census(topics, odom, out)
    n.cell = os.environ.get("CELL", "cell")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.dump_final()
        n.fh.close()


if __name__ == "__main__":
    main()
