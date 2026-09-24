"""Paused-world stepping with fresh scans (run inside the container).

The world stays paused; every change to the scene (person moved, mulcher
turned) is followed by `step_fresh`, which steps 5 physics steps (0.1 s, one
lidar period) and waits for a scan from every lidar that was rendered after
the change.

Freshness is decided per lidar against the newest stamp already received from
that lidar, never against /clock: rclpy delivers one callback per spin, so the
clock falls behind the clouds while stepping and a clock-based test passes
stale scans (seen in the smoke test). While paused, no scan can be rendered
between the change and the step, so the first scan stamped later than the
last one received before the step was rendered after the change -- provided
no older scan is still in flight when the step starts. So every step waits on
all subscribed lidars (never a subset), and a step only counts as done when
all lidars hold scans with the same stamp (one render): a straggler from an
earlier render shows up as a stamp mismatch and is waited out.

That proviso failed with ONE lidar (the check walks, 2026-09-24): with no
second lidar to disagree, a scan still in flight from the previous step was
taken as this step's, and once that happened every later step took the
previous step's render (L2 check walks: the person 0.5 m back in ~100 % of
steps; main walks with 3 lidars: 0 of ~11 000 lidar-steps; smoke/stale_audit.py).
So a scan must also be stamped after the sim time at which the step began,
which is tracked here: read once from the world's stats while paused, then
advanced by every step call (n physics steps of STEP_S each; nothing else
steps the world while a node is stepping it). A render from before the step
has a stamp <= that time. With several lidars the stamp test alone was enough
(no stale scan in the main walks), so there this changes nothing.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

import gz

# ~0.9 MB clouds are dropped under best-effort QoS; reliable, generous depth.
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
STEP_S = 0.02            # physics step of every lookout world (build_world.py max_step_size)


class ScanNode(Node):
    def __init__(self, world, lidars, odoms=(), name="lookout_scans"):
        """lidars: {key: PointCloud2 topic}; odoms: robot names whose
        /<r>/odom_ground_truth is kept (latest)."""
        super().__init__(name)
        self.world = world
        self.keys = list(lidars)
        self.last = {}
        self.n_msgs = 0
        self.odom = {}
        self.retries = 0
        self.t_sim = None        # sim time of the paused world (read at the first step_fresh)
        for k, t in lidars.items():
            self.create_subscription(PointCloud2, t, lambda m, k=k: self._cloud(k, m), RELIABLE)
        for r in odoms:
            self.create_subscription(Odometry, f"/{r}/odom_ground_truth",
                                     lambda m, r=r: self.odom.__setitem__(r, m), qos_profile_sensor_data)

    def _cloud(self, k, m):
        self.last[k] = m
        self.n_msgs += 1

    def spin_until(self, pred, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if pred():
                return True
        return False

    def wait_all(self, timeout=180.0, odoms=()):
        return self.spin_until(lambda: all(k in self.last for k in self.keys)
                               and all(r in self.odom for r in odoms), timeout)

    def drain(self, quiet=0.3, max_wait=10.0):
        """Spin until no cloud has arrived for `quiet` s (wall)."""
        t0 = time.time()
        n, tq = self.n_msgs, time.time()
        while time.time() - t0 < max_wait:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.n_msgs != n:
                n, tq = self.n_msgs, time.time()
            elif time.time() - tq >= quiet:
                return True
        return False

    def stamps(self, keys=None):
        return {k: gz.stamp_sec(self.last[k]) for k in (keys or self.keys) if k in self.last}

    def step_fresh(self, keys=None, n=5, timeout=90.0, retry=5.0):
        """Step the paused world `n` physics steps and return ({key: msg},
        info) with every lidar's first scan rendered after the call (`keys`
        only selects what is returned; all lidars are waited on). A scan lost
        in transport never comes back on a paused world, so after `retry` s of
        wall time the world is stepped again (the scene is unchanged)."""
        if self.t_sim is None:
            self.t_sim, paused = gz.sim_time(self.world)
            if not paused:
                raise RuntimeError(f"step_fresh on a running world ({self.world})")
        t0 = self.t_sim
        prev = self.stamps(self.keys)

        def done():
            if not all(k in self.last for k in self.keys):
                return False
            st = [gz.stamp_sec(self.last[k]) for k in self.keys]
            return (all(s > prev.get(k, -1.0) + 1e-6 and s > t0 + 1e-6 for k, s in zip(self.keys, st))
                    and max(st) - min(st) < 1e-6)

        w0 = time.time()
        gz.step(self.world, n)
        tries = 0
        while not self.spin_until(done, retry):
            if time.time() - w0 > timeout:
                raise RuntimeError(f"no common fresh scan after {timeout} s: sim {t0:.3f} prev {prev} "
                                   f"now {self.stamps()}")
            tries += 1
            self.retries += 1
            gz.step(self.world, n)
        if tries:
            # a retry can leave later renders in flight: take them in (they
            # show the same, unchanged scene), then insist on one common stamp
            self.drain(quiet=0.3)
            self.spin_until(done, retry)
        self.t_sim = t0 + (1 + tries) * n * STEP_S
        msgs = {k: self.last[k] for k in (keys or self.keys)}
        st = self.stamps(self.keys)
        if max(st.values()) > self.t_sim + 1e-6:
            raise RuntimeError(f"scan from the future: stamps {st}, sim {self.t_sim:.3f} (world stepped elsewhere?)")
        return msgs, {"stamps": st, "stamp_spread": max(st.values()) - min(st.values()),
                      "retries": tries, "wall_s": time.time() - w0, "t_sim0": t0}
