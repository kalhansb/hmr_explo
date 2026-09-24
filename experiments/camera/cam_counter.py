#!/usr/bin/env python3
"""Online reader for the simulated NDVI camera (segmentation labels).

Per frame and robot it writes one row per lesion/bark label in view:
  <out>/counts.csv  t,robot,label,npix,x0,x1,y0,y1
and one row per frame with the class histogram:
  <out>/frames.csv  t,robot,bg,l1,l2,l4,l5,l6,lesion_px,bark_px,other_px
Best frame per (robot, tree) is kept as <out>/best_<robot>_<k>.npz (labels,
t), rewritten when that tree's pixel total improves by >= 20 % and >= 5 s
after the last write; a snapshot per robot every 60 s goes to
<out>/snap_<robot>_<t>.npz. The images themselves are not recorded (too big).

Labels (make_lesions.py): lesion 10-123 (tree k = (label-10)//6), bark 130-148.
Usage (started by the run script when CAM_COUNTER is set):
  cam_counter.py --out DIR --robots atlas,bestla --ros-args -p use_sim_time:=true
"""
import argparse, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

LES = (10, 123)
BARK = (130, 148)


class Counter(Node):
    def __init__(self, out, robots):
        super().__init__("cam_counter")
        self.out = out
        os.makedirs(out, exist_ok=True)
        self.fc = open(os.path.join(out, "counts.csv"), "a")
        self.ff = open(os.path.join(out, "frames.csv"), "a")
        if self.fc.tell() == 0:
            self.fc.write("t,robot,label,npix,x0,x1,y0,y1\n")
        if self.ff.tell() == 0:
            self.ff.write("t,robot,bg,l1,l2,l4,l5,l6,lesion_px,bark_px,other_px\n")
        self.best = {}          # (robot, k) -> (score, t_saved)
        self.last_snap = {}
        self.nframes = {r: 0 for r in robots}
        self.first = set()
        self.last_flush = time.time()
        for r in robots:
            self.create_subscription(Image, f"/{r}/segmentation/labels",
                                     lambda m, r=r: self.cb(r, m), qos_profile_sensor_data)
        self.create_timer(30.0, self.report)
        self.get_logger().info(f"listening on {robots} -> {out}")

    def labels(self, m):
        a = np.frombuffer(m.data, np.uint8)
        ch = {"mono8": 1, "8UC1": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(m.encoding)
        if ch is None:
            ch = max(1, m.step // m.width)
        a = a.reshape(m.height, m.step)[:, :m.width * ch].reshape(m.height, m.width, ch)
        return a, ch

    def cb(self, r, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        a, ch = self.labels(m)
        lab = a[:, :, 0]
        if r not in self.first:
            self.first.add(r)
            same = all(np.array_equal(a[:, :, 0], a[:, :, c]) for c in range(1, ch))
            u, n = np.unique(lab, return_counts=True)
            self.get_logger().info(
                f"first frame {r}: t={t:.2f} enc={m.encoding} {m.width}x{m.height} step={m.step} "
                f"channels_equal={same} labels={dict(zip(u.tolist(), n.tolist()))}")
        self.nframes[r] += 1
        h = np.bincount(lab.ravel(), minlength=256)
        les = h[LES[0]:LES[1] + 1].sum(); bark = h[BARK[0]:BARK[1] + 1].sum()
        other = h.sum() - h[[0, 1, 2, 4, 5, 6]].sum() - les - bark
        self.ff.write(f"{t:.3f},{r},{h[0]},{h[1]},{h[2]},{h[4]},{h[5]},{h[6]},{les},{bark},{other}\n")
        tree_px = {}
        for L in np.nonzero(h[LES[0]:BARK[1] + 1])[0] + LES[0]:
            if LES[1] < L < BARK[0]:
                continue
            ys, xs = np.nonzero(lab == L)
            self.fc.write(f"{t:.3f},{r},{L},{len(xs)},{xs.min()},{xs.max()},{ys.min()},{ys.max()}\n")
            k = (L - LES[0]) // 6 if L <= LES[1] else L - BARK[0]
            tree_px[k] = tree_px.get(k, 0) + len(xs)
        for k, s in tree_px.items():
            b, ts = self.best.get((r, k), (0, -1e9))
            if s >= 1.2 * b and t - ts >= 5.0 and s >= 200:
                np.savez_compressed(os.path.join(self.out, f"best_{r}_{k}.npz"), labels=lab, t=t, score=s)
                self.best[(r, k)] = (s, t)
        if t - self.last_snap.get(r, -1e9) >= 60.0:
            np.savez_compressed(os.path.join(self.out, f"snap_{r}_{int(t):05d}.npz"), labels=lab, t=t)
            self.last_snap[r] = t
        if time.time() - self.last_flush > 2.0:
            self.fc.flush(); self.ff.flush(); self.last_flush = time.time()

    def report(self):
        self.get_logger().info(f"frames so far {self.nframes}")
        self.fc.flush(); self.ff.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--robots", default="atlas,bestla")
    argv = sys.argv[1:]
    args = ap.parse_args(argv[:argv.index("--ros-args")] if "--ros-args" in argv else argv)
    rclpy.init(args=sys.argv)
    n = Counter(args.out, args.robots.split(","))
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.fc.flush(); n.ff.flush()


if __name__ == "__main__":
    main()
