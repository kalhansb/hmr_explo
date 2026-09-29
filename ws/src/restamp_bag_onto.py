#!/usr/bin/env python3
"""Shift a bag onto another bag's timeline so both can replay under ONE /clock.

    restamp_bag_onto.py SRC_BAG_DIR REF_BAG_DIR OUT_BAG_DIR

Offset = REF start - SRC start (integer ns, from each bag's metadata), applied to
  * every message's log (receive) time,
  * header.stamp of sensor_msgs/PointCloud2 and sensor_msgs/Imu,
  * header.stamp of every transform in tf2_msgs/TFMessage,
  * a FLOAT64 per-point field named 'timestamp' (Hesai clouds carry ABSOLUTE
    per-point times; leaving them unshifted would break deskew).
Any other topic type is refused rather than copied with a stale clock.

Why: two bags recorded at different times cannot share one sim /clock, and only
one player may publish /clock. With SRC re-stamped onto REF's timeline, play REF
with --clock and SRC without, started together (see run_explo_dual_native.sh).
"""
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage

FLOAT64 = 8


def start_ns(bag_dir):
    meta = yaml.safe_load((Path(bag_dir) / "metadata.yaml").read_text())
    return int(meta["rosbag2_bagfile_information"]["starting_time"]["nanoseconds_since_epoch"])


def shift_stamp(stamp, off_ns):
    ns = stamp.sec * 1_000_000_000 + stamp.nanosec + off_ns
    stamp.sec, stamp.nanosec = divmod(ns, 1_000_000_000)


def shift_cloud(msg, off_ns):
    shift_stamp(msg.header.stamp, off_ns)
    field = next((f for f in msg.fields if f.name == "timestamp"), None)
    if field is None:
        return
    if field.datatype != FLOAT64 or msg.is_bigendian:
        sys.exit(f"cloud field 'timestamp' is datatype {field.datatype} "
                 f"(bigendian={msg.is_bigendian}); only little-endian FLOAT64 is handled")
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step).copy()
    t = buf[:, field.offset:field.offset + 8].copy().view("<f8")
    t += off_ns * 1e-9
    buf[:, field.offset:field.offset + 8] = t.view(np.uint8)
    msg.data = buf.tobytes()


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    src, ref, out = sys.argv[1:]
    off_ns = start_ns(ref) - start_ns(src)
    print(f"offset {off_ns} ns ({off_ns * 1e-9:.3f} s)")
    if Path(out).exists():
        sys.exit(f"{out} exists; remove it first")

    reader = rosbag2_py.SequentialReader()
    # Open the .mcap itself: this Humble reader cannot parse the newer
    # metadata.yaml the bags were recorded with.
    src_mcap = str(next(Path(src).glob("*.mcap")))
    reader.open(rosbag2_py.StorageOptions(uri=src_mcap, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))

    handlers = {
        "sensor_msgs/msg/PointCloud2": (PointCloud2, shift_cloud),
        "sensor_msgs/msg/Imu": (Imu, lambda m, o: shift_stamp(m.header.stamp, o)),
        "tf2_msgs/msg/TFMessage": (TFMessage,
                                   lambda m, o: [shift_stamp(t.header.stamp, o) for t in m.transforms]),
    }
    types = {}
    for topic in reader.get_all_topics_and_types():
        if topic.type not in handlers:
            sys.exit(f"topic {topic.name} has unhandled type {topic.type}")
        types[topic.name] = topic.type
        writer.create_topic(topic)

    n = 0
    while reader.has_next():
        name, data, t_ns = reader.read_next()
        cls, fix = handlers[types[name]]
        msg = deserialize_message(data, cls)
        fix(msg, off_ns)
        writer.write(name, serialize_message(msg), t_ns + off_ns)
        n += 1
        if n % 10000 == 0:
            print(f"  {n} messages")
    del writer
    print(f"wrote {n} messages -> {out}")


if __name__ == "__main__":
    main()
