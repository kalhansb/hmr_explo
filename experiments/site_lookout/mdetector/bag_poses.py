#!/usr/bin/env python3
"""Pass 1 over a site bag: the robot's pose over time (odom -> base frame,
from /tf) and the lidar clouds' stamps and frame, to find the periods where
the robot stood still. Usage: bag_poses.py <bag dir> <lidar topic> <out.npz>"""
import glob, sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2

bag, topic, out = sys.argv[1:4]
files = sorted(glob.glob(f'{bag}/*.mcap'), key=lambda f: int(f.rsplit('_', 1)[1].split('.')[0]))
odom, cl, static, frame = [], [], [], None
for f in files:
    r = SequentialReader(); r.open(StorageOptions(uri=f, storage_id='mcap'), ConverterOptions('', ''))
    r.set_filter(StorageFilter(topics=['/tf', '/tf_static', topic]))
    while r.has_next():
        t, d, ts = r.read_next()
        if t == topic:
            # header only: deserialising the whole cloud is slow but needed for the stamp
            m = deserialize_message(d, PointCloud2)
            cl.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec); frame = m.header.frame_id
            continue
        for x in deserialize_message(d, TFMessage).transforms:
            tr, q = x.transform.translation, x.transform.rotation
            row = (x.header.stamp.sec + 1e-9 * x.header.stamp.nanosec, tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)
            if t == '/tf_static':
                static.append((x.header.frame_id, x.child_frame_id) + row[1:])
            elif x.child_frame_id.startswith('base_link') and x.header.frame_id.startswith('odom'):
                odom.append(row)
    print(f, len(odom), len(cl), flush=True)
np.savez(out, odom=np.array(odom), cloud_t=np.array(cl), frame=frame,
         static=np.array(static, dtype=object), allow_pickle=True)
