#!/usr/bin/env python3
"""Pass 2 over a site bag: lidar frames (sensor frame) and camera images in
chosen time windows, for M-detector with the robot standing still (identity
pose). Usage:
  bag_frames.py <bag dir> <lidar topic> <camera topic> <t0> <windows a:b,c:d (s after t0)> <out dir>
Writes <out>/seg_<a>.bin (md_offline input), seg_<a>_t.npy (stamps),
cam/<t>.jpg (one image per 0.5 s)."""
import glob, os, sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs_py import point_cloud2 as pc2

bag, ltopic, ctopic, t0, wins, out = sys.argv[1:7]
t0 = float(t0)
W = [tuple(map(float, w.split(':'))) for w in wins.split(',')]
os.makedirs(f'{out}/cam', exist_ok=True)
fh = {a: open(f'{out}/seg_{int(a):04d}.bin', 'wb') for a, _ in W}
stamps = {a: [] for a, _ in W}
last_cam = -1e9
I = np.eye(3)
files = sorted(glob.glob(f'{bag}/*.mcap'), key=lambda f: int(f.rsplit('_', 1)[1].split('.')[0]))
for f in files:
    r = SequentialReader(); r.open(StorageOptions(uri=f, storage_id='mcap'), ConverterOptions('', ''))
    r.set_filter(StorageFilter(topics=[ltopic, ctopic]))
    while r.has_next():
        top, d, ts = r.read_next()
        if top == ctopic:
            m = deserialize_message(d, CompressedImage)
            t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec - t0
            if any(a - 1 <= t <= b + 1 for a, b in W) and t - last_cam >= 0.5:
                open(f'{out}/cam/{t:08.2f}.jpg', 'wb').write(bytes(m.data)); last_cam = t
            continue
        m = deserialize_message(d, PointCloud2)
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec - t0
        for a, b in W:
            if a <= t <= b:
                p = pc2.read_points_numpy(m, field_names=['x', 'y', 'z'], skip_nans=True).astype(np.float32)
                p = p[np.linalg.norm(p, axis=1) > 0.3]
                h = fh[a]
                h.write(np.float64(t).tobytes()); h.write(I.astype(np.float64).tobytes())
                h.write(np.zeros(3).tobytes()); h.write(np.int32(len(p)).tobytes()); h.write(p.tobytes())
                stamps[a].append((t, len(p)))
    print(f, {a: len(v) for a, v in stamps.items()}, flush=True)
for a in fh:
    fh[a].close(); np.save(f'{out}/seg_{int(a):04d}_t.npy', np.array(stamps[a]))
