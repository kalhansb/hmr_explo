import sys, numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
uri, topic, out, every = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
r=SequentialReader(); r.open(StorageOptions(uri=uri,storage_id='mcap'),ConverterOptions('',''))
r.set_filter(StorageFilter(topics=[topic])); frames=[]; k=0
while r.has_next():
    t,d,ts=r.read_next(); k+=1
    if (k-1)%every: continue
    m=deserialize_message(d,PointCloud2)
    if not frames: print('fields',[(f.name,f.datatype) for f in m.fields], m.header.frame_id, m.width, m.height)
    p=pc2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=True)
    frames.append((ts, p.astype(np.float32)))
print(uri, 'frames read', k, 'kept', len(frames))
np.savez_compressed(out, ts=np.array([f[0] for f in frames]), **{f'f{i}':f[1] for i,f in enumerate(frames)})
