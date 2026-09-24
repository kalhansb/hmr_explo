#!/bin/bash
# After a camera cell: GT poses + virtual camera (container, --network none),
# then the rendered and virtual readouts (host). Usage: post_run.sh <run name>
# Optional second arg "zband": also campaign z-band M2 at the cue and end.
set -u
C=/home/kalhan/Documents/exploitation_experiments/runs/cam_experiment
P=/home/kalhan/Documents/exploitation_experiments
S=${CAM_SCRATCH:-$P/runs/cam_experiment/scratch}   # writable scratch; holds the Gazebo home (gzhome/)
name=$1
docker run --rm --network none --name "campost_$name" -v $C:/cam -v $P/ws/src/hmr_sim:/hs:ro hmrexplo:humble bash -lc "
  source /opt/ros/humble/setup.bash
  mkdir -p /tmp/bag && cp /cam/$name/rosbag2/metadata.yaml /cam/$name/rosbag2/*.zstd /tmp/bag/
  python3 /cam/gt_poses.py /cam/$name /tmp/bag --hz 10 &&
  python3 /cam/annotate_counts.py /cam/$name &&
  python3 /cam/virtual_cam.py /cam/$name /cam/lesions.json /hs/hmr_sim/worlds/flatforest/flatforestv2.sdf &&
  python3 /cam/virtual_cam.py /cam/$name /cam/lesions.json /hs/hmr_sim/worlds/flatforest/flatforestv2.sdf --frames /cam/$name/cam/frames.csv --out virtual_frames.csv
  chown $(id -u):$(id -g) /cam/$name/gt_poses.npz /cam/$name/gt_path_cum.json /cam/$name/virtual_cam.csv /cam/$name/virtual_frames.csv /cam/$name/cam/counts_geo.csv"
python3 $C/validate_virtual.py $C/$name > $C/$name/validate_virtual.txt
python3 $C/cam_score.py $C/$name --tag rendered > $C/$name/cam_score_rendered.txt
python3 $C/cam_score.py $C/$name --tag virtual --counts virtual_cam.csv > $C/$name/cam_score_virtual.txt
cat $C/$name/cam_score_rendered.txt
if [ "${2:-}" = "zband" ]; then
  cue=$(sed -n 's/^explore_done_cue_t_rel=//p' $C/$name/run_manifest.txt | cut -d. -f1)
  docker run --rm --network none --name "zbc_$name" --cap-add=NET_ADMIN --shm-size=2g \
    -e ARM="../cam_experiment/$name" -e HORIZONS="${cue:-300}" \
    -v $P/ws:/ws -v $P/runs:/runs -v $P/runs/pilot_fine/scoring:/sc -v $S/gzhome:/gzhome \
    hmrexplo:humble /sc/zband_pilot.sh > $C/$name/zband.log 2>&1
  echo "zband rc=$?"
fi
