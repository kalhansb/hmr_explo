# M-detector check

Are the lookout results an artefact of our own detection rules? A check made
after the results (25–26 September 2026, exploratory): every simulated walk of
the lookout experiment (layouts L2 and L3) and of the site experiment (the
oak-model world S1 and the map-point world S1G) was run again with a
published moving-object detector, M-detector (Wu et al., "Moving event
detection from LiDAR point streams", *Nature Communications* 15, 345, 2024;
[hku-mars/M-detector](https://github.com/hku-mars/M-detector), commit
0645dab), as every lidar's detector.

## Answer

The result holds. With M-detector the mulcher alone detects walkers further
out than under the specified rule, about as far as with horizontal
clustering, but mostly only around or after the lookout boundary. With the
lookouts every walker was detected before the boundary, and the first alarm
came a median 23–29 s earlier on the same walk.

| | L2 · 48 walks | L3 · 72 walks | S1 oak models · 48 | S1G map points · 48 |
|---|---|---|---|---|
| detected before the boundary: mulcher alone / with lookouts | 34 / 48 · 48 / 48 | 42 / 72 · 72 / 72 | 18 / 48 · 48 / 48 | 16 / 48 · 48 / 48 |
| median warning: mulcher alone / with lookouts | 2.3 s / 28.8 s | 3.5 s / 26.9 s | −1.7 s / 24.6 s | −4.0 s / 25.0 s |
| first alarm earlier with lookouts, same walk (median; least) | 26.5 s; 0.8 s | 23.1 s; 0.0 s | 25.6 s; 18.1 s | 28.7 s; 20.4 s |
| distance at first alarm (median): mulcher alone / with lookouts | 27.2 m / 59.9 m | 32.1 m / 62.1 m | 24.7 m / 54.1 m | 21.5 m / 55.7 m |
| never detected by the mulcher alone | 2 | 15 | 4 | 4 |
| exact McNemar test, detection before the boundary | p = 1.2 × 10⁻⁴ | p = 1.9 × 10⁻⁹ | p = 1.9 × 10⁻⁹ | p = 4.7 × 10⁻¹⁰ |
| false alarms, every lidar | 0 | 0 | 0 | 0 |

- **Compared with the rules used so far.** Under the specified rule the
  mulcher alone detected no walker before the boundary (0 / 120 in L2 and L3,
  0 / 48 in S1); with
  M-detector it detects 33–71 % of walkers before the boundary across the
  four worlds. Its first detections came at about the distance of the
  horizontal-clustering rule, and 2–3 m short of the ideal detector's.
- **The mulcher's misses** are the walks that came in near its front, where
  the cutting head blocks its lidar, as before.
- **Four L3 walks** are the exception: the mulcher raised the first alarm
  itself, from a few returns on the walker 62–66 m out on open ground.
- **Real data.** On a stretch of the Curt robot's July recording at the
  site, with the robot standing still, M-detector followed a pedestrian
  (confirmed on the camera), but it also produced many short false tracks
  (canopy, ground; 145 tracks in all). The simulation has no wind, no pose
  error and clean scans, so its zero false alarms say little about the
  field; a filter set on real data would be needed before M-detector raises
  alarms there. This was not pursued further.

## Method

- **Scans.** The walks were simulated again exactly as before (same code,
  worlds, seeds and lidar), with every lidar's whole scan saved per walk step
  (`walks.py`, `LOOKOUT_FULLSCANS=1`). Lidar noise is random, so detections
  under the old rules can differ by a step from the original runs.
- **M-detector** (`Dockerfile`, ROS 1 Noetic) is unchanged except its build
  file (`patch_build.py`); `md_offline.cpp` feeds it the scans and exact
  lidar poses from a file, one frame per walk step at walk time (0.3846 s
  apart), and writes its per-point moving labels.
- **Parameters** (`md_vlp16.yaml`): the upstream nuScenes set (HDL-32E, the
  nearest lidar it was tuned for), changed only for the VLP-16 geometry and
  the walk timing. Nothing was tuned on the walks.
- **Alarm rule** (`md_sim.py`, fixed before any M-detector output was seen):
  the points M-detector labels moving after its own clustering are grouped
  with 0.5 m linkage in the horizontal plane; a group of at least 3 points
  that reappears within 1 m in the next step is an alarm; an alarm within
  1 m of the walker is a hit, any other a false alarm.
- **Comparison** (`md_team.py`): as in the lookout experiment, per walk the
  mulcher's first hit against the first hit by any lidar; the boundary is
  each entry's post distance.

## Files

| file | what |
|---|---|
| `Dockerfile`, `patch_build.py`, `md_offline.cpp` | the `mdetector:noetic` image and the offline driver |
| `md_vlp16.yaml`, `md_os128.yaml` | parameters for the simulated VLP-16 and for the Curt robot's Ouster OS1-128 |
| `md_sim.py`, `md_team.py` | M-detector on the saved walks; mulcher alone vs with lookouts |
| `run_md_lookout.sh`, `../run_md_sims.sh` | the runs for L2/L3 and for S1/S1G |
| `bag_poses.py`, `bag_frames.py`, `md_bag.py` | the real-data test on the site recording |

## Reproduce

```bash
# from the repository root; needs ../lookout_experiment (its ws build and runs/lookout)
docker build -t mdetector:noetic experiments/site_lookout/mdetector
experiments/site_lookout/mdetector/run_md_lookout.sh L2 L3     # output runs/lookout_md/
experiments/site_lookout/run_md_sims.sh                         # S1 and S1G walks; output runs/site_lookout/*_md
# then M-detector and the comparison on S1 / S1G, as run_md_lookout.sh does for L2 / L3
```
