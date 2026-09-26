# Simulated exploitation experiments

Three simulated experiments (Ignition Gazebo, Fortress release) on whether
robots that explore a forest gain anything by then looking closely at chosen
places ("exploitation"). Result pages: <https://kalhansb.github.io/hmr_explo/>.

| folder | experiment | dates | outcome | page |
|---|---|---|---|---|
| [`exploitation_map_gain/`](exploitation_map_gain/) | Two Huskies explore; in half the runs they also visit three target oaks. Measure: lidar map completeness of the target trunks. 5 + 5 runs. | 21–22 Sep 2026 | **No result.** The pre-registered test had no usable runs (target visits finished after the 1500 s evaluation time); a later look at 2400 s found no gain (−0.017, p = 0.95). The measure had stopped changing before the visits. | [exploitation](https://kalhansb.github.io/hmr_explo/exploitation/) |
| [`camera/`](camera/) | Explore first, then visit three target oaks drawn at random from a pool of nine; the undrawn six are the comparison. Measure: whether a camera resolves small bark lesions. 6 runs. | 23 Sep 2026 | **Clear gain.** Of lesions still unresolved when exploration ended, the targets' were resolved far more often than the undrawn oaks' (difference +0.76, p = 1.6 × 10⁻⁷, pre-registered runs 1–4; +0.72 over all 6). | [camera](https://kalhansb.github.io/hmr_explo/camera/) |
| [`lookout/`](lookout/) | Robots parked as lookouts on the paths into a forestry mulching site, against the mulcher's own lidar. Measure: whether and how early an approaching pedestrian is detected. 120 trials on two layouts. | 23–24 Sep 2026 | **Clear gain.** With lookouts every pedestrian was detected before the lookout boundary (120 / 120, against 0 / 120), a median 22 s earlier on the same trial. A later check with a published moving-object detector (M-detector) gave the same answer (120 / 120 against 76 / 120, a median 23–27 s earlier); so did the same set-up on the trees of the real site ([`site_lookout/`](site_lookout/)). | [lookout](https://kalhansb.github.io/hmr_explo/lookout/) |

The first experiment's failure shaped the second: the camera experiment
separates exploration from visits in time, randomises the targets and uses a
measure that exploration alone rarely saturates.

Each folder has its own README with the set-up, results, limitations and
how to reproduce. Raw recordings (rosbags, camera frames, per-scan logs) are
large and stay untracked under `runs/`.

## Simulator settings

The lookout experiment changed the Husky lidar in `ws/src/hmr_sim` (minimum
range 0.05 → 0.5 m, maximum 25 → 100 m, range noise 1 → 2 cm) and added a
25 m software crop in `ws/src/explo_planner` so exploration still sees 25 m.
The exploitation and camera runs were made with the earlier settings
(`hmr_sim` 3576be1; the camera runs add their camera and world models from
`camera/overlay/`); each run's `run_manifest.txt` records the exact commits.
Check out that commit of `hmr_sim` to reproduce them exactly.
