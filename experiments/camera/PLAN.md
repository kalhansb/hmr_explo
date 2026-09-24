# Inspection-camera ("NDVI camera") experiment: plan, fixed before any data

Written 2026-09-23, before the first camera run. Anything changed after data
exists is logged under "Deviations" at the bottom with the reason.

## Question

Does exploitation (driving to a target trunk and dwelling at three vantages
facing it) improve what a close-range imaging payload gets from that trunk,
beyond what the same extra driving gives trees that were not targeted?

The lidar map (0.20 m voxels) saturates once a trunk has been seen from 10-20 m,
which is why well-seen trees gain almost nothing on M2. An image keeps
improving with range (pixels on target ~ 1/d^2), and exploitation vantages face
the trunk (yaw = theta + pi, 8 s dwell), so a camera is the sensor that
exploitation was designed for ("exploitation vantages still require yaw: that
capture IS directional", shared_params.yaml).

## Sensor: simulated NDVI camera

Gazebo Fortress has no NDVI sensor. The camera is a **segmentation camera**
(`segmentation_camera`, semantic mode, labels via the label system) whose
labels are turned into an NDVI-like image by a lookup. The label, not a
rendered colour, is what makes the measurement exact: every pixel says which
surface it sees, occlusion included, because it is rendered.

- Optics: MicaSense RedEdge-MX field of view (HFOV 47.2 deg = 0.8238 rad), rendered at
  half resolution, 640 x 480 (fx = fy = 732.5 px), to keep llvmpipe cost down.
- 2 Hz, clip 0.1-40 m, level (pitch 0), forward-facing, on base_link at
  x +0.30, z +0.55 (about 0.68 m above ground, just under the lidar).
- Both robots carry one. Nothing in the planner knows about it; the planner is
  unchanged.

## Targets on the trunks: "lesions"

The quantity to image is small patches on the trunk: flat 3 mm plates, visual
only (no collision), each with its own label.

- The 19 usable in-ROI oaks (all_oaks_trees.txt), 6 lesions each = 114.
- Sizes: 2 x 4 cm, 2 x 8 cm, 2 x 16 cm squares per tree (real bark
  lesions/cankers span roughly this range).
- Height uniform 0.65-1.25 m (the clean-trunk band; below it is root flare,
  above it canopy). Azimuth stratified: one per 60 deg sector, uniform jitter
  inside it, sizes shuffled over sectors. Seed 2026, same layout in every run.
- Each plate sits 2 mm outside the largest bark radius under its footprint
  (from the oak mesh), facing radially out from the trunk axis.
- Labels: lesion = 10 + 6k + j (k = tree index 0..18, j = 0..5) -> 10..123.
  The bark of those 19 oaks gets label 130 + k. Everything else keeps its
  world label (other oaks and all branches 2, robot 6).
- NDVI lookup for display only: leaves 0.80, bark 0.25, lesion 0.05,
  ground 0.30, other 0.10.

Resolution thresholds, at the rendered 640 x 480 optics, fixed now:
a lesion is **resolved** at time t if some frame up to t shows it with
>= 25 pixels (5 x 5, enough for an index estimate from interior pixels).
Sensitivity: 9 px (detected) and 100 px (fine).
Approximate frontal ranges for 25 px: 4 cm <= 5.9 m, 8 cm <= 11.7 m,
16 cm <= 23 m.

## Design: explore-then-exploit, randomised targets

Same design as seq_clean: RELEASE_ON_DONE=1, so every planner first finishes
exploration (the cue), then the three targets are released together. One run
is its own control; readouts are at the cue and at the end.

- Clean (no pose/lidar noise), ROI +-25, DONE_UNKNOWN 0.10, streak criterion,
  COMMS 0, FINE_BAND 0 (campaign default; the fine band is not what is measured).
- Target pool (fixed now): oaks whose whole 3.8 m vantage ring lies inside
  the ROI with 1 m margin, nearest other tree >= 8 m, >= 10 m from spawn:
  **4, 18, 36, 39, 40, 42, 58, 69, 73** (Oak 19 is out: its ring sits on the
  spawn point).
- Replicate k draws its 3 targets with `random.Random(k).sample(pool, 3)`:

  | rep | targets (oak #) |
  |---|---|
  | 1 | 18, 36, 73 |
  | 2 | 4, 18, 73 |
  | 3 | 36, 39, 69 |
  | 4 | 4, 39, 40 |
  | 5 | 40, 42, 58 |
  | 6 | 18, 58, 69 |

  Run reps 1-4; 5-6 only if time allows. The run order is the rep order.

## Metrics (fixed now)

Per lesion: best single-frame pixel count up to t (either robot).

1. **Primary: conditional gain.** Of the lesions NOT resolved at the cue,
   the fraction resolved by the end. Computed separately for lesions on
   targets and on controls; the contrast is target minus control. It is
   ceiling-proof: a tree already fully resolved at the cue does not count.
   - Primary control set: the pool oaks that were not drawn (6 per run),
     because they share the eligibility geometry (interior, isolated).
   - Secondary control set: all 16 non-targets.
2. Resolved fraction at cue and end, by lesion size (4/8/16 cm), per group.
3. Best pixel count per lesion (log10), change cue -> end; implied index error
   sigma_px / sqrt(n) with sigma_px = 0.05 NDVI.
4. Best bark pixel count per tree (whole-trunk resolution).
5. Cost: seconds from cue to end, GT path metres after the cue.
6. Continuity: campaign z-band M2 per tree at cue and end (scovox_bin replay).

Statistics: n = runs. Report every run's contrast, not only the mean. Inference
is a randomisation test that mirrors the assignment: within each run, re-draw
the 3 "targets" from the 9-oak pool (all 84 sets), recompute the pooled
statistic, and compare. Valid under the sharp null (targeting changes no tree's
outcome) because the targets were drawn at random from that pool.

## Phase A: virtual camera on existing bags (no new sim)

A geometric model of the same camera on the GT odometry of runs we already
have: lesion corners projected through the pinhole, clipped to the image;
back-facing lesions dropped; occlusion from other trunks (vertical cylinders)
and the maize/stone objects. Two uses:
- Validate it against the rendered counts from the camera runs (per-lesion
  best-pixel agreement). Only if it agrees (median ratio within 0.8-1.25 and
  resolved/not-resolved agreement >= 90 %) is it used further.
- Then score the 5 campaign on/off pairs (on_rep1-5, off_rep1-5, ROI +-50,
  3600 s) and the pilot runs with it, for a second, independent n.
- Phase A statistic (fixed 2026-09-23 06:35, before any campaign cell was
  scored). The campaign targets are fixed (oaks 42, 4, 39), not randomised, so
  the test is the pairing. Per rep and horizon h: DiD = (T_on - T_off) -
  (Cp_on - Cp_off), using resolved fraction at 25 px (secondary: C in place of
  Cp; 9 and 100 px). The primary horizon is end (3600 s); the secondary is
  1200 s, 480 s after the last release. Report the mean DiD over the 5 reps and
  every rep's value, with an exact sign-flip p over the 2^5 = 32 assignments.

## Decision rules for running unattended

- Smoke test first (short run): labels arrive on ROS, label values are right,
  lesions render, RTF measured. If the segmentation camera cannot be made to
  render, fall back to Phase A only and say so.
- If RTF with the camera is below 0.25 (a run over ~2 h wall), drop to
  480 x 360 and scale the pixel thresholds by (480/640)^2.
- A run that crashes or never reaches the cue in 2400 s sim: re-run once with
  the same seed, and record both.
- One sim at a time (CPU). Scorers on `--network none`.
- Stop launching if free disk < 20 GB. Never delete run outputs.

## Layout

- `make_lesions.py` -> `lesions.json`, `overlay/flatforestv2_cam.sdf`
- `overlay/husky_cam.sdf` (lidar Husky + camera), `overlay/scenario_cam.yaml`
- `cam_counter.py`: online node, per-frame per-label pixel counts ->
  `<run>/cam/counts.csv`, best frames `<run>/cam/best_*.npz`
- `targets/targets_rep<k>.yaml`, `run_cam_cell.sh <name> <rep>`
- `cam_score.py` (per-run readout), `cam_pool.py` (all runs + randomisation test)
- `virtual_cam.py` (Phase A)
- Run outputs: `runs/cam_experiment/<name>/`

## Status log

(append as things happen; run_queue.sh appends per-rep lines to PLAN.status)

- 2026-09-23 05:17 smoke1 (rep1 targets, 300 s): the camera runs at about 2 Hz per
  robot and the labels come out as designed. RTF is about 0.48, the same as without
  the camera. The run ended censored at 306 s with no cue, as expected in 300 s.
- 2026-09-23 06:10 Virtual camera validated against smoke1 renders: ACCEPT
  (see Deviation 1 for the front-view filter).
- 2026-09-23 06:22 run_queue.sh 1 2 3 4 started (2400 s cap per rep).
- 2026-09-23 06:55 Phase A done (phaseA.sh, phaseA_report.py, phaseA_report_thr{9,25,100}.txt).
  Virtual camera on the 5 campaign pairs, DiD vs Cp at 25 px:
  - end (primary): +0.16, 4/5 > 0, sign-flip p 0.062 (1/32 = 0.031 is the floor at n = 5).
  - 1200 s (secondary): +0.32, 5/5, p 0.031.
  - vs all non-targets at end: +0.17, 5/5, p 0.031.
  At 100 px the end DiD is +0.25 (5/5, p 0.031); at 9 px it is +0.08 (4/5, p 0.062).
  The off arm's targets catch up over time (T_off 0.72-0.94 at end); the close-up
  (100 px) gap persists.
- 2026-09-23 07:25 rep1 (targets oaks 18, 36, 73): cue 324 s, all_done at 1329 s.
  Rendered, 25 px conditional gain: T 6/6 = 1.00 vs Cp 3/22 = 0.14, D = +0.86. The
  target set ranks 1st of 84 (p = 0.012, the floor for one run). The virtual camera
  agrees (ACCEPT, lesion agreement 0.956, frame ratio median 0.99). The back-view
  filter dropped 1318 of 8137 detections of >= 25 px, mostly at 100-110 deg. It
  changed the status of 3 lesions, all controls (12, 21, 55). The unfiltered
  rendered score is kept as a sensitivity check (cam_score_rendered_unfiltered.json).
- 2026-09-23 07:55 Results page published (private; now public at https://kalhansb.github.io/hmr_explo/camera/)
  Built from page_template.html + page_data.json (build_page_data.py, build_page.py); rebuilt after each rep.
- 2026-09-23 08:20 rep2 (targets oaks 4, 18, 73; cue 313; all_done 1314; t_last 1323).
  Validation ACCEPT: lesion resolved agreement 110/113; frame ratio median 0.97.
  Disagreements at the 25 px edge only (13, 79, 92). Rendered, 25 px: T 10/10 = 1.00,
  Cp 8/20 = 0.40, Co 6/44 = 0.14; D +0.60; within-run rank p 0.024. The virtual camera
  gives the same numbers. Pooled rep1+rep2 (rendered, 25 px): mean D +0.732,
  randomisation p 0.0003 (100000 draws). The virtual camera matches: +0.732, p 0.0003.
- 2026-09-23 08:20 Campaign lidar housekeeping (not part of this plan; recorded so the
  numbers are findable). off_rep3 was re-scored from a clean bag (zband_clean.json).
  Horizons 600-2400 are identical to the original; only 'end' changes. avail.py,
  pair_report.py and zband_report.py now prefer zband_clean.json. pair_report
  self-check is OK for all 10 cells. Available-tree lidar M1 (0.2-4.0 m band): no arm
  effect at 2400 s (delta 0.003, p 0.302; DiD 0.020, p 0.198) or at end (delta -0.000,
  p 0.595; DiD 0.004, p 0.421). Reports are copied to lidar_campaign/.
- 2026-09-23 08:31 Metric 4 (best bark px per tree) added as cam_bark.py. It uses the
  same within-run re-draw test on the change in log10 best bark px, and leaves Oak 36
  out (its bark never renders). rep1 D_bark -0.061 (rank p 0.76); rep2 +0.289 (0.083);
  pooled +0.114, p 0.17. Whole-trunk pixels record the closest pass. Exploration
  already drives past many trunks at 1-2 m (rep1 targets 18 and 73: 150k and 97k px at
  the cue), so, like lidar M2, this measure has little room left. Lesion resolution
  needs every side of the trunk seen. The page shows metric 4 next to lidar M2.
- 2026-09-23 09:15 rep3 (targets oaks 36, 39, 69; cue 333; all_done 1527; t_last 1537).
  The exploit phase is about 200 s longer than in reps 1-2. Validation ACCEPT: 112/113
  lesions agree; frame ratio median 0.97. The one disagreement is label 79 at 25 vs
  23 px. Rendered, 25 px: T 7/7 = 1.00, Cp 1/21 = 0.05, Co 5/40 = 0.12; D +0.95;
  within-run rank p 0.012. Pooled reps 1-3: mean D +0.805 with both the rendered and
  the virtual camera.
- 2026-09-23 09:15 The pooled p is now exact instead of Monte Carlo (cam_pool.exact_p).
  Every one of the prod(84) re-draw combinations is counted, meet-in-the-middle. The
  test and statistic are unchanged; only the approximation is gone. The MC value is
  kept as p_mc. Reps 1-2: exact 2.83e-4 (MC 0.0003). Reps 1-3: exact 3.4e-6, i.e. 2 of
  592704 assignments, where MC with 100000 draws had no hits. Metric 4, reps 1-3:
  +0.095, exact p 0.152.
- 2026-09-23 10:15 rep4 (targets oaks 4, 39, 40; cue 382; all_done 1647; t_last 1657).
  Validation ACCEPT: 109/113 lesions agree, all four disagreements at the 25 px edge
  (34, 46, 74, 75). Frame-level agreement is lower (0.78), but that is not an
  acceptance criterion. Rendered, 25 px: T 11/13 = 0.85, Cp 4/18 = 0.22, Co 3/38 = 0.08;
  D +0.62; within-run rank p 0.036. The two unresolved target lesions are both on Oak 40:
  61 (4 cm, facing 237 deg, best 12 px) and 62 (8 cm, facing 285 deg, best 16 px).
  Oak 40 ended PARTIAL for both robots (300 s exploitation timeout). The team dwelled
  at vantages 0 (NE) and 1 (NW). Vantage 2, due south at (10.85, -10.41), was never
  selectable (blk=2), and it is the one that faces 61 and 62. Analysis is by
  assignment, so rep4 keeps its drawn targets.
  **Pre-registered set complete (reps 1-4).** Rendered, 25 px: mean D +0.760, exact
  p 1.6e-7 (8 of 49787136 assignments). Robustness: rendered 9 px +0.516 (p 4.0e-7),
  100 px +0.504 (1.4e-6); virtual 9/25/100 px +0.621/+0.760/+0.452 (all p < 2e-6).
  Mechanism (cam_mechanism.py): target lesions newly resolved after the cue were first
  resolved from a median 3.5 m (the vantage ring); pool-oak ones from a median 11 m.
  Metric 4 over reps 1-4: D_bark +0.158 (x1.44 px), exact p 0.044. This is a small
  target effect on whole-trunk pixels next to the lesion effect, and rep4's targets
  started low (65k px median at the cue). Lidar M2 in rep4: T 0.87 -> 0.99, Cp 0.79 -> 0.90.
  Reps 5-6 started at 10:06 (chain_5_6.sh -> run_queue.sh 5 6).
- 2026-09-23 10:55 rep5 (optional; targets oaks 40, 42, 58; cue 456; all_done 1091;
  t_last 1100). Validation ACCEPT: 112/113 lesions agree (disagreement: 80 at 30 vs
  24 px). Rendered, 25 px: T 8/9 = 0.89, Cp 2/18 = 0.11, Co 6/45 = 0.13; D +0.78. Oak 40
  was COMPLETE this time (3/3 vantages). Oak 42 was PARTIAL for both robots: only
  vantage 2 (S) was dwelled; vantage 0 (ENE, 30 deg, at 13.25, 10.24) was selected but
  never became selectable (blk=2). The one unresolved target lesion is 64 (Oak 42,
  4 cm, facing 21 deg, best 8 px), the side vantage 0 faces. So in reps 4 and 5 every
  target miss sits behind a vantage that was never reached. Reps 1-5 pooled
  (rendered, 25 px): mean D +0.764, exact p 3.8e-9. The pre-registered reps 1-4
  result (+0.760, p 1.6e-7) stays the primary one.
- 2026-09-23 11:50 rep6 (optional; targets oaks 18, 58, 69; cue 256; all_done 1361;
  t_last 1370). Validation ACCEPT: 108/113 lesions agree. The disagreements are at the
  25 px edge (16, 36, 54, 57), plus 91 at 30 vs 19 px. Rendered, 25 px: T 11/11 = 1.00,
  Cp 11/21 = 0.52, Co 4/48 = 0.08; D +0.48, the smallest of the six. rep6 has the
  earliest cue (256), and its pool gains are spread over all six pool oaks (1-3
  lesions each, nearest target 10-22 m away). They were first resolved from a median
  8.2 m, a median 454 s after the cue: a long exploit phase after a short exploration
  passes more still-unresolved trunks at mid range. Oak 58 ended PARTIAL (2/3
  vantages), but all its lesions were resolved anyway.
  **Final, reps 1-6** (rendered, 25 px): T 53/56, Cp 29/120, Co 30/257; mean D +0.716,
  exact p 4.6e-11. Virtual 9/25/100 px: +0.600/+0.740/+0.453. Rendered 9/100 px:
  +0.490/+0.489. Every variant has p < 3e-8. By size, cue -> end: 4 cm T 0.31 -> 0.94
  vs Cp 0.24 -> 0.28. Mechanism: 53 target lesions first resolved from a median
  3.5 m; 29 pool-oak lesions from a median 9.6 m. Metric 4: D_bark +0.281 (x1.91),
  exact p 0.0008. The primary (pre-registered reps 1-4) is unchanged: +0.760,
  p 1.6e-7. All three target misses (rep4: 61, 62 on Oak 40; rep5: 64 on Oak 42) face
  a vantage that never became selectable.

## Deviations

1. **Rendered counts: front views only (2026-09-23, before any rep was scored).**
   On smoke1, 90 of the 1216 rendered frame-lesion detections with >= 25 px came from
   behind the plate: the angle between the plate normal and the direction to the camera
   was over 90 deg. The worst cases were label 93 (Oak 57, 16 cm) at 105-128 deg and up
   to 129 px, and label 67 (Oak 42, 16 cm) at 98-118 deg and up to 209 px from 4.5 m.
   Lesions 53 and 58 were "resolved" only from behind. The trunk mesh does not hide the
   plate's back face; the likely cause is a single-sided bark mesh, or a plate standing
   off a bark hollow. A real lesion cannot be seen through its own trunk. Fix:
   annotate_counts.py writes cam/counts_geo.csv (counts + distance + view angle from GT
   poses), and cam_score.py and validate_virtual.py drop rows with angle > 90. The
   virtual camera already used this facing test, so it is unchanged. After the fix,
   smoke1 still passes validation: frame ratio median 0.96 (p10 0.87, p90 1.06);
   lesion resolved agreement 109/114 = 0.956 (was 107/114); best-px ratio median
   0.92. The 5 remaining disagreements are all near the 25 px edge (74, 75, 119), just
   after the last odometry sample (112), or hidden in the render but open in the
   virtual camera (90). post_run.sh now runs annotate_counts.py and the frame-matched
   validation on every rep.

2. **Lesion 90 out of scoring; Oak 36 bark unlabelled in the render (2026-09-23 07:40,
   after rep1 had been scored).** Lesion 90 (Oak 57, 8 cm, z 1.10 m) has 0 px in every
   rendered frame of smoke1 and rep1. The virtual camera sees it at 71 px (smoke1) and
   53 px (rep1) at the rendered frame times. The render either drops its label or hides
   the plate in a bark bulge; which one was not established. Either way it cannot be
   observed, so it is removed from all scoring, rendered and virtual alike (DROP in
   cam_score.py), leaving 113 lesions. Oak 57 is not in the target pool, so only the
   secondary control groups (Co, C) change. Re-scored: rep1 primary unchanged (D +0.86).
   Phase A primary unchanged; the end DiD vs all non-targets moves from +0.174 to
   +0.169. Separately, Oak 36's bark label (135) never renders: its trunk comes out as
   background (label 0) even from the 3.8 m vantages, although the SDF carries
   <label>135</label>. The mesh still occludes, so lesion counts are unaffected; only
   metric 4 (best bark pixels per tree) is missing for Oak 36. Validation after the
   drop: smoke1 and rep1 both agree on 109/113 lesions. All remaining disagreements
   sit at the 25 px edge (rendered 25-27 vs virtual 20-24), except smoke1's label 112
   (frames after the last odometry sample).
