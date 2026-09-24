# Camera ("NDVI camera") experiment

Does exploitation help? The lidar can't show it, because trunk coverage saturates.
This experiment measures it with the sensor exploitation was designed for: a
close-range camera. The plan was fixed before any data (PLAN.md), with a running status
log and deviations. Results page: <https://kalhansb.github.io/hmr_explo/camera/>
(built from `page_template.html` + `page_data.json` by `build_page.py`).

This folder holds the scripts, the plan, the pooled results and a small per-run
extract (`reps/rep<n>/`: camera scores, run manifest, planner logs). The scripts
were run from `runs/cam_experiment/` (untracked, ~4 GB with bags and camera
frames) and read and write there; they also use the harness in
`runs/pilot_fine/` (overlay sources, scoring), which is untracked too.

**Status: complete.** The pre-registered set (reps 1-4) is the primary result. The
optional reps 5-6 ("if time allows") were also run and are reported next to it. Phase A
is complete.

## Set-up in one paragraph

Both Huskies carry a simulated camera: an Ignition segmentation camera, 640 x 480,
HFOV 0.82 rad, 2 Hz. Each pixel carries the label of the surface it sees, so occlusion
is exact. The 19 usable oaks each get 6 small visual-only "lesion" plates (2 x 4, 2 x 8
and 2 x 16 cm, one per 60 deg sector, 0.65-1.25 m up), 113 scored in all. A lesion is
*resolved* once a single frame shows it with at least 25 px (sensitivity: 9 and
100 px). Each run explores until the 2D map is done (the cue). Then 3 target oaks,
drawn at random from a fixed pool of 9, are released. The robots visit a 3-vantage ring
(3.8 m) on each target. The planner does not know the camera exists.

## Main result (rendered camera, 25 px)

Of the lesions not yet resolved at the cue, the share resolved by the end:

| run | targets (oaks) | targets | undrawn pool oaks | other oaks | D (T - pool) | rank p of 84 |
|---|---|---|---|---|---|---|
| rep1 | 18, 36, 73 | 6/6 | 3/22 | 6/42 | +0.86 | 0.012 |
| rep2 | 4, 18, 73 | 10/10 | 8/20 | 6/44 | +0.60 | 0.024 |
| rep3 | 36, 39, 69 | 7/7 | 1/21 | 5/40 | +0.95 | 0.012 |
| rep4 | 4, 39, 40 | 11/13 | 4/18 | 3/38 | +0.62 | 0.036 |
| **reps 1-4 (pre-registered)** | | 34/36 | 16/81 | 20/164 | **+0.76** | **p = 1.6e-7** |
| rep5 | 40, 42, 58 | 8/9 | 2/18 | 6/45 | +0.78 | 0.024 |
| rep6 | 18, 58, 69 | 11/11 | 11/21 | 4/48 | +0.48 | 0.012 |
| all 6 reps | | 53/56 | 29/120 | 30/257 | +0.72 | p = 4.6e-11 |

The p value is an exact randomisation test that mirrors the assignment. Within each run,
all 84 ways of calling 3 pool oaks "targets" are re-drawn, and every one of the 84^n
combinations (n runs) is counted. The result holds at every threshold and with every camera
(reps 1-4 | all 6):

| | 9 px | 25 px | 100 px |
|---|---|---|---|
| rendered | +0.52 (p 4e-7) \| +0.49 (p 2.5e-8) | +0.76 (p 2e-7) \| +0.72 (p 5e-11) | +0.50 (p 1e-6) \| +0.49 (p 1e-9) |
| virtual camera | +0.62 (p 2e-7) \| +0.60 (p 2e-9) | +0.76 (p 2e-7) \| +0.74 (p 8e-11) | +0.45 (p 2e-6) \| +0.45 (p 9e-10) |
| rendered, back views kept | +0.51 (p 1e-5) \| +0.51 (p 4e-7) | +0.76 (p 2e-7) \| +0.73 (p 6e-11) | +0.49 (p 2e-6) \| +0.47 (p 4e-9) |

By size, resolved fraction cue -> end over all 6 reps: 4 cm targets 0.31 -> 0.94 vs pool
0.24 -> 0.28; 8 cm 0.47 -> 0.97 vs 0.46 -> 0.62; 16 cm 0.67 -> 1.00 vs 0.64 -> 0.83.
The smallest lesions essentially never get resolved without exploitation.

**Where the gains came from** (`cam_mechanism.py`, all 6 reps). The 53 target lesions
newly resolved after the cue were first resolved from a median 3.5 m, the vantage ring.
The 29 pool-oak lesions resolved after the cue were caught in passing from a median
9.6 m.

**The three misses** all sit behind a vantage the robots never reached. In rep4, Oak
40's south vantage was never selectable, and both robots timed out on it after 300 s
(PARTIAL), having dwelled only at the NE and NW vantages. The two missed lesions, 61 and
62 (best 12 and 16 px), face south. In rep5, Oak 42's east-north-east vantage was never
selectable, and the missed lesion 64 faces 21 deg. Runs are analysed by the drawn
targets, so these shortfalls stay in the result.

**The weakest run** is rep6 (+0.48). It had the earliest cue (256 s), and its long
exploit drive passed every pool oak at mid range, picking up 11 of their 21 open
lesions from a median 8.2 m. A short exploration leaves more for incidental passes to
find, so the contrast shrinks, but the targets still got 11/11.

**Cost.** The exploit phase lasts 644-1275 s after the cue. The two robots drive
174-280 m in total after it.

## Why the lidar couldn't show this

| run | lidar M2 targets | lidar M2 pool | bark px targets | lesions resolved, targets | lesions resolved, pool |
|---|---|---|---|---|---|
| rep1 | 0.98 -> 0.99 | 0.84 -> 0.89 | 124k -> 124k | 0.67 -> 1.00 | 0.39 -> 0.47 |
| rep2 | 0.80 -> 0.99 | 0.89 -> 0.98 | 37k -> 100k | 0.44 -> 1.00 | 0.44 -> 0.67 |
| rep3 | 0.98 -> 0.99 | 0.84 -> 0.87 | 91k -> 106k | 0.61 -> 1.00 | 0.42 -> 0.44 |
| rep4 | 0.87 -> 0.99 | 0.79 -> 0.90 | 65k -> 102k | 0.28 -> 0.89 | 0.50 -> 0.61 |
| rep5 | 0.98 -> 0.99 | 0.89 -> 0.90 | 40k -> 97k | 0.50 -> 0.94 | 0.50 -> 0.56 |
| rep6 | 0.81 -> 0.99 | 0.97 -> 0.99 | 22k -> 99k | 0.39 -> 1.00 | 0.42 -> 0.72 |

(cue -> end; lidar M2 = observed fraction of the 0.2-1.2 m trunk band on the 0.20 m
voxel map; bark px = the most bark any single frame shows, median over trees.)

Lidar trunk coverage is near 1 on most targets by the cue. Where a target starts low
(e.g. Oak 4 in reps 2 and 4), exploiting it lifts the lidar too. But the lidar
cannot tell a trunk seen from every side up close from one that was not. Oak 40 in rep4
reads M2 0.991 at the cue and 0.992 at the end, while its south-facing lesions were
never resolved. Whole-trunk image size (PLAN metric 4) shows a target effect too:
x1.44 over reps 1-4 (exact p 0.044) and x1.91 over all 6 (p 0.0008). It is a much
weaker contrast than the lesions, because it records the
closest pass, and exploration already drives past many trunks at 1-2 m. Small lesions
need every side of the trunk seen from close range, and the vantage ring adds exactly
that.

## Phase A: the same camera replayed on the campaign on/off pairs

A geometric "virtual camera" runs on ground-truth poses. It was validated against the
rendered camera on the smoke test and every rep (ACCEPT: 108-112 of 113 lesions
agree). The disagreements sit at or near the 25 px edge, apart from one smoke-test lesion
seen after the last odometry sample and rep6's lesion 91 (30 vs 19 px). It was then run on the 5 campaign on/off pairs (targets
42, 4 and 39, released at 120/420/720 s). DiD = (T_on - T_off) - (pool_on - pool_off),
with an exact sign-flip p over 32:

| horizon | mean DiD (pool) | pairs > 0 | p |
|---|---|---|---|
| 1200 s | +0.32 | 5/5 | 0.031 |
| 3600 s (primary) | +0.16 | 4/5 | 0.062 |

The advantage is largest soon after the releases. By 3600 s the off arm catches up,
because its robots eventually drive past the targets too. With only 5 pairs, 0.031 is
the smallest p possible. Details are in `phaseA_report_thr*.txt`.

## Honest limits

- NDVI is a per-label lookup, not a spectral simulation. What is tested is geometry:
  does the robot put enough pixels on each patch.
- Two render artefacts, both logged as deviations in PLAN.md. Plates are visible through
  their own trunk, so back views (> 90 deg) are dropped. Lesion 90 never renders and is
  unscored. Keeping back views changes nothing (table above).
- One world, one lesion layout, clean sensors, n = 4 pre-registered runs (6 in all).
  Target sets overlap between runs. The test accounts for this, but generalisation to
  other layouts is untested.
- The control is "the same extra driving, other trees". It does not compare against a
  planner that knows about the camera.

## Files

| file | what |
|---|---|
| PLAN.md | pre-registered plan, status log, deviations |
| make_lesions.py, lesions.json, overlay/ | lesion layout and the camera world, robot and scenario |
| run_cam_cell.sh, run_queue.sh, chain_5_6.sh, targets/ | one camera run; the rep queue; reps 5-6 launcher; per-rep target draws |
| cam_counter.py | online per-frame label counts -> `<run>/cam/counts.csv`, best frames |
| post_run.sh | per rep: GT poses, view angles, virtual camera, validation, scores, z-band |
| cam_score.py, cam_pool.py, cam_bark.py, cam_mechanism.py | per-run readout; pooled exact test; metric 4; where resolutions came from |
| virtual_cam.py, validate_virtual.py, phaseA.sh, phaseA_report.py | Phase A |
| ndvi_png.py, ndvi_tiles.py | false-colour NDVI images of the best frames |
| build_page_data.py, build_page.py, page_template.html | results page |
| lidar_campaign/ | campaign lidar re-analysis (available-tree M1, pair self-check) |
| rep*/ | runs: bags, logs, `cam/`, `cam_score_*.{txt,json}`, `zband.json` |
