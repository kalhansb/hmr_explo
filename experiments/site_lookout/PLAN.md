# Site lookouts: plan (written 2026-09-25, before any walk)

## Question

At the real site (the `gt_map` of `hmr_localisation`, explorer ROI), with
the mulcher parked among the trees at map (-2, 0.6), do robots parked as
lookouts on the path warn the mulcher of a walker earlier than the mulcher's
own lidar?

This repeats the lookout experiment (`experiments/lookout`, layout L2: one
path, a lookout at each end) on the site's tree layout. Everything that is
not about the site is unchanged.

## What is the same as the lookout experiment

Code in `experiments/lookout` run as is (walks.py, detect.py, scans.py,
gz.py, spawn_static.py, sim_up.sh, run_layout.sh, gonogo.py, analyse.py),
in the lookout experiment's own environment (`docker_run.sh` here: its
workspace build, run folder and calibration):

- Lidar: VLP-16 model, 0.5-100 m, 2 cm noise, 1800 x 16, 10 Hz, on the
  mulcher (roof, 2.1 m) and on each lookout Husky.
- Mulcher: static body and head boxes, turned through 12 headings.
- Detection: main rule m5 (>= 5 new points, 3-D linkage 0.5 m, >= 2 rings,
  2 consecutive steps within 1 m); sensitivity rules m3, m10, m20, xy5 and
  the ideal detector; background of 10 scans; hit within 1 m.
- Walker: one person mesh, 1.75 m, 0.5 m steps at 1.3 m/s, starting 45 m
  (+-5 m) before the post line, +-0.5 m across the path.
- Walks: 12 headings x 2 entries x 2 repeats = 48; 12 check walks, run
  "away" and "absent".
- Posts: `posts.py` unchanged (free spot 12-28 m out beside the path, safe
  radio link under the unchanged comms model, earliest view of a walker).
- Open-ground calibration: `runs/lookout/calib_gpu` of the lookout experiment.

## What is new

- **Trees:** the 38 trees found in the map by `trees.py` (canopy height
  model, peaks >= 3 m, >= 3.5 m apart), all as our oak model (the
  narrow-crown ones, called pines before, are oaks too), each scaled to its
  measured height (4.8-21.9 m) and crown radius (1.4-4.5 m). No other trees:
  outside the ROI the ground is open.
- **Ground:** flat plane (no slope, walls, fences or bushes).
- **Path:** one straight path south of the tree band (layout.py path S,
  the lane next to tree pine_9), 220 m long. Its nearest trunk is 3.3 m
  from its centre line. It passes **14.8 m** from the mulcher at its closest.
- **Walk end:** the walks end at the path's closest approach, 14.8 m from
  the mulcher (L2: 10 m).
- **Posts:** entry A (from the west) 23.0 m from the mulcher, entry B (from
  the east) 28.0 m. Lookouts lksa, lksb.
- Frame: the mulcher at the origin; map = world + (-2.0, 0.6).

## Measures (as the lookout experiment)

Primary: walks caught before the post line, mulcher alone vs mulcher and
lookouts, main rule; McNemar exact. Secondary: paired difference of first
alarm times (team minus mulcher alone), distance at first alarm, false
alarms, per sensitivity rule; check walks: the mulcher's first alarm with and
without the lookouts.

Expected from the calibration, before any walk: under the main rule the
mulcher cannot fire beyond ~12.5-14 m, and the walker never comes closer
than 14.8 m, so the mulcher alone will probably catch no walker at all under
the main rule. The sensitivity rules (xy5, ideal detector) reach further
and give the comparison that is not decided by the walk's end point.

## Procedure

1. Pilot: 6 walks and the check walks (`run_layout.sh S1 ... pilot`).
2. Go/no-go (`gonogo.py`), as in the lookout experiment. On NO-GO, fix what
   failed, record it here, re-run the pilot. Nothing in the lidar, rules,
   thresholds, walks or posts is changed after seeing results.
3. Full: 48 walks and 12 + 12 check walks (`run_layout.sh S1 ... full`).
4. Analysis: `analyse.py --layout S1=...`.

## Status log

- 2026-09-25: plan written; layout built (`build_layout.py`).
- 2026-09-25: pilot (6 walks + 3 + 3 check walks, `runs/site_lookout/S1_pilot`): GO
  (`runs/site_lookout/gonogo_pilot.txt`). Nothing changed. Full run started.
- 2026-09-25: full run (48 walks, 12 + 12 check walks, `runs/site_lookout/S1_full`):
  GO (`results/gonogo_full.txt`). Analysis: `analyse.py` stopped on three
  printing steps that assumed the mulcher detects someone and the L2/L3 figure
  notes; fixed in how it prints only (L2/L3 output unchanged). Results in
  `results/` and README.md.
- 2026-09-25, after the results (exploratory, asked for): do the trees stop
  the mulcher seeing walkers? S1's 12 check walks (mulcher only) repeated
  with every tree removed (`no_trees.py`, layout S1N, `runs/site_lookout/S1N_check`).
  Specified rule: 0/12 with and without trees. Horizontal clustering and
  ideal detector: 11/12 both ways (the same walk missed, in the blind
  wedge); the first detection at the same distance in 8 of 11, 0.5-0.9 m
  further without trees in 2, 6.7 m (horizontal) / 3.7 m (ideal) further in 1
  (h090, entry B).
- 2026-09-25, after the results (exploratory, asked for): does the real site, not only its trees,
  block the mulcher's lidar? S1's 12 check walks repeated with the oak models replaced by the gt_map's
  own points (`gtmap_world.py`, layout S1G, `runs/site_lookout/S1G_check`): ROI points more than 0.35 m
  above the ground, on the flat plane, as 5 cm voxels (656k voxels, 6.9 M triangles, visual only);
  190 points in the mulcher's footprint removed; no map point within 1.5 m of the path line.
  Specified rule: 0/12 (as with oaks and with no trees). Horizontal clustering and ideal detector:
  11/12, the same walk missed (h330-B, blind wedge); no false alarms. First detection later than
  among the oak models in the 5 detected walks from the east: by 1.7-14.2 m (median 4.6 m)
  horizontal, 2.7-7.7 m (median 5.8 m) ideal; from the west unchanged, except h060-A under the
  ideal detector (9.3 m later).
- 2026-09-25, after the results (exploratory, asked for): M-detector (hku-mars, Nature Communications
  2024; `mdetector/`) as every lidar's detector. S1 and S1G re-run with whole scans saved
  (`run_md_sims.sh`, `runs/site_lookout/S1_md`, `S1G_md`); M-detector's upstream nuScenes parameters
  changed only for the VLP-16 geometry and the walk timing (`md_vlp16.yaml`); alarm rule fixed before
  its output was seen (`md_sim.py`: frame-out moving points, 0.5 m horizontal linkage, >= 3 points,
  2 consecutive steps within 1 m, hit within 1 m). 48 main walks (`md_team.py`), oak models / map points:
  mulcher detected 44/44 of 48 at all (missed the 4 blind-wedge walks), before the post line 18 / 16
  of 48, median warning -1.7 / -4.0 s; with lookouts 48/48 before the post line, median 24.6 / 25.0 s;
  first alarm earlier with lookouts on the same walk, median 25.6 / 28.7 s (least 18.1 / 20.4 s);
  false alarms 0 in every walk and lidar.
- 2026-09-26: the lookout experiment's layouts L2 and L3 re-run the same way with M-detector
  (`mdetector/run_md_lookout.sh`, `runs/lookout_md/`). Mulcher alone / with lookouts, before the post
  line: L2 34/48 / 48/48 (median warning 2.3 / 28.8 s), L3 42/72 / 72/72 (3.5 / 26.9 s); same walk,
  lookouts earlier by a median 26.5 s (L2) and 23.1 s (L3); in 4 L3 walks the mulcher alarmed first
  (sparse hits on the walker 62-66 m out); false alarms 0. Recorded in `../lookout/README.md`.
