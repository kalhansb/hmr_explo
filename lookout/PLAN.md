# Path-lookout experiment: plan

Written 2026-09-23, before any walk. The design (lidar settings, detection rule,
thresholds, procedure) is fixed by the task brief and is not changed after results
are seen; anything that cannot work as written is logged under **Deviations** with
the reason, before the change.

## Where things live

- Separate checkout `~/Documents/lookout_experiment`, branch `lookout_experiment` in
  the superproject and every submodule, cut from the `exploitation_experiment` tips
  (explo_planner 2c09835, simple_nav_3d cadcaa7, scovox a63ee01, hmr_sim 3576be1).
  `~/Documents/exploitation_experiments` (its build, its runs/, the cam pilot) is not
  touched.
- Experiment code, configs, analysis, figures, README: `lookout/` (tracked).
  Raw per-scan logs: `runs/lookout/` (gitignored, large).
- Simulation: docker `hmrexplo:humble`, Ignition Fortress + ROS 2 Humble, ogre2 on
  the NVIDIA GPU (headless EGL, `LOOKOUT_GPU=1`; llvmpipe before 2026-09-23),
  one container per layout, `--network none`, own `IGN_PARTITION`/`ROS_DOMAIN_ID`.

## What exists (survey)

- **Exploit phase:** C++ only, `explo_planner/src/explo_planner_node.cpp` (19k lines,
  `main()` in the same file, so node logic is only testable by source scans or by
  moving logic into `explo_planner_lib`). Tree mode = vantage ring around a released
  `TreeTarget` (EXPLOIT_PLAN -> NAVIGATE -> EXPLOIT_DWELL). Goals go out as
  `PoseStamped` on `/<r>/goal_pose` to simple_nav_3d; arrival is from TF with
  `goal_xy_tolerance`/`goal_yaw_tolerance` 0.4. No "in position" topic exists.
- **June mulcher experiment:** not found. Searched every local branch of all six repos,
  sibling workspaces under ~/Documents, and `git ls-remote` of likely repo names under
  kalhansb on GitHub (`hmr_exploitation_experiment`, `run_sweep.sh` and
  `build_flatforest_arena.py` do not exist; the only `run_sweep.sh` on disk is an
  unrelated SLIM-VDB sweep). The mulcher and head boxes, the forest builder and the
  person stepping are therefore built new here, to the dimensions in the brief.
- **Radio model:** `hmr_sim/src/hmr_comms_sim_node.cpp` (ROS node, COMMS=1): 70 dB per
  trunk in the Fresnel corridor, hard 30 m horizon (+200 dB beyond). No Python port.
  A lookout sits ~90 m from the mulcher, so under this model it never has a direct
  link. (The 90 m posts; since 2026-09-24 the posts are inside the horizon, see
  **Design change**.) Ported offline (no fade term) to fill the optional radio column; expected
  result "no link" everywhere, reported as such.
- **Everything that relies on the 25 m clip:**
  - scovox_node (lidar path): drops non-finite and range outside [1, 20] m before
    integrating; carving only from accepted hits (no max-range clearing).
  - nav costmap (simple_nav_3d `local_mapper`): drops non-finite and outside
    [0.8, 20] m.
  - explo_planner: never reads the cloud (sensor model `fov_max_range` 20 m).
  - The model.sdf comment, the gen-9 audit in `exploitation_map_gain_experiment.md`,
    RViz config (display only). Bags never record the cloud.
  So the crop is a safeguard: mapping and navigation get a relayed cloud
  `/<r>/velodyne_points_25m` with every return beyond 25 m set to +inf, which is
  what the old far clip produced. Only detection reads `/<r>/velodyne_points`.
- **Person model:** `hmr_sim/models/MaleVisitorOnPhone` (standing, licence not
  recorded). Using instead Gazebo Fuel **OpenRobotics/Walking person** (Marina
  Kollmitz, MakeHuman; **CC0 1.0**, OK for publication): static mid-stride mesh,
  35k triangles, 1.87 m as shipped, scaled to 1.75 m.

## What gets built

1. **Lidar (hmr_sim model.sdf):** max 100 m, min 0.5 m, Gaussian sigma 0.02 m;
   channels, 1800 samples, 10 Hz, 1 cm resolution unchanged. Crop relay node per
   robot; simple_nav_3d gets a `points_topic` launch arg (default unchanged).
2. **Lookout exploit mode (explo_planner):** `exploit_mode` = `tree` (default,
   untouched) | `lookout`; the point comes as `lookout_x`, `lookout_y`, `lookout_yaw`
   parameters from a per-layout ROS params-file YAML (one block per robot name);
   `lookout_start` = `on_done` (default: after exploration is DONE, the
   map-then-support order) | `immediate`. States LOOKOUT_NAV -> LOOKOUT_HOLD. Drives
   with the existing goal path (simple_nav_3d), holds, publishes
   `/<r>/lookout/in_position` (Bool, latched) and `/<r>/lookout/pose`, logs a
   `lookout_in_position` event with the time taken. Parsing and the arrival/hold
   logic live in `explo_planner/lookout.hpp` (unit-tested, `test_lookout`). simple_nav_3d gets launch args
   for its global map size/origin (default 80 m / -40, unchanged), because a 90 m
   lookout is outside the hard-coded 80 m map. Built and tested; since 2026-09-24
   the experiment does not use it (the lookouts are spawned on their posts).
3. **World builder (`lookout/build_world.py`):** oaks (the flatforest v2 Fuel oak,
   same density 0.0086 /m^2 and min spacing 6.5 m as flatforest v2, Poisson-disc,
   fixed seed) over a disc of radius 160 m; lanes 3 m wide cleared (no trunk centre
   within 1.5 m + 1.7 m root flare of the centreline); lookout points calculated
   from the map by `posts.py` (since 2026-09-24; before, a 2 m pad was cleared at
   each 90 m point, now a point must already be that free); 8 m cleared round the
   mulcher; flat ground 400 m. Paths are
   polylines with gentle bends in `lookout/config/L2.yaml`, `L3.yaml`.
   Mulcher: body box 2.59 x 1.68 x 1.94, head box 0.80 x 1.70 x 0.80 centred 1.5 m
   ahead at 2.0 m, rendered (visual geometry), the same VLP-16 lidar on the roof
   centre at 2.1 m. Person: Walking person, a static visual-only model moved with
   `set_pose`; the mulcher is a static model too, turned with `set_pose`.
4. **Detection (`lookout/detect.py`):** exactly the brief's Part 4 (background
   median over 10 scans per beam; new = >= 0.3 m closer or background none; cluster
   at 0.5 m; alarm = >= 5 pts on >= 2 rings, persisting 2 steps within 1 m; hit
   within 1 m of truth). Best-case rule: >= 5 returns inside the person's box,
   trunk returns ignored (a return in the box on a beam whose background range is
   within 0.1 m of it is a static surface: trunk, branch or ground). Per scan and
   lidar the log keeps every new point at a looser 0.1 m threshold (x, y, z, ring,
   range, background range), so every rule and threshold can be recomputed offline.
5. **Walk driver (`lookout/walks.py`):** paused world; move the person, step the
   world 5 physics steps (0.1 s, one lidar period), take the first scan from every
   lidar rendered after the move (`lookout/scans.py`: per-lidar stamps, all lidars
   on one common stamp; never /clock), log. Lidar
   poses from Gazebo ground truth at scan time (robot odom + sensor offset; mulcher
   pose as set and read back), never hard-coded heights.
6. **Warning delivery (messenger, added 2026-09-23 on request; radio model
   unchanged). Dropped 2026-09-24 (see Design change); the code is kept.**
   - Link: `lookout/radio_node.py` runs the comms emulator's link budget
     unchanged (`radio.py`: p0 + 20 log d + 70 dB per trunk in the Fresnel
     corridor + AR(1) shadow fade sigma 4.8 dB, alpha 0.9, 200 dB past the 30 m
     horizon, the 3-tier rate ladder with its 8-sample hysteresis at 5 Hz) between
     each lookout and the static mulcher, from ground-truth positions and the
     layout's trunk list, and publishes `/<r>/lookout/link_up`. Only the relaying
     of traffic is left out.
   - Planner (lookout mode): while driving to its post, it records the last pose
     where the link was up and knows the mulcher position (parameters). On an
     alarm (`/<r>/lookout/alarm`) at its post: if linked, it sends at once
     (`/<r>/lookout/warning`). If not, it drives to the last-link point, checking
     the link all the way, and sends the moment it comes up. If the link is
     still down there, it drives towards the mulcher (goal 5 m short of it) until
     it comes up. Then it drives back to its post and holds. While it is away,
     `in_position` is false: it is not watching (its lidar is moving). Alarms
     that arrive while it is away are logged and dropped.
   - Trips (`lookout/trips.py`): the mulcher is static and the link depends only
     on positions and trunks, so each lookout's trip is the same every walk. So
     trips are measured once per layout, after the lookouts first reach their
     posts and before the walks: 3 trips per lookout, one lookout at a time.
     Logged: time until link up, distance driven, average speed, where the
     link came back, time to return to the post (settled).
   - Per walk (offline, from the walk logs; the walk driver is not changed):
     delivery time = the lookout's first true alarm time + its mean trip time
     (0 if linked at the alarm); the person's position then, along the path at
     1.3 m/s (clamped at the walk's end); delivered before the mulcher's own
     first detection, and by how many s; delivered before the 90 m and 50 m
     lines; time the entry is unwatched (mean trip out and back). Team =
     earliest delivery. False trips: every false alarm of a lookout would send
     it on a trip. Reported as raw false alarms and as trips (a trip absorbs
     the alarms raised during its unwatched time), with the unwatched time they
     cost, and a flag for a false trip that left the entry unwatched when the
     true alarm came.
   - Analysis: next to the detection curve, % of walkers whose warning is
     *delivered* at least T s before the 90 m line, T = 0-30 s; % of walks where
     the delivered warning beats the mulcher's own detection. README: delivery
     results depend on the model's 30 m radio horizon; the Husky's measured trip
     speed compared with the walker's 1.3 m/s.
7. **Calibration, pilot, full run, analysis:** as the brief, in that order. The
   pilot runs 1 trip per lookout (a check that delivery works), the full run 3.
   (No trips since 2026-09-24.)

## Assumptions

- The lookouts are spawned on their posts (since 2026-09-24). In a real mission
  they would explore first, calculate the posts from their map and drive there;
  exploration and the drive-out are out of scope. The map for the calculation is
  the world's trunk list (a perfect map). The world is paused for stepped walks;
  lookout poses are logged at every step to show they held. (Until 2026-09-24:
  they drove to their points with `lookout_start: immediate` from beside the
  mulcher, then the stacks were stopped before the walks.)
- Detection sees ground-truth poses of the lidars (as the brief says).
- Oak density and spacing come from flatforest v2, since the June forest's own
  numbers are not available.
- 2 cm range noise applies to the mapping clouds too; the crop cannot undo it.

## Status log

- 2026-09-23: survey done, plan written, workspace cloned and building.
- Part 1 done: model.sdf 0.5-100 m, sigma 0.02; `lidar_crop.py` relay at 25 m feeds
  nav and scovox (`lidar_points_topic`); `nav_global_map_size_m` launch arg.
- Part 2 done: lookout mode in explo_planner, 14 new tests (`test_lookout`), full
  explo_planner suite 710 tests, 0 failures. Tree mode code paths unchanged (the
  tests assert both hooks are gated on `exploit_mode_ == LOOKOUT`).
- Part 3 worlds built (`build_world.py`, seed 20260923): L2 680 oaks, L3 663 oaks,
  calib (open ground). Person scaled from its *rendered* extents (measured with a
  vertical "gauge" lidar): 1.739 m tall as rendered.
- Smoke test (`lookout/smoke/`, open ground):
  - Stepping: one step of 5 gives a scan from every lidar in ~0.45 s wall, no retries.
  - Self-returns: Husky none (nearest 2.43 m). Mulcher: channels 0-3 (-15..-9 deg)
    hit its own roof all round at 0.55-1.0 m; channel 4 (-7 deg) clears the roof.
  - **Head check:** channels 0-14 are blocked over the whole +-30 deg wedge
    (channels 5-13 out to +-37.5 deg, 14 to +-32.3 deg). Channel 15 (+15 deg) is
    blocked only within +-11.3 deg (38 % of the wedge): it passes over the 2.4 m
    head top. It points 15 deg up (3.4 m high at 5 m), so it cannot see a person
    anywhere. So 15 of 16 channels over +-30 deg, not 16; the geometry is as
    specified and is not changed.
  - Person in the box (points / rings), open ground: Husky 47/5 at 10 m, 13/2 at
    25 m, 7/2 at 25.8 m, 4/1 at 28.3 m, 0 at 32 m, 2/1 at 39 m; mulcher 31/4 at
    12 m, 16/3 at 20 m (rear), 4/2 at 30.4 m, 2/1 at 30 and 45 m, 0 at 15 m inside
    the head wedge. On these spot checks the 5-point, 2-ring rule would stop near
    25-26 m from a Husky, below the brief's ~30 m sanity figure; Part 5 measures it
    properly.
  - (Corrected.) A "4.4 deg tilt at rest" and the grazing-return jitter reported
    here were the spawn transient, not the resting state: a spawned Husky takes
    ~20 s of sim time to settle (z 0.1322 m, pitch ~0.014 deg, roll ~0). A tilt
    change of 0.02 deg turns hundreds of grazing ground returns "new". Every
    background is now learned only after the Husky is at rest
    (`walks.settle`: 2 s window, <= 0.2 mm and <= 0.002 deg), and a lidar whose
    pose moves by more than that between steps is re-learned at the next walk
    boundary. After the fix: 3-8 new points per scan, no drift false alarms.
  - Mulcher `set_pose` (`setpose3.py`): at 8 poses (yaw 0, 90, 30, -60, 180, 330,
    translation 5 m) lk0's bearing in the mulcher frame matches the commanded pose
    within 0.5 deg, the head stays at bearing 0, and lk0 sees the body turn. (At
    -60 deg lk0 is at -30 deg, inside the head shadow, and is not seen.)

- Walk driver (`walks.py`) built and tested on a 90-step L2 walk (lk2b's first
  main-rule alarm at 13.7 m from it, 0 false alarms). `ign service` lost ~3 % of
  replies and cost ~0.4 s per call ("Host unreachable" in the server log; one
  walk crashed on it): replaced by a persistent ign-transport client
  (`gzsvc.cc`, `build_gzsvc.sh`; retries 4x, every retry logged) with the CLI as
  fallback. A forest step (3 lidars) takes ~1.1 s wall: L2 full ~4 h, L3 ~7 h.
- Part 5 calibration done (`calib.sh`, `calib.py`, runs/lookout/calib; open
  ground, 5-40 m in 2.5 m steps, 10 scans, toward and across). Reliable / max
  range (m):

  | mount, orientation | main rule (m5, 3D) | xy5 (horizontal linkage) | best case |
  |---|---|---|---|
  | Husky, toward | 12.5 / 12.5 | 32.5 / 32.5 | 32.5 / 32.5 |
  | Husky, across | 12.5 / 12.5 | 27.5 / 27.5 | 27.5 / 27.5 |
  | Mulcher, toward | 12.5 / 12.5 | 27.5 / 37.5 | 27.5 / 40 |
  | Mulcher, across | 12.5 / 12.5 | 30 / 37.5 | 30 / 37.5 |

  The main rule stops at 12.5 m for every mount: a geometric ceiling of the rule
  as written (see Deviations), not a sensor limit. The best case and xy5 land on
  the brief's ~30 m reliable / ~38 m max sanity figures.
- Messenger built (item 6): radio_node.py, planner LOOKOUT_DELIVER state, 5 new
  tests; explo_planner suite 715 tests, 0 failures. `run_layout.sh` runs a
  layout end to end (stacks, drive-out, trips, stop, walks); `trips.py` measures
  the drive-out and the trips. The sim runs at ~0.1 real-time with two nav
  stacks up (lidar rendering on llvmpipe), so a drive-out is ~15 min wall.
- 2026-09-23, rendering moved to the GPU. The first pilots were stopped at the
  drive-out (RTF ~0.1) and are re-run from scratch. With the NVIDIA driver fixed
  (reboot), `LOOKOUT_GPU=1` passes the GPU into the container and ogre2 renders
  headless through EGL (`gpu/10_nvidia.json`, `--headless-rendering`); ogre2.log
  confirms the GTX 1650 Ti. Paused-world stepping with the walk's 3 lidars went
  from ~1.1 s to 0.66 s per 0.1 s step. Checked and ruled out as the bottleneck
  (smoke worlds, deleted after): the lidar range (50/25 m no faster than 100 m)
  and the oak collision meshes (none: no faster). The cost is ~0.2 s per lidar
  scan in the single-threaded gz server; sensors render one after another
  (gz-sensors #81), so it adds up per lidar.
- Gazebo settings reviewed (web sources and the numbers above, no benchmark runs)
  and left unchanged:
  - physics: DART stays. Bullet is only preliminary in Fortress and TPE has no
    dynamics (the Husky needs wheel contact to drive and settle); physics is not
    where the time goes.
  - step: 0.02 s stays. A walk step is one lidar period (0.1 s), and fewer
    physics steps per render would not reduce the renders.
  - real_time_factor 2.0 is a cap the sim never reaches (0.1-0.5), so no effect.
  - shadows are already off and the world has no lights; lidar depth rendering
    does not use lighting.
  - the bridge carries only the point clouds, IMU, odometry, cmd_vel and clock
    that the stacks use.
  - lidar samples, rate, range and noise are fixed by the design.
  So the only speed-ups are the GPU and running layouts in parallel (8 cores,
  one per gz server). The calibration is re-run on the GPU (`runs/lookout/calib_gpu`,
  used from now on; the llvmpipe table above is kept for comparison).
- GPU calibration (`runs/lookout/calib_gpu`, ~3 min): 43 of 44 range entries equal
  the llvmpipe run; the one difference is a sensitivity variant (Husky across,
  m20 reliable 12.5 m vs 10 m). Main rule, xy5 and best case identical.
- Pilot incident (infrastructure, not procedure): `run_layout.sh` wrapped
  trips.py in a 2 h wall-clock `timeout`, sized for a faster sim. At RTF ~0.1
  it killed the L2 pilot's trips.py during lk2b's return (lk2a's trip was
  complete); run_layout then stopped without walks, as designed. Raised to 48 h
  (trips.py already times out each leg in sim time). L2 pilot re-run from
  scratch; the killed run is kept in `runs/lookout/L2_pilot_trips_timeout`.
  L3's wrapper was paused (SIGSTOP) 2 min before it fired, so its trips ran to
  the end, and the rest of run_layout (stop stacks, zero cmd_vel, pilot and
  pilot_check walks) was run by hand in the same container with the same
  commands; its run.out will still end with "trips failed" when the paused
  wrapper is released after the walks.

- Messenger pilots on the GPU with the 90 m posts
  (`runs/lookout/L2_pilot_return_behaviour`, `L3_pilot_return_behaviour`; both
  stopped before any walk, no walk has run with 90 m posts): every lookout
  reached its point (0.18-0.20 m off, heading -18..+9 deg, 0.43-0.47 m/s); on
  the way out the link was last up 27.9-30.4 m from the mulcher. A carried
  warning needed 137-148 s (~61 m at ~0.44 m/s) to reach the link. A walker
  first seen by a 90 m post (~102 m out) reaches the 10 m stop in ~71 s, so the
  warning would arrive ~70 s after the walker has stopped. The messenger cannot
  warn in time, and without a link a lookout cannot report "all clear" either.
- 2026-09-24: design change, user-directed, before any walk (next section).
  Posts calculated from the map (`posts.py`); worlds and configs rebuilt (L2 680
  oaks, L3 664: no pads cleared now; the calibration world is byte-identical, so
  the GPU calibration stands); 90 m worlds and configs kept in
  `runs/lookout/worlds_r90_posts`. Pilots re-run with the lookouts spawned on
  their posts (`runs/lookout/L2_pilot`, `L3_pilot`).
- Post-spawn pilots, first attempt (L2 and L3 in parallel; kept in
  `runs/lookout/L2_pilot_crashed`, `L3_pilot_crashed`, `L3_pilot_crashed2`).
  Every lookout settled (L2 59.8 s sim, L3 98.3 s; lk3a rocked longest) and
  stood 0-10 mm from its calculated point, linked (24.0-28.0 m, 52.9-54.2 dB,
  no trunk on the link). L2 ran 4 of its 6 walks, no false alarms, every
  lookout alarm linked. Then two infrastructure faults, no design change:
  (1) `ign model -m` (the mulcher readback at each heading, sensor poses at
  session start) rebuilds the whole world state and took 44-72 s with two sims
  on the host; its 60 s timeout fired (L3 at the first heading, L2 at the
  fifth). Timeout now 600 s, and the call is repeated like the other `ign`
  calls. (2) walks.py paused the world at session start and never unpaused
  it, so the check session that follows the main walks waited for clouds a
  paused world never sends ("no clouds/odom from ['mulcher']"); no run had
  reached its check walks before. walks.py now unpauses before a session
  waits for clouds (the session pauses it again at once). Pilots re-run from
  scratch, one layout at a time: two sims in parallel step at ~1.3 s each,
  one alone at 0.66 s, so parallel saves nothing and loads the CLI.
- Post-spawn pilots, second attempt (L2 then L3): **go/no-go check 4 fails on
  L2 (stopped for the user).** lk2b, settled and on its post, held within
  0.006 deg for 2.5 walks, then tilted 0.52 deg in one 0.1 s sim step (walk
  h120-A step 108; the walker 40 m away, nothing near the Husky; not contact
  with an oak's collision hull: the hull reaches <= 3.1 m at 0.9 m height and
  no part of any lookout's footprint is inside one). It kept creeping after
  the relearn at the next walk boundary (0.03 -> 0.28 deg over walk h180-B).
  At grazing incidence that turns ~2000-5700 ground returns "new": lk2b false
  alarm scans 162 (h120-A) and 906 (h180-B), against 0 for lk2a and the
  mulcher. Every lidar pose stayed inside the hold check (< 0.01 m). lk3a
  rocked the same way while settling (98 s sim). The Husky on its suspension
  in DART is not a still sensor platform; this is a sim artifact, but fixing
  it changes the procedure, so it goes to the user first.
  Second blocker, same run: the check walks cannot start. Removing a lookout
  (`ign service .../remove`) crashes the Gazebo server (ogre2 assert in
  `HlmsDatablock::~HlmsDatablock` inside `Ogre2GpuRays::Render`, gz.log): a
  Fortress bug when a model with a gpu_lidar is removed. The check session then
  finds no clouds. Also for the user (options: check walks in a second sim with
  no lookouts spawned, or move the lookouts far away instead of removing them).
  L2's 6 main walks are complete (`runs/lookout/L2_pilot`). gonogo.py on L2:
  NO-GO, only on false alarms (409 true / 1800 false alarm scans, all 1800
  from lk2b). L3's 6 main walks (`runs/lookout/L3_pilot`): every check passes,
  379 true / 0 false, no lookout tilted more than 0.0006 deg from its
  background. So the tip was one event on one robot in 12 walks, but it wrecked
  L2's false-alarm count. Neither layout's check walks could run (the crash
  above).
- Why lk2b tipped (user: "check why wobble happened. fix it. check walks do
  both"). `smoke/wobble.sh`: ten Huskies on open flat ground (no trees, no
  mulcher, no person), two at each lookout heading, spawned as in the pilots
  (z 0.3, spawn_robot.launch.py), 600 s sim, every ground-truth odom message
  logged; `smoke/wobble_stats.py` applies walks.py's settle test and reports
  what follows. With the worlds' collision detector (DART + bullet,
  `runs/lookout/smoke_wobble_bullet`): settled after 17-79 s, and after that 4
  of 10 moved past the 0.01 deg relearn threshold: wb7 tipped 0.53 deg in one
  0.1 s step (2.6 mm) 2.6 s after passing the settle test, the same size as
  lk2b's 0.52 deg; wb2 and wb4 crept 0.05 deg, wb3 0.012 deg. The same ten with
  the ode detector (`smoke_wobble_ode`): all settled by 17.5-25 s, then
  0.0000 deg and <= 0.12 mm over the remaining ~580 s. So the cause is the
  bullet contact between the Husky's cylinder wheels and the ground plane:
  any parked Husky can rock at any time, nothing in the forest or the post
  triggers it, and a settle test cannot rule it out (wb7 passed it and tipped
  2.6 s later). **Fix: the lookouts are spawned static on their posts**
  (`spawn_static.py`, called by sim_up.sh with AT_POST=1): the model and
  namespacing of spawn_robot.launch.py (hmr_sim untouched) with `<static>`
  set, base level at z 0.1322 m, where a settled Husky sits on the plane. The
  lidar is then at 0.848 m, as in the pilots (steps.csv `lz`) and the
  calibration. The mulcher, the person and the trees were static already, so
  nothing in the world is left to physics. Switching the world to ode instead
  would change collision for the whole forest, not only the robots; not done.
  walks.py's settle test and hold check stay as guards (a static lookout
  passes them at once). `smoke/static.sh` (`runs/lookout/smoke_static`): two
  static lookouts on the flat world: ground-truth odom and clouds arrive
  (~17 Hz wall at RTF ~2), `ign model -m` gives the post pose and the lidar
  offset (0.0012, 0, 0.716) m, set_pose moves a static lookout (needed by the
  "away" check walks), and over 492 s of sim time the pose did not change at
  all (0.000 mm, 0.00000 deg, ~24600 odom messages each).
- Check walks, both ways (the user's choice): run_layout.sh runs the main
  walks, then the check walks "away" in the same sim (walks.py
  `--check-how away`: each lookout set_pose'd to (1000 + 10 i, 1000), outside
  every lidar's 100 m), then stops that sim, starts a fresh one without the
  lookouts (sim_up.sh `NO_LOOKOUTS=1`, logs tagged `_absent`) and runs the
  check walks "absent". Walk ids `<walk>-chk-away`, `<walk>-chk-absent`;
  gonogo.py leaves every `-chk` walk out of its main-walk checks. analyse.py
  compares each kind with the matching main walk's mulcher-alone first
  detection: same step, and within one walk step (0.5 m, ~0.4 s), since the
  lidar noise (sigma 0.02 m) alone can move a repeated walk's first detection
  by about one step.
- The second-attempt pilots (dynamic lookouts) are kept in
  `runs/lookout/L2_pilot_dynamic`, `L3_pilot_dynamic`.
- Static-lookout L2 pilot, stopped after 4 of its 6 walks when the user asked
  to go straight to the full runs (`runs/lookout/L2_pilot_static_partial`).
  Both lookouts passed the settle test at 2.1 s sim, 0.0 m from their posts,
  linked; gonogo.py on those 4 walks: GO, 264 true / 0 false alarm scans
  (lk2a 111/0, lk2b 114/0, mulcher 39/0), lookouts held 0.0000 m. The same
  walks gave lk2b 162 (h120-A) and 906 (h180-B) false alarm scans with the
  dynamic Husky.
- Full runs started (L2 then L3, `runs/lookout/L2_full`, `L3_full`) without
  the pilots, at the user's request; the go/no-go checks run on the full runs'
  data instead. L2 after 8 walks: GO (599 true / 0 false alarm scans; lk2a
  222/0, lk2b 230/0, mulcher 147/0; both lookouts on their posts, linked,
  held); ~108 s wall per walk.
- L2's 48 main walks done (1.6 h). **Bug found in the first check walks
  and fixed (code, not procedure):** with only the mulcher's lidar in the
  session, `scans.step_fresh` took a scan still in flight from the previous
  step as this step's, and then did so at every step: the check walks saw the
  person where it had been one step (0.5 m) earlier. Found because the check
  walks' box counts ran ~1.5 returns below the matching main walks all along
  the path; `smoke/stale_audit.py` (the returns near the person, their offset
  along the path) then showed 239 of 241 check-walk mulcher scans stale and 0
  of 11 220 main-walk lidar-steps (mulcher and both lookouts; the several-lidar
  stamp test holds). The calibration (two lidars) shows no stale first scan
  at any of its 60 positions. Fix: a scan must also be stamped after the sim
  time at which the step began (read from the world's stats once while
  paused, then advanced by each step call); see scans.py. It cannot change a
  scan the main walks took. The 4 stale check walks (session
  20260924T035543) are moved to `runs/lookout/L2_full/stale_check_20260924T035543`
  and re-run. analyse.py now counts only the session that finished a walk in
  every log (it filtered by walk id only, so an interrupted walk's partial
  rows would have been counted too). After the fix: the first check walk
  0/109 stale; its mulcher box counts equal the main walk's at 140 of 141
  steps, best case at the same step, main rule one step later (step 120 vs
  119, at 14 m, the 3D-linkage ceiling).
- L2 check walks "away" re-run: 12 walks, 0 of 900 mulcher scans stale.
  They ran in a new sim (run_layout.sh restarted; its main walks were all
  done, so walks.py skipped them), with the lookouts spawned and then moved
  1 km away, as in L3's same-sim "away" walks; analyse.py's label now says
  "lookouts spawned, then moved 1 km away". analyse.py's "within one step"
  compared rounded times (t_s, 3 decimals) against the step length and missed
  a one-step gap; it now compares step indices (L2 away: 11/12 same step,
  12/12 within one).
  The sim restart without lookouts works (sim_up.sh NO_LOOKOUTS=1); the first
  "absent" walk: 0/109 stale, main-rule alarm at the same distance as the main
  walk (14.032 m); ~47 s wall per walk with only the mulcher's lidar rendered.
- Full runs complete (2026-09-24 09:39): L2 48 main + 12 away + 12 absent;
  L3 72 main + 12 away + 12 absent; both exit 0, no crash, 0 service retries.
  Stale audit (smoke/stale_audit.py): 0 stale in every lidar-step of both
  layouts (L2 0/12 561, L3 main 0/16 453, L3 check 0/1504).
- **gonogo.py on the full runs: NO-GO**, on one check only, "walks finished,
  every field filled": the empty fields are all in the check-walk rows and
  all in the lookouts' own columns (`lkX_*_before_post`, `lkX_*_before50`,
  `fa_lkX`, `hold_max_*_lkX`). A check walk runs without the lookouts
  (walks.py: `self.lk = []`), so those columns have nothing to hold;
  NO_DETECT_OK allows only the first-detection columns to be empty. Every
  main-walk field is filled. Every other check passes on both layouts:
  every lidar sees the person, every lookout 0.0 m from its post and linked
  and held (0.0000 m every walk), false alarms 0 of 3477 (L2) and 0 of 4581
  (L3) true alarm scans. Stopped here as the brief says; gonogo.py not
  changed. analyse.py was run (`runs/lookout/results`), provisional until
  this is settled.
- The user approved the change: gonogo.py lets a lookout's columns be empty
  on a walk with `lookouts_present` 0 (the check walks). Re-run on both full
  runs: **GO**, every check passes (`runs/lookout/gonogo_full.txt`). The
  results below are final. analyse.py gained one summary line per layout
  (the team's first alarm vs the mulcher's on the same walk, and the
  distances at both) and puts its figure legends below the panels; no
  number it already reported changed. The output is copied to
  `lookout/results/`; the write-up is `lookout/README.md`.

## Results (2026-09-24, full runs; `results/results.md`, README.md)

Main rule (the brief's rule as written); Wilson 95 % intervals, approximate.

| | L2 (48 walks) | L3 (72 walks) |
|---|---|---|
| caught before the post line: mulcher alone / team | 0 % (0-7) / 100 % (93-100) | 0 % (0-5) / 100 % (95-100) |
| caught before 50 m: mulcher alone / team | 0 % / 0 % | 0 % / 0 % |
| median warning before the post line: mulcher / team | -12.5 s / 11.0 s | -11.2 s / 10.8 s |
| team's first alarm earlier than the mulcher's, same walk (quartiles) | 20.0, 23.1, 23.8 s | 21.9, 21.9, 22.3 s |
| distance from the mulcher at the first alarm, median: mulcher / team | 13.9 / 39.4 m | 13.7 / 41.7 m |
| walks the mulcher never caught (all from its front) | 4 | 18 |
| McNemar exact, before the post line | p = 7.1e-15 | p = 4.2e-22 |
| false-alarm scans, every lidar | 0 | 0 |

Sensitivities (declared before the walks): with `xy5` or the best case the
mulcher alone catches 54-79 % before the post line (median warning 1-8 s),
the team 100 %, before 50 m too (27-29 s). Each lookout is the first catcher
on its own entry (24 walks each). Check walks: the mulcher's first alarm at
the same step in 23 of 24 per layout, within one step in 24 of 24; the
lookouts do not change what the mulcher sees. The main rule cannot fire past
~14 m (3D-linkage ceiling, Deviations), so the mulcher's 0 % before the post
line and everyone's 0 % before 50 m follow from it; the paired time gain and
the sensitivities are the fairer comparison.

## Design change (2026-09-24, user-directed, before any walk)

Why: the messenger pilots (status log) showed that a carried warning arrives
~70 s after the walker reaches the 10 m stop (Husky ~0.44 m/s, walker 1.3 m/s).
Asked for lookout positions that see further than the mulcher while keeping
the network, the agreed answer was posts inside the radio horizon.

What changed:
- **Posts:** calculated from the map (`posts.py`, next section) instead of the
  90 m circle. L2: lk2a 28 m, lk2b 24 m; L3: lk3a, lk3b, lk3c 28 m. Every post
  is linked to the mulcher.
- **Procedure:** the lookouts are spawned on their posts; no planner, nav stack,
  drive-out or trips. Only the person moves (`run_layout.sh`).
- **No messenger:** an alarm from a linked post reaches the mulcher at once.
  Delivery and trip analysis are dropped. The link is checked on each post
  (`walks.py` "at_post" event, go/no-go) and at every lookout alarm.
- **Catch lines:** "before 90 m" becomes "before the post line": where the path
  crosses its post's radius (28 m; 24 m for L2 entry B). "Before 50 m" is kept.
  The warning-time curve (T = 0-30 s) is measured to the post line.
- **Walks:** start 45 m (+-5 m) before the post line (so 67-78 m from the mulcher;
  59-69 m for L2 entry B), end 10 m from the mulcher. Headings, repeats, offsets and seeds unchanged.

Unchanged: the comparison (mulcher alone vs mulcher + lookouts, the same walks,
paired), the lidar, the detection rule and thresholds, the sensitivity variants,
the check walks, the GPU calibration, the radio model.

Kept, not used: the lookout mode's messenger (explo_planner `LOOKOUT_DELIVER`),
`trips.py`, `radio_node.py`, `run_layout_messenger.sh`.

## Lookout placement (`posts.py`)

In the application the robots explore first, so the map (trunk positions, path
centrelines) is known and the posts can be calculated from it. Here the map is
the world's trunk list with the lanes cleared. For each path entry:

1. Candidates every 0.5 m along the path, 12-28 m from the mulcher, 2 m to either
   side (along the circle through that point), facing out along the path.
2. A spot is kept only if it is **free** (no trunk centre within 2 m + 1.7 m root
   flare: the pad the builder used to clear; a robot cannot clear trees) and its
   **radio link is safe**: linked by the comms model (unchanged), at most 28 m
   out (2 m inside the 30 m horizon), and every trunk at least 0.5 m outside the
   link's Fresnel corridor.
3. **Score:** a person walks in along the centreline; the score is their
   distance from the mulcher when first within 12.5 m of the spot (the calibrated
   main-rule reach) with no trunk (radius 0.35 m) on the line of sight. Ties: the
   same at 30 m (best case), then the larger corridor clearance.
4. The highest score wins.

| post | radius, side | first seen, main / best (m from the mulcher) | nearest trunk (m) | link clearance (m) |
|---|---|---|---|---|
| lk2a | 28 m, ccw | 40.3 / 57.8 | 8.7 | 3.3 |
| lk2b | 24 m, cw | 35.7 / 51.4 | 3.8 | 2.6 |
| lk3a | 28 m, cw | 40.2 / 57.5 | 4.3 | 3.1 |
| lk3b | 28 m, cw | 40.2 / 57.6 | 7.7 | 1.3 |
| lk3c | 28 m, ccw | 40.0 / 57.7 | 10.0 | 2.8 |

lk2b is at 24 m because every spot from 24.5 to 28 m, on both sides, has a trunk
within 3.7 m (at 28 m: 2.0 m and 3.4 m). Elsewhere the cleared lanes give a long
view, so the radio limit decides: each metre further out is seen ~1 m (~0.8 s)
earlier. The "first seen" numbers are geometry only; the walks measure detection.

Limits: trunks only (no branches or canopy); the person is a point on the
centreline; the reaches are from open ground. The 28 m follows from the comms
model's 30 m horizon and 70 dB per trunk, which are deliberate stress settings
(`hmr_comms_sim_node.cpp`), not measured radio: a real link could reach further
or not as far.

## Deviations

- The lookout point comes in as three ROS parameters from a params-file YAML per
  layout, not a separate `lookout_points_file`: same content (one x, y, yaw per
  robot, from YAML), no new file parser in the node.
- Head blocking is 15 of 16 channels over +-30 deg (see the status log); reported,
  geometry unchanged.
- Smoke-test bug found and fixed before any measurement: a scan was taken as fresh
  if stamped after the last /clock value received, but /clock lags behind the clouds
  in rclpy, so stale scans passed. First smoke numbers (and a false "set_pose does
  not turn the static mulcher") came from this; all were re-measured with the
  per-lidar test in `scans.py`.
- Radio column (brief: optional link at first detection): the comms model has a
  30 m horizon, so a lookout at 90 m is never linked; the column is fixed by the
  model. Replaced by warning delivery (item 6 above), at the user's request; the
  radio model itself is unchanged. Since 2026-09-24 every post is linked, so the
  column (`linked_at_alarm_*`) should read 1 at every lookout alarm.
- **3D-linkage ceiling of the main rule (reported, rule unchanged).** VLP-16
  rings are 2 deg apart, so returns on neighbouring rings from the same body are
  ~0.035 d apart vertically; with 0.5 m single linkage in 3D no cluster spans
  two rings beyond ~14.3 m, so the rule "5 points on 2 rings" cannot fire
  further out. The calibration confirms it (12.5 m for every mount). The rule is
  kept exactly as written and is the main result. Declared before the pilot, as
  a labelled sensitivity only: `xy5`, the same rule with the 0.5 m linkage in
  the horizontal plane (`detect.clusters(..., xy=True)`), logged alongside
  (m3/m10/m20 are the brief's point-count sensitivities, all 3D).
- Renderer: llvmpipe -> NVIDIA GPU before the pilot (speed only; same ogre2
  engine, lidar settings and world). Depth precision can differ between the two,
  so the calibration is re-run on the GPU and the go/no-go uses that run.
- Trips are measured once per layout (the link depends only on positions and
  trunks) and delivery per walk is computed offline from them (item 6); the walk
  driver is unchanged, as asked. (Superseded 2026-09-24: no trips.)
- Posts and procedure changed 2026-09-24, at the user's direction, before any
  walk: see **Design change**. The walk driver changed only in its reference
  line (post line instead of 90 m) and one log event (the link on each post).
- Lookouts static (2026-09-24, after the L2 pilot failed go/no-go on lk2b's
  tip; the user asked for the cause and a fix): a parked Husky rocks in DART +
  bullet (status log), so the lookouts are spawned static on their posts at
  the settled height. Lidar, mount height, rules, thresholds and walks
  unchanged; the pilots are re-run.
- Check walks (2026-09-24, user's choice): removing a lookout crashes
  Fortress, so the check walks run twice, with the lookouts moved 1 km away in
  the same sim and in a fresh sim where they were never spawned, instead of
  once with them removed. analyse.py adds a "within one step" count beside
  "same step", because of the lidar noise.
- gonogo.py check 3 amended after the full runs (2026-09-24, user-approved):
  the lookouts' columns may be empty on the check walks, which run without
  the lookouts; it failed only on those columns (status log).
- No completed pilot for the static lookouts (2026-09-24, user's request): the
  full runs went ahead after 4 static-lookout pilot walks on L2 (GO) and none
  on L3; gonogo.py is applied to the full runs' data. The check walks ("away",
  "absent") first run inside the full runs.
