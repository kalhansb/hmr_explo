# Experiment Plan — Multi-Robot Forest Inspection

Physical trials of the coupled exploration/exploitation experiment
(*Experiment Plan: Multi-Robot Multispectral Forest Inspection*) on this
workspace's stack.

- **Under test:** `explo_planner` (exploration + exploitation goal selection,
  role switching) and scovox + dscovox distributed mapping.

All coordinates below are in the `map` frame of `gt_map_us050.pcd`.

## 1. Robots

All robots **explore and exploit** (`exploitation_enabled: true` on every
planner). Only `curt` carries the multispectral payload. Batteries last
**one hour**, which caps every run (§7). The campaign runs in two phases:
**phase 1 fields `bunker` + `curt` only**; `go1` joins in phase 2.

| Robot | Sensors | Capture at vantage dwells |
| --- | --- | --- |
| `curt` | Ouster LiDAR, IMU, RGB-D, multispectral | multispectral + RGB-D + LiDAR |
| `go1` | Hesai LiDAR, IMU, RGB-D | RGB-D + LiDAR |
| `bunker` | Hesai LiDAR, IMU, RGB-D (seg input) | RGB-D + LiDAR |

Crew: Director (cues, launches), Safety Officer (e-stops, panic call),
Wrangler (robot handling, batteries, calibration panel).

## 2. Area of Operations

The plantation rows run at **24.8°** to the map x-axis (hedge line fitted from
the gt map). AO and sub-areas are defined in the **row frame**:
`u = x·cos θ + y·sin θ`, `v = −x·sin θ + y·cos θ`, θ = 24.82°.

**AO = grid cells I8–O10** (see `doc/ao_topdown_grid.png`): a **140 × 60 m**
rectangle, u ∈ [−40, 100] × v ∈ [−24.16, 35.84], southern long edge on the
hedge line. Map-frame corners: (−51.3, 15.7), (−26.2, −38.7), (100.9, 20.1),
(75.7, 74.5). Ground falls **~14 m** along the AO, from +5.3 (west end, I8) to
−8.6 (east end, O9).

Sub-areas for Type B split the long axis at grid lines u = 20 and u = 60
(trunk counts nearly equal):

| Sub-area | Cells | u range | Size | Trunks | Ground z | Targets |
| --- | --- | --- | --- | --- | --- | --- |
| SA-1 | I8–K10 | [−40, 20] | 60 × 60 m | 15 | −1.2 … +5.3 | T01–T03 |
| SA-2 | L8–M10 | [20, 60] | 40 × 60 m | 15 | −8.2 … +2.1 | T04–T06 |
| SA-3 | N8–O10 | [60, 100] | 40 × 60 m | 14 | −8.6 … −2.0 | T07–T09 |

**Phase 1 covers SA-1 + SA-2 only** (cells I8–M10, u ∈ [−40, 60],
100 × 60 m); SA-3 joins in phase 2 with `go1`, scoped from phase-1 measured
times.

Deployment point: map origin (0, 0) — inside the AO (in SA-1, 20 m from its
eastern edge); the verified near-origin NDT seed still applies.

The planner ROI (`roi_min/max_x/y`) is **axis-aligned**, so the rotated AO
needs either `roi_yaw` support in `explo_planner` (open item 2) or the
bounding-box fallback x ∈ [−51.3, 100.9] × y ∈ [−38.7, 74.5] with out-of-AO
cells painted lethal in the GT costmap.

## 3. Exploitation targets

44 trunk candidates sit inside the AO; 9 are targets — **one per robot per
sub-area** (per-robot target topics). T01 and T04 are already verified in
`targets_map_test2.yaml` and ride with `curt` (the multispectral robot); the
rest have cluster-fit radii (upper bounds — tape DBH on site → final radii).

| ID | x | y | ground z | radius | Cell | Sub-area | Robot |
| --- | --- | --- | --- | --- | --- | --- | --- |
| T01 | 5.43 | −8.10 | −1.70 | 0.25 | K10 | SA-1 | curt |
| T02 | −17.46 | 24.37 | +3.26 | ~0.7 | J8 | SA-1 | bunker |
| T03 | −18.32 | −19.75 | +2.42 | ~0.9 | I10 | SA-1 | go1 |
| T04 | 15.93 | 16.57 | −2.65 | 0.27 | L9 | SA-2 | curt |
| T05 | 41.82 | 26.24 | −4.31 | ~0.7 | M9 | SA-2 | bunker |
| T06 | 54.82 | 3.54 | −8.21 | ~1.1 | M10 | SA-2 | go1 |
| T07 | 83.46 | 34.88 | −8.56 | ~0.7 | O9 | SA-3 | curt |
| T08 | 70.47 | 22.23 | −8.56 | ~1.1 | N10 | SA-3 | bunker |
| T09 | 68.88 | 37.83 | −6.67 | ~0.4 | N9 | SA-3 | go1 |

Phase 1 uses T01–T06 with go1's trees reassigned (T03 → bunker, T06 → curt,
three targets per robot); phase 2 restores go1's assignments and adds T07–T09.

Caveat: exploration candidate z (`candidate_robot_z`, ROI z band) is absolute
today; over the AO's ~14 m relief that is unsound and needs the
terrain-relative fix before field day (open item 4). Vantage z follows the
per-target ground z above, so exploitation is unaffected.

## 4. Runs

**Phase 1** — `bunker` + `curt`, SA-1 + SA-2, targets T01–T06:

| Run | Type | Exploration | Targets released |
| --- | --- | --- | --- |
| RS | Shakedown (field) | SA-1 | T01 |
| RA-1…3 | **Type A: explore all, then exploit** | SA-1 + SA-2 | T01–T06 after coverage-done |
| RB-1…3 | **Type B: per-sub-area interleaved** | SA-1 → SA-2 | per sub-area at its coverage-done |

**Phase 2** — all three robots, full AO (adds SA-3 and T07–T09); run count
and time budgets set from phase-1 measured rates.

The campaign is **two field days: day 1 = phase 1, day 2 = phase 2**.
Day 1 is RS + 6 runs plus turnarounds — tight; if time runs short, drop the
third repetitions (RA-3/RB-3 are the stretch runs) rather than rushing
setup checks.

RA vs RB is the *Dynamic vs Static Role Efficiency* comparison — compare only
within a phase (fleet size differs across phases). Repetitions → mean ±
spread.

## 5. Per-run setup

**Starting poses** (map frame; also the NDT seeds; mark them physically):

| Robot | x | y | yaw |
| --- | --- | --- | --- |
| curt | 0.0 | +3.0 | 25° |
| bunker | 0.0 | 0.0 | 25° |
| go1 (phase 2) | 0.0 | −3.0 | 25° |

Yaw 25° faces the robots down-row (+u), into the AO's long axis.

**Parameters at onset** (`exploration_params.yaml`; log the config git SHA):

| Parameter | Value |
| --- | --- |
| `use_sim_time` | false (live) |
| ROI x/y | AO or current sub-area, row frame (§2; needs `roi_yaw`, open item 2) |
| ROI z | −10 … +7 (interim, covers AO relief; sync scovox/dscovox `share_roi_z_*`; open item 4) |
| `coordination_enabled` | true |
| `exploitation_enabled` | true (all robots) |
| `targets_topic` | `/exploration/targets/<robot>` |
| `n_vantages` / `min_vantages_required` | 3 / 3 |
| `vantage_standoff_m` / `vantage_start_angle_deg` | 2.0 / 30 |
| `exploit_dwell_sec` | 8.0 |
| `done_unknown_fraction` | 0.05 (×3 steps) — re-calibrate at RS: the scovox column measure plateaus differently than the old 2D one (open item 5) |
| `done_coverage_source` | auto → scovox 2.5D column coverage of the fused dscovox map (no planning_map in this setup) |
| `done_action` | idle — planner stays up at DONE and exploits targets released at the cue (field runs never use shutdown) |
| `candidate_enable_polar` | true at RS — then check goal spacing: if goals bunch near the robot, go frontier-only (false) for the timed runs (open item 9) |
| `max_steps` | 200 |
| `require_planning_map` | false (no planning_map; Nav2 GT costmap owns feasibility) |
| `output_csv` | `/tmp/<run_id>_<robot>.csv` |
| NDT map | `gt_map_us050.pcd`, per-robot configs |

Clock sync: no base station — chrony over the mesh with `bunker` as time
master; offset < 50 ms on the other robots (gate; log it).

## 6. Run procedure

1. Stage robots on marked poses; verify chrony, disk ≥ 250 GB/robot.
2. Launch: NDT (all active) → scovox + dscovox merger per robot → seg (bunker)
   → Nav2 with GT costmap per robot. Check `/pcl_pose` within 0.5 m of stage.
3. Multispectral calibration panel capture + lighting log (abort if lighting
   outside agreed envelope). E-stop test. Start all recordings.
4. **T0:** launch the planners (two in phase 1) → all EXPLORE.
5. **Coverage-done cue** (planner logs `ROI unknown fraction … source=scovox`,
   then enters DONE-idle): Director launches the target scheduler(s) —
   Type A: all lists at once; Type B: current sub-area's list, then move
   planners to the next sub-area as robots free up. With `done_action: idle`
   the planners stay up at DONE, so the release cannot race a shutdown.
6. Robots with targets switch EXPLORE → EXPLOIT (3 vantages/tree, 8 s dwells),
   revert on empty queue.
7. End when all targets closed and exploration DONE. Stop planners, robots
   home under RC, stop recordings last, `docker stop` containers (never pkill).
8. Offload: bags (`ros2 bag info` check), planner CSVs, multispectral SD,
   closing calibration panel, run sheet (switch counts, anomalies, batteries).

## 7. Expected switches and durations

**One-hour batteries hard-cap every run at 55 min** (5 min reserve to RC
home). Phase-1 estimates below are rough — treat RS and RA-1 as the timing
calibration and rescale the rest of the campaign (and phase-2 scope) from the
measured coverage rate.

| Run (phase 1) | Expected EXPLORE↔EXPLOIT switches | Duration (expected) | Hard cap |
| --- | --- | --- | --- |
| RS | 2 (curt) | ~25 min | 45 min |
| Type A | 2 per robot (4 total) | 40–60 explore + ~15 parallel exploit | 55 min |
| Type B | 2 per robot per sub-area (8 total) | ~50 min (2 scenes × ~25 min) | 55 min |

If Type A hasn't cued coverage-done by 40 min, the Director releases the
targets anyway — partial-coverage run, note it on the sheet. If explore rates
run long, the first lever is the frontier-only candidate switch (open item 9),
then shrinking the per-run area.

More switches than expected = partial-target retries: note on run sheet, don't
abort. Battery swaps between runs only — a full day is up to 7 runs per
robot, so bring enough charged packs (or field charging) to sustain that.

## 8. Panic stop

Anyone calls "PANIC STOP" → Safety Officer e-stops all robots → Director stops
planners, then containers (`docker stop`). Recording is stopped last.

Triggers (any one):

1. Geofence: any robot > 5 m outside the AO polygon — row frame u ∉ [−45, 105]
   or v ∉ [−29.2, 40.8] (Director watches `pcl_pose` in RViz against the AO
   overlay, §2).
2. Robot–robot < 1.5 m closing, person within 1 m, or contact with a tree.
3. Localisation divergence: pose jump > 2 m, or footprint visibly off the map
   (one robot diverging → stop that robot; two or more → full stop).
4. Runaway nav: ≥ 3 blacklist cycles on the same spot or > 3 min oscillation.
5. Platform health: battery < 20 %, motor fault, tilt > 20°.
6. Mesh comms loss to any robot > 15 s.
7. Any recording stops growing.
8. Person/animal in plot, rain, or lighting leaves the multispectral envelope.

Classify after: **resumable** (brief single-robot hiccup, map intact) vs
**void** (divergence, comms > 60 s, environment breach → rerun).

## 9. Data collection and metrics

| Device | Records |
| --- | --- |
| Each robot's onboard PC | rosbag2 of its own raw sensors (LiDAR, IMU, RGB-D) + planner/mapping topics (raw data never crosses the mesh) |
| bunker additionally | seg output |
| curt additionally | multispectral trigger/meta |
| Multispectral camera | full-band frames to its own SD (synced via trigger timestamps) |
| Run sheet | config SHA, chrony offsets, lighting log, taped DBH, switch counts, anomalies |

No base station: every topic lives in the per-robot bags (§10) and planner
CSVs stay on each robot until the §6.8 offload.

**Metrics** (all computable offline from bags + CSVs):

| Metric | From |
| --- | --- |
| Per-robot mapping contribution (FBM) | `scovox_bin` streams — % of final-map voxels first observed per robot; dips while exploiting = coupling cost |
| Map synchronisation latency (FBM) | `scovox_bin` publish vs receive stamps vs next plan tick (CSV + `/rosout`) |
| Vantage point validity (FBM) | planner CSV (`n_vantages_valid`, `vantage_los_clear`) + offline re-raytrace in final map |
| Dynamic vs static efficiency (TBM) | RA vs RB: time from first release to 100 % target coverage |
| Coverage completeness (TBM) | % targets with ≥ 3 clear-LoS dwells + offline image/scan QA per assigned sensor |
| Sim2Real gap (TBM) | existing map-test-2 bag-replay results vs field: map RMSE vs `gt_map.ply`, vantage validity, coverage deltas |
| Supporting | NDT health (`/diagnostics`), path length (TF), per-target time/path, `scovox_bin` bandwidth |

## 10. ROS topics to record

Per robot `<r>` ∈ {curt, bunker, go1}, on its own PC (`--compression-format zstd`):

| Topic | Why |
| --- | --- |
| `/<r>/ouster/points` or `/<r>/hesai/points` | **raw LiDAR — a-posteriori replay backbone** |
| `/<r>/imu/data` | raw IMU |
| `/<r>/camera/color/image_raw/compressed`, `/<r>/camera/aligned_depth_to_color/image_raw` + both `camera_info` | **raw RGB-D — every robot** |
| `/<r>/odom` | platform odometry |
| `/<r>/pcl_pose` | NDT pose |
| `/tf`, `/tf_static` | pose chains, trajectories |
| `/<r>/scovox_node/scovox_bin` | per-robot map deltas (map growth + provenance) |
| `/<r>/dscovox_node/scovox` (throttled ~0.2 Hz) | merged view behind each decision |
| `/<r>/goal_pose`, `/<r>/cmd_vel` | commands in/out of Nav2 |
| Nav2 path + goal status topics (per bringup) | nav outcome forensics |
| `/exploration/targets/<r>` | stimulus timeline |
| `/exploration/intents` | MinPos claims — coordination forensics (shared topic, on every robot's bag) |
| `/<r>/explo_planner/candidates` | candidate/vantage viz for figures |
| `/diagnostics`, `/rosout` | NDT health; planner state transitions |
| bunker only: `/scovox/segmentation/colored` | seg replay |
| curt only: multispectral trigger topic | payload frame ↔ bag time alignment |

No base station: the three per-robot bags are the only recordings, so their
union must be complete — the shared topics (`/exploration/intents`, each
robot's `scovox_bin` as received) appear redundantly across bags, which is the
intended fallback if one robot's bag is lost.

## 11. Open items before field day

1. Nav2 bringup with the GT costmap on all three platforms + `goal_pose` →
   `NavigateToPose` bridge.
2. Rotated ROI: add `roi_yaw` to `explo_planner` (preferred), or run the
   bounding-box ROI with out-of-AO cells lethal in the GT costmap and accept
   wasted candidates + a manual coverage cue (§2).
3. Namespace all topics per robot — `bunker` and `go1` both default to
   `/hesai/points` and `/imu/data` (collision when run together).
4. Terrain-relative candidate z — **required**: the AO spans ~14 m of relief,
   so absolute `candidate_robot_z` / ROI z clamping misplaces exploration
   candidates and their EIG raycasts. Interim widened z band per §5.
5. Coverage-done trigger — **resolved in `explo_planner`**:
   `done_coverage_source: auto` falls back to a 2.5D column-coverage measure
   on the fused dscovox map when no planning_map exists
   (`MapCache::unknownColumnFraction`), and `done_action: idle` keeps
   planners alive at DONE so the release cue can't race a shutdown.
   Residual: calibrate `done_unknown_fraction` at RS (watch where the logged
   `source=scovox` fraction plateaus). NB the measure shares the planner's
   axis-aligned ROI box — under item 2's bounding-box fallback, never-visited
   out-of-AO columns put a permanent floor under the fraction, so either
   calibrate above that floor or land `roi_yaw` first.
6. Validate per-robot target-topic remap end-to-end at RS (no cross-robot
   target claiming exists; redundant shared-topic mode would also need
   `coord_claim_radius_m` retuned to ~2 m).
7. Multispectral driver: trigger topic + SD sync procedure.
8. Tape DBH of T02–T03 and T05–T09 (T01/T04 verified); update target YAMLs.
   Set NDT-fitness panic threshold from RS.
9. Exploration-goal spacing check (RS): watch whether selected goals sit far
   apart or bunch near the robot — the polar candidate ring
   (`candidate_enable_polar`, 96 near-robot samples) can outscore frontier
   goals and burn battery on short hops. If so, run frontier-only
   (`candidate_enable_polar: false`) for the timed runs.
