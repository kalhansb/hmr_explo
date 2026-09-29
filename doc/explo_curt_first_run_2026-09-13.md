# explo_planner + SCovox on the Jetson Orin — native run over curtmini_jetson

2026-09-13. No Docker (socket unreachable for this user), no `map-test-2` bag.

## What was actually run

The repo's own runners could not be used:

- `run_explo_experiment.sh` needs bag `2026_06_19_18_19_06__kalhan-map-test-2_`;
  `run_explo_curtmini.sh` needs `2026_07_31_11_24_12__kalhan_2_CURTMINI`. Neither
  is on this host (only the two localisation bags), and the README puts the bag
  set at ~54 GB against 8 GB free.
- Both are `docker compose` based.

So `run_explo_experiment.sh`'s topology was rebuilt natively against the bag that
is here. That script's bag "carries raw sensors only" and it synthesises the TF
tree with an NDT localizer — which is exactly what `curtmini_jetson` is, and
`gt_map_us050.pcd` is present. The extrinsics it hardcodes
(`base_link->os_lidar` 0.1105/0/0.404 yaw 180, `base_link->imu` 0.062/0/0.015
yaw 90) are bit-for-bit what curtmini's own `/tf_static` carries, so it is the
same rig at a different site.

```
bag -> NDT localizer (map->odom) + static odom->base_link_curt
    -> scovox_node (rolling mapper)  -> scovox_bin
    -> dscovox_node (merger)         -> /robot1/dscovox_node/scovox
    -> explo_planner
```

Script: `run_explo_native.sh` (a one-off, not kept; `ws/src/run_explo_pinned.sh` supersedes it). Outputs in `~/explo-output/` on the Jetson.

**Open loop by design**, per the repo's own `dscovox_exploration_run.md`:
"Nothing navigates to the goal — the robot replays the recorded trajectory, so
each NAVIGATE leg ends on the watchdog and the planner replans from wherever the
bag walk is." This exercises candidate generation, EIG scoring and goal
selection. It cannot judge whether the goals are *good*.

## Blocker: explo_planner does not build against scovox main

```
scovox_msgs/msg/refinement_region.hpp: No such file or directory
```

explo_planner main (`66c28cc`, 2026-09-07) requires `scovox_msgs/RefinementRegion`;
scovox main (`34b854f`, 2026-07-31) does not ship it, and scovox main *is* the
pinned submodule — so this is not a stale pin. The planner's own comment points
at "scovox_msgs/msg/RefinementRegion.msg", a file that does not exist. **This
affects the Docker path too**; it is not native-only.

Worked around by reconstructing the message from its only writer
(`publishRefinementRegion`: `id, x, y, base_z, radius, remove`) plus the stated
"field-compatible with TreeTarget" constraint, which fixes `uint32 id` and
`float32 radius`. Marked as a local reconstruction in both the `.msg` and the
`CMakeLists.txt`.

**There is no consumer** — scovox has no subscriber for that topic at this
commit, so the planner publishes into the void. Harmless for exploration, which
never enters the exploitation path that emits these. Do not trust this
workaround for exploitation work; get the real definition from scovox's owner.

Also required: `nav2_msgs` (hard dependency, absent here) — fetched with
`apt-get download` + `dpkg -x` into `~/ros-extra`, no root.

## The merger is not optional

`explo_planner` subscribes to the map `TRANSIENT_LOCAL`; `scovox_node` publishes
`~/scovox` `VOLATILE`, so they never match and the planner sits at `map=0`
forever. Only `dscovox_node` publishes with matching QoS. The committed
`exploration_fused_bag.yaml` default (`/scovox_node/scovox`) therefore cannot
deliver a map — a latent repo bug, already documented in
`run_explo_curtmini.sh`'s header. `dscovox_topic` is overridden here.

## Result: the planner works, the mapper cannot keep up

Two runs, both 120 s, both produced 5 plan steps and zero all-rejected ticks.

| | value |
|---|---|
| plan steps | 5 |
| candidates | 190 -> 321 (96 radial + 94-225 frontier centroids) |
| plan time | 250 -> 750 ms, growing with map size |
| rejections | 0 (close/map/unreach/blk/minpos all ~0) |
| observed voxels | 354 k -> 2.09 M |
| frontier voxels | 338 k -> 1.91 M |
| mean EIG | 0.069 -> 0.038 |
| **planner CPU** | **~1.08 cores peak** (p99 1.07, max 1.52), bursts to 1.6 s; median 0.00. RSS **1038-1829 MB** |

### The planner's average CPU is not a real number — do not quote it

An earlier version of this doc said "0.17 cores avg, p95 0.45". Both were
artefacts of a 2 s sampling interval against a 250-750 ms burst, which smears it
by ~3x. Re-sampled at 0.1 s (1185 samples):

```
median 0.00   p95 0.99   p99 1.07   MAX 1.52
123/1185 samples > 0.8 cores, longest burst ~1.6 s
```

The planner is effectively **single-threaded and saturates a core while
working** (~1.08), and idle the rest of the time. The *mean* only measures this
harness's open-loop duty cycle -- 15-27 s idle per cycle on a navigation
watchdog that can never succeed -- not the planner's cost. In a closed loop,
replanning fires far more often and the average would be much higher.

Bursts (1.6 s) run longer than the self-reported `plan_time_ms` (275-475 ms), so
they include map ingest, not just the plan computation.

### Reproducibility is poor here

Unlike the GLIM/localizer work (+/-1-2% across 3 runs), two explo runs differed
substantially: planner RSS 1038 vs 1829 MB, scovox RSS 725 vs 985 MB, step-1
voxels 0.67 M vs 1.16 M. Single runs in this pipeline carry real uncertainty.

| **scovox CPU** | **0.81 cores** avg, but **1.03 cores whenever it integrates** — see below. Peak RSS **725 MB** |

### Nothing here was pinned

Unlike the GLIM/localizer work, this run used **no `taskset`** — both nodes had
all 12 cores. `run_explo_native.sh` contains no `PIN_CORES`/`taskset`.

### SCovox is single-threaded and already saturating one core

The 0.81 average is diluted by idle gaps. The instantaneous series (2 s samples)
sits at **1.03 cores with median 1.02, and 41 of 67 samples above 0.95**:

```
0.00 0.02 0.11 1.04 1.03 1.03 1.03 1.03 1.03 1.03 1.01 0.06 0.07 0.52 1.03 ...
```

So scovox pegs one core whenever it has a frame to integrate. The gaps are not
spare capacity: **`gated=267` of `recv=312`** — 86% of received clouds are
rejected by the rolling-map gate before integration, so the node has nothing to
do in between.

That sharpens the conclusion. SCovox is not 18x short of budget with headroom to
tune; it is *already at one core* and would need roughly 18x more throughput. At
current per-frame efficiency that is ~18 cores, so this needs an algorithmic fix
(resolution, range, carving, or the TSDF stage), not more parallelism.

### Measurement method

CPU is `/proc/<pid>/stat` fields 14+15 (utime+stime) sampled every 2 s — the same
source `htop`/`top` read. Validated against an independent tool on a known
single-thread load pinned with `taskset -c 3`: `/proc` reported **1.000 cores**,
`top -b` **0.992**. (`htop` is not installed on this host and has no batch mode,
so it cannot record; `top -b` is the scriptable equivalent.)

### THREE CORRECTIONS — earlier revisions of this doc were wrong

**C1. "TSDF is 80% of frame time" is a MISLABEL. TSDF is switched off.**
The node logs `TSDF: sdf_trunc=0.000 m space_carving=0 band_only=0 (sdf_trunc=0
means off)` and `SCovox ready … fused_walker=1`. With `fused_walker=1`,
`scovox_map_split.hpp:552-565` documents `tsdf_ms` as the **combined
TSDF+semantic integration time covering the ray walks plus the carve flush**. So
the 1479 ms is the **fused ray-walker (occupancy carving)**, not TSDF, and the
earlier recommendation to "fix the TSDF stage" pointed at a stage that is not
running. This also means the prior profiling that fingered **full-ray carving**
as scovox's hot path was right and this doc previously contradicted it in error.

**C2. The gate is a LOCALIZATION-DIVERGENCE gate, not routine keyframing.**
The log carries 8 × `Runtime TF jump N m > 1.00 m — localization diverged;
pausing integration until pose re-stabilizes`, with jumps of **1.48, 2.22, 6.06,
2.11 m**. The planner independently logs 8 × `Pose jump … exceeds
max_pose_jump_m=1.00` in the same run. So the NDT localizer **diverged eight
times during the run**, and scovox correctly paused integration each time.
Earlier revisions described this as the "rolling-map gate" and concluded "the
node has nothing to do in between" — presenting a localization failure as normal
throughput behaviour. Every number in this document was collected on a diverging
localizer and should be treated as such.

**C3. Most of the map starvation happens at DDS, before the gate.**
scovox received `recv=312` clouds; **GLIM on the same bag and window received
1153**. The cloud QoS is `keep_last / depth 5 / best_effort` and scovox blocks
~1.85 s per callback, so ~73% of clouds are discarded by DDS before the gate
ever sees them. The gate then rejects 86% of the remainder. Net ~45 of ~1155
scans (3.9%) — but the dominant loss is the DDS drop, not the gate.

**SCovox stage breakdown** (`tsdf_ms` relabelled per C1):

| scovox stage | mean over 44 logged frames |
|---|---|
| frame | **1855 ms** (budget 100 ms at 10 Hz) |
| integrate | 1834 ms (99% of frame) |
| TSDF | 1479 ms (80% of frame) |
| tf | 21.6 ms |
| downsample | 131072 -> 59836 points (46%) |

**SCovox runs 18.5x over its real-time budget** at `resolution 0.10`,
`max_range 20`, `carve_band -1.0` (full-ray). It integrates at ~0.54 Hz against a
10 Hz input, so the planner is scoring a map roughly 18x behind the robot. The
goals it produces are still structurally sensible, but they are computed on a
stale map — which in a closed loop would matter a great deal.

TSDF dominates at 80% of frame time in this configuration.

## Caveats

- Different site from the campaign bags; curtmini is a localisation bag, not an
  exploration one, and the "robot walk" is a 20 m localisation trajectory.
- 120 s window. Plan time was still growing at the end (250 -> 750 ms), so it has
  not plateaued — a longer run would cost more per step.
- Open loop: goal *quality* is untested.
- Memory is heavy: planner 1038 MB + scovox 725 MB + localizer ~210 MB ~= 2 GB
  for the stack.
- `RefinementRegion` is a local reconstruction (above).
