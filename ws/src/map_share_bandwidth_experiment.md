# SCovox Multi-Robot Map-Sharing Bandwidth Experiment Plan

Measuring the inter-robot uplink of the SCovox stack: `scovox_node` (mode=`rolling`) senders → `dscovox_node` merger, over the `<robot_ns>/scovox_node/scovox_bin` `ScovoxMapBinary` delta stream.

> **Verification note (checked against the tree 2026-07-02):** `scovox_multi_robot.launch.py`, `fastdds_shm.xml`, and executable `dscovox_mapping_node` all exist; `mode` code-default is `rolling` (`scovox_node.cpp:259`) but the shipped fused config overrides to `persistent` (`run_fused_experiment.sh:189`); shipped `downsample_voxel_size:=0.1` (`run_fused_experiment.sh:192`). Baseline `ds` below is set to **0.1** to match the shipped config.

---

## 1. Objective & Hypotheses

**Objective.** Quantify, in bytes/s per robot and aggregated at the merger, the true inter-robot bandwidth of SCovox map sharing under the shipped wire format (envelope `version=5`, blob codec `FORMAT_VERSION=5`, `K_TOP=2`, `share_tsdf=false`), and characterize how that bandwidth scales with the map-sharing knobs and with fleet size `N`.

**What "bandwidth" means here.** The only bytes a robot-to-robot radio would carry are the LZ4-compressed `ScovoxMapBinary.data` deltas on `/robotK/scovox_node/scovox_bin`. Everything else (`~/scovox` full map, `~/pointcloud`, `~/planning_map`, `/dscovox/pointcloud`) is a *local* viz/planning output and must never be counted as link bandwidth.

**Hypotheses.**

| # | Hypothesis | Expected scaling |
|---|-----------|------------------|
| H1 | Steady-state uplink is proportional to per-scan **map change** (touched voxels), not total map size. | flat-ish after warm-up; spikes on novel geometry |
| H2 | Aggregate merger bandwidth scales ~linearly in `N` for **independent** coverage; and near-exactly `= N ×` single-robot for the **duplicate-source** upper bound. | `BW_agg ≈ N · BW_1` |
| H3 | `resolution` is the dominant knob: touched-voxel count scales ~1/res² (surface) to ~1/res³ (carved free space); halving res multiplies deltas 4–8×. | super-linear in 1/res |
| H4 | `downsample_voxel_size` and `carve_band` cut the Beta free-space stream hardest (rays collapsed / free volume not marked). | large, sub-linear |
| H5 | `share_tsdf=true` ≈ doubles the geometry payload (adds a 20 B/voxel TSDF delta on top of Beta). | ~2× geometry stream |
| H6 | `K_TOP` adds 6 B/dir-voxel per +1 (compile-locked); `num_classes`/`alpha_0` are header-only → **negligible** on bandwidth. | +6 B·Ndir per K_TOP step; ~0 for classes |
| H7 | The delta stream is **1–2 orders of magnitude** smaller than the full `ScovoxMap` snapshot comparator and **>2 orders** smaller than raw `PointCloud2` "ship the cloud". | delta ≪ snapshot ≪ cloud |

---

## 2. What Exactly Is Being Shared, and What Is Measured

**The shared payload.** `scovox_msgs/msg/ScovoxMapBinary` published on `~/scovox_bin` → resolves to `/robotK/scovox_node/scovox_bin`. Fields: `std_msgs/Header` + `uint8 version` + `bool little_endian` + `geometry_msgs/Transform map_from_source` + `uint8[] data`, where `map_from_source` is the producer's `map ← header.frame_id` pose (carried so the merger needs no TF; fixed 56 B on the wire) and `data` is an **LZ4-compressed BinarySerializer frame** carrying only the voxels touched since the last publish (`drainTouchedBeta` / `drainTouchedDir`). `share_tsdf=false` → TSDF is **not** on the wire.

**Two regimes to separate cleanly:**

1. **Steady-state DELTA bytes/s** — the number we want. Each integration cycle emits only touched (non-prior) Beta + Dir voxels (`scovox_node.cpp:1517-1556`, at-prior gates at `:1521-1526`/`:1541-1547`). This is proportional to per-scan map change.
2. **One-time NEW-SUBSCRIBER full snapshot** — when a subscriber first connects (`snapshot = cur_sub > prev_sub_count_`, `scovox_node.cpp:1480`), a **full `forEachCell` snapshot** is sent once. Attaching `ros2 topic bw` is itself a new subscriber → it triggers one snapshot spike. **Exclude the first message from steady-state averages; report it separately as "cold-start transfer".**

**Critical gates (both must be satisfied or there is zero traffic):**
- `mode:=rolling` — `bin_pub_` is only created in rolling mode (`scovox_node.cpp:526-536`). The shipped configs use `mode:=persistent` → **no binary publisher at all** → nothing to measure.
- **A subscriber must exist before deltas start** — with zero subs, touched sets are drained and nothing is published (`scovox_node.cpp:1479-1487`). Start the merger (or the `ros2 topic bw` probe) **first**.

---

## 3. Analytical Bandwidth Model (from the byte budget)

**Per-frame uncompressed bytes** (before LZ4), from the authoritative budget:

| Component | Formula | At defaults (K_TOP=2) |
|-----------|---------|-----------------------|
| Fixed header | MAGIC(4)+VER(1)+resolution(4)+num_classes(2)+K_TOP(1)+alpha_0(4)+tsdf_count(4)+beta_count(4)+dir_count(4) | **28 B/frame** |
| Per Beta voxel | coord i32×3 (12) + a_occ f32 (4) + a_free f32 (4) | **20 B** |
| Per Dir voxel | coord (12) + other f32 (4) + cnt f32×K_TOP (4·K) + cls u16×K_TOP (2·K) = 12 + 4 + 6·K_TOP | **28 B** @K=2 |
| Per TSDF voxel (only if `share_tsdf=true`) | 20 B | 0 (default off) |

**Uncompressed frame size:**

```
S_raw(frame) = 28 + 20·N_beta + 28·N_dir  (+ 20·N_tsdf if share_tsdf)   [bytes]
```

**On-wire per-robot bandwidth:**

```
BW_robot ≈ r_LZ4 · S_raw(frame) · f_pub  +  E_env · f_pub
BW_agg   ≈ Σ_k BW_robot,k   (≈ N · BW_1 for duplicate source)
```

where:
- `f_pub` = actual emit cadence ≈ once **per integrated scan** (bounded by sensor rate; `scovox_publish_rate` drives the *viz* `sm_timer`, NOT `scovox_bin`).
- `r_LZ4` = **empirically unknown compression ratio** (0<r≤1). Voxel coords in a scan are spatially clustered → LZ4 should do well on the 12-byte coord triples; **measure it, do not assume.** Report `r_LZ4 = mean(serialized data.size) / S_raw`.
- `E_env` = ROS2 envelope + RTPS/CDR framing per message: `Header` (stamp 8 B + `frame_id` string) + `version` u8 + `little_endian` bool + `map_from_source` (7×float64 = 56 B, fixed) + `data` length prefix (4 B) + CDR alignment + RMW overhead. Order ~tens–hundreds of bytes/msg; only material at low `f_pub`/tiny deltas — and `map_from_source` adds a constant 56 B to every message, so it matters most in that same low-`f_pub`/tiny-delta regime.

**Worked example (order-of-magnitude, to be replaced by measured numbers).**
Assume one scan touches `N_beta = 4000` occupancy voxels (surface + full-ray carved free space at res 0.10 m) and `N_dir = 800` semantic voxels:

```
S_raw = 28 + 20·4000 + 28·800 = 28 + 80 000 + 22 400 = 102 428 B ≈ 100 KB/frame
```

At `f_pub = 1 Hz` and an assumed `r_LZ4 = 0.35`:

```
BW_1 ≈ 0.35 · 100 KB · 1 Hz ≈ 35 KB/s  per robot
BW_agg(N=2) ≈ 70 KB/s ;  BW_agg(N=3) ≈ 105 KB/s
```

Cold-start snapshot (full map, say 300 K Beta + 60 K Dir voxels): `S_raw ≈ 28 + 20·300k + 28·60k ≈ 7.7 MB` → after LZ4 (~0.35) ≈ **2.7 MB one-off** per new subscriber. **These are placeholders; the experiment replaces every `N_beta`, `N_dir`, `r_LZ4`, `f_pub` with measured values.**

---

## 4. Variables

**Independent (swept):** `mode` (gate, always rolling for measurement), `resolution`, `downsample_voxel_size`, `carve_band`, `max_range`, `share_tsdf`, `K_TOP` (compile-locked → needs rebuild), `num_classes`/`dirichlet_prior` (header-only control check), `N` robots (1→2→3), and comparator arm (delta vs full snapshot vs raw cloud).

**Dependent (measured):** per-robot bytes/s (`ros2 topic bw`), aggregate bytes/s (Σ), message cadence (`ros2 topic hz`), per-message serialized size distribution, LZ4 ratio `r_LZ4`, cold-start snapshot size, and — for the Pareto — map quality (APE vs `gt_map_us050`, semantic accuracy).

**Controlled (held fixed, MUST be fleet-consistent):** single bag `/ws/bags/2026_06_19_18_19_06__kalhan-map-test-2_`; the same absolute sensor topics (`/ouster/points`, `/scovox/depth/image_raw`, `/scovox/depth/camera_info`, `/scovox/segmentation/colored`, `/imu/data`); `rmw_fastrtps_cpp`; `ROS_DOMAIN_ID=0`; QoS KeepLast(50) RELIABLE; **c-slam disabled** (static `map→r1_map/r2_map` identity TFs); homogeneous little-endian; **identical `K_TOP`, `num_classes`, `alpha_0`, Beta prior, codec rev 5, envelope v4** across the fleet (else frames dropped/corrupted per `dscovox_node.cpp:298-321`); one docker image for all mappers + merger.

---

## 5. Experimental Conditions / Matrix

All arms run `mode:=rolling`. Baseline (A0) is the shipped fused LiDAR+RGB-D config at defaults but rolling.

### 5a. Delta-stream sweeps (the map-sharing channel)

| Arm | Knob varied | Values | Others held at baseline | Expect |
|-----|-------------|--------|-------------------------|--------|
| A0 | baseline | res 0.10, ds 0.1, carve −1.0, max_range per config, K_TOP 2, classes 14, tsdf off, N=1 | — | reference BW_1 |
| B1 | `resolution` | 0.05, 0.10, 0.20, 0.35, 0.50 m | fleet-wide identical | super-linear ↑ as res↓ (H3) |
| B2 | `downsample_voxel_size` | 0.0, 0.1, 0.25, 0.5, 1.0 m | — | strong ↓ with coarser ds (H4) |
| B3 | `carve_band` | −1.0 (full-ray), 0.0 (endpoint), 0.5, 1.5, 3.0 m | — | free-space Beta stream collapses (H4) |
| B4 | `max_range` | 6, 10, 20, 50 m | — | ↑ carved volume → ↑ Beta |
| B5 | `share_tsdf` | false, true | — | ~2× geometry (H5) |
| B6 | `K_TOP` (**recompile fleet**) | 1, 2, 4 | rebuild scovox_core, bump nothing else | +6 B/dir per step (H6) |
| B7 | `num_classes`/`alpha_0` | 14/0.01, 20/0.01, 14/0.05 | header-only control | ≈ no change (H6) |
| B8 | `N` robots | 1, 2, 3 | duplicate-source AND coverage-split variants | linear ↑ (H2) |

### 5b. Comparator arms (what "sharing the map" would cost with a different payload)

| Arm | Payload | Topic | Per-unit cost | Note |
|-----|---------|-------|---------------|------|
| C-delta | `ScovoxMapBinary` delta (this stack) | `/robotK/scovox_node/scovox_bin` | 20 B/Beta + 28 B/Dir, LZ4, delta-only | the real inter-robot channel |
| C-snap | full `ScovoxMap` snapshot | `~/scovox` (dscovox latched, transient_local) | `ScovoxVoxel`: pos 12 B + a_occ/a_free/a_unk 12 B + `semantic_evidence[]` @6 B/class | every voxel, uncompressed, MUCH bigger |
| C-cloud | raw `PointCloud2` "ship the cloud" | `~/pointcloud` | 11 float/uint fields per voxel | naive baseline; orders larger |

Measure C-snap and C-cloud on the **same bag/warm-up window** so the ratios are apples-to-apples. Report `BW_snap / BW_delta` and `BW_cloud / BW_delta`.

---

## 6. Measurement Methodology

**The transport reality (do not fall for the SHM-inflation trap).** All containers run `network_mode:host` + `ipc:host` + `ROS_DOMAIN_ID=0` + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Map-sharing bytes move over **SHM (`/dev/shm`, 256 MB segment via `fastdds_shm.xml`) or UDPv4 loopback** — they never touch a physical NIC. Two consequences:
- **`tcpdump -i <NIC>` sees nothing** (SHM leaves no packets; loopback is not the NIC). Do not try to sniff a real interface.
- **`ros2 topic bw` reports the serialized application message size/sec at the subscriber** — which for this payload (an already-LZ4-compressed serialized voxel delta) **≈ the exact bytes a robot-to-robot radio link carries** (± a few % RTPS/UDP framing). **This is the correct, transport-independent inter-robot bandwidth figure.**

**Primary tools (run from a second `docker compose exec scovox bash`, domain 0):**

```bash
# Cadence
ros2 topic hz /robot1/scovox_node/scovox_bin
ros2 topic hz /robot2/scovox_node/scovox_bin
# Bandwidth (serialized bytes/s ≈ on-link bytes)
ros2 topic bw /robot1/scovox_node/scovox_bin
ros2 topic bw /robot2/scovox_node/scovox_bin
# Aggregate = sum of the two (and /robot3 for N=3)
```

**Per-message size logging (to get the distribution, r_LZ4, and separate the snapshot spike):**
- Record the stream: `ros2 bag record -o /tmp/binbag /robot1/scovox_node/scovox_bin`, then `du -b` the bag / count messages, or parse the bag to get `len(msg.data)` per message.
- Cross-check against the mapper's own accounting: `publishBinaryMap()` returns per-frame bytes stored in `bin_bytes_` (`scovox_node.cpp:804`, `:1252`). **Grep the scovox log for per-publish binary size** and reconcile with `ros2 topic bw` averages.
- Compute `r_LZ4 = mean(len(data)) / S_raw`, where `S_raw = 28 + 20·N_beta + 28·N_dir` and `N_beta`/`N_dir` come from `beta_count`/`dir_count` in the frame header (or from the mapper's drain counters if logged).

**Optional true RTPS/UDP framing overhead (only if you want it on top of payload):** force a **UDP-only** Fast DDS profile (drop the SHM profile), then measure on the container **veth / `lo`** with `ifstat`/`nload`/`iftop`, or `tshark -i lo -f 'udp' -q -z io,stat,1` filtered to the RTPS flows, or `tcpdump -i lo ... -w` then byte-count. This yields payload + header overhead; the delta of (loopback bytes − serialized bytes) = the RTPS/UDP framing tax and the **reliable-QoS retransmit** overhead. Not required for the headline number.

**Warm-up & window.** The scovox_bin cadence ≈ one delta per integrated scan (verify with `ros2 topic hz`). Discard the first message (new-subscriber snapshot) and the first ~15–30 s of exploration transient; then average over a fixed steady-state window (e.g. 60 s of the bag's mid-trajectory). Report mean, p95, and max bytes/s.

---

## 7. Step-by-Step Procedure (two-mapper + merger)

Base stack reuses `ws/src/run_fused_experiment.sh` unchanged for hmr_loc + hmr_seg + scovox.

**STEP 0 — base stack.** `dc_loc/dc_seg/dc_scovox up -d`; launch the EKF+NDT+3-static-extrinsics block in `hmr_loc` and `seg_node` in `hmr_seg` (script lines 119-159). Single bag is the only sensor source; both mappers subscribe to the same absolute topics.

**STEP 1 — two identity TFs so the merger keys two distinct sources.** dscovox keys one source grid per `header.frame_id`, and `scovox_node` sets `bin.header.frame_id = integration_frame` (`scovox_node.cpp:1570`). In `hmr_loc` (or scovox):
```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id r1_map
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id r2_map
```

**STEP 2 — start the merger FIRST** (subscriber-gated publish; `scovox_node.cpp:1482`):
```bash
dc_scovox exec -d scovox bash -lc 'source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash; \
ros2 run scovox_mapping dscovox_mapping_node --ros-args -r __node:=dscovox_node -p use_sim_time:=true \
 -p "input_topics:=['"'"'/robot1/scovox_node/scovox_bin'"'"','"'"'/robot2/scovox_node/scovox_bin'"'"']" \
 -p map_frame:=map -p pointcloud_topic:=/dscovox/pointcloud > /tmp/dscovox.log 2>&1'
```
(For a pure bw run without a merger, attach `ros2 topic bw` first instead — but that itself is the "first subscriber" that eats the snapshot.)

**STEP 3 — mapper #1** = the EXACT fused `scovox_mapping_node` command from `run_fused_experiment.sh:169-198`, with only three edits:
1. add `-r __ns:=/robot1` right after `-r __node:=scovox_node` → publishes `/robot1/scovox_node/scovox_bin`;
2. `-p mode:=rolling` (was `persistent` at line 189) — rolling creates `bin_pub_` (`scovox_node.cpp:526-536`);
3. `-p integration_frame:=r1_map` (was `map` at line 180).
Keep everything else identical (`fuse_lidar_rgbd:=true`, depth/seg/pointcloud/imu topics, lidar_/rgbd_ weights, resolution, palette).

**STEP 4 — mapper #2**: same command with `-r __ns:=/robot2` and `-p integration_frame:=r2_map`.

**STEP 5 — play the bag** exactly as `run_fused_experiment.sh:229`.
- **Duplicate-source (2× upper bound):** keep both mappers alive the whole playback; deltas are byte-identical; `BW_agg = 2·BW_1`.
- **Coverage-split (realistic):** play `[0, T/2]` with only robot1 subscribed, then `[T/2, T]` with only robot2 — or stagger mapper #2's connection by tens of seconds so deltas diverge.

**STEP 6 — measure** (Section 6): `ros2 topic bw` + `ros2 topic hz` on each `scovox_bin`, record the bag for per-message sizes, grep `bin_bytes_` from the mapper log. Then tear down with the compose-stop commands the orchestrator prints, plus the two extra mapper processes and the dscovox process. **Use `docker compose stop`, never `pkill`.**

For each sweep arm, restart the affected mapper(s) with the changed param (rebuild scovox_core for the `K_TOP` arm B6), re-warm-up, re-capture the same window.

---

## 8. Analysis & Plots

1. **Bytes/s vs each knob** (one line per arm B1–B8): resolution (log-x), downsample, carve_band, max_range, share_tsdf (bar), K_TOP (bar), num_classes (bar — expect flat), N (linear). Plot mean with p95/max whiskers.
2. **Stream decomposition:** stacked area of `20·N_beta` (occupancy) vs `28·N_dir` (semantics) vs TSDF (when on) per frame, so the reader sees which stream each knob moves. Overlay measured LZ4-compressed size to show `r_LZ4` per arm.
3. **Comparator bar chart:** `BW_delta` vs `BW_snap` (`~/scovox`) vs `BW_cloud` (`~/pointcloud`) on a log axis; annotate the ratios (H7).
4. **Per-robot vs merger-aggregate:** `BW_agg` vs `N` for duplicate-source (expect exactly linear) and coverage-split (expect sub-linear where regions overlap); overlay the analytical `N·BW_1` line.
5. **Bandwidth ↔ map-quality Pareto:** x = steady-state bytes/s, y = map quality, one point per sweep setting. Quality axes: **APE** of the fused/local occupancy map vs `gt_map_us050.pcd` (geometry) and **semantic accuracy** (class agreement vs labels). Identify the knee (e.g. res 0.10 + downsample 0.5, per prior findings) that minimizes bandwidth for acceptable quality.
6. **Cold-start vs steady-state:** report the one-time new-subscriber snapshot size separately from steady-state bytes/s; show total-transfer = snapshot + ∫(delta bytes/s) over the run.
7. **Reliable-QoS overhead (optional):** from the UDP-only loopback measurement, `(loopback bytes − serialized bytes)` = RTPS/UDP framing + KeepLast(50) reliable retransmits; report as a % tax on the payload figure.

---

## 9. Risks / Validity Threats

| Threat | Effect | Mitigation |
|--------|--------|-----------|
| **SHM masking real bytes** | tcpdump on NIC sees nothing; naive "no traffic" conclusion | Use `ros2 topic bw` (serialized size ≈ link bytes); only use loopback/veth sniffing under a UDP-only profile if framing overhead is wanted |
| **Single-bag correlation between "robots"** | duplicate-source deltas are byte-identical → merged occupancy double-counts (`a_fused = a1+a2−1`); not a physical fused map | Use duplicate-source only for the 2× BW upper bound; use temporal coverage-split for any quality/fusion claim |
| **LZ4 variance** | `r_LZ4` differs per arm (coord clustering changes with res/carve) | Measure `r_LZ4` per arm; never assume a fixed ratio; report distribution not just mean |
| **New-subscriber snapshot spike** | one-off full map skews the bytes/s average | Drop the first message; report snapshot separately |
| **Persistent-mode zero traffic** | `bin_pub_` never created → nothing to measure | Assert `mode:=rolling` on every mapper |
| **Subscriber-gated early loss** | deltas before any sub are drained, never resent | Start merger / bw probe **before** mappers |
| **Frames must be distinct** | both at `integration_frame:=map` collapse to one merger source | Distinct `r1_map`/`r2_map` identity TFs |
| **c-slam / TF jump** | cached source→map TF never refreshed → ghosted voxels + invalid map | Keep c-slam OFF, TFs static (dscovox banner :24-33) |
| **Mixed builds** | K_TOP / codec-rev / prior / endianness mismatch → frames dropped or fused mass corrupted | One image, one build, whole fleet; verify envelope v4 + codec rev 5 + K_TOP 2 identical |
| **`scovox_publish_rate` confusion** | it drives viz `sm_timer`, not `scovox_bin` | Bin cadence = per integrated scan; measure with `ros2 topic hz`, don't assume the param value |
| **Measuring the wrong topic** | `~/pointcloud`, `/dscovox/pointcloud`, `~/planning_map` overstate link BW by orders of magnitude | Only measure the two `scovox_bin` topics for link BW |

---

## 10. Deliverables & First-Run Checklist

**Deliverables.**
- A results table: per arm → mean/p95/max bytes/s per robot, aggregate, cadence Hz, mean `N_beta`/`N_dir`, `S_raw`, measured `r_LZ4`, cold-start snapshot MB.
- The 7 plots of Section 8 (raw CSV + figures).
- The comparator ratio numbers `BW_snap/BW_delta` and `BW_cloud/BW_delta`.
- A one-paragraph headline: single-robot steady-state uplink at defaults, and the recommended bandwidth-vs-quality knee.
- Reproducibility appendix: exact mapper/merger commands per arm, bag window used, git SHA of scovox build (+ K_TOP recompile note for B6).

**Concrete first-run checklist (baseline A0, N=2 duplicate-source, upper bound):**

- [ ] `dc_loc/dc_seg/dc_scovox up -d`; EKF+NDT+3 static extrinsics up in `hmr_loc`; `seg_node` up in `hmr_seg` (run_fused lines 119-159).
- [ ] Publish `map→r1_map` and `map→r2_map` identity static TFs (STEP 1).
- [ ] Confirm c-slam OFF and only static source frames (no pose-graph / loop closures).
- [ ] Start `dscovox_mapping_node` FIRST with `input_topics:=[/robot1/.../scovox_bin, /robot2/.../scovox_bin]` (STEP 2). Confirm it is subscribed.
- [ ] Start mapper #1: run_fused scovox cmd + `-r __ns:=/robot1` + `mode:=rolling` + `integration_frame:=r1_map`.
- [ ] Start mapper #2: same + `-r __ns:=/robot2` + `integration_frame:=r2_map`.
- [ ] Verify publishers exist: `ros2 topic list | grep scovox_bin` shows both `/robot1/...` and `/robot2/...`.
- [ ] In a 2nd shell: `ros2 topic hz` and `ros2 topic bw` on both `scovox_bin` topics; start `ros2 bag record` of both for per-message sizes.
- [ ] Play the bag (run_fused line 229).
- [ ] Discard message #1 (snapshot) + first ~30 s; average steady-state bytes/s over a fixed 60 s window; log snapshot size separately.
- [ ] Reconcile `ros2 topic bw` mean against grepped `bin_bytes_` per-publish size from the mapper log.
- [ ] Compute `r_LZ4` from `len(data)` vs `S_raw = 28 + 20·N_beta + 28·N_dir` (header `beta_count`/`dir_count`).
- [ ] Tear down: `docker compose stop` per the orchestrator's printed commands + stop the two mappers and dscovox (never `pkill`).

**Key file references:**
- `ws/src/run_fused_experiment.sh` (base stack; scovox cmd 169-198, `mode:=persistent`@189, `integration_frame:=map`@180, `downsample_voxel_size:=0.1`@192, bag play 229)
- `ws/src/scovox/src/scovox_mapping/launch/scovox_multi_robot.launch.py` (namespaced multi-robot + dscovox `input_topics`)
- `ws/src/scovox/src/scovox_mapping/src/scovox_node.cpp` (`bin_pub_` rolling-only 526-536; `publishBinaryMap` 1475-1591; header.frame_id=int_frame 1570; envelope v4 1571-1575; at-prior gates 1521-1556; mode default 259; integration_frame default 233)
- `ws/src/scovox/src/scovox_mapping/src/dscovox_node.cpp` (c-slam-disabled 24-33; source keyed by frame_id 98-104; input_topics + reliable bin_qos 133-210; version/endian reject 298-321)
- `ws/src/scovox/src/scovox_core/include/scovox/binary_serializer.hpp` (FORMAT_VERSION=5 @84, MAX_NUM_CLASSES 4096, per-beta/per-dir sizes, share_tsdf); `voxel.hpp` (`K_TOP=2`)
- `ws/src/scovox/src/scovox_msgs/msg/ScovoxMapBinary.msg` (header + version + little_endian + `geometry_msgs/Transform map_from_source` + `uint8[] data`)
- `ws/src/hmr_localisation/config/fastdds_shm.xml` (SHM default, 256 MB, UDPv4 fallback)
- Sweep-default configs: `ws/src/scovox/src/scovox_mapping/config/lidar_mapping.yaml`, `rgbd_semantic_mapping.yaml`

## 11. Update (2026-07-02): distributed-fusion topology test

`run_mapshare_experiment.sh` now also tests the REAL fleet topology, not just
the wire: **one dscovox merger per robot** (`/robotK/dscovox_node`, base params
from `dscovox_params.yaml`), each fusing ALL robots' `scovox_bin` streams, so
every robot ends the run holding its own copy of the global map — exactly the
multi-robot design in the scovox README ("Multi-robot mapping").

**Fusion proof — `SPLIT_BAND=1` (default when `NROBOTS>=2`):** robot K *shares*
only slice K of `[SPLIT_ZMIN, SPLIT_ZMAX]` (default `[-0.5, 2.0]`, the
real-robot band in `scovox_robot_share.yaml`), while still mapping everything
locally. Any voxel robot K's fused map holds inside a *peer's* slice can only
have arrived over that peer's `scovox_bin` stream. After playback, step 8 of
the script:

1. greps each merger log's last `dscovox_diag` line — every merger must have
   seen all N sources, with non-zero and cross-robot-symmetric fused totals;
2. calls each `/robotK/dscovox_node/get_region` (services still answer after
   `/clock` stops) with a thin probe slab at the centre of every sender slice
   and requires non-zero voxels in every *peer* slab;
3. prints `FUSION VERIFY: PASS/FAIL`.

For pure bandwidth sweeps (Sections 1–10), run with `SPLIT_BAND=0` to restore
the original whole-map (or `SHARE_Z_MIN/MAX`-banded) duplicate-source arms.
