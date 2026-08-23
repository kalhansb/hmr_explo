# Reconnection Modes Under Emulated Comms: Focused Experiment Plan

**One question.** Robots re-merge their maps automatically whenever they come
back into radio range — reconnection itself is opportunistic and free, and the
downstream work hinges on that in-range mechanism. The `reconnect_mode`
manoeuvres (`rendezvous` / `pursuit` / `hybrid`) are **deliberate reconnection
path planning** layered on top: they spend travel to make the next contact
happen sooner instead of waiting for luck. With a reliable map relay a
disconnection therefore costs **latency, not data** — within the relay's
backlog cap (§3.3, §8; scoped in R5) — and the baseline failure is not a
permanently incomplete map. The question is efficiency: **do the
manoeuvres reach complete team knowledge sooner and cheaper than chance
re-contact alone**, and is the shipped default (`hybrid`) the right one?

Single experiment: **4 arms (`off` = opportunistic-only, `rendezvous`,
`pursuit`, `hybrid`) at one calibrated severity, paired seeds.** Everything
else in this document exists only to make that one comparison valid. Because
every arm rides the same in-range merge mechanism, the pilot doubles as the
validation of that mechanism (B0a) — the piece the downstream work depends on
regardless of which mode wins.

> **Verification note (checked against the tree 2026-08-12; adversarially
> re-verified 2026-08-13 against `origin/new_experiments` @ 78ec72f):** the
> reconnection code is on `explo_planner@origin/new_experiments`, NOT the
> checked-out `main` — `ReconnectMode` at `planner_util.hpp:57`, the `PURSUE`
> state at `explo_planner_node.cpp:104`, `reconnect_mode` / `pursuit_*` at
> `shared_params.yaml:385,398,409`. The comms emulator
> (`hmr_sim/src/hmr_comms_sim_node.cpp`) is built and installed but not
> committed — node/config/launch/test/docs **untracked**, CMakeLists.txt and
> package.xml **modified-tracked**. `done_action: idle` is the yaml value
> (`shared_params.yaml:161`) but the **C++ default is `shutdown`** and the
> idle requirement is warn-only; an unrecognized string (even `"Idle"` — the
> compare is case-sensitive) falls back to `shutdown`, the dangerous value
> (`explo_planner_node.cpp:345,860-864,1004-1010`). `step_` increments only in
> LOG_STEP (`explo_planner_node.cpp:3347`), so manoeuvres themselves consume
> **no steps** — but each reconnect cycle resets the coverage streak
> (`explo_planner_node.cpp:2652,2719,2898`) and costs ≥ 3 re-confirmation
> steps before DONE can re-latch, so mode arms *do* leave a step signature.
> The harness `DURATION_S` cap (`run_explo_sim_rviz.sh:61`, default 0 =
> unbounded) is **sim-time**, as are the planner, logger, and emulator clocks
> under the default wiring.

> **Revisions.** R1 (2026-08-12): fixed the unreachable-termination trap (§2.1),
> single-firing manoeuvres (§2.2), the staleness-gate mode collapse (§2.3), and
> the known-vs-observed coverage conflation (§4.1). R2 (2026-08-12): replaced
> the 2×4 factorial with an E1/E2/E3 split; primary endpoint changed to
> time-to-team-knowledge-complete (coverage-at-budget is near mode-invariant by
> causal structure — modes act at exhaustion, after coverage accrues — and
> would have refuted hybrid spuriously); common-random-number pairing;
> exploration-only. R3 (2026-08-13): **scoped to the reconnection methods
> only.** E1 (damage curve) and E3 (mild-severity overhead) cut — they
> characterise the comms model, not the manoeuvres. Single severity, four
> arms; the `moderate` calibration target and the `ideal` matrix arm are gone
> (the control pilot doubles as the reference). H1 and F3 (E1-only) dropped;
> H2 reframed as a manipulation check that gates interpretation. R4
> (2026-08-13): **baseline reframed** — robots re-merge automatically whenever
> back in range, so `off` is *opportunistic-reconnection-only*, not "no
> reconnection"; disconnection costs latency, not data. F2 demoted from
> expected outcome to rare end-state failure; B0 split into B0a (in-range
> merge mechanism works — the downstream work hinges on it) and B0b (headroom:
> `off` ≫ control); merge attribution (opportunistic vs deliberate) added as a
> core diagnostic; timer-driven logging added during manoeuvre states; primary
> endpoint unchanged — its rationale is now cleaner. R5 (2026-08-13):
> **adversarial review folded in.** "Latency, not data" scoped to the relay's
> 64 MiB/link backlog cap — the emulator **evicts oldest** on overflow — with
> `drop_overflow` promoted to a run-invalidating gate, plus two more loss
> paths (discovery race, rx-history overwrite) gated (§3.6, §8). Paired-seeds
> claim corrected: the seed pins only the emulator fading trace — the planner
> has no RNG and sim sensor noise is unseeded — so the pilot gains a
> same-seed repeat and grows to 2–3 seeds × 4 arms (§6, §7). Heartbeat is
> state-gated, so "peer missing" conflates outage with planner-busy —
> suppression logging added (§3.8, §8). §2.1's conclusion corrected
> (step-budget termination also dispatches; the trap kills *repeat* cycles)
> and the stuck-in-PLAN third ending gated (§2.7). Timer logging extended to
> all states with forced map re-ingest — the R4 version would have logged
> frozen voxel counts (§3.4). Analysis pre-registered as paired RMST at T,
> with a second contrast (hybrid vs best single mode) — hybrid-vs-off alone
> cannot answer the shipped-default question (§1). H-P corrected: a
> staleness-declined pure pursuit parks and beacons, not "back to chance"
> (§1). B0b sharpened with a delay-attribution requirement and a
> solo-reachability check on the saturation criterion (§1, §5.2).

---

## 1. Claims Under Test

**What a disconnection costs.** The map relay is reliable (§3.3) **up to its
64 MiB/link backlog cap**: below the cap nothing is lost, only *late*; at the
cap the emulator evicts oldest and voxels are genuinely gone — which is why
`drop_overflow > 0` invalidates a run (§3.6, §8). Within the gates, while
disconnected, each robot's known map falls
behind what the team has observed; at the next contact — chance or engineered —
the backlog drains and the gap snaps shut. The damage is therefore **when**
knowledge arrives, not **whether**. Outright failure is the exception:

- **F1 Incoherent finish:** the team does not reach a common DONE within the
  run cap T.
- **F2 End-state divergence:** the run ends with a robot still missing > 10 %
  of what the team physically observed (§5.1) — possible when the final
  stretch is explored while disconnected and the robots then idle out of
  range, so no further contact ever drains the backlog. Reported as a rate;
  **not** expected in most `off` runs.

**Checks and hypotheses.**

| # | Claim | Prediction / criterion |
|---|-------|------------------------|
| **B0a** (mechanism check) | In-range reconnection works: at each re-contact the reliable backlog drains and robot-known jumps toward team-observed. The downstream work hinges on this mechanism — the pilot doubles as its validation. | step-change in robot-known at contact times (contacts from `~/link_states`); backlog drain visible as `backlog_bytes` in `~/stats` (reliable-side starvation never touches the drop counters) |
| **B0b** (headroom check) | Under `off`, completion waits on chance re-contact: time-to-team-knowledge-complete materially exceeds the control's — **and the delay is attributable to waiting-on-contact** (late gap-drain events in the §5.1 trajectory), not merely longer travel from duplicated coverage. A pass driven by lost *coordination* (no division of labour) is a pass for the wrong reason: the manoeuvres fire at exhaustion and cannot recover duplication cost (§5.2 solo-reachability check). | `off` ≫ control **with the gap-drain signature**. **If `off` ≈ control, chance contact is already near-optimal and the scenario cannot discriminate modes — recalibrate (§4), do not proceed. If `off` ≫ control but the gap-drain signature is absent, the endpoint is measuring duplication, not reconnection — fix the criterion before the matrix.** |
| **H-R** | Rendezvous converts the merge from lucky to scheduled — a deterministic meeting closes the gap every cycle, at the highest travel/time cost of the three. | best success-within-T; worst reconnect overhead |
| **H-P** | Pursuit reconnects fastest when the last-contact record is fresh; when stale it declines — and then **parks and beacons at its current pose** (`holdForTeam`: a stationary, findable target, not a return to pure chance). Only hybrid gets the meeting-point fallback on a declined chase. | best time-to-reconnect given success; most censored runs |
| **H-H** | Hybrid ≈ rendezvous on success, ≈ pursuit on speed — the shipped-default claim. | Pareto-better on (success, time) |

**Pre-registered primary:** time-to-team-knowledge-complete (§5.2), **`hybrid`
vs `off`**, paired by seed, censored at T. This is an intention-to-treat
comparison of **policies**, not channels: mode arms still enjoy chance
merges, and `off` still merges whenever luck brings the robots into range —
the estimand is the value *added* by deliberate manoeuvres on top of the
shared in-range mechanism, not "reconnection vs none". **Analysis: paired
restricted-mean survival time (RMST) at T** — "success-within-T plus
time-given-success" conditions on success and silently drops exactly the
pairs where the effect was largest; RMST keeps every pair under censoring.
**Second pre-registered contrast:** `hybrid` vs the better of {`rendezvous`,
`pursuit`} on the same endpoint — `hybrid` vs `off` alone cannot answer the
shipped-default half of the One Question, and the mode-mode delta is the
smaller, harder contrast. ("Cheaper" throughout means total time and
distance to team-knowledge-complete — the `off` arm's *reconnect* overhead
is zero by construction.) Everything else exploratory — effect sizes with
intervals, no per-hypothesis significance claims; the H-table predictions
are arm rankings, and rankings over four arms at n ≤ 10 are noise-prone —
keep them out of conclusions.

Safety is observed, not tested (§5.4): proximity peer poses ride the same gated
1 Hz heartbeat, and pursuit deliberately drives at the peer's last known
position while blind — expect pursuit ≥ hybrid > rendezvous ≈ off on close
approaches, reported descriptively (events are too rare at this n for a test).

**Explicitly out of scope (cut in R3):** the comms damage curve (how much
degradation hurts exploration efficiency — duplication, distance per voxel,
`rejected_by_minpos` mechanics), overhead under mild degradation, and any
severity sweep. Also out: knowledge *latency during exploration* — stale peer
intents driving duplicated coverage — which is the cut damage curve's
territory, not the manoeuvres'. One severity, chosen to make the modes' job
real.

---

## 2. Scenario Corrections (prerequisite, not code)

All are prerequisites for a valid mode comparison — unchanged by the R3
rescope; §2.7 added in R5.

### 2.1 The shipped ROI makes coverage termination unreachable

The harness passes `shared_params.yaml` unchanged, so the planner uses the
field site's ROI: x ∈ [−51.3, 100.9], y ∈ [−38.7, 74.5]
(`shared_params.yaml:202-205`) ≈ 17,229 m². The flatforest ground plane is
110 × 110 m centred at origin; the 80 oaks span x ∈ [−43.9, 49.6],
y ∈ [−49.7, 49.6]. **≥ 42 % of the ROI footprint has no geometry**; those
columns never leave the prior, `coverageUnknownFraction`
(`explo_planner_node.cpp:2245`) can never fall below `done_unknown_fraction:
0.05`, and every run would end at `max_steps`. **Correction (R5):** that does
*not* mean no manoeuvre fires — step-budget termination also routes through
`finishOrRendezvous` (`explo_planner_node.cpp:1852,3358`; the code comment
calls it the dominant termination path in dense terrain), so the broken ROI
still fires **exactly one** manoeuvre at `max_steps` and produces a
plausible-looking run. What it kills is the repeat reconnect → re-disperse
cycles (§2.2) — subtler, and worse, because nothing would look wrong.

**Fix:** sim ROI **x, y ∈ [−50, 50]**, then measure the achievable unknown
floor in the control pilot (occlusion keeps it above zero) and set
`done_unknown_fraction` above that floor. *Empirical check bundled into the
pilot: confirm unreturned rays leave columns unknown (the analysis above infers
it — no return ⇒ no ray ⇒ no carve); the control's unknown-fraction plateau
confirms or corrects the 42 % figure.*

**MEASURED (2026-08-15): the floor is 0.4922, and `done_unknown_fraction` is
now 0.55.** A full-length control run (5411 sim s, both robots ~1930 m
travelled) drove the ROI unknown fraction to 0.4922 by t ≈ 2900 s, after which
it did **not move at all** for the remaining 2600 sim s — identical to four
decimal places on both robots — while they covered a further ~950 m each and
the fused map grew 0.9 %. The last hundred goals stayed inside
x ∈ [−27.7, 42.5], y ∈ [−17.6, 37.5]. That is a floor, not a slow asymptote.

The 42 % geometry estimate above is therefore *optimistic*, and the reason is
the planner rather than the world. Utility is `info_gain / (ε + path_cost)`,
and while the map is mostly unknown `info_gain` is near-constant across the
candidate set — measured 6 % spread against a ninefold spread in `path_cost` —
so `argmax(U)` degenerates to `argmin(cost)`. That makes the planner a
diffusive nearest-frontier crawler; a forest's trunk shadows regenerate
frontier clusters inside the region it has already covered, so a distant
unexplored corner never wins on cost. It saturates at roughly **half** a ±50
ROI and then cycles there indefinitely. Three related fixes were needed before
it explored at all (§3.13); none of them changes this asymptotic behaviour,
they only stop it deadlocking in the first minute.

Consequence for the endpoint: **time-to-team-knowledge-complete now means
"time to reach the coverage a well-connected team reaches", not "time to map
the ROI"**. At 0.55 the criterion lands in the fast early phase, where map
sharing is what separates the arms, and all three control runs terminated
naturally rather than censoring at T. The threshold *is* the endpoint
definition, so it must be identical across every arm; it is recorded in each
run manifest. This also raises the stakes on §5.2's solo-reachability check,
since a criterion this far above zero is more plausibly reachable by one robot
alone — which is exactly the B0b failure the check exists to catch.

**Control pilot, 3 runs at `tx_power_dbm: 160` (link never drops):** makespans
1362 / 1938 / 1969 sim s, 25–36 min wall each, none censored. Because the
fading trace is inert at that power, seeds 1–3 differ *only* through unseeded
noise (sim physics, DDS timing) — so their **CV of 19 %** is a direct
measurement of the unseeded-noise floor that §7 phase 3 was meant to obtain
from a same-seed repeat. Pairing on the fade stream cannot strip variance that
does not come from the fade stream, so this argues for **more** seeds in the
matrix, not fewer.

### 2.2 Manoeuvres must fire more than once

`finishOrRendezvous` runs only at exhaustion, so one reconnect event per run.
The corrected ROI brings saturation early enough for reconnect → re-disperse
cycles to repeat. **Pilot gate: ≥ 3 reconnect cycles per run;** if < 2, shrink
the ROI toward ±35 and re-run §4 calibration (coupled — a smaller world also
shrinks the separations that break the link).

### 2.3 The staleness gate must not collapse the modes

`pursuit_staleness_max_sec: 180`: if the robots never re-approach after
dispersal, the record at exhaustion is minutes old, `pursuitBudgetSec` → 0,
pursuit declines, hybrid degrades to ≈ rendezvous, pure pursuit merely holds —
all arms identical and the mode comparison measures nothing. Calibration (§4)
must therefore hit a **window**: outages > `coord_claim_ttl_sec` (5.0 s) so the
peer reads as missing, AND time-since-contact at exhaustion < 180 s in most
runs so the chase actually arms. **Pilot gate: log every `pursuitBudgetSec`
outcome; the chase must be attempted in most pilot runs, not
staleness-skipped.**

### 2.4 `have_anchor_`

Set on any received peer intent (every one, in fact — but never without one);
without it `shouldRendezvous` is false and every arm silently degenerates to
`off` — and the DONE log line then claims "full team present (0/1 peers)",
actively misleading. Satisfied by the 3 m spawn — assert it in the pilot
anyway.

### 2.5 Run cap

Set `DURATION_S` to ~3× the control's observed makespan (sized in the pilot,
not guessed). Pure pursuit holds forever by design and rendezvous waits
unbounded (`rendezvous_max_wait_sec: 0`) — without a cap, time-to-event
metrics are undefined in exactly the arms where modes differ most. Runs hitting
the cap are **censored observations, not discarded runs** (§6). Two additions
(R5): the cap is **sim-time** (the harness watches `/clock`), and runs can
also hang mid-exploration without terminating (§2.7) — budget matrix
wall-clock at worst-case T per run, not mean makespan. Optionally end a run
once every robot passes saturation + margin plus a grace window: the primary
endpoint is already decided, and only F1/F2 and the safety observation
truncate.

### 2.6 Travel budget sanity

`max_range: 20 m` (`dscovox_multi_robot.launch.py:94`; the `simple_nav_3d`
**lidar** path matches at `simple_nav_3d.launch.py:324` — the RGB-D branches
inherit the node default 10.0 and do not) ⇒ ~40 m swath ⇒ order-250 m minimum
travel for 10,000 m² — comfortably inside `MAX_STEPS=500`, but confirm in the
pilot that steps never bind before the coverage criterion, remembering each
reconnect cycle now costs ≥ 3 re-confirmation steps (verification note).

### 2.7 Stuck-in-PLAN is a silent third ending

When every candidate is rejected, the planner loops in PLAN forever
(`explo_planner_node.cpp:2153-2161`) — no manoeuvre, no DONE, and **no
LOG_STEP rows**, so the step budget never advances while stuck; only
`DURATION_S` bounds the run. The harness default `FRONTIER_ONLY=1` disables
the polar candidate fallback that exists to prevent exactly this
(`run_explo_sim_rviz.sh:66-78`). **Gate: the harness must flag a planner that
emits no CSV row for N minutes as invalid-hung.** Same state-gating side
effect: a robot stuck in PLAN > 5 s stops heartbeating and reads *missing* to
its peer under perfect comms (§3.8, §8).

---

## 3. Build Gap (all opt-in; defaults bit-identical)

The seven R3 items survive the rescope, plus an eighth added in R5 — each is
needed for the mode comparison itself:

1. **Split the intent topic** (the only planner change). The planner pubs and
   subs one global `coord_intent_topic` (`explo_planner_node.cpp:1280-1283`), so
   no external bridge can gate it — and gated intents are what makes a peer
   *read as missing*, the trigger for every manoeuvre. Add
   `coord_intent_pub_topic` + `coord_intent_sub_topics` (list), defaults =
   today's behaviour. Harness sets pub `/<r>/exploration/intents`, sub
   `[/<r>/rx/<peer>/exploration/intents]`, matching the emulator's relay
   convention (`hmr_comms_sim_node.cpp:19,555`). Build the pub topic absolute
   from `robot_name_` per the file's own convention
   (`explo_planner_node.cpp:1099`). ~30 lines + a parse test.
2. **Emulator config:** `best_effort_topics: ["exploration/intents"]` — a lost
   1 Hz heartbeat should read as lost, not late. (The parameter exists today,
   with drop-on-disconnect + BER + airtime semantics — but it is inert until
   item 1 lands: the emulator subscribes per-robot topics and the intent topic
   is currently global.)
3. **Map stream needs a launch change in a third repo** — kept in the rescope
   because without gated maps there is **no knowledge gap for the modes to
   close**: robots would always hold each other's maps and the primary endpoint
   would be mode-invariant by construction. `simple_nav_3d.launch.py:251,358`
   hardcodes `dscovox_inputs = [f"/{r}/scovox_node/scovox_bin" for r in
   [robot]+peers]` (parameter `input_topics`, a list). Add a
   `peer_bin_topic_pattern` launch arg defaulting to today's string; harness
   points peers at `/<r>/rx/<peer>/…`; own stream stays local. Keep the
   **reliable** relay policy: map degradation appears as staleness, never holed
   voxels — *while the §3.6 overflow gate holds* (the queue evicts oldest at
   its 64 MiB cap) — state this in the writeup; results do not generalise to
   lossy map transport.
4. **CSV:** add `state`, `reconnect_distance_m`, `reconnect_elapsed_sec`
   (`metrics_logger.cpp:29-37` has `phase` only). Without these H-R/H-P/H-H are
   not measurable. **Emit rows on a timer in *all* states, all arms — and the
   timer row must re-ingest the map first.** Two R5 corrections to the R4
   version of this item: (a) `total_observed_voxels` comes from the map as
   loaded by `loadLatestMap()`, which runs only in WAIT_FOR_MAP/PLAN
   (`explo_planner_node.cpp:1544,1689,1878`) — a timer row emitted during a
   manoeuvre *without re-ingesting* would log a **frozen** count and reproduce
   exactly the bias it exists to remove; (b) the `off` arm's blind spot is
   smaller but real — its merges are also visible only at the next LOG_STEP,
   potentially tens of seconds away — so timer logging everywhere both fixes
   the one-sided manoeuvre bias and removes the shared quantization noise.
   Don't assume `reconnect_elapsed_sec` ≤ the 240 s chase ceiling:
   proximity-hold time is refunded to the pursuit clock
   (`explo_planner_node.cpp:3244-3250`) and the hybrid fallback leg adds up to
   `nav_max_timeout_sec` (180 s) on top.
5. **Harness:** `COMMS=0|1` and `SEED`; launch the emulator with the scenario;
   flip intent + dscovox remaps; record `~/link_states`, `~/robot_index`,
   `~/stats`; `exploitation_enabled:=false` for all matrix runs
   (exploration-only — target detours add noise and answer a different
   question; skip the scheduler). **Seed plumbing is unbuilt:**
   `comms_sim.launch.py` does not expose `seed` as a launch argument — it
   flows only through the params file — and the harness has no seed variable
   at all; the matrix's per-run seeds need both ends added. The seed pins
   *only* the emulator fading trace (§6).
6. **Four bring-up gates** (all silent failures *look like results*):
   **leakage** — fail the run if any subscriber outside `hmr_comms_sim` plus
   an explicit node-name allowlist (rosbag, rviz — the §5.1 oracle *requires*
   bagging cross-robot topics) sits on a cross-robot topic, and the check must
   explicitly cover `/exploration/intents`, which bypasses the emulator
   entirely until item 1 lands; **QoS match** — intents are
   `KeepLast(8).reliable()` (`explo_planner_node.cpp:1279`); today's emulator
   mirrors the source publisher's reliability (rclcpp fallback: reliable), so
   the feared mismatch cannot currently arise for intents — keep the gate
   anyway against heterogeneous publishers and future edits; **overflow** —
   `drop_overflow > 0` on any link invalidates the run (the reliable queue
   evicts oldest at 64 MiB/link: "Deltas lost on this link — the receiver's
   merged map will be missing voxels", `hmr_comms_sim_node.cpp:605-613`); also
   raise `rx_qos_depth` from 100 — drained messages are scheduled
   near-simultaneously, and a reliable `KeepLast(100)` rx publisher can
   overwrite unacked samples mid-drain; **odom watchdog** — `has_pose_`
   latches true and never times out, so a mid-run odom death freezes that
   link's state forever, possibly *connected*. Bring-up order matters too:
   relay subscriptions appear via 1 Hz topic discovery, so **start the
   emulator before the mappers** or the initial full map snapshot is never
   relayed — and no drop stat counts it.
7. **Commit the emulator** (node, config, launch, smoke test, docs untracked;
   CMakeLists.txt and package.xml carry uncommitted modifications). Runs
   against uncommitted code are not reproducible. Log the build hash per run —
   `staged` is a `RobotIntent` wire field and the intent TTL is
   producer-advertised on the wire too (`RobotIntent.msg:40,79`), so a
   build-skewed peer can both hold barriers spuriously and lengthen how long
   it reads "present" in everyone else's table.
8. **Log heartbeat suppression** (new in R5). `heartbeatTick` skips
   WAIT_FOR_MAP, PLAN and LOG_STEP (`explo_planner_node.cpp:3031-3036`), so a
   planner busy > 5 s (= `coord_claim_ttl_sec`) reads **missing** to its peer
   under perfect comms — indistinguishable from an outage in
   `coord_active_peers`, and able to arm a manoeuvre against a healthy
   teammate. Log own planner state (or suppression intervals) at heartbeat
   cadence so every peer-missing episode can be classified outage vs
   suppression against `link_states`. Without this, merge attribution (§5.3)
   and the manoeuvre triggers themselves are confounded.

### 3.9 explo_planner ↔ scovox rev-8 compatibility — CHECKED, clear (2026-08-14)

`scovox@1a689a6` landed a **rev-8 wire codec** (u8 sqrt-companded evidence
payloads + packed u8 class ids) and `SCOVOX_K_TOP` work. Since
`total_observed_voxels` is the primary endpoint's input, this was audited before
building anything on top of it. **The planner is unaffected**, for four
independent reasons:

1. **No `.msg` changed.** `1a689a6` touches no file under `scovox_msgs/`; rev 8
   is the *binary* codec (`ScovoxMapBinary`, scovox_node → dscovox_node). The
   planner subscribes to `ScovoxMap`, which dscovox publishes **after** the
   decode, so it sits downstream of every rev-8 change.
2. **The planner never reads semantics.** `MapCache::updateFromScovoxMap`
   (`map_cache.cpp:46-88`) reads `position`, `a_occ`, `a_free` and nothing else,
   so packed class ids cannot reach it. `class_id` is `uint16` in
   `ScovoxSemanticEvidence.msg` regardless — the u8 packing is wire-only and is
   unpacked on receipt. (`tree_detector_node.cpp:260-264` does read
   `semantic_evidence`, but it reads the same unchanged msg field and is not
   launched by this harness.)
3. **The voxel count's boundary is preserved by design.** `total_voxels` is a
   raw cell count of the ROI-clipped grid (`map_cache.cpp`, `computeStats`), not
   an evidence threshold — so the only way rev 8 could move it is by changing
   which records survive the receiver's refold. That is exactly what the sqrt
   companding exists to prevent: linear u8 would have opened a dead band around
   the prior and dropped ~0.8 % of records (young voxels vanishing from peers'
   maps), whereas companding keeps the q=0→1 step equal to rev 7's u16 step and
   reconstructs the at-prior value **bit-exactly**
   (`binary_serializer.hpp:78-93`).
4. **Skew fails loud, not silent.** `deserialize` throws `bad VERSION` on a
   codec mismatch and the frame is dropped with a warning
   (`binary_serializer.hpp:343-345`); `K_TOP_wire` is asserted to match the
   receiver's.

**Residual risk is build skew, not code.** Encoder and decoder both live in
`scovox`, so a stale install of either rejects *every* frame and the planner
never leaves `WAIT_FOR_MAP`.

⚠️ **The "empty CSV" tell this section used to rely on is gone.** With
`metrics_period_sec: 5.0` the sampler emits a row every 5 s in *every* state,
`WAIT_FOR_MAP` included — so total codec skew now produces a full-length CSV of
zeros (`total_observed_voxels=0`, `mean_eig=0`, `unknown_fraction=1`) rather
than an empty file. Detect it from the logs and from `state`, not from file
size.

Pre-campaign steps, both required:

1. **Rebuild `scovox` and confirm no `bad VERSION` / `K_TOP` warnings** in the
   scovox_node and dscovox_node logs before the first timed run. Note that
   `colcon build --packages-select scovox_mapping` on its own compiles against
   the *installed* `scovox_core` headers and fails on a stale install; build
   `scovox_core scovox_mapping` together.
2. **Check `state` in the first CSV rows.** A run stuck in `WAIT_FOR_MAP` is
   now indistinguishable from a healthy one by row count alone.

**§3.9's scope claim has also narrowed.** It argued the planner is insulated
from scovox because it consumes only the decoded `ScovoxMap` from dscovox. That
is no longer the whole interface: the planner now also subscribes to an
`OccupancyGrid` published directly by `scovox_node`
(`~/global_planning_map`, §3.10), so scovox's 2D projection is on the planner's
critical path for candidate acceptance and reachability. The rev-8 argument
above is unaffected — the projection shares no code with the wire codec — but
"the planner does not touch scovox" is no longer true as stated.

---

### 3.10 The 2D planning map in sim — enabled, and what it changed (2026-08-15)

Sim runs now enable the planner's 2D planning map (`use_planning_map:=true`).
Getting there required a new publisher, because neither existing one works:

| Candidate source | Why not |
|---|---|
| `/<r>/dscovox_node/planning_map` — the planner's **default** topic | Does not exist. `dscovox_node` publishes no `OccupancyGrid` at all; it exposes only a `GetOccupancyGrid` service. |
| `/<r>/scovox_node/planning_map` | A 20 m robot-centred crop, and its extent **is** `simple_nav_3d`'s local-planner window (that planner has no window param of its own). Consuming it rejects every frontier beyond ~10 m — `isCellOccupied` treats out-of-bounds as occupied — and widening it puts the whole world through the local A* on the control path. |
| `mode: persistent` (fixed envelope) | Disables the `ScovoxMapBinary` publish that multi-robot map sharing depends on. |

**Resolution:** `scovox_node` gained a second, world-fixed planning-map
publisher on `~/global_planning_map`, sharing the projection code with the
rolling one but with its own envelope, resolution and publish period. Default
off. The harness sizes it from `ROI_HALF` (3× = 150 m @ 0.4 m/cell) and
rate-limits it to 1 Hz — the projection + inflation run on `scovox_node`'s
integration thread, so the period is a real-time budget, not just bandwidth.

Three switches were pinned deliberately, each guarding against a *silent*
change of meaning rather than a crash:

- **`done_coverage_source:=scovox`.** The `auto` default switches to the 2D
  planning map the instant one is received. Enabling the map would therefore
  have swapped the termination metric from 2.5D column coverage to 2D cell
  coverage against the same `done_unknown_fraction`, changing when every run
  ends — with nothing in the output saying so.
- **`cost_grid_radius_cap_m:=10 × ROI_HALF`.** The reachability filter is dead
  code while `use_planning_map` is false, so its auto cap
  (`candidate_max_radius + 2` = **10 m**) has never been exercised. That bound
  is sized for *polar* candidates; frontier centroids have no range limit, and
  `FRONTIER_ONLY=1` removes the polar set entirely. Left on auto, every frontier
  more than 10 m of walked distance away returns `kInfCost` and is rejected —
  exploration degenerates to 10 m hops or stalls, and looks like a result.
  (The auto default is arguably wrong for any frontier-driven planner; it is
  latent in the field only because field runs publish no planning map.)
- **A hard readiness gate on the topic.** With `use_planning_map=true` the
  planner *blocks in INIT* until a map arrives, so a topic typo or a launch arg
  defaulting to `0.0` presents as two planners that never start — an hour in,
  with no error anywhere.

**Comparability note:** this makes sim runs non-comparable with the 2026-08-02
campaign CSVs. Both the candidate free/occupied filter and the cost-grid
reachability filter were entirely inactive there, so `rejected_by_unreachable`,
`rejected_by_minpos` and goal selection do not mean the same thing.

### 3.11 Adversarial review — defects found and fixed (2026-08-15)

Two independent reviews of the §3 build work. Confirmed defects, all fixed:

| # | Defect | Consequence if unfixed |
|---|---|---|
| 1 | `if gates.py … \| tee …; then` tested **tee's** exit status (no `pipefail`) | Every gate failure reported success; `GATES_STRICT` was dead code |
| 2 | `RECONNECT_MODE=off` was not implementable — `reconnectModeFromString` falls back to `RENDEZVOUS` on any unknown string | The control arm silently ran `rendezvous` twice; both pre-registered contrasts compared an arm with itself |
| 3 | `exploitation_enabled` left at the yaml's `true`, scheduler still running | §3.5 requires exploration-only; comms severity would drive exploitation stalls straight into the primary endpoint |
| 4 | Metrics sampler re-ingests the fused map + walks every voxel every 5 s on a **single-threaded** executor | Delays the 1 Hz beacon past `coord_claim_ttl_sec` → peers read MISSING with the link healthy → spurious manoeuvre charged to the radio |
| 5 | `reconnect_distance_m` was range-**to-goal**, documented as distance travelled | Inverts the overhead comparison: arriving reads ~0, giving up early reads large |
| 6 | The unknown fraction — the primary endpoint *and* the DONE criterion — was in no CSV column | Runs could not be scored on their own stopping rule |
| 7 | `gate_qos` skipped topics with no publisher (`if not pubs: continue`) | The "relay never formed" case its docstring names first went undetected |
| 8 | `gate_overflow` used a **wall** timeout against a **sim-clock** publisher | Healthy runs flagged invalid for low RTF |
| 9 | Nothing asserted the independent variable varied | A COMMS=1 run where the link never dropped yields a complete, plausible dataset that is really a control run |
| 10 | No hang gate; the new timer rows made the specified "no CSV row for N minutes" detector unable to fire | A permanently stuck planner produces a full-length, flat, plausible run |
| 11 | No run manifest | Arm, seed and `tx_power_dbm` existed only in the operator's scrollback |
| 12 | `/hmr_comms_sim/stats` not bagged | `backlog_bytes` / `drop_airtime` — the only evidence for the B0a gate and the shared-airtime confound — unrecoverable |
| 13 | `rx_qos_depth: 500` raised only the **publisher**; `dscovox_node` read at `KeepLast(50)` | Reconnect burst discarded at the reader; silent voxel loss, no counter |
| 14 | planning_map stored with no frame check, then indexed with raw world XY | Correct only while `map→odom` is identity; nothing enforced or reported it |

**Since fixed (2026-08-15):** the 8 `pine_*` includes now count toward
attenuation. `tree_name_substring` (scalar, `"tree"`) became
`tree_name_substrings` (list, `["tree", "pine"]`), and an `<include>` is matched
on its instance name **or** its model URI, so `model://cmu_pine_tree` qualifies
either way. Verified at startup: 88 tree positions loaded from `flatforestv2`,
equal to the world's real trunk count (80 `Oak tree_*` + 8 `pine_*`), with no
double-counting of pines that match both substrings. Both species share the one
`tree_radius_m` / `tree_attenuation_db`; neither model has an analytic trunk
(both collide as meshes), so the world affords no per-species radius. All 8 pines
lie inside the ±50 m sim ROI, so this changes link traces rather than being
cosmetic — **any link statistics captured before this date are not comparable**
with runs after it. Quantified over 20k uniform robot pairs in ±50 m at the
shipped defaults: mean trees per link 1.399 → 1.540 (+10 %), 13.3 % of link
geometries change count, and **2.27 % of links flip 0 → ≥1 trees**, which is the
one that matters — that flip switches the BER model from AWGN to Rayleigh, a
discontinuity rather than a 12 dB nudge. At the §4 operating point it costs
≈2 pp connectivity, worth ≈1.1 dB of `tx_power_dbm`. **Corollary for §4:** a
calibration performed against 80 trees is now ~1 dB optimistic. It likely still
lands inside the 55–70 % acceptance band, but it must be re-checked, not assumed.

Because the change is uncommitted, `-dirty` alone could not tell an 80-tree run
from an 88-tree one — both carried the same provenance string. The harness now
writes `comms_trees_loaded=` into the run manifest and suffixes each `-dirty`
marker with a hash of that repo's working tree, so runs either side of this fix
are distinguishable from the artifacts alone.

Two same-class defects fixed alongside, found by the review: `"pine"` does not
match `pinus_pinaster` (20 instances in each of three shipped forest worlds), so
`"pinus"`, `"euca"` and `"ulex"` joined the defaults — `realistic_forest` went
from 45 matched plants to 110. And the node now logs the distinct names it
*rejected*, so a world whose species this build cannot see says so at startup
instead of quietly reporting a plausible number. Shrubs stay excluded on purpose:
`tree_attenuation_db` is a trunk figure.

The perimeter walls stay uncounted, and this is correct rather than a
concession: the 56 `wall_*` includes form a **closed square at ±52 m**, and the
sim ROI is ±50 m (`ROI_HALF`). A straight segment between two points inside a
convex region cannot cross that region's boundary, so no robot-to-robot link
that stays inside the ROI can ever be occluded by a wall — the attenuation they
would contribute is exactly zero, not merely small. (Two caveats, both benign: a
link hugging the boundary can bring a wall centre within the ~2 m Fresnel+trunk
width, and the point-obstacle test would model a 7.5 m panel as a point anyway.
Neither arises while the planner keeps goals inside the ROI.)

Not fixed, carried as known limitations: `prox_hold_count` measures "close **and**
connected" because peer poses arrive only via the gated intent beacon, so it
under-reports the close approaches pursuit is predicted to cause; `hmr_sim` is
copy-installed, not symlink-installed, so edits to `comms_sim_params.yaml`
require a rebuild to take effect.

### 3.12 Voxel resolution 0.10 → 0.20 (2026-08-15)

At 0.10 m a run long enough to reach coverage termination is not
computationally feasible on this box, and the failure mode is silent. The
lidar path carves free space along the **whole** ray (`carve_band: -1`) out to
`max_range: 20`, so every beam writes ~200 voxels. The fused map passed 12.7 M
voxels by t ≈ 550 s and was still growing linearly with explored area.
Everything downstream scales with it — dscovox integration, the full
`ScovoxMap` publish, and the planner's ingest plus whole-grid walk — and past
a few million voxels the planner's map subscription simply stops keeping up:
one robot ran **three minutes on a frozen map**, still driving, still logging
steps, its coverage curve flat while its teammate's kept climbing, with no
error anywhere. `voxel_resolution_m` is now a `simple_nav_3d` launch argument
(default 0.10, unchanged) and the harness passes 0.20. Same run then holds
~700 k voxels and reaches a *lower* unknown fraction at equal sim time.

This is **not** a free knob under `COMMS=1`, and it is why §4 must be
calibrated at the resolution the campaign runs at: the `scovox_bin` deltas are
what the radio carries, so an ~8× change in payload is an ~8× change in the
offered load the airtime model must move. No `tx_power_dbm` transfers across it.

### 3.13 Exploration deadlocked before it explored (2026-08-15)

The first control pilot never explored. Both robots entered a two-point
oscillation within a minute and stayed in it — atlas between goals 0.87 m
apart, bestla 0.63 m — for the whole run. Nothing reported it: every process
healthy, steps advancing (so the harness hang gate, which watches the step
counter, saw progress), CSV filling, unknown fraction pinned at 0.87 having
moved 0.001 in 100 sim s. It would have produced a full-length run with a flat,
plausible coverage curve.

Cause is the same `info/(ε+cost)` degeneracy described in §2.1. Three
independent things then make "the nearest frontier" a fixed point, and all
three needed fixing (each opt-in, each defaulting to the shipped behaviour):

1. **`candidate_min_goal_dist_m`** (harness: 4.0). The nearest frontier is
   usually under a metre away, and a VLP-16's ±15° vertical FOV covers a band
   ~0.3 m tall at that range — the voxels that made it a frontier are
   physically unobservable from it. `goal_xy_tolerance` (0.4 m) is far too
   small to catch this, and the failed-goal blacklist never fires because
   every one of these goals is *reached*, on time.
2. **`frontier_z_lo_offset_m` / `frontier_z_hi_offset_m`** (harness: 5.7 /
   2.5, giving absolute z ∈ [0.2, 1.5]). The frontier search band was the full
   9.5 m ROI slab; a lidar's free space is a wedge bounded by its vertical FOV,
   so the whole upper and lower surface of that wedge is frontier at every
   range, permanently. With fix 1 alone the ping-pong just moved out to 4.3 m.
3. **`visited_goal_radius_m` / `visited_goal_ttl_sec`** (harness: 6.0 / 180).
   Even with a consumable band, trunk shadows regenerate clusters however
   thoroughly an area is observed, so two neighbouring clusters trade places as
   "nearest" forever. With fixes 1+2 the robots still alternated between two
   goals 4.7 m apart for six consecutive steps.

With all three, goals walk out across the ROI and effective speed roughly
doubles (0.14 → 0.31 m/s). **This changes what every arm does**, so no run
recorded before 2026-08-15 may be pooled with one after, and the three values
are recorded in every run manifest.

### 3.14 The link was lossy, not just slow — first mode pilot discarded (2026-08-15)

The whole design assumes the radio is a **delay**: a robot's deltas queue during
an outage and *drain at the next contact*, which is what B0a is (§5.1, "the gap
opens during each outage and snaps shut at each contact"). The emulator does not
guarantee that. `reliable_queue_max_bytes` defaulted to 64 MiB and on overflow
`hmr_comms_sim_node` drops the **oldest** queued delta and never retransmits it,
so those voxels are gone from the receiver's merged map for the rest of the run
— the gap can never snap shut, and the receiver's unknown fraction, which is
both the primary endpoint and the DONE criterion, is biased upward.

It fired at the calibrated operating point. Counted from the bag rather than
inferred: atlas published **3345** `scovox_bin` deltas and bestla received
**3190** — 155 missing, matching the emulator's own `drop_overflow=153`. The
`off` arm lost **1180 in both directions**. Severity tracked the arm's outage
exposure (off 0.77 disconnected, modes 0.29), which makes the loss a
*treatment-correlated* confound rather than symmetric noise: the arm that
disconnects most also loses the most map, and then explores worse, and drifts
further apart. All three phase-3 mode runs are discarded (archived under
`_invalid_overflow/`).

What survives, and why:

- **Phase 1 controls (tx=160): clean, zero drops.** The link never leaves the
  top tier, so nothing queues. The 0.4922 floor and the 0.55 criterion stand.
- **Phase 2 calibration: unaffected.** The sweep computes link states from
  replayed geometry — SNR/bandwidth/connected never depend on queue occupancy —
  so `tx_power_dbm = -14.0` is still the right severity.
- **Solo run (tx=-60): 6193 drops, result still valid.** At 0.00 % connected
  nothing is ever delivered, so dropping a queued delta and holding it are
  indistinguishable in outcome. The gate cannot know that; the reading of it
  can. Treat a deliberately-blacked-out arm's overflow count as uninformative.

Two fixes, both in the harness rather than the emulator's defaults, since the
required depth is offered-load × longest-outage and both move with voxel
resolution and `tx_power_dbm`:

1. `reliable_queue_max_bytes` is now a `comms_sim.launch.py` argument, set by
   `RELAY_QUEUE_BYTES` (default **1 GiB**) and recorded in the manifest.
   Sizing: ~120 kB/s per direction (2 Hz × ~60 kB) against a full T=3600 s
   blackout is ~430 MB, so 1 GiB carries 2.4× margin at ≤2 GiB of RAM for both
   directions. This raises the cap so the gate stops firing for real — the
   overflow counter remains a hard gate.
2. **`GATES_STRICT=1` now fails the run on the run-time gates, not only the
   bring-up gates.** It previously covered bring-up alone, so all three invalid
   runs exited 0 and the campaign driver logged `OK rc=0` on the line directly
   below its own `RUN INVALID` banner. The verdict is now written to the
   manifest as `run_gates_verdict=`, and `run_campaign.sh` re-runs an INVALID
   cell instead of skipping it as complete — otherwise resume would preserve
   corrupted cells forever.

The general lesson is the one worth keeping: **a validity gate that cannot fail
the run is a log message.** The gate was correct, fired every 30 s for the
entire campaign, and printed the exact diagnosis; nothing consumed its verdict.

### 3.15 The radio was lossy for the wrong reason, and no gate ever said so (2026-08-15)

Two defects found by an independent adversarial review, both of which invalidate
every phase-3 run made before this entry.

**The port changed the radio model.** `hmr_comms_sim_node` was derived from the
Gazebo plugin `hmr_sim/src/HMRNetSim.cc`. There, `ber`/`per` are computed *only
to be published*, and delivery is decided by the bandwidth state machine alone:
PDR 1.0 below the SNR ≥ 2 dB boundary, 1e-8 above it. The port promoted that
diagnostic into a per-message coin flip. It is wrong on its own terms —
`AwgnQam64Ber`/`RayleighQam64Ber` hardcode `spectral_efficiency = 72e6/20e6`,
i.e. 64-QAM at the **top** rate, regardless of which tier `NextBandwidth`
selected. A link that correctly downshifted to 7.2 Mbps was charged the error
rate of a gear it was not in: slow *and* lossy for one weak signal, the opposite
of what rate adaptation is for.

Measured at the calibrated `tx_power_dbm = -14.0`: median BER on **connected**
samples 0.116, which kills a 200-byte beacon with probability 1−1e-70. Robots
exchanged **3544 intent beacons; 7 arrived (0.2 %)**. Since `last_contact_` is
fed by intents, peer records never formed, `startPursuit` declined every time
("record of 'bestla' is 1611s old (max 180s)"), and **`PURSUE` appears in zero
CSV rows campaign-wide** — the §2.3 mode-collapse trap, fired structurally
rather than by bad luck. The artifact was also backwards: 60 kB map deltas flowed
anyway because the reliable path capped retransmission cost at `retx_cap = 4.0`,
so the model killed small control messages and spared large bulk ones.

Fixed by deferring to the state machine, as the plugin does: gear 0 delivers
nothing, any other gear delivers everything at that gear's speed, and
degradation reaches the experiment as airtime pressure and backlog. Verified end
to end at 20 m and tx = −14 (link BER 0.0749): `relayed 142, drop_ber 0`.

**No run-time gate has ever adjudicated a run.** All 10 runs on disk logged
`gateswatch ignored SIGINT`; not one contains a `watch` or `outage` line. The
cause is not handler logic — SIGINT never reached a handler. The harness starts
the watcher as a background job of a *non-interactive* shell, which POSIX gives
`SIGINT` as `SIG_IGN`, and CPython honours an inherited `SIG_IGN` rather than
installing its `KeyboardInterrupt` handler. Teardown then escalated to SIGKILL,
which cannot be caught, taking the summary with it. So `gate_outage_occurred` —
the check that **the independent variable actually varied** — has never returned
a verdict, while runs carried `run_gates_verdict=CLEAN` earned on bring-up gates
alone. Registering SIGINT explicitly is the fix; the harness now reads a missing
watch summary as SUSPECT rather than CLEAN.

**Consequences for §4 and §5.2.** The −14 dBm calibration was scored on
`connected`, which under the ported model did not imply delivery; under the
restored model it does, so the sweep's premise holds again and no re-sweep is
required. Every phase-3 number recorded before this entry is void.

**Corrections to earlier entries, from the same review.** §2.1's control
makespans (1362/1938/1969) are *run-end* times; the criterion crossings are
1350.2/1895.4/1950.1 — two different definitions were mixed. The floor is
0.49218 (atlas) vs 0.49235 (bestla), not identical to four decimal places.
"0.52 crossed at ≈ 2100–2200" is actually ≈ 1777, which *understates* how cheap
tightening the criterion is. §5.2's "bestla never reached 0.55" is literally
correct — its sub-threshold samples begin at t = 3678, 21 s past the horizon —
but it survives by 0.00056, and §2.5's own rule (T ≈ 3× control makespan ≈
5800 s) was not followed; at the specified horizon bestla crosses and the B0b
defence collapses. Treat it as unresolved, not as measured.

**Open, not yet fixed:** `analyze_runs.py` has no horizon bound (it parses one
and never uses it), so it scores censored runs as finished from teardown data;
it pools arms across transmit powers; it discards an outage still open at trace
end (1225 s in one pursuit run); `reconnect_elapsed_sec` is read one row too
late and is always −1. Re-running INVALID cells (§3.14 fix 2) is selection on a
post-treatment variable and should be restricted to infrastructure failures.
And §3.14's bag evidence can no longer be re-derived — the bags were deleted to
reclaim disk, which was a mistake.

### 3.16 The outcome is settled before the treatment is applied (2026-08-16)

The p3b pilot (8 cells, 4 arms × 2 seeds, tx = −14 dBm, T = 5400 s) ran clean:
8/8 complete, 0 failures, every cell CLEAN on a *real* run-time verdict — the
first block in which §3.15's watcher fix let the outage gate adjudicate at all.
All four mechanisms were then observed executing end to end. Pursuit: *"'bestla'
out of comms (record 9s old) → chasing its trail head (−42.07, −18.03), budget
240s"* then *"team reconnected mid-chase (1/1) → re-planning against merged
map"*. Rendezvous: *"team incomplete (0/1 peers) → returning to last-connected
anchor"* then *"team reconnected en route"*. Hybrid: chase first, barrier
fallback. Under the §3.15 radio the peer record was 1611 s stale and `PURSUE`
appeared in zero rows campaign-wide; it now fires with records 5–19 s old.

The mechanisms work. The experiment still cannot see them, for a structural
reason.

`finishOrRendezvous` is reached only at exhaustion. In `p3b_pursuit_seed1` atlas
crosses the 0.55 criterion at t = 855.4 s and enters `PURSUE` at t = 900.8 s —
**45 s after the stated outcome is already determined**. No arm can move
time-to-criterion at any n, with any radio. The pilot's crossing times (off
1395 s, rendezvous 1046 s, pursuit 1045 s, hybrid 1686 s) are samples of
trajectory luck, not an ordering.

Two further properties of that metric, both measured here:

- **It is badly conditioned.** Near 0.55 the coverage curve is flat —
  `p1control_off_seed3` stays within 0.0004 of 0.5660 for 126 consecutive
  seconds, a slope of ~8e−05 per second, so a coverage difference of 0.001
  becomes 12 s of crossing time. Read the same five runs two ways: CV 28.7 % on
  time-to-criterion against 3.0 % on mean unknown fraction over a fixed window.
  That, not exploration variance, is the "19 % noise floor" of §2.5.
- **Better conditioning does not rescue it.** An integrated metric over the
  exploration window is a well-conditioned readout of the phase in which every
  arm is identical.

The outcome must therefore be **post-exhaustion map completeness** — the final
`unknown_fraction`, the quantity the manoeuvre exists to improve — with
time-to-criterion demoted to descriptive. Post-exhaustion *gain* (saturation
minus final) is a useful diagnostic of whether the manoeuvre did anything, but
must not be the primary: it rewards saturating at a worse level, and in this
pilot ranked pursuit far above `off` where absolute completeness put them
0.0016 apart.

### 3.17 The robots choose how bad their own comms are (2026-08-16)

Every p3b cell ran at the single calibrated power tx = −14 dBm. Realised
pre-treatment outage duty across those cells: **0.426 to 0.867**. The §4 sweep
moved duty from 0.45 at −6 dBm to 0.92 at −26 dBm, so run-to-run variation at
*one* power spans nearly the whole range that 20 dB of transmit power was used
to control.

The cause is geometry, not the radio. Median inter-robot distance ranges 25 m to
68 m and mean trees-on-link 0.66 to 1.43 across the same cells; path loss goes
as 20·log10(d) with 11.98 dB per intervening trunk. Where the robots go sets
their own link budget, and where they go depends on what they have merged, which
depends on the link. Severity is an outcome, not a treatment.

Consequences for measurement:

- **Duty must be cut before any robot stops exploring.** A chase closes distance
  and lifts the duty, so an arm partly determines its own reported severity —
  hybrid reads 0.745 over the full run against 0.867 pre-treatment. Only the
  pre-treatment figure is admissible as a covariate.
- **The treatment applies only conditionally.** `shouldRendezvous` requires the
  peer to be *missing* at exhaustion. In `p3b_pursuit_seed2` both robots reached
  exhaustion with the team present, the manoeuvre correctly declined, and the
  arm degenerated to `off`. An arm's mean therefore mixes fired and not-fired
  runs and dilutes whatever effect exists.

### 3.18 The pilot finds no arm effect, and says so (2026-08-16)

Final map completeness (lower is better), with pre-treatment severity and the
number of manoeuvres actually fired:

    arm          seed  duty_pre  team_final    gain  fired
    hybrid          1     0.867      0.5030  0.0563      5
    hybrid          2     0.783      0.5365  0.0137      3
    off             1     0.574      0.5179  0.0088      0
    off             2     0.773      0.5097  0.0377      0
    pursuit         1     0.426      0.5163  0.0447      2
    pursuit         2     0.743      0.5487  0.0009      0
    rendezvous      1     0.784      0.5257  0.0137      1
    rendezvous      2     0.642      0.5266  0.0306      3

    arm mean:  off 0.5138 | hybrid 0.5198 | rendezvous 0.5261 | pursuit 0.5325

**The `fired` column above undercounts — see §3.22.** It was read off the
sampled CSV `state` column, which cannot see manoeuvres shorter than its ~5–10 s
period. Log-derived, hybrid fired **7** and **4**, not 5 and 3; the other six
cells are unchanged. The conclusion below does not turn on it — the arm that
fired most still did not win — but no later analysis should use that column.

**The control arm has the best mean final map.** Within-arm spread (hybrid
0.0335, pursuit 0.0324) exceeds the spread of the arm means (0.0187), so nothing
here is separable at n = 2. Regressing completeness on duty_pre across all eight
cells gives a slope of +0.0028 over a 0.44 range of duty — flat. Severity does
not explain the outcome either.

The sharpest single number: `off` on seed 2 recovered 0.0377 of map after
exhaustion **with zero manoeuvres**, more than hybrid (0.0137) or pursuit
(0.0009) managed on the same seed *with* their policies firing. Opportunistic
reconnection during a long run does much of what the manoeuvres are for.

This is a negative result at pilot scale, not a refutation. It is consistent
with a real effect hidden by §3.16 and §3.17, and equally consistent with no
effect. What it does settle is that **the design as specified in §2.5 cannot
answer the question**: a fixed-power, fixed-n, unadjusted comparison of arm
means is measuring severity and trajectory luck. Phase 4 needs a design that
spans severity deliberately and fits completeness against duty_pre per arm,
conditions on whether the manoeuvre fired, and scores the phase in which the
arms actually differ.

One hypothesis worth pre-registering rather than discovering post hoc: a harsh
link leaves each robot holding map the other lacks, so there is more to recover
when contact is made, while an easy link has already shared everything
continuously and leaves the manoeuvre nothing to do. If so the value of a
reconnection policy *rises* with severity — a sharper claim than "policy beats
no policy", and one this pilot is too small to test.

### 3.19 Transmit power was never a legal variable (2026-08-16)

Every severity level in this plan so far was produced by moving
`tx_power_dbm` — the control at 160, the calibrated condition at −14, total loss
at −60, and the whole §4 sweep from −6 to −26. That is not a variable. Both
robots carry the same radio, its power is fixed hardware, and no field
experiment can turn it down. In the link model

    SNR = tx_power + 101 − 49.17 − 20·log10(d) − 11.98·N_trees − fade

`tx_power` is a constant offset, so lowering it 8 dB is *arithmetically* the
same as moving the robots twice as far apart or adding two thirds of a trunk —
but it is not the same experiment, because a power cut changes nothing about
where the robots can drive or what the lidar sees.

The arithmetic shows what the knob was really standing in for. The shipped radio
is 30 dBm and these runs used −14: 44 dB of detuning, and 44 ÷ 11.98 = **3.7 tree
trunks**. The transmit power was a silent proxy for tree density all along.

It had to be, because the world cannot break a real radio. At 30 dBm with the
geometry these runs produced (25–68 m apart, 0.66–1.43 trunks on the link):

    30 + 101 − 49.17 − 33.98 − 16.77 = 31.1 dB   →  top tier, never disconnects

`flatforestv2` carries 88 trees over the ±50 m ROI, about 88 stems/ha — open
woodland. Reaching the 2 dB cutoff at 50 m needs ~3.8 trunks on the link, i.e.
roughly 240 stems/ha, which is an ordinary managed forest. **There were no
outages to study, so the radio was detuned until outages appeared.**

§3.17 already had the evidence and stopped short of this conclusion: at one
fixed power, realised duty ran 0.43–0.87, while the entire 20 dB sweep produced
0.45–0.92. The environment alone was already generating the whole range the
radio knob was being used to generate.

Consequences. Severity must come from the forest — tree density is the honest
treatment, and it is honest precisely because it also changes navigation and
occlusion, which a power cut cannot imitate. The ideal-comms control is now
`--comms 0` (no emulator, one broadcast domain), never a magic 160 dBm radio.
Phase 4 as written is withdrawn. A denser world needs its own Phase 1, since the
0.4922 floor and the 0.55 criterion were calibrated in `flatforestv2` and do not
transfer.

What survives: §3.16's mechanism verification is about the planner, not the
radio. What does not: the severity ladder, and any reading of §4 as calibration
of a physical variable.

### 3.20 Inter-robot map divergence separates the comms conditions (2026-08-16)

Time-to-criterion is settled before the arm is reachable (§3.16) and is badly
conditioned near 0.55; final unknown_fraction scores the stopping rule, because
each condition stops at a different sim time. Scored instead at *matched* sim
time, the better-informed robot advances at nearly the same rate however bad the
link is. Degraded comms does not slow the team down — **it pulls the two robots
apart**. The quantity that moves is

    divergence(t) = |unknown_fraction_A(t) − unknown_fraction_B(t)|

taken as the median over matched times (`sim/map_divergence.py`):

    condition                    median divergence      n     × logger noise
    ideal comms (tx=160)         0.00000 – 0.00027      3        0.0 – 0.1
    realistic   (tx=−14)         0.01090 – 0.05816      8        3.7 – 56.5
    no comms    (tx=−60)         0.01724 – 0.02135      2       20.7 – 26.2

No overlap; the worst ideal run is 41× below the best realistic one.

This metric is the only one tried with an **exact null**: if every delta reaches
both robots, both MapCaches hold the same union and report the same
unknown_fraction. That is a property of delivery, not of the planner — which is
what makes the number usable at all here, because the ideal cells ran on
`explo_planner 2d9cd44-dirty` / `hmr_sim c7608f1` against the current
`a75f14b` / `2e9e6c1`, two of the three repos dirty. Their *coverage rate* is
not comparable across that gap and must not be quoted; their divergence is,
because no planner version can make two identical maps disagree. The check
confirms it: ideal-comms divergence lands at 0.0–0.1× the logger's own noise
floor, i.e. below one sampling step.

**The link causes it, and the response is lagged.** A between-condition gap on
its own proves only that these runs differ somehow, so the causal claim was
tested inside single runs, pooled over the 8 p3b cells. Interval-by-interval it
looks dead: `spearman(connected fraction, change in divergence) = −0.043` over
577 adjacent 20 s intervals. That is not absence of an effect but its timescale —
a delta queued during an outage is delivered on reconnect and only moves
`unknown_fraction` once the robot actually covers ground, which a 20 s window
cannot resolve. Widened to sustained transitions (link holds its new state 60 s)
and a ±120 s comparison window:

    sustained outage onset    n=22   mean Δdiv +0.01282   16/22 rose   sign p≈0.026
    sustained reconnect       n= 7   mean Δdiv −0.00887    5/7 fell    sign p≈0.23

Outages drive the maps apart, reconnects pull them back, and the two means
straddle zero with opposite signs as the mechanism requires. The outage
direction is significant on a sign test; the reconnect direction is the right
sign but n=7 — sustained reconnects are rare when duty runs 0.43–0.87 — so it is
suggestive only, not established.

The practical consequence: divergence is an integrator, not a live link monitor.
Do not read it as an instantaneous connectivity signal; it lags by minutes.

Two limits that the table hides.

**Aggregate over the run; never classify an instant.** Under realistic comms
divergence collapses to zero on every reconnect — 7 of the 8 realistic runs touch
values below the ideal runs' *peak* at some point. The run median separates
cleanly; a spot reading does not.

**Divergence is not monotone in severity.** It is zero when the link is perfect
and near-zero again when the link never comes up (both robots stay equally
ignorant, symmetrically); it peaks at *partial* connectivity, where one robot
takes a merge burst the other misses. So it answers "is comms intact?" and "how
unequal is the team's knowledge?", not "how bad is this link?". Read beside the
laggard's own coverage — low divergence also describes two robots that are
equally uninformed, which is what pursuit produced in §3.18.

Still owed: an ideal-comms control at the *current* build via `--comms 0`. The
structural argument above is what licenses using the stale cells in the meantime,
and it should not be leaned on any longer than necessary.

### 3.21 "Time to exploration saturation" is not a separate metric (2026-08-16)

Proposed as a replacement for time-to-criterion, on the reasoning that it is not
read off a flat part of the curve and it sits upstream of the arm. Measured, it
turns out to be the same event under another name.

Every run on disk leaves EXPLORE at an unknown fraction of 0.511–0.549 — all of
them just under the 0.55 criterion. `explo_planner_node.cpp:2194` is why: the
transition fires on `unk < done_unknown_fraction_`. **Saturation time IS
time-to-criterion**, so §3.16 applies to it unchanged.

It also fails to separate the conditions, and fails in the direction that shows
why:

    condition     n   mean saturation   range
    perfect       3        1775 s       1389–1995
    realistic     8        1130 s        820–1445
    none          2        2580 s       2055–3106

The ranges overlap (1389–1445) and *perfect is slower than realistic*, which is
backwards. That is the §3.20 build gap: saturation time is a coverage-rate
quantity, so unlike divergence it does **not** survive comparison across the
stale control build. It cannot be quoted until the `--comms 0` control is re-run.

This also corrects an earlier reading in this file's working notes: perfect
1702 s vs realistic 2374 s, reported as a clean 1.39× dose-response. That used
only the two `off` cells for realistic, one of which (`p3b_off_seed2`, both
robots crossing at 3476 s) is a large outlier. With all 8 realistic cells the
ordering reverses. There was never a monotonic time effect.

There is no genuine saturation to measure instead. `frontier_voxels` grows
monotonically for the whole run (93k → 1.54M in `p3b_off_seed1`); it counts the
boundary of an expanding map, not work remaining, and it never falls. The
planner's own comment at line 1000 says why: *"EIG scores don't fall sharply as
the map saturates (the FOV raycast always finds some unobserved voxels at the
cone edge), so unknown fraction is the reliable signal here."* The exploration
curve never flattens to zero, so 0.55 is an arbitrary line drawn across a curve
that has no natural end — which is exactly why crossing it is badly conditioned.

To make true exhaustion observable at all, `done_unknown_fraction <= 0` disables
coverage termination (line 1002) and runs go until the planner genuinely runs
out of candidates. That is a different and much longer experiment — the solo run
needed 3880 s with the criterion active — and it would first need the §3.19
denser world, since candidate supply in an open ROI is what the frontier-only
caveat at `run_explo_sim_rviz.sh:82-87` warns about.

Polar candidates were already off for every run on disk
(`candidate_enable_polar=false`, from the `FRONTIER_ONLY=1` default), so nothing
in the results above involves the polar grid.

### 3.22 Counted per event, the manoeuvre evidence is 7 firings (2026-08-16)

Every campaign run so far scores the *run*. The manoeuvres are what the plan is
about, so `sim/manoeuvre_events.py` re-reads the same runs with the **firing**
as the unit. It changes the picture at every step.

Figures below are the snapshot at 24 firings (p3b complete, p4mild seed 1
complete, p4mild seed 2 still running). Re-run the tool for current numbers;
the structure of the finding does not depend on the count.

**Firings are undercounted by the CSV, badly.** §3.18 read the CSV `state`
column, which is sampled on a ~5–10 s timer, and manoeuvre episodes are
routinely shorter than that: 7 of the 24 leave no manoeuvre row at all
(durations 0.0, 0.0, 0.7, 1.1, 1.8, 2.3, 4.4 s). A state-column census
undercounts, and undercounts the *fast* reconnections preferentially — exactly
the ones the modes claim as their advantage. Firings must be parsed from the
planner log; `analyze_runs.py` now reports both counts and flags the four runs
where they disagree.

Two parsing traps, both live in this data. `Rendezvous: exploration ended [...]
with full team present -> DONE` (`explo_planner_node.cpp:2877`) is the manoeuvre
correctly *declining* — it matches any grep for `Rendezvous:` and inflates
counts. And `holdForTeam` logs `Rendezvous: waiting for team at the barrier`
**in the pure pursuit arm** (`explo_planner_node.cpp:2866-2871`), so classifying
firings by log wording files pursuit's park-and-beacon under rendezvous. The arm
comes from the manifest; the kind comes from the decision line. There are four
kinds, not three: `chase`, `meeting_point`, `anchor_return`, `hold`.

**9 of the 24 armed against a teammate that was in radio range.** Classifying
each firing by the link trace over the 10 s before it armed, and cross-checking
the *peer's* own `Heartbeat resumed after X s suppressed` lines (§3.8 — a robot
cannot observe its own silence, so this must be read from the other log):

    genuine outage   link down >50% of the 10 s before arming     15
    planner artifact link up AND peer heartbeat suppressed         4
    unexplained      link up, no suppression found                 5

The artifacts armed with the channel *busy* — 5.7, 12.5, 9.2 and 5.8 messages/s
being delivered at the moment the robot declared its teammate lost. The sharpest
single case is in `p4mild_hybrid_seed1`: atlas armed a chase against a teammate
**7.1 m away**, link up across the whole window, 5.9 messages/s flowing, and the
manoeuvre ended 0.0 s later. §3.8 warned this confound existed; measured, it is
17–38 % of all firings.

**8 more are degenerate — they arm and dissolve before the robot moves.** Eight
firings ended within 5 s having travelled under 1 m, two of them logging
`Reconnect manoeuvre ended after 0.0 s sim`. The test that ARMS the manoeuvre
(peer missing) and the test that ENDS it (team present) disagree within a single
cycle, so the manoeuvre fires and is cancelled on the next tick. No trail is
followed, no waypoint reached, no path planned. This is a defect in its own
right — an arm/disarm race — and it is also what makes the chase look instant.

What survives:

     24  firings parsed from the logs
     15  armed during a genuine outage
     10  of those not degenerate
      8  of those actually drove a route (holds park in place by design)
         — across 5 runs, 6 robot-runs; up to 79 m driven, 6–180 s to outcome

**Eight events, clustered in six robot-runs, is the entire evidence base for the
reconnection path planning.** No re-analysis of these runs will make it more.
Restricted to genuine outages the raw pattern is: chases reconnect fast (n=7,
median 6 s), anchor returns slower (n=2 at tx −14, median 51 s), meeting points
did not reconnect at all (n=2: one arrived-and-waited, one unreachable within
budget), and both pure-pursuit holds ran to the horizon without reconnecting.
That is a hypothesis to test, not a result — the n are 2 and 7, and they cluster.

Two structural findings fall out that are not about sample size:

- **Pure pursuit's hold has no timeout and censors the run.** In
  `p4mild_pursuit_seed1` both robots declined their chase (records 652 s and
  204 s old, gate 180 s), held at their current pose, and waited at the barrier
  with `rendezvous_max_wait_sec: 0` until T. The run censored at 5807 s. §2.5
  predicted this; it has now happened.
- **The arm/disarm race must be fixed before any manoeuvre experiment.** A
  manoeuvre that ends 0.0 s after it arms is not a policy being exercised.

This is descriptive mechanism evidence. Events cluster hard — five come from one
robot-run — so the effective sample is the run count, and nothing here compares
arms.

*Method note.* The planner log stamps are system clock (rcutils does not follow
`use_sim_time`) while every CSV is absolute sim time. They are reconciled per
run by least squares over the `Step N logged:` lines, each of which has a
matching `LOG_STEP` row carrying that step's sim time: 88–149 anchors per run,
spread across the whole run, max residual 0.70 s. The anchors do not depend on
any manoeuvre having happened, which matters — a run whose only firings were too
short to be sampled still gets a time base.

### 3.23 At the shipped radio, realistic comms IS perfect comms (2026-08-16)

The first comparison of the two conditions with **nothing detuned**: both arms
carry `tx_power_dbm = 30.0`, the shipped radio, on both robots. The only
difference is whether the emulator is in the path at all — `--comms 0` leaves
both robots on their direct topics in one broadcast domain, `--comms 1` puts the
relay hop, delay, airtime budget, fading and disconnection logic between them.
Same build, same world, paired seeds, `off` arm so no manoeuvre confounds it.
6/6 cells clean.

Inter-robot map divergence (§3.20), median over matched sim times:

    p5perfect_off_seed3   0.00000      p5real_off_seed2   0.00000
    p5perfect_off_seed1   0.00008      p5real_off_seed1   0.00003
    p5perfect_off_seed2   0.00027      p5real_off_seed3   0.00003

**Completely interleaved, every cell at 0.0-0.2x the logger noise floor.** The
realistic arm is if anything the tighter of the two. There is no effect to
measure.

That is not because the link is flawless. It does drop:

    run                 duty    outages   median   longest
    p5real_off_seed1  0.0256          7     6.6 s    16.6 s
    p5real_off_seed2  0.0078          3     4.4 s     7.6 s
    p5real_off_seed3  0.0296          9     3.4 s    19.4 s

19 outages pooled, driven by geometry exactly as the model says — they happen
when the robots reach 82-91 m apart with up to 5 trunks on the path, and the
deep fades take SNR to −20 dB. But every one of them is **under 30 s, median
3.8 s**, and the reliable relay drains the backlog on reconnect long before
either robot's coverage can fall behind. The team loses ~35 s of connectivity
out of a ~1700 s run and not one voxel of shared knowledge.

Set against the detuned runs this is the whole story of the project so far:

    condition                     duty          divergence      x noise
    perfect (--comms 0)           0.000         0.00000-0.00027   0.0-0.2
    realistic (30 dBm, honest)    0.008-0.030   0.00000-0.00003   0.0
    detuned (tx = -14 dBm)        0.426-0.867   0.01090-0.05816   3.7-56.5

Detuning the radio 44 dB multiplied outage duty by 15-100x. **That factor, not
any property of comms, is what every "severity level" in this plan measured.**

Consequences, and they are the point of the exercise:

- **§3.20's separation stands, and is now correctly attributed.** Divergence
  does cleanly separate a *degraded* link from a healthy one. What it separates
  is not realistic-versus-perfect; it is detuned-versus-honest.
- **The reconnection manoeuvres have nothing to do here.** They fire at
  exploration exhaustion against a peer missing beyond the claim TTL. A 4 s
  outage never reaches that test. In this world, at this radio, the manoeuvre is
  unreachable by construction — which is the deepest form of §3.16's objection.
- **A one-line correction to a claim made earlier today.** A 300 s probe at
  30 dBm showed 100 % connectivity and was read as "the honest radio never
  drops". It does; the probe simply ended while the robots were still 8.5 m
  apart. Short runs cannot see this because the mechanism is dispersal. The
  full-length runs are the evidence.

The remaining question is unchanged and now has a number attached: an honest
comms experiment needs a world where outages last minutes rather than seconds.
`flatforest_dense` (250 stems/ha against flatforest's 74, `densify_forest.py`)
is built for exactly that — it puts ~3.9 trunks in the Fresnel corridor of a
50 m link where 3.83 is the cutoff. It has NOT been run yet, and it needs its
own §2.1 calibration first.

---

### 3.24 Which metrics can see comms at all (2026-08-16)

§3.23 reported a null. A null is only informative if the instrument that
produced it can detect an effect when one exists, so before running the dense
world the whole candidate endpoint list was put on one bench:
`sim/comms_metrics.py`. Eleven metrics per run, every time-indexed quantity read
at a **matched sim-time horizon** — the minimum over all runs in the comparison
of the last time both robots had logged — because each run stops at its own
coverage threshold and an endpoint read at run end scores the stopping rule as
much as the condition.

Two of the eleven are not endpoints and are meant to be read first:

- **`peer_visible_frac`** — manipulation check. The fraction of planner rows in
  which the robot could see a live peer. If this does not separate, the emulator
  did nothing in this world, and no downstream metric can be carrying a real
  effect; any that appear to are noise.
- **`plan_ms_p50`** — negative control. Median planner solve time has no causal
  path from the radio. If it separates, the two arms differed in CPU load rather
  than comms, and §3.8's warning applies: executor lag is indistinguishable from
  a radio outage in this output.

**The instrument check.** Run against the withdrawn detuned cells (§3.19), where
a genuinely broken link is known to exist, the battery does see it:

    metric                perfect    detuned    separation
    peer visible frac      0.9953     0.2468    CLEAN
    map divergence         0.0001     0.0086    CLEAN     (+11155%)
    t to matched level    740.1 s   1178.0 s    overlap      (+59%)
    m to matched level    514.7 m    805.3 m    overlap      (+56%)
    plan ms p50            420 ms     355 ms    overlap   (control, quiet)

So the battery is sensitive, the negative control stays quiet, and at n≈3 only
two metrics separate *cleanly*: the manipulation check and map divergence.
Effort costs move by more than half in the right direction but their between-run
variance swallows it — they need seeds, not a better definition.

**The same battery on §3.23's Phase 5 pair finds nothing, including the
manipulation check**: `peer_visible_frac` 0.9953 perfect vs 0.9748 realistic. The
emulator was in the path and had almost nothing to do. That is the correct
reading of the Phase 5 null — not "these metrics are blind" but "there was no
treatment". Every other row overlapped, and the largest apparent gap
(`deconflict rej frac`, +194%) sat at permutation p = 1.000.

**A metric this bench threw out.** The first effort metric was team distance
divided by the unknown-fraction reduction since t=200 s. It separated cleanly —
in the *wrong direction*, scoring the detuned runs as 24% more efficient. The
cause is the denominator: by t=200 s the perfect-comms robots have already
merged maps, so they start the window with less room left to improve, before any
robot has done extra work. **A ratio anchored to a condition-dependent baseline
measures the baseline.** It was replaced with endpoints matched on coverage level
rather than time — time-to-level and distance-to-level at the deepest unknown
fraction every run reaches — which have no such anchor.

**Report separation, not significance.** At n = 3 vs 3 the exact permutation null
over C(6,3) = 20 splits pairs every split with its complement, so the smallest
attainable two-sided p is 2/20 = **0.10**. Nothing at this n can be significant,
and the tool prints its own floor beside the p column to stop that number being
read as evidence. The reportable quantity is whether the two groups' ranges
overlap at all; a clean split at n = 3 is a reason to run more seeds, not a
result.

### 3.25 The forest, not the radio, is the severity knob (2026-08-16)

The dense stand was run at the **shipped 30 dBm on both robots**, changing only
the world. Measured over full runs rather than probes:

    world              stems/ha   conn%   SNR med   trees med   outages   longest
    flatforest               81    97.4    41.0 dB        1.0       3-9      19 s
    flatforest_dense        250    44.5    -1.3 dB        4.0     37-38     861 s

`p6dense_off_seed2` spent **73% of its run disconnected** with a median SNR of
−24 dB and a single outage lasting 860.6 s. That is past the 180 s
`pursuit_staleness_max_sec` bound, past the 5 s claim TTL, and 45× the longest
outage the sparse world ever produced — obtained without touching a single robot
parameter. Both dense cells still reached `all_done` with CLEAN gates, so the
coverage endpoint survives the denser stand and the pilot is not being run into a
world that cannot terminate.

This is the answer to §3.19's open question. Severity is available, it is
environmental, and it is roughly an order of magnitude bigger than anything the
sparse world could reach.

**The 0.55 criterion transfers, and the reason it is safe is narrower than
that.** The worry (§3.23) was that a denser stand shadows more voxels and would
move the achievable floor. Measured, it does not: dense runs bottom out at
0.5268–0.5477 against the sparse world's 0.5407–0.5410, so 0.55 is reachable in
both and every dense cell so far ended `all_done` rather than censored. But the
comparison this criterion is used for — realistic vs perfect (§3.26), and the
four modes (Phase 7) — is **entirely within one world**, so the threshold enters
both arms as a shared constant and cannot confound them whatever it means
physically. The number that would need real calibration is a *cross-world*
coverage comparison, and this plan does not make one: a dense-world
`unknown_fraction` counts trunk voxels as observed that the sparse world does not
contain at all, so the two are not the same quantity and must not be pooled.

### 3.26 Comms does not slow exploration — it slows completion (2026-08-16)

The endpoint question, answered. **Degraded comms does not slow exploration
down.** Across every condition on disk, from a 97%-connected link to a
27%-connected one, the first robot to reach the coverage criterion does so at
1336–1845 s and the ranges overlap completely. The radio does not make a robot
explore more slowly.

What it delays is **completion**, because the run does not end until the
*second* robot reaches the criterion:

    condition                  leader crosses   laggard trails by   laggard drove
    sparse perfect              1336-1701 s            0-10 s          1.4-4.3 m
    sparse realistic 30 dBm     1440-1800 s             0-4 s          2.7-6.5 m
    dense realistic 30 dBm      1365-1845 s          10-1060 s        3.6-378.5 m
    sparse detuned -14 dBm      1365-1385 s          10-2045 s        8.4-736.8 m

**This is §5.2's pre-registered primary endpoint, and it survives contact with
the data.** §5.2 already defined the endpoint as sim time until *every* robot
reaches saturation, and its B0b probe already showed the signature: bestla never
arrived despite driving 1271 m, 75% further than atlas. What §3.26 adds is the
decomposition — the leader's time is near-invariant, so the **entire** comms
effect lives in the gap — and a price for it.

**The laggard is not waiting on the radio.** The expected picture was a robot
stalled until a backlog drained, then jumping. That is not what happens. During
its lag window the laggard drives at 0.357–0.364 m/s against a 0.320–0.352 m/s
whole-run average — full speed, the whole time. It is out covering ground to
learn for itself what its partner already knew. In the dense forest that cost
**378 m of driving**; detuned, **737 m**. Under perfect comms the same gap costs
2–5 m.

So the cost of realistic comms is not latency and not lost coverage. It is
**duplicated robot-metres by the robot that was cut off**, and a mission that
cannot be declared finished until that robot has driven them.

Consequences for the design:

- **`laggard_lag` is promoted to the reported primary**, with `lag_dist` (the
  cost in robot metres) beside it and `t_lead_cross` as the near-invariant
  companion that shows the effect is not in exploration rate.
- **Censoring is now load-bearing, not a nuisance.** A run whose laggard never
  arrives is the *worst* case for its condition, not a missing observation.
  Dropping it biases the comparison toward "no effect" exactly when the effect
  is largest, so `comms_metrics.py` records those as lower bounds and flags any
  group median that is itself a lower bound. §5.2's B0b probe is precisely such
  a run.
- **This is the mechanism the reconnection modes claim to fix.** Their value is
  supposed to be moving the laggard's merge earlier and making it scheduled
  rather than lucky. That claim is now stated as a number they must move:
  laggard lag, 10–1060 s in the dense forest under `off`. Phase 7 tests it.

One caution carried forward. `unknown_fraction` cannot distinguish the laggard
re-covering its partner's ground from exploring genuinely new ground — both look
identical in the laggard's own map. The quantity that does separate them is
distance-to-matched-coverage-level, which on the detuned check read 515 m for
perfect against 805 m degraded (+56%).

> **CORRECTION (2026-08-16, same day, from the first dense ideal-comms cell).**
> The caution immediately above is not a caution. It is the dominant effect, and
> it falsifies this section's second claim.
>
> `p6denseperfect_off_seed1` — dense world, `--comms 0`, both robots holding an
> identical map — crossed the criterion at **2700 s**, against 1365–1845 s for
> the *realistic* dense runs. Its laggard lag was 0 s exactly, as predicted, but
> its leader was far slower, not invariant. Its `unknown_fraction` sat at
> 0.5504 → 0.5502 for 1000 s while the robots drove 254 m.
>
> Late-run information yield, t = 1400 → 2200, shows why:
>
>     condition             robot    drove    new voxels   voxels/m
>     dense perfect         atlas   253.7 m       31 912        126
>     dense perfect         bestla  168.2 m       23 183        138
>     dense realistic s1    bestla  174.3 m      142 823        820
>     dense realistic s2    bestla   73.5 m      132 099       1796
>
> The ideal-comms robots drive **further for a sixth of the information**.
> `unknown_fraction` is measured on each robot's OWN map, so it **rewards
> redundant coverage**: a degraded-comms robot has cheap unknown right beside it
> — the ground its partner already covered — and harvests it at 6–14× the yield
> per metre, while a robot that already holds the union has only the genuinely
> hard, far-away voxels left. **The per-robot criterion therefore systematically
> favours degraded comms late in a run.**
>
> What stands: the laggard measurements. Under realistic comms it trails by
> 10–1060 s and drives up to 378 m at full speed, not idling — measured within
> the realistic runs and untouched by this.
>
> What is withdrawn: "the leader's time is near-invariant, so the entire comms
> effect lives in the gap." The leader's time moved, in the opposite direction,
> for an artifactual reason.
>
> What this means for the design: **the stopping rule is not a team-knowledge
> criterion.** §5.2 defines the endpoint as every robot's *own* map reaching
> saturation, and that definition is satisfiable by duplication. The endpoint
> that is not is one that charges for travel —
> distance-to-matched-coverage-level — or one evaluated on the union map, which
> these `--record 0` runs cannot reconstruct. Until that is settled, no
> comparison between a shared-map arm and a partitioned-map arm should be read
> off crossing times alone.
>
> n = 1 for the dense ideal arm; two more cells are running. But the yield
> figures are within-run measurements and do not depend on n.
>
> **THE CORRECTION ABOVE IS ITSELF WITHDRAWN IN PART (§3.27).** It was written
> off `p6denseperfect_off_seed1` alone. Seeds 2 and 3 crossed at 885.6 s and
> 1423.9 s, so the ideal arm's leader range is 886–2700 s against the realistic
> arm's 1365–1845 s: **overlapping, with the ideal median FASTER** (1424 vs
> 1550). There is no systematic leader slowdown. Seed 1's 2700 s plateau was one
> run, not the arm. `vox_per_m_late` likewise overlaps at n = 3 and points the
> other way (ideal 2428 vs realistic 1347). What survives is §3.26's original
> claim — the leader is near-invariant — and the observation that seed 1
> plateaued. What does not survive is "the criterion systematically favours
> degraded comms", which remains a live hypothesis with one run behind it.

### 3.27 The dense comparison separates — and is confounded by a silent QoS drop (2026-08-16)

> **PARTIALLY WITHDRAWN by §3.28 (2026-08-17).** The separation result in the
> first half of this section stands. The QoS-overflow diagnosis in the second
> half — everything under "the realistic arm's merged maps are holed" — is
> **wrong** and was refuted by the re-run it demanded. The map gap is undrained
> backlog, not lost voxels, and the dense cells did **not** need re-running.
> Read §3.28 before citing anything below about `rx_qos_depth`.

Phase 6, 6/6 cells clean: the dense forest at the shipped 30 dBm (`p6dense`)
against the same world with no emulator (`p6denseperfect`), `off` arm both sides.

**The manipulation finally bites.** `peer_visible_frac` 0.9959 vs 0.4327, a clean
split — against Phase 5's 0.9953 vs 0.9748, which was no treatment at all. The
environment lever did what §3.25 said it would.

    metric              perfect            realistic          sep
    peer visible frac   0.996/0.996/1.000  0.433/0.523/0.317  CLEAN
    laggard lag s       0.11/4.48/4.99     10.1/125.0/1060.5  CLEAN
    laggard drove m     0.04/2.10/4.15     4.6/45.4/378.5     CLEAN
    map divergence      0.0000 all three   0.0011/0.0000/0.0002 CLEAN
    t LEADER cross s    2700/886/1424      1845/1550/1365     overlap
    plan ms p50         295/290/330        280/260/300        overlap (control quiet)

So the leader's crossing time does NOT separate — §3.26's original claim stands
and the §3.26 correction's counter-claim does not. The whole measurable effect
is again in the laggard.

**But the realistic arm's merged maps are holed, and the loss has no counter.**
Final disagreement between the two robots' copies of the merged map:

    p5real   sparse realistic     0.05 %   0.01 %
    p6denseperfect                0.04 %
    p6dense  dense realistic      1.76 %   1.49 %   1.78 %   <-- all three seeds

30–40× the others. It is not the reliable backlog: `reliable_topics` carries
`scovox_node/scovox_bin`, the 64 MiB→1 GiB pre-relay queue never overflowed, and
`drop_overflow` is zero. It is `rx_qos_depth: 500`, whose own comment predicts
this exactly — *"samples past the depth are dropped SILENTLY: drop_overflow only
sees the pre-relay queue, so the loss would land as missing voxels in the merged
map with no statistic anywhere saying so."* Deltas run ~2 Hz, so the 861 s outage
queues ~1720 messages against a depth of 500. The depth was raised 100→500 for
the SPARSE world, whose worst outage was 19 s ≈ 40 messages. The dense world is
45× that. `dscovox_node`'s own `scovox_bin_qos_depth` (also 500) must be raised
with it or the burst simply overruns the subscriber instead.

**Consequence: Phase 6's separation is real but its size is not attributable.**
Part of the laggard lag and all of the residual divergence may be permanently
missing voxels rather than delayed ones — and the config comment says outright
that this "would confound the map-merging experiments". The dense cells must be
re-run with both depths sized to the run length, not the sparse world's outages.
Phase 7 was auto-started into the same world by the phase-6 chain and was
stopped 38 s in for this reason.

**Two lesser reading hazards in the same table.** The matched horizon collapsed
to 1060 s (set by the fastest ideal cell) while runs reach 2875 s, so every
horizon-clipped metric — `vox_per_m_late`, `unknown_at_dist`, divergence, the
`unknown_*` family — is measured over the first third of the longest runs and
says nothing about late-run behaviour. `laggard_lag`, `lag_dist` and the crossing
times are computed over the full run and are unaffected. And `vox_per_m_late`'s
window here (773–1060 s) is not "late" at all, which is why it disagrees with the
hand-cut t = 1400–2200 slice in §3.26's correction.

---

### 3.28 WITHDRAWN: the map gap is drained backlog, not lost voxels (2026-08-17)

§3.27 concluded that the dense cells' 1.5–1.8 % merged-map disagreement was a
silent `rx_qos_depth: 500` overflow, and demanded a re-run before Phase 7 could
proceed. The re-run was done. **It refuted the diagnosis.**

    rx_qos_depth / scovox_bin_qos_depth = 500     1.76 %   1.49 %   1.78 %
    rx_qos_depth / scovox_bin_qos_depth = 4000    3.58 %   0.10 %

3.58 % at depth 4000 is worse than every run at depth 500. Deepening a queue
cannot make an overflow worse, so overflow is not the mechanism. The arithmetic
in §3.27 was sound (861 s × ~2 Hz ≈ 1720 deltas against a depth of 500) and the
config comment did predict exactly this failure — which is precisely why it was
believed. A prediction that fits is not a measurement.

**Ruled out second: a measurement artifact.** §3.27's number came from comparing
each robot's *last* CSV row, so a robot that logged longer would look like it had
a fuller map. Recomputing at the latest **common** sim time moves nothing
(3.58 → 3.60, 1.76 → 1.76, 1.49 → 1.49); the two planners stop within 0.0–7.5 s
of each other.

**What it actually is.** Traced through a run the gap is transient, not
cumulative — percent disagreement sampled every 100 s over the last 600 s:

    dense realistic   27.9  28.8  36.9  38.3   7.0   2.5    opens, then drains
    dense realistic    8.0   8.0   6.8   6.7   5.0   1.1
    dense realistic    1.2   0.7   7.8   5.3   1.0   0.1
    dense ideal        0.0   0.0   0.0   0.0   0.0   0.0    never opens at all

The gap opens **during** an outage, which is the treatment working rather than
failing: each robot keeps mapping from its own sensors while the peer's deltas
sit undelivered in the emulator's reliable queue, so the two merged maps are
legitimately different for as long as the radio is down. On reconnect the backlog
drains and the gap collapses. A 38 % mid-run gap healing to 2.5 % is a link
recovering. Under perfect comms it is flat zero, because there is nothing to
drain.

So the end-of-run number is not an integrity measure. It is **how much backlog
was still draining at the instant the run hit `all_done`**, and it scales with
outage severity — sparse 0.01–0.05 %, dense up to 3.6 %, ideal 0.03–0.09 %. That
makes it a treatment-intensity reading and a legitimate **secondary** map-
completeness metric, which is how `modes_compare.py` now reports it (`map_end`
alongside `map_peak`).

**Three consequences.**

1. **§3.27's re-run demand is void.** The dense cells were never corrupted, so
   Phase 6's separation is fully attributable after all. The laggard lag is
   delayed voxels, not missing ones.

2. **Failing runs on this was backwards.** `map_agreement.py` was written as a
   strict gate at 0.5 %. With `GATES_STRICT=1` and `run_campaign.sh` aborting
   after three consecutive failures, and 2 of 3 cells failing it, the gate was on
   course to kill the 12-cell Phase 7 matrix over a non-defect — and it would
   have discarded preferentially the cells where the comms treatment bit
   *hardest*, leaving a matrix biased toward runs where the radio barely
   mattered. It is now report-only.

3. **Queue depth stays at 4000, but as insurance, not as a fix.** Overflow is
   unobserved, not refuted: `drop_overflow` only ever sees the relay's own
   pre-relay queue, so nothing in this experiment measures the reader-side burst
   independently. 240 MB per queue is cheap on a 62 GB box. The sizing rule in
   `comms_sim_params.yaml` is kept.

**What would still be a real defect**, and is worth keeping the peak column for:
a gap that opens and never closes while the link is up. Deltas carry absolute
voxel state and the receiver snapshot-replaces, so any voxel observed again
self-heals; permanent loss can only survive in voxels never revisited. `end` far
below `peak` is a drained backlog; `end ≈ peak` with the link long restored is
not.

**Method note.** The failure mode here was not the wrong arithmetic — it was
treating a mechanism that *explained* the data as a mechanism that *caused* it,
and then hard-coding it into a gate before testing it. The test that settled it
cost one re-run and one 40-line script.

---

### 3.29 The pipeline's own noise is larger than the effect under test (2026-08-17)

Three adversarial reviews of the Phase 7 design converged on one number, and it
was already sitting on disk.

**`seed` does not seed the world.** It reaches only the comms emulator
(`run_explo_sim_rviz.sh:651`, inside `if [ "$COMMS" = "1" ]`). The scenario, the
spawn poses and the planner carry no RNG. So an **ideal-comms** campaign run at
three different seeds is three runs of *one identical configuration* —
`p6denseperfect_off_seed{1,2}` even share a planner build, and their manifests
differ in exactly one inert line. Their team completion times:

    t_team    890 s     1429 s     2700 s      <-- 3.03x spread, CV 45 %

**That band is wider than any arm difference this campaign has produced.** The
Phase 3b deltas that looked like a clean 2× win for every manoeuvre — −1212,
−1305, −1241 s against control — all sit inside it. They were never an effect.

**It is the endpoint, not the physics.** The same three runs, re-read at higher
thresholds:

    unknown<=0.55    890  1429  2700     3.03x    CV 45.4 %
    unknown<=0.60    594   685   955     1.61x    CV 20.6 %
    unknown<=0.65    499   550   600     1.20x    CV  7.5 %
    unknown<=0.75    225   229   250     1.11x    CV  4.7 %

By 0.55 the coverage curve has gone nearly flat — one series spent 77 minutes of
sim time to gain three percentage points — so time-to-threshold inverts a flat
function and converts ROS/Gazebo scheduling jitter into minutes of apparent
difference. The amplification varies ~900× *between seeds of the same arm*, which
is why the variance is not just large but wildly heteroscedastic.

**Power.** At CV 0.56 and n=5 per arm, an exact permutation test on medians has
80 % power only against a **~70 % speed-up**. The effect's own ceiling is the
measured penalty of realistic versus perfect comms — the most a perfect
reconnection policy could possibly recover — and in the dense world that came out
**−24 %, p = 0.70, with the wrong sign**: realistic comms finished *faster* than
perfect comms. Detecting a plausible 10–20 % effect would need roughly 86–382
runs per arm, against ~30–60 min per run.

**Two arithmetic traps worth recording.** The difference-of-medians statistic at
n=5 has only 12 of 252 splits tied at the extreme, so the smallest p it can ever
emit is 12/252 = 0.0476 — meaning a Bonferroni threshold of 0.0167 for three arms
is *unreachable for any data whatsoever*. And pairing on seed, which looks like
the obvious fix, is worse: the within-seed correlation is 0.21 (an 11 % cut in
the sd of the difference), while the paired permutation floor at n=5 pairs is
2/2⁵ = 0.0625, so p < 0.05 goes from improbable to arithmetically impossible.

**Consequence: Phase 7 is a pilot, not a ranking.** The honest output is an
effect size with its interval plus the noise floor it must be read against — at
n=5 that interval spans roughly ×0.47 to ×2.13, i.e. consistent with the arm
being twice as fast or twice as slow. `modes_compare.py` now prints the noise
floor from replicates, marks each delta INSIDE it or clearing it by *N*×, and
carries a power banner computed from the arms' own pooled CV.

**The one free rescue** is the threshold ladder. The noise collapses from CV 45 %
to 7.5 % between 0.55 and 0.65 *on data already collected*, so re-reading the
same runs at 0.60–0.65 costs nothing and is far better powered. 0.55 remains the
headline — it is the planner's own DONE rule and the completion time the team
actually pays in wall clock — but a ranking that exists only at 0.55 is a ranking
of where the threshold fell on each seed's curve, and the ladder says so out
loud. The ladder also distinguishes a flip *near* completion (the endpoint is
noise) from an early-vs-late reversal, which is a **result**: a manoeuvre spends
time not exploring, so it can trail at 0.70 and lead at 0.55.

**What must not be written**, however the matrix lands: that any mode "wins", is
"best", or is "recommended"; that an arm "outperformed the control (p = 0.048)"
— 0.0476 is the arithmetic floor, not evidence; or that a null result shows the
modes are equivalent, since with an MDE of 70 % a null excludes essentially
nothing.

---

### 3.30 The manoeuvre is terminal, so ~90 % of the endpoint is untreatable (2026-08-17)

§3.29 said the design is underpowered. This is *why*, and it is mechanical rather
than statistical.

**The trigger.** `finishOrRendezvous()` is the sole entry point to all three
modes — `startPursuit` is called from inside it
(`explo_planner_node.cpp:2917`) — and it has exactly three call sites:

    :2191   finishOrRendezvous("step-budget")
    :2244   finishOrRendezvous("coverage-saturated")
    :3987   finishOrRendezvous("step-budget")        (exploit path, off here)

Every one means **this robot has finished exploring**. There is no
mid-exploration trigger anywhere in the node. The reconnection "policy" is a
terminal barrier, not something that operates while the team explores.

> **SUPERSEDED 2026-08-17 (statement of fact only; the analysis below still
> holds for p7modes).** The node now has a mid-exploration trigger —
> `explo_planner_node.cpp:2852-2894`, added because *this* finding said the
> policy could not act until exploration was already over. It fires after
> `reconnect_midrun_silence_sec` (240 s) of continuous peer silence, capped at
> `reconnect_midrun_max_attempts` (6), with a cooldown stamped at manoeuvre
> end; a mid-run barrier that expires resumes exploring rather than ending the
> run. The 240 s threshold sits above the measured 180 s beacon-suppression
> tail so the trigger does not fire at healthy, silently-planning teammates.
>
> Everything below — the ~89 %/~11 % decomposition, the "no arm can alter the
> leader's own exploration" consequence, and the conclusion that ~90 % of the
> endpoint was untreatable — remains a correct reading of **the p7modes runs**,
> which were collected under the terminal-only policy. It is no longer a
> statement about the shipped planner. Any post-redesign campaign must
> re-derive the decomposition rather than cite this one, because the mid-run
> trigger is precisely an attempt to move mass out of `t_lead` and into the
> treatable window.

**Consequence.** No arm can alter the leader's own exploration, so `t_lead` is
identical in expectation across arms *by construction*. The manoeuvre can act
only in the window between the leader finishing and the laggard finishing, by
carrying a map backlog to the laggard. Decompose the endpoint:

    t_team  =  t_lead  +  lag
               ~89 %       ~11 %        (median 130 s of a median t_team 1191 s)

Measuring `t_team` therefore reads the treated signal through nine parts of
untreatable variance — and `t_lead` is the *noisiest* part, since it carries the
3.03× replicate band of §3.29. Every null result this campaign has produced is
consistent with a working manoeuvre whose effect was diluted below the noise by
the choice of endpoint.

**So `lag` is the mechanism-aligned endpoint.** `t_team` stays the headline
because it is the completion time the team actually pays in wall clock and it is
what was asked for — but any claim *about a mode* belongs on `lag`, and
`modes_compare.py` now prints the mechanism window above the comparison table so
this cannot be read past.

**It also explains the relabelled-control problem.** Because the trigger is
terminal, an arm declines whenever the team happens to be together at the finish,
and by then both robots have usually converged on the last frontiers.
`p7modes_rendezvous_seed1` is the live example: peer invisible for **56 %** of the
run — the comms treatment plainly bit — yet **zero firings**, the log declining
twice with `full team present -> DONE`. Its completion time (1891 s, against
3090 s for `off` at the same seed) is *not* evidence about rendezvous. It is two
draws from the same distribution, and the 1.63× gap between them is a second
measurement of the noise floor. Declining is the DEFAULT here, not an edge case.

**What this implies for the design, stated plainly.** To measure a reconnection
policy on exploration completion time you need either (a) a mid-exploration
trigger, so the policy can act during the 89 % of the run it currently cannot
touch, or (b) a scenario where robots are still separated when the first one
finishes, so the terminal trigger actually fires. Neither is a re-analysis; both
are changes to what is run. Until one of them exists, the campaign measures a
policy that mostly does not execute, and `fire` counts belong in every table.

---

### 3.31 Pure pursuit cannot reconnect in this world, and the reason is exact (2026-08-17)

§3.29 and §3.30 say the *timing* comparison is underpowered and mostly
untreatable. This section answers "which reconnection method works best" from
the mechanism instead, where the evidence is strong — because it is per-event
outcome data, not a difference of noisy medians.

> **COUNT CORRECTED (see the end of this section).** An earlier draft said
> "every hold event ever recorded failed" and later "7 of 7". The final tally is
> **6 hold events, 5 failed, 1 recovered — and the one recovery was the peer's
> doing, not the hold's.** The mechanism conclusion is unchanged and the reason
> is sharper; the arithmetic was wrong and is fixed below.

**Five of the six `hold` events failed outright.** Four of them here, from two
independent runs in two different worlds:

    run                    robot   stale  sep_m  trees   outcome           dist
    p4mild_pursuit_seed1   atlas    652   71.6     5     open_at_horizon    0 m
    p4mild_pursuit_seed1   bestla   204   47.2     1     open_at_horizon    0 m
    p7modes_pursuit_seed1  atlas    266   50.3     5     open_at_horizon    0 m
    p7modes_pursuit_seed1  bestla   194   55.1     5     open_at_horizon    0 m

Zero reconnections, zero metres driven, 4/4 still disconnected at the horizon.

**The causal chain, each link verified.**

1. The dense stand produces outages up to 861 s (§3.25).
2. So by the time a robot finishes exploring, the missing peer's contact record
   is **194–652 s** stale — every one of the four above.
3. `pursuit_staleness_max_sec` defaults to **180 s**
   (`explo_planner_node.cpp:479`, `:1293`). All four exceed it, so `startPursuit`
   declines: the trail head is too old to mean anything.
4. Pure pursuit's fallback is `holdForTeam` — park and beacon. This is
   deliberate, and the code says so: *"Pure pursuit has no agreed fallback point
   by design (that is the A/B against hybrid): a chase that never started waits
   right here."*
5. Both robots reach the same state, so **both park and neither moves**. The
   separation is frozen at 47–72 m, which is beyond the ~50 m link budget at this
   density. Nothing in the system can then change the geometry, and the link
   cannot reopen. It is a mutual-hold deadlock.

**The gate is 4.8× too short for the world it runs in.** 180 s against outages of
861 s. Pursuit is not underperforming here; it is inoperative by construction.

**The moving fallbacks do work**, which is exactly the A/B the design intended:

    fallback                      events  reconnected  arrived_waiting  moves?
    pursuit      hold                  4       0              0         no (0 m)
    hybrid       meeting_point         3       1              1         yes (48-79 m)
    rendezvous   anchor_return         7       5              2         yes (0-39 m)

Rendezvous' return-to-anchor is the most reliable: both robots converge on a
point they already agreed on, so it does not depend on a fresh peer estimate at
all — note its `stale` column is `--`, i.e. the staleness gate does not apply.
Hybrid's meeting point also moves both robots and reconnects, but it *does* need
a contact record to midpoint from, so it inherits part of pursuit's fragility.

**The paired A/B, same world and same seed — the cleanest result here.** Phase 7
put pursuit and hybrid through *identical* conditions at seed 1, and both hit the
same staleness failure. Only the fallback differed:

    cell                    chase              fallback        drove   outcome
    p7modes_pursuit_seed1   declined (266 s)   hold             0 m    open_at_horizon
    p7modes_pursuit_seed1   declined (194 s)   hold             0 m    open_at_horizon
    p7modes_hybrid_seed1    declined (237 s)   meeting_point   46 m    RECONNECTED

Same scenario, same seed, same radio realisation, same staleness regime, both
chases declined for the same reason. Hybrid's meeting point then **recovered a
90.2 m separation through 6 trees in 106 s**, while pursuit's hold sat still and
never recovered at 50–55 m. This is the A/B the design was built to run, and it
is decided by the fallback alone — not by a difference of medians, and not by
anything that needs n=5.

**But note the cost, because it cuts the other way on the headline metric.**
Hybrid was the SLOWEST arm at seed 1 (t_sim 3391 vs off 3090), and rendezvous —
which fired nothing at all — was the fastest (1891). A successful reconnection
costs driving time, and because the trigger is terminal (§3.30) the map it
delivers arrives after exploration is essentially over. So "the manoeuvre worked"
and "the team finished sooner" are not the same claim here, and on present
evidence they point in opposite directions. That is a real trade, not a
measurement artifact, and it is the strongest argument in this document for
reporting `lag` and `fire` beside any completion time.

**The one recovery, and why it strengthens the argument.** Phase 7's seed-3 cell
produced the only `hold` that ever reconnected — and it did so without the
holding robot doing anything:

    p7modes_pursuit_seed3  atlas  hold  stale 688 s  sep 75.5 m  RECONNECTED after 169 s

    during that 169 s window:   atlas (holding)      moved   1 m
                                bestla (exploring)   moved  83 m

atlas parked, as the policy specifies. bestla was still exploring, drove 83 m,
and wandered back inside the link budget. The link reopened because of the
**peer's** independent motion, not because of anything the hold did. Same policy,
same failure to move, opposite outcome — decided entirely by whether the other
robot happened to still be exploring.

That is the precise statement, and it is stronger than "hold never works":

    a hold contributes no motion (0-1 m in all six events), so it cannot itself
    restore a link. Whether it recovers is decided by the peer. When the peer is
    also holding, nothing moves and recovery is impossible by construction —
    seed 1, both robots held, both open_at_horizon; seed 2, held, and the mission
    was censored (§3.32).

**Final tally, corrected.** Six hold events across three worlds:

    p4mild_pursuit_seed1   atlas    open_at_horizon
    p4mild_pursuit_seed1   bestla   open_at_horizon
    p7modes_pursuit_seed1  atlas    open_at_horizon     both held -> deadlock
    p7modes_pursuit_seed1  bestla   open_at_horizon
    p7modes_pursuit_seed2  atlas    open_at_horizon     -> mission censored
    p7modes_pursuit_seed3  atlas    reconnected         peer drove 83 m; holder 1 m

Five failures, one peer-driven recovery, zero recoveries attributable to the
policy itself.

**Read the rates as mechanism, not as an effect estimate.** Events cluster hard
within runs, so the effective sample is the run count, not the event count, and
these rates are not independent samples. But the pursuit claim does not rest on a
rate: it rests on a deadlock that is visible in the code, predicted from the
parameter, and confirmed by all six hold events contributing 0-1 m — with a same-seed
control showing the alternative fallback recovering under the same failure.

**Actionable, in order of confidence.**

1. `pursuit_staleness_max_sec` must exceed the world's outage distribution, or
   the chase never arms. 180 s is a sparse-world number.
2. Pure pursuit needs a fallback that moves *someone*. A mutual hold cannot
   recover by construction, whatever the timeout.
3. Between hybrid and rendezvous the evidence is genuinely split, and the two
   worlds disagree — see the correction immediately below. Both work. Pursuit
   does not. That is the only ranking claim this campaign supports, and it is a
   mechanism argument; the timing comparison in §3.29–3.30 cannot support a
   ranking and must not be cited as if it did.

**CORRECTION (later the same day).** An earlier draft of point 3 read "on present
evidence rendezvous is the method to prefer", drawn from the p3b and p4mild
worlds where its anchor return reconnected 5 times in 7 events against hybrid's
1 in 3. The Phase 7 dense world reverses that:

    arm          fallback         events   outcome
    hybrid       meeting_point       2     2 reconnected
    rendezvous   anchor_return       2     2 arrived_waiting
    pursuit      hold                3     3 open_at_horizon

Two points of nuance before reading that as "hybrid wins". First, `arrived_waiting`
describes the *barrier*, not the radio: rendezvous' seed-3 cell drained its map
gap from a 26.63 % peak to 0.70 % and completed normally at 2200 s, so the link
plainly reopened and the backlog moved even though the barrier never counted the
peer. Functionally that cell succeeded. Second, these are 2 and 2 events, from 1
and 1 runs, clustered — nowhere near enough to separate two working policies.

So the honest position is: **hybrid and rendezvous both work, and which is better
is unresolved; pursuit fails in every world tested.** The pursuit conclusion is
robust because it rests on a deadlock in the code confirmed by all six hold
events across three worlds contributing 0-1 m of motion, plus a mission failure
(§3.32). Its single recovery was driven by the peer moving 83 m, not by the hold. The
hybrid-vs-rendezvous question needs a campaign designed for it, with far more
firings than a terminal trigger produces.

---

### 3.32 The deadlock costs a mission, not just a reconnection (2026-08-17)

§3.31 showed pursuit's hold never reconnects. `p7modes_pursuit_seed2` shows what
that costs, and the chain is verified end to end.

**It is the only censored run in Phase 7.** `run_end_reason=censored_at_T`,
t_sim 5812 against the 5800 s horizon — where the other seed-2 cells finished in
1256–2378 s. On the primary endpoint this is the worst outcome any arm can
produce, and pursuit produced it.

**The chain, each link measured.**

1. atlas finishes its coverage criterion at t≈985 and calls the manoeuvre. The
   chase declines: staleness 186 s against the 180 s gate (§3.31).
2. Pure pursuit's fallback is `hold`. Separation 73.4 m through 8 trees — beyond
   the ~50 m link budget at this density.
3. atlas parks. Its odometry is frozen at 322 m for the remaining **4870 s**, and
   the event outcome is `open_at_horizon`: the link never reopens.
4. So atlas's map backlog is never delivered. The two merged-map copies freeze:

       t_sim    atlas_vox   bestla_vox    gap     atlas_unk  bestla_unk
        1000     1754890      1604773    8.55 %     0.539      0.572
        3000     1755024      1606508    8.46 %     0.539      0.572
        5800     1755035      1607184    8.42 %     0.539      0.572

   Peak 18.80 % at t=700, and then flat for 4800 s. Compare the same-seed-1
   hybrid cell, whose meeting point reconnected: peak 19.24 % → **end 0.43 %,
   drained**.
5. The consequence is the run's outcome. atlas's copy sits at unk 0.539, *below*
   the 0.55 threshold. bestla's copy sits at 0.572, *above* it. The world needed
   to finish is already mapped — it is simply in the wrong robot's copy.
6. bestla explores alone for 4870 s and 2077 m, against atlas's 322 m, and still
   cannot cross. The run hits the horizon.

**This is the signature `map_agreement.py` was kept for.** §3.28 defanged the
0.5 % gate because a gap that opens on an outage and drains on reconnect is the
treatment working. It also said what a genuine defect would look like: *a gap
that opens and never closes*. That is exactly this — not a defect in the map
layer, but the map layer faithfully reporting a link that never came back. The
`peak` and `end` columns together distinguish the two cases, which is why both
are printed.

**What it establishes.** A reconnection policy is not merely a convenience for
map freshness: when it deadlocks, the team can fail to complete a mission it had
already collectively explored. The information existed; no policy moved it.
Pursuit's mutual hold is the only mode here that can produce that state, because
it is the only fallback in which *neither* robot moves.

**Caveat, stated plainly.** This is one run. The 3.03× noise band of §3.29 means
a single censored cell is not an effect estimate, and pursuit's seed-1 cell
completed normally at 2295 s. What is *not* noise is the mechanism: the frozen
8.4 % gap, the 4870 s of parked odometry, and `open_at_horizon` are direct
observations, and they are the predicted consequence of a deadlock identified
independently from the code. Treat the censoring as a demonstrated failure mode
with a known cause, not as a measured rate.

### 3.33 The redesign: a mid-run trigger and the fixes §3.30–3.32 demanded (2026-08-17)

With the p7modes matrix complete, the user authorised planner-behaviour changes
("check timing issue"). The design went through three adversarial reviews
before any edit; what shipped is the amended version, and the amendments were
not cosmetic — the reviews caught one planner-crashing bug and one
run-never-terminates loop in the original proposal, and refuted the proposal's
own endpoint change with the campaign's data. Full trail:
`explo_planner/sim/reconnect_redesign_2026-08-17.md`; the review verdicts are
summarised in its REVIEW OUTCOMES block.

**What changed in the planner** (all default-off / default-legacy; the sim
harness opts in per campaign, and every knob is echoed into `run_manifest.txt`
— the p7modes lesson that an unrecorded param is a confound):

1. **Mid-exploration trigger** (`reconnect_midrun_silence_sec`, default 0).
   §3.30 proved the manoeuvre was terminal-only, leaving ~89 % of t_team
   untreatable. Now a robot whose team has been continuously incomplete for
   the threshold interrupts exploration, runs its arm's manoeuvre via the
   extracted `dispatchReconnect()`, and — decisive design point — a failed
   mid-run attempt RESUMES EXPLORING (never DONE), on a short mid-run barrier
   cap (`reconnect_midrun_max_wait_sec`, 240 s). The re-dispatch cooldown is
   stamped at manoeuvre END, not dispatch (review: a dispatch-stamped cooldown
   expires during the manoeuvre — `missing_for` stays satisfied for the whole
   outage — and the "resume" becomes a one-tick interlude in an infinite
   loop). Attempt cap per run (6), live-count re-check at the trigger, and the
   threshold must clear the measured heartbeat-suppression tail (180 s;
   campaign uses 240 s) so a silently-planning healthy teammate cannot trip
   it.
2. **The chase can actually arm, and stale chases target the right thing**
   (`pursuit_staleness_max_sec` raised per-world via the harness;
   `pursuit_goal_stale_sec`, default 180). §3.31's gate autopsy stands: at the
   old 180 s gate no dense-world chase ever armed. But the review killed the
   naive fix (raise to 900 and chase as before): the budget formula's ceiling
   saturates, making it "always chase the stale goal for 240 s" — up to
   ~190 m in the wrong direction. Instead, past `pursuit_goal_stale_sec` the
   trail drops the peer's GOAL (dead hypothesis — the peer re-planned long
   ago) and drives to its CONTACT POSE alone, on a distance-true budget, and
   only when the budget covers the whole trail. Two chasers that both complete
   contact-pose trails end at the swapped contact pair — mutually within
   former link range, the same geometric argument the anchor return rests on,
   and one that does not decay with staleness. An uncoverable trail declines
   to the mode's fallback instead of dying mid-trail at an arbitrary
   disconnected point.
3. **The mutual-hold deadlock is broken** (`hold_escalate`, default false).
   §3.31–3.32: five of six holds never reconnected, one cost a mission. A
   TERMINAL barrier that expires now escalates ONCE to the last-connected
   anchor (sticky per-manoeuvre flag — the review showed a positional
   "if not already there" guard loops forever on an unreachable anchor) and
   waits a shorter `hold_escalate_wait_sec` (300 s) before giving up for
   real. Both robots converging on their own last-contact poses restores the
   pair geometry the link last worked at. Review caveat kept honest: the two
   anchors are same-window, not same-instant (one-way packet losses can
   displace one side's record), and a link that existed on a +2σ fade
   excursion can stay dark at the restored geometry — escalation improves the
   odds and bounds the cost; it is not a guarantee.
4. **Manoeuvre legs are exempt from `nav_max_timeout_sec`** — the 180 s
   ceiling was sized for exploration hops and covers only ~60 m of real
   driving; chases and returns were dying tens of metres short by
   construction (review attack 3/4). Distance-true budgets; the no-progress
   window remains the watchdog.
5. **Release flicker guard** (`reconnect_release_confirm_sec`, default 0).
   One live claim inside the 5 s TTL used to release a manoeuvre and reset
   the silence clock — crediting a "reconnection" on a range-edge flicker
   that drained no map deltas. Mid-run, that corrupts the primary metric; the
   campaign requires the release condition to hold 3 s.

**What the reviews refuted in the original proposal, kept on the record:**
moving the finish line to unknown ≤ 0.62 (my own §3.29 noise ladder was
computed on PERFECT-comms replicates; on realistic-comms replicates 0.62 buys
nothing over 0.55, 0.60 is the worst threshold of all at 4.45×, and only 0.65
is low-noise — where the mechanism window vanishes). So p8trigger keeps
termination at 0.55 and reads t@0.65 as the low-noise secondary via the
existing ladder. Also refuted: silence 300 s (fires ~0.4×/run — at an early
finish, ~0.09×/run), a 20-cell four-arm night (17.4 h at observed rates), and
the uninitialised `rclcpp::Time` in my trigger sketch, which would have thrown
on first firing (default-constructed Time is SYSTEM clock; subtracting it from
a sim-time now() is a runtime error — the reviewer found the codebase already
guards this idiom twice).

**p8trigger design:** arms off / rendezvous / hybrid × seeds 1–5 (pursuit arm
dropped: its verdict is established and §3.31-patched, its chase mechanism
still runs inside hybrid; 15 cells ≈ 13 h at observed cell rates, exact
permutation floor 2/252 ≈ 0.008), DONE_UNKNOWN 0.55, duration 5800 s,
MIDRUN_SILENCE 240, MIDRUN_MAX_WAIT 240, RECONNECT_RELEASE_CONFIRM 3,
PURSUIT_STALENESS 900, PURSUIT_BUDGET_MAX 900, HOLD_ESCALATE 1, one build
pinned before the first cell and no repo commits while it runs (the p7modes
matrix recorded four planner hashes for one binary — true but post-hoc
unprovable). The hypothesis the trigger makes testable at last: a mid-run
reconnection delivers the peer's queued deltas while they can still prune this
robot's remaining frontiers, so a mode that reconnects faster should now
finish sooner — t_team becomes treatable, and the off arm prices what
deliberate reconnection is worth over opportunistic contact.

**Defaults flipped to ON (2026-08-17, user decision).** The changes above
shipped default-off so that every pre-existing path stayed bit-identical.
That is the right default for a patch and the wrong one for a fix: the
terminal-only trigger it replaces *cannot* reconnect a team before the
exploration it was meant to shorten is already over, and a 180 s staleness
gate declined every chase ever asked of it in a world whose outages run to
861 s. Defaults are now `reconnect_midrun_silence_sec` 240 (was 0),
`pursuit_staleness_max_sec` 900 (was 180), `reconnect_release_confirm_sec` 3
(was 0), `hold_escalate` true (was false); `pursuit_budget_max_sec` stays 240
and `pursuit_goal_stale_sec` stays 180. Set `reconnect_midrun_silence_sec:=0`
to recover the old behaviour exactly.

Two couplings this exposes, both recorded rather than silently resolved:

1. **Chase budget must not outlast the barrier waiting for it.** A waiting
   teammate treats `pursuit_budget_max_sec` as the worst case it may assume
   about its pursuer. The harness sets `RDV_MAX_WAIT=600`, so the planned
   `PURSUIT_BUDGET_MAX=900` would have let a chase outlive the wait that
   justified it. **p8trigger uses 600** — still 2.5× the old ceiling, so the
   budget-saturation problem of §3.31 is relieved, and the invariant
   *chase budget ≤ waiter patience* holds by construction. The code default
   stays 240: a conservative worst case is the right thing for a number other
   robots reason about.
2. **Hold escalation is on but latent in the field.** It fires on terminal
   barrier expiry, and the field default `rendezvous_max_wait_sec=0` means
   wait forever, so no terminal barrier expires and no escalation happens.
   It acts only where a finite escape hatch is already configured (the sim
   harness, and any hardware yaml that sets one) — there it converts a
   give-up into one more attempt. Making the deadlock break unconditional in
   the field would mean revisiting wait-forever, which is a separate,
   deliberately-chosen policy and not this change's to make.

### 3.34 p7modes readout, n=3 — the pilot that measured its own instrument

12/12 cells, 0 failures, 11 `all_done` + 1 `censored_at_T` (pursuit/seed2,
§3.32). Build fairness **settled rather than assumed**: the matrix records four
planner commits, but all six pairwise `git diff` over `*.cpp *.hpp config/` are
empty, and all 12 cells record a clean (non-`dirty`) `git_explo_planner`. If
the tracked compiled sources are byte-identical across every recorded commit
and no cell ran with uncommitted changes, no commit difference can reach the
binary.

*Corrected:* I first corroborated this with "the installed binary predates cell
1 by ten days". That was wrong — this is a symlink-install workspace, `stat`
does not dereference by default, and the date read was the symlink's, not the
binary's. The real binary lives in `build/` and its pre-rebuild mtime is now
unrecoverable. The conclusion is unaffected (it rests on the diffs, not the
timestamp), but the manifest now records `sha256(explo_planner_node)` so binary
identity across a matrix is a recorded fact rather than a reconstruction.

**No arm is ranked on t_team, and none can be.** Pooled within-arm CV is 0.28 at
n=3, the exact permutation floor is 0.200 (nothing can reach p<0.05), and
replicates of one *identical* config span 3.03× at this threshold. `comms_metrics`
returns "NO metric separates off from hybrid at this n".

What the matrix does establish is *why* — decomposing t_team into the leader's
crossing and the laggard's lag, where only the second is reachable by a terminal
manoeuvre (§3.30):

| arm | Δt_team vs off | Δt_lead (unreachable) | Δlag (the only treatable part) |
|---|---|---|---|
| hybrid | +1830 s | +1885 s | **−55 s** |
| rendezvous | +310 s | +555 s | **−168 s (−55 %)** |
| pursuit | +415 s | +325 s | **−83 s** (1 censored; a lower bound) |

Every arm looks *slower* than the control on the primary metric and every arm is
*faster* in the only window it can act in. For hybrid the identity is exact:
1885 + (−55) = 1830. The headline number is, to within rounding, entirely the
leader-crossing draw — a quantity the arm provably cannot influence, because the
trigger cannot fire until a robot has already finished. This is §3.30 measured
rather than argued, and it is the justification for the mid-run trigger.

**Mechanism (manoeuvre_events, all 9 firings):**
- **Every chase declined — 6 for 6.** Staleness at arm 186–688 s against a 180 s
  gate. Pure pursuit's defining behaviour never executed once in the world it
  was built for (§3.31), and the `mid` column is 0 everywhere, as it must be for
  a pre-redesign binary.
- **Pursuit's fallback is a car park.** 3 of 4 holds gave up after ~600 s having
  travelled 0–1 m. Its map divergence is +595 % vs control — the split-map
  signature of §3.32.
- **Hybrid's meeting point is the one manoeuvre that works: 3/3 reconnected**, in
  79–108 s over 32–46 m. Rendezvous' anchor return went 0/2 in seed3 (644 s,
  742 s) and never armed at all in seeds 1–2.

**Caveat, stated because it cuts toward the arms:** realized comms are not
matched across arms — rendezvous ran at 0.303 peer-visible fraction vs the
control's 0.419, the one *cleanly separated* metric in the whole readout. Fading
is a function of (seed, tick) **and poses** (§4), so once behaviour diverges the
comms each arm experiences diverge too. Rendezvous cut lag 55 % while seeing
markedly worse comms; this is not a confound to explain away, but it does mean
arm-vs-arm severity is not controlled and n=5 will not fix that.

**Status: pilot.** It sized the noise, priced the mechanism window at 12.3 % of
the endpoint, killed one arm on mission-safety grounds, and refuted the
threshold change I proposed to fix it (§3.33). It ranks nothing.

---

## 4. Calibration — one severity (offline, no Gazebo, cheap)

Emulator fading is a function of `(seed, tick)`, independent of traffic
(exactly — separate RNGs), and runs against replayed poses. Two purity
caveats: a pair with no pose yet skips its RNG draw, so the trace's tick
alignment can shift by a tick or two at startup; and *connectivity* also
depends on poses and trees, so traces diverge across live arms once behaviour
does. Record `/<r>/odom_ground_truth` from 2–3
control runs, replay through `comms_sim.launch.py` sweeping `tx_power_dbm`
(single monotone severity scalar with a physical reading — a weaker radio;
everything else fixed), read `~/link_states` (col 8, **0-indexed**, of the 9
fields per pair = `connected`; 5 Hz). **Freeze `link_rate_hz`:** fade memory
(`fade_alpha` is per-tick) and the 8-tick connect hysteresis both scale with
it — changing the rate silently changes severity, breaking `tx_power_dbm` as
the single knob. The ≥ 3-of-8-sample hysteresis (~0.6 s) also floors the
achievable outage granularity.

**Acceptance — one `tx_power_dbm` value meeting all three:** duty cycle ≈
55–70 % disconnected; median outage > 5 s (claim TTL, §2.3); time-since-contact
at exhaustion < 180 s in most traces (§2.3 window). No second severity level is
calibrated — R3 cut the damage curve.

**DONE (2026-08-15): `tx_power_dbm = -14.0`.** Swept −6/−10/−14/−18/−22/−26
against all three phase-1 control bags (`calibrate_txpower.sh`, 18 points).
−14 is the *only* value passing all three criteria on all three bags:

| bag | duty | outages | median outage | tsc at exhaustion |
|---|---|---|---|---|
| seed 1 | 0.573 | 53 | 8.4 s | 26.0 s |
| seed 2 | 0.616 | 43 | 7.0 s | 0.0 s |
| seed 3 | 0.604 | 63 | 9.0 s | 23.6 s |

−18 passed on seed 1 alone (seed 2 median outage 4.4 s, seed 3 duty 0.712);
−10 on seed 2 alone. The window is narrow and −14 sits mid-window.

Two things make this number **non-transferable**, both of which change the
offered load rather than the radio:

- It is calibrated at `voxel_resolution_m: 0.20`, not the launch default 0.10
  (§3.12). The `scovox_bin` deltas *are* what the link carries, so an ~8×
  change in payload is an ~8× change in what the airtime model has to move.
- It is calibrated against 88 trees, not 80 (§3.11).

It is also far below the shipped 30 dBm, and that is a property of the
scenario, not an error: the two robots stay close. Measured over the three
control runs (27 763 link samples), median separation is 35 m (p90 66 m,
p99 92 m) and trees per link average 0.86. A closed-form estimate from that
geometry — `SNR = tx + 101 − 49.17 − 20 log10 d − 11.98 N`, connected at
SNR > 2 — puts the 55–70 % band at −14…−18 dBm before any sweep was run,
and the sweep landed on −14. Anything near 30 dBm keeps the link up ~98 %
of the time at these separations.

**Sweep hygiene, learned the hard way.** The control bags contain
`/hmr_comms_sim/link_states` recorded at the control's own `tx_power_dbm`.
Replaying the bag wholesale republishes those rows onto the topic the sweep's
logger subscribes to, interleaving control-power rows with swept-power rows —
and since the control is deliberately run where the link never drops, every
sweep point reads far more connected than it is. Play only `/clock` and the
pose topics. The check that catches it: `snr_db + path_loss_db − 101`
recovers the transmit power each row was computed at, and it must equal the
swept value on every row.

**Carry into analysis:** replayed duty cycle is an estimate — once comms change
behaviour, trajectories change. Re-measure **realized** disconnection fraction
per run from that run's own `link_states`; report it per arm — **but never
adjust or condition on it**: realized severity is post-treatment (behaviour
determines exposure), and adjusting it away biases the intention-to-treat
estimate. It is a manipulation report, not a covariate. Note also that the
§2.3 window is evaluated on *control* trajectories whose exhaustion timing
will shift under real degradation — calibration only aims; the phase-3
`pursuitBudgetSec` gate is the real guard.

---

## 5. Metrics

### 5.1 Two coverage numbers, never one

- **Team-observed** (oracle): offline re-merge of both robots' recorded
  sender-side `scovox_bin` streams through an ungated dscovox — what the team
  physically captured. (Recording sender-side streams means bag subscribers on
  cross-robot topics — allowlisted by node name in the §3.6 leakage gate.)
- **Robot-known:** each planner's `total_observed_voxels` — what arrived.

Track the robot-known ↔ team-observed gap **over time, not just at T**: it
opens during each outage and snaps shut at each contact (B0a is exactly this
signature). Its trajectory carries the story; its end state is merely F2. The
two-robot known gap is the unmerged-maps signature.

### 5.2 Primary endpoint: time-to-team-knowledge-complete

> **DECOMPOSED, AND PARTLY BROKEN (2026-08-16, §3.26 + its correction).**
> Report this endpoint decomposed — `t_lead_cross`, `laggard_lag`, `lag_dist`,
> via `sim/comms_metrics.py`. The laggard gap is real: 0–4 s at the shipped
> radio in the sparse world, 10–1060 s in the dense one, and the laggard spends
> it driving at full speed, up to 378 m.
>
> **But this endpoint as defined is satisfiable by duplication and biased
> toward degraded comms.** It asks each robot's OWN map to saturate, and a robot
> cut off from its partner has cheap unknown beside it — its partner's ground —
> which it harvests at 6–14× the voxels per metre of a robot that already holds
> the union. The first dense ideal-comms cell crossed at 2700 s against 1365–
> 1845 s realistic *because* it had nothing cheap left to cover. The
> solo-reachability caveat below anticipated exactly this failure mode; it is now
> observed, and it is larger than the effect the endpoint was meant to measure.
> Do not read a shared-map arm against a partitioned-map arm off crossing times
> alone.

Sim time until **every** robot's known map reaches the saturation criterion
(the §2.1 floor + margin), censored at T. Under `off` a robot gets there when
a chance re-contact drains the remaining backlog — or, failing that, by
covering the ground itself; under the modes, at the manoeuvre-driven merge.
The modes' entire claimed value is **moving that event earlier and making it
scheduled rather than lucky** — which is precisely the difference this
endpoint measures. Companions: success-within-T, and the overhead pair
(`reconnect_distance_m`, `reconnect_elapsed_sec`) absolute and as a fraction
of total. Primary analysis is paired RMST at T (§1).

**Solo-reachability check (pilot, load-bearing — new in R5):** the §2.1 floor
is measured in the control, where sharing is near-ideal; whether a *single*
robot can reach floor + margin alone is unknown. If it can, the endpoint
barely depends on merging (a robot just covers the ground itself) and B0b can
pass on duplication alone; if it cannot, `off` completes only through contact
and the endpoint measures exactly what it claims. Measure both in the pilot:
solo time-to-criterion vs merge-driven time-to-criterion.

**MEASURED (2026-08-15) — B0b survives, but on the second robot, not the
first.** Run at `tx_power_dbm: -60` (link 0.00 % connected across every
sample, verified), so each robot explores on its own map with the teammate
physically present:

| | reached 0.55 | at | distance |
|---|---|---|---|
| atlas, solo | yes | 2010 s | 727 m |
| bestla, solo | **never** (T = 3600 s) | — | 1271 m |
| team, control (merged) | yes | median 1938 s | ~490 m each |

So a single robot *can* get there alone, and atlas did it only ~4 % slower
than the fully-merged team. Taken per-robot that is close to the B0b failure
mode. What protects the endpoint is that it is a **team** makespan over
*every* robot: bestla never arrived despite driving 1271 m — 75 % further than
atlas — so the team run censored. Merging is what rescues the unlucky robot,
not what carries the lucky one.

Carry as a validity caveat, and watch it in phase 3: if `off` comes close to
the mode arms, the first thing to suspect is that 0.55 is lenient enough for
duplication to substitute for merging. The lever is to tighten the criterion
toward the 0.4922 floor (0.52 was crossed at ≈ t 2100–2200 in the floor
probe), at the cost of longer runs and more censoring. Note the solo numbers
cannot be re-read at a stricter threshold from this run: atlas went DONE on
crossing 0.55 and stopped exploring, so its curve ends there by construction.

### 5.3 Mode diagnostics

**Merge attribution — the mechanism evidence.** Classify every contact event
(from `~/link_states`, 5 Hz) by the planner `state` at contact time: exploring
→ *opportunistic*; `PURSUE`/`RETURN_NAV`/`RETURN_SYNC` → *deliberate*;
`PROXIMITY_HOLD`, barrier-wait, and DONE-idle get their own explicit classes
(a DONE-idle contact is opportunistic-terminal, not exploration). Mask the
startup window: `link_states` has no validity column — before the first odom
lands, rows read `connected=0` with zeroed physics, so naive extraction counts
a spurious "reconnection" at first pose. Keep attribution windows tolerant to
the connect hysteresis (3-of-8 samples ≈ 0.6 s lag behind physical proximity),
and cross-check every peer-missing episode against the §3.8 suppression log —
a planner-busy "outage" attributed to comms corrupts the attribution. Report
per arm: contact count and share of recovered knowledge per channel. In `off`
every merge is opportunistic by construction; in the mode arms this is the
direct evidence that the manoeuvres — and not luck — did the work. If a mode
arm wins the primary but its merges are mostly opportunistic, the win is
noise, not mechanism. Descriptive companion to the primary, not the estimand
(§1).

Per-manoeuvre: which state fired (`state` column), time-to-reconnect,
distance travelled during the manoeuvre, wait time at the meeting point.
Pursuit-arm specifics: chase attempted vs staleness-skipped, budget spent vs
reconnected early — this doubles as the empirical test of `planner_method.md`
§10.2's claim that the clamp makes staleness the only knob that matters.
Sanity: `coord_active_peers` (cheapest check the emulator is actually reaching
the planner).

### 5.4 Safety (descriptive only)

Min inter-robot distance, approaches < 1.5 m, `prox_hold_count` /
`prox_hold_total_sec` (already logged). Broken out by mode; no significance
tests at this n.

---

## 6. The Experiment

| Arms | Seeds | Runs |
|---|---|---|
| calibrated severity × {`off`, `rendezvous`, `pursuit`, `hybrid`} | 8–10, **same seed set across arms** | 32–40 (+ ~13–17 pilot: 3 control, 2–3 seeds × 4 arms, one same-seed repeat) |

- **Common random numbers — scoped honestly (R5):** the seed pins **only the
  emulator's fading trace** (`fade_rng_` = seed, `drop_rng_` = seed+1). The
  planner has no RNG at all (the "random" scorer is a constant), and the
  sim's lidar gaussian noise, IMU noise model, and physics/DDS timing are
  unseeded — a "paired" comparison shares the fade stream and nothing else.
  Pairing remains a valid block design; *how much* variance it strips is an
  empirical question the pilot's same-seed repeat (§7) answers — that number,
  not an assumption, sets the final seed count. Analyse paired (RMST
  differences at T, §1).
- **No `ideal` arm.** The phase-1 control pilot (run through the emulator at
  high `tx_power_dbm`, so the relay path — extra hop, `delay_ms: 2.5`, rx QoS —
  is present) supplies the reference makespan and unknown floor. Under ideal
  comms no manoeuvre can fire, so an ideal mode arm would be degenerate.
- Censored time-to-event at T (§2.5): report success-within-T and
  time-given-success **as descriptives only** — never average censored times
  as if observed, and never let time-given-success carry the paired
  comparison (it conditions on success and drops the most informative
  pairs). The paired primary is RMST at T (§1).
- `off` = `rendezvous_enabled:=false` — verified sound: it gates the entire
  mode dispatch in `finishOrRendezvous`, disabling all three manoeuvres
  together. This is the **opportunistic-only baseline**, not "no
  reconnection": robots still re-merge whenever chance brings them back into
  range.
- Two robots is what the code claims (`ros_api.md:327` — 3+ robots pair one
  missing peer at a time). Do not extrapolate to N.

---

## 7. Phasing

| Phase | Work | Gate |
|---|---|---|
| 0 | §3 build gap; commit emulator; seed plumbing; branch hygiene (planner work lands on `new_experiments` while the checkout is `main`; params load from the **install share** — rebuild after every yaml edit) | smoke test green; one `COMMS=1` run completes; leakage + QoS + overflow + odom-watchdog gates pass |
| 1 | §2.1 ROI fix + control pilot (3 runs, high `tx_power_dbm`) | natural coverage termination; unknown floor measured; makespan → sets T and confirms steps never bind; wall-clock per run → confirms seed count is affordable |
| 2 | §4 calibration sweep | one `tx_power_dbm` value meeting all three criteria |
| 3 | Mode pilot: **2–3 seeds × 4 arms + one same-seed repeat pair (10–14 runs)** — 1 run per arm cannot estimate variance, and "chase attempted in most runs" is meaningless at n = 1 | every manoeuvre fires (`state` column); `have_anchor_` true; ≥ 3 reconnect cycles/run; chase attempted in most pursuit runs; **B0a demonstrated** (robot-known jumps at contact, `backlog_bytes` drain in `~/stats`); **B0b visible with the gap-drain signature** (§1); solo-reachability measured (§5.2); peer-missing episodes classified outage vs suppression (§3.8); same-seed repeat → unseeded-noise floor; between-seed variance → final seed count |
| 4 | Matrix (32–40 runs) | — |
| 5 | Oracle re-merge, analysis, writeup | — |

Each early phase has already killed one version of this plan: 1 the
no-repeat-cycles trap (né unreachable-termination — corrected in R5, §2.1),
2 the outage-window trap, 3 the mode-collapse trap. Do not skip them.

---

## 8. Threats to Validity

- **Leakage / QoS / overflow / odom-watchdog** — §3.6, automated because all
  of them look like results.
- **Opportunistic merges dilute the contrast — they don't invalidate it.**
  Mode arms keep their chance re-contacts; the estimand is the policy delta
  (§1). Don't "clean" opportunistic merges out of mode arms — attribution
  (§5.3) reports them instead. The converse risk is real too: in a small
  world, chance contact may be so frequent that `off` nearly matches the
  modes — that is what B0b exists to catch *before* the matrix runs.
- **Reliable map relay = staleness only below the cap.** On overflow the
  emulator **evicts oldest** ("Deltas lost on this link — the receiver's
  merged map will be missing voxels", `hmr_comms_sim_node.cpp:605-613`) —
  gated via `drop_overflow` in §3.6. Two further loss paths live at the
  edges: the 1 Hz discovery race (initial snapshot never relayed — start the
  emulator first) and rx-history overwrite during drains (raise
  `rx_qos_depth`). With the gates green, results still do not generalise to
  lossy transport.
- **Realized ≠ nominal severity** — §4; report per arm, **never adjust**
  (post-treatment).
- **Airtime coupling — worse than R4 stated, and the R4 diagnostic doesn't
  exist.** The drain is not a burst: admission is token-gated
  (`airtime_burst_s: 0.1`, refill 1 s/s), so 64 MiB costs ≥ ~7.5 s of channel
  time (more with retx), tokens go arbitrarily negative (one large delta can
  stall the channel for seconds), and the token pool is **shared across all
  links** — a map drain starves best-effort intents on *every* link exactly
  during the reconnect-detection window. `drop_airtime` is **per directional
  link, not per topic** — diagnose with per-link `drop_airtime`,
  `backlog_bytes` trajectories, and logged per-message sizes. If it bites,
  report as a modelling artifact, not a physical finding — and decide whether
  a shared (vs per-link) airtime pool is the model you want *before*
  calibration.
- **ROI ↔ calibration coupling** — any ROI change invalidates §4; re-run it
  (§2.2). Don't tune the two in parallel.
- **`done_action` must stay `idle`** — and "default" means the *yaml*: the
  C++ default is `shutdown`, enforcement is a warning only, and an
  unrecognized value (even `"Idle"`; the compare is case-sensitive) silently
  falls back to `shutdown`, which makes the first finisher go dark and every
  barrier result an artefact. Treat the startup warning as run-invalidating
  and assert the resolved param in the harness.
- **"Peer missing" ≠ outage.** The heartbeat is state-gated (§3.8): a planner
  busy > 5 s in PLAN reads missing under perfect comms and can arm a
  manoeuvre against a healthy teammate. Attribution and the triggers
  themselves are confounded until suppression is logged and classified
  against `link_states`.
- **A fourth degenerate branch exists:** if the missing teammate was never
  itself heard, `missingPeerRecord` is null even at dispatch
  (`explo_planner_node.cpp:2511-2517`) — hybrid and rendezvous degrade to the
  own-anchor return and pure pursuit holds in place. Rare with two robots
  spawned 3 m apart, but classify it in the `state` diagnostics rather than
  letting it masquerade as a normal manoeuvre.
- **Emulator scope** (its own caveats): no MAC retries beyond the airtime cost
  factor, no fragmentation, no DDS discovery traffic, no congestion dynamics.
  If challenged, validate a subset against `tc netem` and show the curves
  match.
- **Exploration-only scope:** exploitation under comms stress (the vantage-ring
  barrier waiting on gated claims) is a real question — deliberately deferred,
  not smuggled into this matrix. Likewise the damage curve (cut E1) — if a
  reviewer asks "how bad are the comms you're recovering from", the answer is
  the realized disconnection fraction reported per arm, not a separate sweep.

---

# 2026-08-21 — Metric set revised, tail mechanism found, lever choice

All numbers below are from the post-2026-08-20 binary (`build_stamp`
`Aug 20 2026 14:11:15`). Everything older was archived to
`~/hmr_campaign_archive` and is void as numbers.

Pool: 14 completed cells — off n=9, hybrid n=3, rendezvous n=1, pursuit n=1.
Pooling is verified by comparing `run_start.params` across cells (ignoring
path/seed keys), **not** by comparing `run_manifest.txt`: the manifest does not
record the env overrides (`UTIL_GAMMA`, `FRONTIER_ONLY`,
`RECONNECT_MIN_SHARE_VOX`, silence window), so matching manifests never was
evidence that two cells are comparable.

## 1. The metric set changed

Was: completion time, chase time, shared map voxels at reconnect.
Now: **completion time** and **redundant coverage**.

Both dropped metrics had already failed on the data before they were dropped:

- **Chase time is a tautology.** The off arm has zero chase by construction, so
  "off 0.0 m vs hybrid 78.4 m, p=0.001" tests only that the feature is switched
  off. It measures the manipulation, not an outcome.
- **Shared voxels at reconnect collapsed under pooling.** From a single cell it
  looked as though commanded merges were reliably large (4 of 4 above 340k).
  Over 9 off cells: off 14/58 merges ≥300k (24%) vs hybrid 4/16 (25%),
  p=0.674. The 300k threshold had also been chosen after seeing the single
  cell. **This claim is retracted.**

**Redundant coverage** replaces them. It is the *consequence* map sharing is
supposed to buy, rather than the *volume* of the transfer, which was never the
object of interest. Measured geometrically from the logged `step` poses: sweep a
disc (R=5 m, 1 m grid, path interpolated to 1 m) along each robot's path, and
take |A∩B| / |A∪B|. It needs no assumption about what a merge actually
transferred. Sensitivity checked at R=3/5/8 m.

**An informed/blind split was attempted and abandoned.** The intent was to count
only overlap where the two robots had had no contact between the two first
visits. It scored 0.0% in 13 of 14 cells because it treats any contact as
transferring the whole map, and merges here are truncated (~103k voxels). It is
a floor produced by an assumption known to be false, not a measurement. Do not
revive it without per-voxel transfer logging.

## 2. The tail is redundant coverage, not slow exploration

Roughly a fifth of cells finish at ≥1.6× the median (1199–1895 s vs ~713 s).
Three probes, in order:

- **`stallprobe.py`** — the run ends when `unknown_fraction` < 0.60 for 3
  consecutive steps. Threshold flicker is refuted: **0 re-crossings in 14/14
  cells**. Once below 0.60 it stays below. Completion time is not measuring
  stopping-rule noise.
- **`stepprobe.py`** — the decomposition `finish ≈ n_steps × step_duration`.
  Tail vs normal: **n_steps 2.76×, seconds per step 1.00×, metres per step
  0.95×.** Nothing is slow and no target is farther away. There are simply more
  legs. (This factorisation is not circular the way vox/m was — vox/m is
  fixed-map over distance and distance is ≈ speed×time, so it can only restate
  the clock.)
- **`redundancy2.py`** — where the extra legs go. Tail vs normal, off cells:
  area covered 1.23×, run length 2.59×, **overlapping area 5.24×**. Overlap is
  the only quantity moving faster than the clock.

**Confound killed.** A 1828 s run lays down 2.3× the path inside a bounded ROI,
so overlap at run end cannot distinguish cause from consequence. Recomputing
every cell truncated to a common horizon T = 713 s (the pooled median finish):

| | vs finish |
|---|---|
| overlap at run end | rho +0.45, p=0.080 |
| **overlap in first T s** | **rho +0.51, p=0.041** |
| area covered by T | rho +0.25, p=0.374 |

Truncated overlap predicts finish *better* than final overlap, and area by T is
flat (tail/normal ratio 1.02). By 713 s every cell has covered the same ~3,500
m²; the future-tail cells have already spent ~3× more of it on ground the
partner covered. **Redundancy leads the tail.** The test is conservative:
normal cells finish before T and so get their whole run to accumulate overlap,
while tail cells are cut off at 713 s, and the tails still show 3×.

Caveat: tail n=3, and `pb1_hybrid_seed3` is a tail with only 7.9% overlap while
`pb2_off_seed7` has 30.6% and is not a tail. Redundancy is neither necessary nor
sufficient — it is the strongest single predictor available, at n=14.

## 3. Where the numbers stand (off n=9 vs hybrid n=3)

- **Completion time: no detectable effect.** geo-mean ratio 1.019 (+1.9%),
  95% CI [0.706, 1.504], MWU p=0.926. Anything below ~50% is invisible at this
  n. Medians: off 719 s, hybrid 707 s.
- **Redundant coverage:** off median 8.6%, hybrid 1.5%; median difference
  95% CI [−30.0, +4.4] pts, p=0.195. Direction favours hybrid; not resolved.

**Correction.** A previously reported near-significant result — time apart, off
62.4% vs hybrid 48.6%, p=0.052 — **does not survive**. Recomputed per-link from
both robots' views it is off 47.8% vs hybrid 49.5%, p=0.926. It was derived from
`peer_lost`/`peer_seen`, which debounce 5 s of silence and therefore do not
report the link. Use `link_states.csv`, which is ground truth at 5 Hz.

## 4. Choosing the next lever

**The dose is not low.** Read from `link_states.csv` rather than from events, at
tx=30 the pair is already silent **65.9%** of the run, with **42.3%** inside
outages ≥60 s — the only ones the midrun gate (`MIDRUN_MIN_SILENCE=60`) can act
on. Typically ~2 long outages plus ~43 sub-second flickers that the 5 s
heartbeat debounce hides. An earlier framing of "dose too low" was wrong.

**The link is occlusion-gated, not power-gated.** Across 64,228 link samples:

| | connected | disconnected |
|---|---|---|
| `trees_on_link` median | 1 | 6 |
| `distance_m` median | 30.5 | 64.9 |
| `snr_db` median | 34.9 | −22.9 |
| **share with `trees_on_link`=0** | **27.9%** | **0.0%** |

**Not one dropout in 64,228 samples occurred with clear line of sight.** Forest
density is therefore a direct lever on the comms channel, and
`flatforest_2robot_lidar.yaml` (sparse) already exists beside the dense
scenario.

**A tempting wrong conclusion, recorded so it is not drawn again.** Outage dose
is *negatively* associated with redundancy — trig% vs overlap%, off cells,
rho=−0.60, p=0.047 (all three dose measures agree in sign, in both groupings).
It is tempting to conclude that lowering tx is counterproductive. **That
inference is invalid**: it is observational and confounded by separation. Robots
that are far apart get both more outage *and* less overlap. Lowering tx is an
intervention that adds outage at *fixed* separation, including at close range
where overlap is high. The correlation cannot settle it; a direct probe can.

**Also invalid: the replayed tx dose curve.** `txdose.py` assumed `connected`
was an SNR threshold and replayed lower tx by shifting SNR 1:1. It is not a
threshold: connected=1 occurs down to −23.4 dB and connected=0 up to +53.0 dB.
Occlusion decides. The replayed curve is discarded; tx levels must be chosen by
running a probe.

**Plan.** Probe before committing, in both directions, because a wrong guess
costs ~10 h:

1. **tx probe** — off + hybrid at reduced tx, verify from `link_states.csv`
   that silent%/trig% actually moved *and* that overlap rose. If overlap does
   not rise, tx is confirmed dead as a lever and the campaign is not run.
2. **environment probe** — off + hybrid on the sparse forest. Note the ROI ↔
   calibration coupling warned about above: `--done-unknown 0.60` is calibrated
   for the dense ROI and may trip instantly or never in a sparse one. The probe
   checks the `unknown_fraction` trajectory before any campaign is committed.
3. **tx=30 dense baseline continues regardless** — it is the control anchor and
   the reference dose point for both comparisons. A well-powered null at tx=30
   is itself a result.

## 5. Standing constraints reconfirmed this session

- Cells run **strictly sequentially**; CPU contention is indistinguishable from
  a comms or planner effect. Verified under challenge: a `ros2 run` cell shows
  **4** planner processes, not 2 — two python CLI wrappers each with a real
  binary child. Count parents, not matches.
- **The sim is not reproducible from a seed.** Identical config produced 856.4 s
  and 1803.3 s. Seeds are labels, not blocks. Do not pair or block on them;
  completion-time CV is ~51%.
- **Planner diagnostic logging is largely dead** and cannot currently diagnose
  the tail: `target_id` (−1), `vantage_index` (−1), `n_vantages_valid` (0),
  `prox_hold_count` (0), `prox_hold_total_sec` (0), `dwell_sec` (0),
  `rejected_by_unreachable` (0) across all cells. `selected_path_cost`,
  `selected_score`, `mean_info_gain` populate only on PLAN rows (~59/417).
  Repairing these is the highest-value planner work identified so far.
- **Arm state names differ** — pursuit/hybrid use `PURSUE`; rendezvous uses
  `RETURN_NAV` then `RETURN_SYNC`. Reading only `PURSUE` scored rendezvous at
  0 m of chase while it was demonstrably dispatching four times.
- **Terminal handovers are not midrun reconnections** — split on
  `trigger == "terminal"` or pursuit appears to dispatch 5× at a 28 s median
  chase when it actually made 1 midrun attempt per robot at 58 s.
- **The 113–127 s `peer_lost` → `reconnect_dispatch` lag is the information
  gate waiting on purpose**, not planning latency: `gate_sec` is 61–65 s on the
  first outage and 149–166 s on the second (designed 60/190). Residual
  planning latency is only ~12–18 s. This was nearly reported as a defect.

---

## 2026-08-21 (later) — Where redundant coverage actually comes from

Redundancy leads the tail, so the question became *why the planner permits it*.
Four measurements, run in the order below because each one killed the reading
the previous one suggested. The last is the one to keep.

### A retraction, before the findings

Overlap-creation rate was first compared inside matched separation bins (from
`link_states.csv` `distance_m`, so geometry is held fixed) and gave a clean
connected/disconnected ratio of **0.60–0.75** — apparently "the planner
duplicates at two-thirds the blind rate even while connected."

**That number does not survive its own robustness checks and must not be
quoted.** Two independent checks break it:

- **Lag sensitivity.** A robot commits to a target up to one planning step
  (~33 s) before it sweeps the ground, so re-attributing each event 33 s earlier
  tests whether the link state at duplication is the state it *decided* under.
  The 40–60 m bin flips from **0.64 to 1.95**.
- **Per-cell pairing.** Pooling events and exposure across cells lets a few
  high-overlap cells drive the ratio. Per cell (20–60 m, both states >60 s
  exposure): median ratio **1.19**, range 0.29–28.79, sign test **4/8 cells,
  p=1.000**.

The pooled table was a pooling artefact. Compare within cells.

### 1. Half of redundant coverage happens with the radio UP

This is a **count**, not a rate — each duplicated grid cell classified by link
state at the moment the second robot swept it — so exposure and binning cannot
distort it.

| arm | share of duplicated ground swept while connected (lag 0 / lag 33 s) |
|---|---|
| off (n=9) | median **41.9%** / 71.5% |
| hybrid (n=3) | 100% / 100% |
| all (n=14) | 81.1% / — |

Every cell lies in 16–100%; none is near zero. **Comms can only ever explain the
disconnected share**, so even an instant, free, zero-lag reconnection policy
leaves roughly half of the off arm's redundancy untouched. This is a ceiling on
what the tx lever can buy, established before spending cells on it.

Near-field breakdown — duplication while connected **and** within 20 m, a state
where comms cannot be blamed under any reading (the link never drops below 20 m
across 14 cells of samples, 0 s of disconnected exposure):

    pb1_hybrid_seed3  49.0%   mc1_pursuit_seed20  31.8%
    rb1_off_seed3     19.4%   pb2_hybrid_seed2    13.0%
    off arm median     0.0%   (8 of 9 cells exactly zero)

The arms that **command robots toward each other** are the ones generating
near-field duplication. That is the chase re-sweeping the partner's ground — a
cost of the treatment, consistent with chase costing 21–24% of pair distance.

### 2. Duplication is SEQUENTIAL, so goal deconfliction is irrelevant

Revisit gap dt = |firstB − firstA| per overlapped cell:

| bucket | off arm | all arms |
|---|---|---|
| 0–30 s | **0.0%** | 1.0% |
| 30–60 s | 0.7% | 1.1% |
| 60–180 s | 5.7% | 10.7% |
| 180–600 s | **86.1%** | 71.6% |
| 600 s+ | 7.5% | 15.6% |
| median | **420 s** | 409 s |

Robot B sweeps ground robot A finished ~7 minutes earlier. Of those sequential
duplications, **61.6% occur while connected**.

MinPos claims (`coordination.hpp`) cover the peer's *current goal disc* with a
TTL of seconds, so they can only stop **simultaneous** duplication — which
essentially never happens here. Measured directly: across 16 cells the planner
had a peer visible for **2,784 ticks and rejected 3 candidates total**, with
14/16 cells rejecting none, `max coord_active_peers = 1`.

**The deconfliction being inert is not the defect.** It is structurally
incapable of addressing sequential re-covering, and filing it as a bug would
have been wrong. `rejected_by_minpos` is live, not dead — it is genuinely ~0.

### 3. It is not legitimate transit either

The confound that had to die: overlap counts ground within 5 m of the path, so a
robot **crossing** known territory to reach a fresh frontier scores as
duplication while behaving correctly.

Discriminator, using `step.observed_voxels`: per planning step, take the cells
newly swept by that robot, split by whether the peer had swept them first, and
compare voxel yield per new cell. If the peer's coverage reached this robot's
map, that ground adds ~nothing (transit). If it did not, it yields like virgin
ground.

    virgin vs peer-already-swept yield ratio
      off        median 0.63  (n=9)      hybrid  median 0.59  (n=3)
      all        median 0.61  (n=14, range 0.17–4.47)

**Redundant coverage is real exploration work, not empty transit** — pure
transit would score near 0.

### 4. What is still blocked, and why it matters

Ratio 0.61 is intermediate and does **not** cleanly separate merge-truncation
from EIG-scoring, because of a confound that cannot be removed with current
logging: a robot crossing the same ground on a **different heading sees
different 3D surfaces**, so even a perfect merge leaves the ratio well above
zero.

Separating them needs **per-voxel transfer logging at the merge** — which voxels
actually arrived, not how many. This is now the **third** analysis blocked by
that same gap:

1. the informed/blind redundancy split (scored 0.0% in 13/14, abandoned);
2. "commanded merges are reliably large" (retracted, p=0.674);
3. this one.

The code already half-knows the problem: `experiment_log.hpp:198-201` notes
`est_unshared_vox` overcounts because "the two robots often re-observe the same
region." The duplication was a design-time concern that was never quantified.

**This displaces repairing the dead diagnostic columns as the top planner task.**
Requires a build window — no colcon build while a campaign runs.

### Consequence for the queued levers

The tx probe keeps its slot and its ordering (user's call, 4 cells, ~1.4 h), but
its ceiling is now known in advance: it can only address the ~half of redundancy
that happens while disconnected. Forest density remains the stronger lever
because it moves exploration geometry as well as the channel — and geometry is
what generates sequential re-covering.

### 5. The structural gap — nothing carries where the peer has BEEN

Two follow-ups, both negative results worth keeping.

**The no-build shortcut fails.** The hope was to detect merges in existing data:
voxels gained by DRIVING should scale with distance, voxels gained from a MERGE
arrive independently of motion, so a jump at low travel would mark a merge. It
does not separate. Across 16 cells / 820 step transitions the robots are
essentially never stationary between planning ticks — median travel **9.8 m**,
only 9 transitions under 5 m — and voxel gain tracks distance monotonically
(median 25k at 5–10 m, 54k at 10–20 m, 129k at 20+ m). There is no stationary
population to read merges from. Per-voxel transfer logging still needs a build.

**Where peer information can enter the planner at all** (`create_subscription`
sites, `explo_planner_node.cpp`):

| channel | what it carries |
|---|---|
| `ScovoxMap` / `OccupancyGrid` | the fused map — the merge |
| `RobotIntent` | `goal_pos`, `robot_pos`, `claim_radius_m`, `ttl_sec` |
| `PoseWithCovarianceStamped` | peer's **current** pose |
| `TreeTarget` | target reports |

`RobotIntent.msg` is explicit that it is a claim on the *currently selected
viewpoint*. **No channel carries the peer's coverage history.** Dwell credit
(`mergePeerDwells`) is per-target, not free space. So the only mechanism that
can suppress a 7-minute-old re-cover is the fused map.

That is the structural gap, and it lines up exactly with the measured failure
mode:

- duplication is **sequential** (median 420 s), so goal claims cannot help —
  and measurably do not (3 rejections in 2,784 peer-visible ticks);
- **61.6%** of sequential duplication happens while **connected**, so the
  information channel was open at the time;
- yet the only thing able to carry coverage is the merge, which is truncated.

**Design implication.** A coarse peer-coverage trail — a downsampled polyline of
recent poses, or a hashed set of swept cells — carried on the intent channel
would target the dominant duplication mode directly. The asymmetry that makes
this attractive: intents are tens of bytes and get through where a ~103k-voxel
map merge truncates. It would also degrade gracefully, since a stale trail is
still valid (ground swept stays swept), unlike a goal claim which must expire.

This is a proposal, not a result. It is the concrete form of "improve the EIG
planner" that the redundancy work points to, and it should be validated against
the existing off-arm cells before any implementation.

### 6. The link was available for 99% of duplications

The trail proposal needed a number before it was worth anything. Necessary
condition: for a coverage trail to prevent a given duplication, there must have
been contact between robot A sweeping the cell and robot B re-sweeping it.

Over **7,762 duplicated cells across 14 cells**:

| criterion | share of duplications |
|---|---|
| any contact in the window | **100.0%** |
| contact ≥33 s before the re-sweep (time to re-plan) | 99.0% |
| **≥5 s cumulative contact, ≥33 s of lead** | **99.0%** |

The strict column exists because the link shows ~43 sub-second flickers per run
and a 0.2 s flicker is not a delivery opportunity. It changes nothing: 99.0%.

This is **saturated**, so read it as a negative result rather than a validation
of the proposal. It does not show a trail would work — it assumes perfect
uptake and ignores that re-covered ground has some independent value (the 0.61
voxel-yield ratio). What it does show, cleanly:

**Comms availability is not the binding constraint on this duplication.** In 99%
of cases the robots had seconds of open link and a full planning step of lead
time between one sweeping the ground and the other re-sweeping it. The channel
was open; there was no message to put on it.

That is the sharpest statement the current data supports, and it is consistent
across all four measurements: sequential gaps (420 s median) leave ample contact
opportunity, ~half of duplication happens outright while connected, goal claims
fire 3 times in 2,784 peer-visible ticks, and no channel carries coverage.

**It also caps the tx lever from the other side.** Lowering transmit power adds
outage, but outage is not what permits this duplication — 99% of it already had
the link it needed. The tx probe keeps its slot (4 cells, user's ordering), but
this is now a prior expectation that it will not move redundancy much, and it
was established for free while the queue ran.

## 2026-08-21 (later still) — Metric validity audit, and a metric that has to be dropped

Triggered by a 49 s discrepancy: the harness notification for `pb2_hybrid_seed3`
reported `t_sim 1216` while `advantage.py` reported `1265`. That is not rounding,
and if the reporting script reads a different clock than the harness, every
completion-time number in this document is suspect. Checking it first surfaced
three things about the instrumentation and then two audits of the headline.

### 7. Instrumentation facts found on the way

- **`run_manifest.txt` has no `t_sim_sec` line.** It carries `run_end_reason=`
  only. The harness/notification time comes from `exploration_complete`;
  `advantage.py` reads `run_end`. The two legitimately differ — that is the 49 s.
- **`exploration_complete` fires more than once per robot**, at an identical
  `unknown_fraction`. Count per cell (`n_ec`): exactly **2 in all 10 off cells**,
  **2–6 in hybrid**, **5 in pursuit**. `experiment_log.cpp:467` names the
  mechanism — a merged map arrives carrying new frontiers, so the robot explores
  again and exhausts a second time. Only the reconnection arms re-declare.

That second fact is not a curiosity; it is a direct threat to the headline, and
it was checked before anything else was reported.

### 8. Two threats to the completion-time comparison, both cleared

**Threat A — arm-dependent teardown.** If the clock keeps running after
exploration ends, and it runs longer for arms with machinery still in flight (a
chase, a rendezvous barrier), the measured hybrid penalty is partly an artefact
of when the clock stops. Against an off median of 754 s, a ~130 s tail matters.

| arm | tail (t_end − t_first) | median t_first | median t_end |
|---|---|---|---|
| off | **123 s** | 604 s | 754 s |
| hybrid | **129 s** | 904 s | 1199 s |

Near-identical. Recomputing the ratio from `t_first` gives 1.497 against 1.589
from `t_end` — same direction, same order. **Common overhead, not an artefact.**

**Threat B — unequal work, the more damaging one.** If off finishes fast because
it never learns what it is missing, while hybrid keeps working because
reconnection tells it the job is not done, then faster means *less complete* and
the comparison is rigged against reconnection.

| arm | n | final unknown_fraction | max voxels | pair distance |
|---|---|---|---|---|
| off | 10 | 0.5589 | 1,634,495 | 435 m |
| hybrid | 5 | **0.5559** | 1,600,671 | **681 m** |
| pursuit | 1 | 0.5448 | 1,735,380 | 1,195 m |
| rendezvous | 1 | 0.5806 | 1,516,201 | 431 m |

Difference in completeness: **−0.0030**, hybrid marginally *more* complete, well
inside the 0.005 band set in advance. Equal voxel totals. **The arms solve the
same problem; hybrid's extra time is extra time, not extra work.**

And the extra time has a physical counterpart: hybrid drives **57% farther** for
the same map.

### 9. Which metrics this design can actually resolve

With the comparison shown to be fair, the question becomes whether it can ever
reach significance. Required cells per arm at 80% power, α=0.05, for the effect
size *actually observed* at n=15 (log scale, matching the bootstrap):

| metric | off | hybrid | ratio | CV(log) | **cells/arm needed** |
|---|---|---|---|---|---|
| completion time | 754 s | 1199 s | 1.292 | 35.4% | **30** |
| pair distance | 435 m | 681 m | 1.274 | 30.2% | **24** |
| voxels per metre | 7185 | 4527 | 0.766 | 29.3% | 19 |
| **redundant coverage** | 8.8% | 11.5% | 1.083 | **134.4%** | **4451** |

Three conclusions, in order of consequence.

**Redundant coverage must be dropped as a between-arm metric.** It needs ~4,451
cells per arm. That is not a shortfall to be closed by running longer; it is
unmeasurable in this design at any cell count the campaign can pay for. This
also explains, retrospectively, the earlier apparent hybrid redundancy advantage
(1.5% vs 8.6%) evaporating to a null (8.1% vs 8.7%, p=0.759) as n went 3→5: it
was never a signal. §1 recorded the metric set as *completion time + redundant
coverage*; **half of that set does not work.** Redundancy stays valuable as a
*mechanism* variable — every finding in §§1–6 above rests on it, measured within
the off arm — but it cannot rank arms.

**Completion time survives, at ~30 cells/arm.** This is a better position than
the CV-51% estimate in [[sim-run-to-run-nondeterminism]] implied (~74/arm): that
figure was raw-scale and assumed a 200 s effect, where the observed effect is
29%. pb2 delivers n=16/side, so ~30 is roughly two more stages — reachable.

**Voxels-per-metre is not independent evidence.** Its ratio 0.766 is within 2%
of 1/1.274, i.e. it is pair distance in disguise, since the numerator is
near-constant across arms by §8. It is quoted here only because the small extra
power is real; it must not be reported as a third agreeing metric.

### Consequence for the queued levers — the tx probe just got more valuable

Required n scales as 1/effect². The tx=5 probe exists to lengthen outage, which
should *widen* the arm gap. If it doubles the effect (1.29 → ~1.6), required
cells fall from 30 to **~9 per arm** — the difference between a campaign that
concludes and one that does not. The user's ordering (tx before environments)
was a preference; it is now the quantitatively correct order as well.

This does **not** contradict §6. That said lower tx will not move *redundancy*
much, because 99% of duplication already had the link it needed. Redundancy is
exactly the metric just dropped. The tx probe is now justified on completion time
and distance, which are the metrics that survive.

## 10. Is the arm effect confounded with the campaign batch? (No — but check part 4)

The completed cells are **not one experiment**. They come from prefixes
`mc1` / `rb1` / `pb1` / `rep1` / `pb2`, and the arms are badly unbalanced across
them: **6 off cells are `rb1` and zero hybrid cells are.** If `rb1` differed from
`pb2` in anything — tx power, scenario, ROI — the pooled off median would be
dragged toward `rb1` and the hybrid penalty inflated by exactly that amount. The
headline would be measuring the batch.

**The off arm is stable across batches:**

| batch | n | median | min | max |
|---|---|---|---|---|
| mc1 | 1 | 691 s | — | — |
| rb1 | 6 | **691 s** | 569 | 1828 |
| pb2 | 3 | 792 s | 594 | 900 |
| rep1 | 1 | 882 s | — | — |
| pooled | 11 | 719 s | | |

The large `rb1` block sits on the pooled median, not away from it.

**The within-batch contrast reproduces the pooled one.** `pb2` is the only
prefix containing both arms, and it runs them **alternating**, so it is balanced
on time-order as well as on batch:

| contrast | off | hybrid | time ratio | distance ratio |
|---|---|---|---|---|
| **within pb2 only** | 792 s (n=3) | 1265 s (n=3) | **1.597** | **1.662** |
| pooled (all prefixes) | 719 s (n=11) | 1199 s (n=5) | 1.668 | 1.584 |

Within-batch is if anything *larger* on time. **Pooling is not manufacturing the
effect**, and the matched design replicates it independently at n=3v3. (These
are median ratios; the geometric-mean ratio with a CI remains 1.292
[0.889, 1.856] — do not mix the two.)

### 4. The pooling key cannot protect the next stage

All 18 completed cells carry **distinct `run_start.params` signatures** — one
per cell, no two alike. Something seed- or instance-specific lives in `params`,
so the signature varies with the cell and therefore **cannot separate batches at
all**; grouping by it would produce 18 groups of one.

That removes it as a safety net and sharpens the hazard already recorded: when
the tx=5 and sparse-forest cells land, **the tag prefix is the only thing keeping
them out of the tx=30 dense baseline.** TX power and scenario are harness/sim
arguments that never enter planner params, so nothing in the data will flag the
mistake if a prefix slips. Verify the prefix filter before reporting any probe.

*Minor, resolved:* `pb2_off_seed9` reported 546 s by notification and 594 s at
`run_end` — the same `t_first` / `t_end` difference as §7, not a new anomaly.

## 11. Where hybrid's extra distance actually goes — and it is not mostly the chase

Hybrid reaches the same completeness as off but drives **664 m against 413 m**,
~251 m more per pair. The chase alone cannot account for that, so every metre
was attributed.

**A method error worth recording so it is not repeated.** The first attempt
assigned each step leg to "the state at the `step` event" and produced
`chase = 0.0%` in every cell **including pursuit**, which is impossible. Cause:
the state machine runs `PLAN → NAVIGATE → INTEGRATE → LOG_STEP`, and `step`
events — the only ones carrying `distance_m` — fire in `LOG_STEP`. Every leg was
attributed to `LOG_STEP`. A leg spans several states and cannot be assigned to
one. The corrected method uses `state_change.from_dwell_sec` to build per-state
intervals and splits each leg in proportion to time spent in the two *moving*
states (`NAVIGATE`, `PURSUE`). Validated by construction: off cells enter
`PURSUE` zero times and now score exactly 0.0% chase; hybrid enters 3–6 times.

### Decomposition of the 251 m excess

| category | metres | share of excess |
|---|---|---|
| chase (`PURSUE`) | 81 m | **32.3%** |
| extra re-exploration after first completion | 16 m | 6.4% |
| **ordinary exploration** | **154 m** | **61.4%** |

Two corrections follow. **The chase is a minority of the cost** — and the
"21–24% of pair distance" figure recorded earlier came from the two most extreme
cells; across n=5 the median is **12.2%** (range 6.7–24.1%). And the
re-exploration threat raised in §7, where hybrid re-declares completion 2–6
times, is **quantitatively negligible at 6.4%** of the excess.

### The chase's cost is paid in yield, not in displacement

Obvious hypothesis: a chase drags the robot off its frontier, so the return trip
is logged as ordinary `NAVIGATE` and the chase escapes blame. **Refuted.**
Comparing the first 2 legs after `pursuit-released` against all other legs in
the same cell (so per-cell speed and terrain cancel):

| measure | post-chase vs baseline |
|---|---|
| leg length | **0.80×** — median over 6 cells, 2/6 above 1.0, sign test p=0.688 |
| **voxels per metre** | **0.40×** — all 5 hybrid cells below 1.0 (0.53, 0.42, 0.35, 0.39, 0.05) |

Post-chase legs are *not longer*. They are **much less productive per metre.**
That reframes the 61% "ordinary exploration" excess: those metres are not extra
because legs grew, they are extra because **each metre buys less map**.

The mechanism is the chase's own geometry. A chase ends with the robot standing
next to its partner, on ground the partner has just swept — the least
informative place in the map. It then explores outward from there at 0.4× the
normal information rate. This is the same asymmetry already recorded in §1:
connected-and-within-20 m duplication is 0.0% in 8 of 9 off cells but 13–49% in
hybrid and pursuit cells. **The chase creates its own redundancy.**

**Caveats, both real.** Five cells all pointing one way is p=0.0625 on a sign
test — suggestive, not established. And `observed_voxels` jumps when a merge
lands, which happens exactly at reconnection; the pursuit cell's 8.72 is almost
certainly one such spike (n=1). For hybrid that bias *inflates* post-chase yield,
so 0.40× is conservative — but a clean number needs **per-voxel transfer logging
at the merge, now blocking a fourth analysis.**

### What this means for the lever

If the chase were merely long, the fix would be trigger tuning or a shorter
chase radius (the latter already rejected by the user). Neither addresses what
was found. The cost is **where the chase leaves the robot**, so the candidate
lever is what happens on `pursuit-released`: currently the robot resumes
exploring from the rendezvous point, which is by construction inside its
partner's fresh coverage. Sending it back toward the frontier it abandoned — or
scoring the post-release goal against the merged map before committing — targets
61% of the excess rather than 32%. **This is a hypothesis with a mechanism and a
measurement, not a validated fix**; it needs the merge logging to be tested
properly.

## 12. Inference method correction — stop quoting the bootstrap CI as the headline

At off n=11 vs hybrid n=6 the three inference methods available disagreed, which
had to be resolved rather than reported around.

| metric | ratio | percentile bootstrap 95% CI | Mann-Whitney | **exact permutation** |
|---|---|---|---|---|
| completion time | 1.393 | [1.068, 1.800] — excludes 1.0 | p=0.145 | **p=0.0786** |
| pair distance | 1.409 | [1.089, 1.823] — excludes 1.0 | p=0.097 | **p=0.0468** |

With n1=11 and n2=6 there are only **C(17,6) = 12,376** splits, so the
permutation null was enumerated **completely** — no sampling, no distributional
assumption, no small-n approximation. It tests the same statistic the bootstrap
CI is built on (difference in mean log), unlike Mann-Whitney which tests ranks.
The cells are exchangeable replicates (the sim is not reproducible from a seed),
so every split is equally likely under the null and the test is exact.

**The percentile bootstrap under-covers here** — it resamples 6 values to
estimate the tail of their own mean and produced an interval excluding 1.0 for
both metrics when the exact test puts completion time at p=0.079. Mann-Whitney
errs the other way. **From here the exact permutation p is the primary
inference**; the geometric-mean ratio stays as the effect size, and the
bootstrap CI is reported only as a descriptive spread. Every earlier CI in this
document should be read with that correction — the direction and magnitude of
those results are unaffected, but their intervals were narrower than warranted.

### What the data actually supports right now

**Distance crosses 0.05 (p=0.047); completion time does not (p=0.079).** That is
not two independent confirmations — distance and time are near-duplicate
measures of the same run, so this is one marginal result seen twice. It should
not be presented as an established effect, and one cell could flip it.

**But the magnitude is stable.** Leave-one-out over all 17 cells moves the ratio
only within **1.271–1.547** (time) and **1.304–1.556** (distance). No single
cell carries the result, including the 1828 s off outlier and the 643 s fast
hybrid cell. The effect is robust in size while marginal in significance —
the ordinary signature of being slightly under-powered, not of an artefact.

**Required n must be widened accordingly.** `power.py` derives required-n from
the observed effect, which is itself noisy — it moved 30 → 17 cells/arm when a
single cell landed. Recomputing at the pessimistic leave-one-out ratio (1.271)
gives ~32 cells/arm. **Plan for ~20–30 cells/arm**, which agrees with the
independent estimate in [[sim-run-to-run-nondeterminism]].

### Redundancy: the instability is itself the evidence

The same recomputation moved redundancy's required-n from **4,451 to 205
cells/arm** on one new cell, as the observed gap swung from 8.7 vs 8.1 to
8.9 vs 15.0. A metric whose own sample-size requirement moves by 20× on a single
observation is not measuring anything stable at this n. That is a stronger
argument for dropping it as a between-arm metric than the original CV of 130%,
and it is why the §9 decision stands rather than being revisited each time a
cell lands.

### Concrete demonstration that the bootstrap cannot be trusted here

Running the corrected `advantage.py` produced the point empirically. Two
independent percentile-bootstrap implementations on **identical data** disagree
about the headline:

| implementation | iterations | 95% CI on the ratio | verdict |
|---|---|---|---|
| `distcost.py` | 20,000 | [1.068, 1.800] | excludes 1.0 |
| `advantage.py` | 4,000 | [0.984, 1.919] | **spans 1.0** |

Same cells, same statistic, opposite conclusions — the difference is entirely
RNG and iteration count. The exact permutation p is **0.0786** regardless. Any
result that depends on which bootstrap you happened to run is not a result.

`advantage.py` now leads with the exact permutation p, prints the bootstrap CI
and MWU explicitly labelled "descriptive spread only", and labels redundant
coverage "DESCRIPTIVE ONLY — not a between-arm metric" with the reason inline,
so the §9 decision cannot be quietly forgotten the next time a cell lands.

*Note:* `advantage.py` and `power.py` compute overlap slightly differently
(11.5% vs 15.0% for hybrid). Both are descriptive-only now, so the discrepancy
does not affect any conclusion — but do not quote the two interchangeably.

## 13. Fourth validity threat, checked and cleared: is `all_done` filtering censoring the slow cells?

Every analysis script filters cells on `run_end_reason=all_done`. That is a
**selection filter**, and it points the obvious way: if hybrid runs hit the
5400 s duration cap more often than off runs, the filter drops hybrid's worst
cells and *understates* hybrid's disadvantage. The measured ratio would then be
a floor, and the true effect larger.

Checked directly over every cell directory ever produced:

- **19 of 20 cells ended `all_done`.** The 20th (`pb2_off_seed10`) is in
  flight. **Zero** cells have ever ended in timeout, budget exhaustion, or
  failure — across five batch prefixes and four arms.
- Longest completion observed is **1828 s against the 5400 s cap** — 2.95×
  headroom — and it is an **off** cell (`rb1_off_seed3`), not a hybrid one.
  The four longest hybrid cells are 1199 / 1265 / 1382 / 1719 s.

So the filter is not selecting on outcome; it is only excluding runs that have
not finished yet. No censoring correction is needed, and the reported ratio is
not a floor. This also means the `all_done` guard can stay in the scripts
without a caveat — worth stating explicitly, because it is the first thing a
reviewer asks about a completion-time comparison with a timeout in it.

One thing to re-check later: if the tx=5 probe genuinely doubles the effect as
§9 projects, hybrid cells could approach the cap. **Re-run this check when the
tx cells land** — the conclusion is a property of the current dense/tx=30
configuration, not of the harness.

## 14. The `pursuit-released` hypothesis cannot be tested offline — and why that closes a backlog item

§11 attributed **61%** of hybrid's excess distance to ordinary exploration
rather than the chase, and proposed changing behaviour at `pursuit-released`.
That proposal has two variants implying different planner changes:

- **(a) RESUME** the frontier abandoned when the chase began. Correct only if
  that frontier is still worth visiting at release.
- **(b) RE-SCORE** the goal against the newly merged map. Correct if the partner
  has meanwhile covered it — which is exactly what a good merge would reveal.

Building the wrong one wastes a build window, and if the robot *already*
resumes its old target then (a) is a no-op and the 61% is not goal abandonment
at all. So this had to be split before implementation.

**It cannot be, with the current logs.** No event carries a goal coordinate.
`planner_<robot>.csv` has a `target_id` column that would answer it exactly —
and it is `-1` in every row of every cell. (`resume.py` checks id stability
*first* and aborts rather than reporting a fabricated resume rate.)

### Planner CSV audit — 8,271 rows, 19 cells, both robots

**10 of 32 columns are constant everywhere.** Two are constant *by
configuration*, not broken: `coverage_source` (always `scovox` — one source in
this campaign) and `phase` (always `explore`). The remaining **eight are genuine
instrumentation gaps**:

| column | always | blocks |
|---|---|---|
| `target_id` | -1 | goal identity — resume-vs-rescore, goal churn |
| `vantage_index` | -1 | vantage selection |
| `n_vantages_valid` | 0 | vantage selection |
| `vantage_los_clear` | 0 | vantage selection |
| `dwell_sec` | 0 | — mirrored fine in events, see below |
| `prox_hold_count` | 0 | proximity-hold behaviour |
| `prox_hold_total_sec` | 0 | proximity-hold behaviour |
| `rejected_by_unreachable` | 0 | goal rejection accounting |

Two details worth keeping:

- **`dwell_sec` is dead in the CSV but live in the events** —
  `state_change.from_dwell_sec` is what the §11 decomposition was built on. So
  this is a CSV-writer gap specifically, not a missing measurement. Cheapest of
  the eight to fix.
- **The rejection counters are half-wired**: `rejected_by_minpos` does fire (3
  distinct values, arm-varying) while `rejected_by_unreachable` never does. One
  is wired and the other is not, rather than both being switched off.

The three `reconnect_*` / `rejected_by_minpos` columns are **sparse but not
dead** — `off=1` distinct value against `hybrid=294` is exactly right, since an
off cell can never populate a reconnect field. Not a defect; do not "fix" them.

### Consequence for the build window

The queued planner task was per-voxel transfer logging at the merge (four
blocked analyses). This is now a **fifth**, and the two should be done in the
same window since both are logging-only and neither changes behaviour. Wiring
`target_id` alone unblocks the resume question; the vantage and prox_hold
columns are cheap to add alongside since the columns already exist in the
schema and only need populating.

**Until then, §11's `pursuit-released` proposal stays a hypothesis and must not
be implemented** — there is no evidence yet that goal abandonment is what the
61% consists of.

## 15. Arm balance, and a confound to check in pb2's tail

**Status at 18 cells (off 12, hybrid 6):** completion time ratio **1.379**
(exact p=0.0739), distance **1.393** (exact p=0.0414, 18,564 splits).
Leave-one-out 1.258–1.531 / 1.290–1.539.

**The effect estimate has stopped climbing.** Trend across n: 1.019 → 1.227 →
1.292 → 1.393 → **1.379**. The last two landings moved it by ~1%, where the
early ones moved it by 20+ points. Combined with the stable leave-one-out range
this reads as a real ~1.35–1.40 effect that is simply under-powered, not an
estimate still drifting.

> **WRONG — RETRACTED, see §16.** The very next cell (`pb2_hybrid_seed5`,
> 680 s) moved the ratio to 1.290 and flipped distance from p=0.0414 to
> p=0.0745. Two consecutive small moves are not a plateau; the estimate was
> never stable. Do not quote the plateau claim or the 1.35–1.40 range.

### Off cells are now nearly worthless; hybrid cells are worth 4x

Two-sample power is governed by the harmonic mean of the group sizes, so at
12 off / 6 hybrid the effective n is **8.00**:

| next cell | arms | effective n | gain |
|---|---|---|---|
| +1 off | 13 / 6 | 8.21 | +0.21 |
| +1 hybrid | 12 / 7 | 8.84 | **+0.84** |

**One hybrid cell buys exactly 4.0x what one off cell buys.** This is not an
argument to change the queue — pb2 already runs 14 hybrid against 8 off, so
both arms finish at **16/16** and the imbalance is a transient artifact of the
interleave starting on hybrid plus 8 pre-existing off cells. It IS an argument
against ever topping up the off arm opportunistically, which would look like
free data and buy almost nothing.

### The confound to check when pb2 finishes

pb2's tail is `hybrid:9 hybrid:10 ... hybrid:14` — **six hybrid cells
back-to-back with no off cell between them**, roughly 2.8 h. That is precisely
the drift the interleaving was designed to prevent: any slow change in machine
state over that window loads entirely onto the hybrid arm and is
indistinguishable from an arm effect. It is unavoidable given 14 vs 8, so it
must be checked after the fact instead of designed away.

**The check:** compare `pb2_hybrid` seeds 9–14 against seeds 1–8. Under the null
of no drift these are exchangeable replicates of the same arm and configuration,
so the same exact permutation test applies. If the tail block is systematically
slower, the pooled hybrid median is inflated and the headline ratio must be
recomputed from the interleaved cells only. Add this to the pb2 read-out — it is
the last structural threat to the tx=30 dense comparison.

## 16. Retraction: there was no plateau, and there is no bimodality either

`pb2_hybrid_seed5` landed at **680 s** — a hybrid cell faster than the off
median. One cell:

| | n=18 (off 12 / hyb 6) | n=19 (off 12 / hyb 7) |
|---|---|---|
| completion time | 1.379, p=0.0739 | **1.290, p=0.1353** |
| pair distance | 1.393, **p=0.0414** | **1.319, p=0.0745** |

**The distance result has lost significance.** §15's "the effect estimate has
stopped climbing" is retracted: two consecutive small moves were a coincidence,
not convergence, and the third landing moved the ratio 6.5% and crossed 0.05.

### Why leave-one-out gave false comfort

§15 leaned on the stable LOO range (1.258–1.531). That was the wrong guard.
**LOO can only explore the convex hull of values already observed.** Hybrid's
observed range was 1199–1719 s; the new draw at 680 s fell entirely outside it,
so no LOO subset could have anticipated it. LOO answers "does one cell *carry*
this result?" — it does not answer "will the next cell *change* it?" Only the
second question matters while a campaign is still running. **Do not quote LOO
stability as evidence that an estimate has settled.**

### And the tempting two-regime story is also noise

Hybrid's 7 cells read 643 / 680 / 707 | 1199 / 1265 / 1382 / 1719 — three inside
the off range, an empty stretch, four roughly twice as slow. Two candidate
mechanisms were tested and **both refuted**:

1. **Chase triggering does not explain it.** The fast cells average **3.33**
   midrun dispatches against the slow cells' **2.25** — the wrong direction —
   and the slowest cell (1719 s) has **zero** midrun dispatches, all terminal
   handovers. A hybrid cell is not slow because it reconnects more.
2. **`peer_lost` separates only by artifact.** Raw counts look perfect (max fast
   17 < min slow 18) but the count accumulates over the run, so longer runs
   mechanically collect more. Per-100 s rates overlap completely: fast
   0.59–2.64, slow 1.50–2.89. **Never use raw `peer_lost` counts as a
   predictor** — always rate-normalise.

Then the gap itself was tested against a unimodal null (one log-normal fitted to
hybrid's own values, 200k simulated 7-cell samples): **P(largest gap ≥ observed)
= 0.249**. One sample in four shows a gap this wide. The bimodality is dropped —
there is no two-regime finding here to explain.

### What can be said honestly at n=19

Use the assumption-free effect size instead of the ratio:

> **P(a random hybrid run is slower than a random off run) = 0.667**
> (0.50 = no effect). Three of seven hybrid cells beat the off median; one of
> twelve off cells is slower than the hybrid median.

Within-arm spread is the real obstacle: off ranges 569–1828 s (3.21×, CV(log)
31%), hybrid 643–1719 s (2.67×, CV(log) 40%). The distributions overlap heavily.
This is the same nondeterminism recorded earlier, now measured on both arms, and
it is why ~20–30 cells/arm was the estimate rather than the 17 the optimistic
point-estimate implied.

**Standing rule from this episode: while cells are still landing, report the
current p and the fact that it moves — never characterise the estimate as
settled, converged, or plateaued.**

## 17. The interim p-values are NOT a stopping rule

Distance p across the last four landings: **0.0468 → 0.0414 → 0.0745 →
0.0503**. It has crossed 0.05 in both directions and is currently sitting on it.
Completion time: 0.0786 → 0.0739 → 0.1353 → 0.0981.

Re-running the test after every cell and watching for p < 0.05 is **optional
stopping**, and if the campaign were halted the moment it dipped below, the true
false-positive rate would be far above 5% — with ~20 interim looks it is roughly
3–4× the nominal rate. The oscillation above is not evidence of anything; it is
what an underpowered test does while n grows.

**This campaign is protected because its size was fixed in advance**: `pb2.sh`
declares 14 hybrid + 8 off, both arms ending at 16/16, written before any of
these cells ran. That pre-registration is what makes the final p interpretable.

Rules that follow, and they bind:

- **Do not stop pb2 early**, for any reason including a favourable p. The stated
  end is 16/16.
- **Do not extend pb2** because the result is *nearly* significant. Extending on
  a near-miss is the same error wearing the other hat, and turns a fixed-n test
  into a sequential one with no correction.
- Interim numbers are for **monitoring the harness** (cells completing, no
  timeouts, no drift) — not for inference. Report them with the p and the fact
  that it moves, per §16.
- The **tx probe is a separate, differently-configured experiment**, not a
  continuation of this one. Its cells never pool with tx=30 dense (§10), so
  running it is not extending this test.

If the final 16/16 result lands near 0.05, the honest report is the effect size
with its interval and the statement that the study was powered for ~20–30
cells/arm — not a significance claim either way.

### §16 postscript: the gap filled itself

The very next hybrid cell (`pb2_hybrid_seed6`) landed at **1104 s** — inside the
707–1199 s stretch that was empty when the two-regime story looked tempting.
The gap test said P(gap this wide | one log-normal) = 0.249; one cell later the
gap is gone. Hybrid now reads 643 / 680 / 707 / **1104** / 1199 / 1265 / 1382 /
1719, a continuous spread with no structure to explain. Refuting that story
before writing it up was worth the two scripts it took.

### Running tally

| n | off/hyb | time ratio | time p | dist ratio | dist p | P(hyb slower) |
|---|---|---|---|---|---|---|
| 17 | 11/6 | 1.393 | 0.0786 | 1.409 | 0.0468 | — |
| 18 | 12/6 | 1.379 | 0.0739 | 1.393 | 0.0414 | — |
| 19 | 12/7 | 1.290 | 0.1353 | 1.319 | 0.0745 | 0.667 |
| 20 | 13/7 | 1.321 | 0.0981 | 1.345 | 0.0503 | 0.692 |
| 21 | 13/8 | 1.335 | 0.0673 | 1.378 | 0.0273 | 0.721 |
| ⋯ | ⋯ | *(no rows — see below)* | | | | |
| **32** | **16/16** | **1.299** | **0.0161** | **1.321** | **0.0088** | **0.770** |

Distance p had crossed 0.05 **three times in five landings** by n=21. Per §17
that is monitoring output, not inference, and pb2 ran to its pre-declared 16/16
regardless of where p sat on the way.

**The gap in the table is the guard working.** Rows 22–31 are missing on
purpose: §17's stopping rule makes running the permutation test an interim look,
and ~20 such looks inflate the false-positive rate 3–4×. Landings after n=21
were recorded as bookkeeping only (`peek.py` prints per-cell clocks and refuses
to aggregate). The 16/16 row is the **first legitimate inference since the guard
was set**, which is exactly what makes it reportable at face value.

**Final numbers.** Completion time: geometric-mean ratio **1.299** (hybrid 29.9%
slower), exact-permutation-by-Monte-Carlo **p = 0.0161 ± 0.0002** over 2,000,000
draws — C(32,16) = 601,080,390 is past the enumeration limit, so MC is the
declared fallback (§18). `advantage.py` independently returns 0.0164 with a
different generator, agreeing inside its own error. Distance: ratio **1.321**,
**p = 0.0088 ± 0.0001**. Leave-one-out moves completion over 1.252–1.376 and
distance over 1.276–1.392; both stay clear of 1.0, but per §16 that is hull
exploration, **not** convergence evidence.

> **Scope of this number (§22.3).** It measures the reconnection penalty *under
> greedy local navigation*. The nav **global** planner has never planned a path
> in any cell of any campaign — it subscribes to a dscovox topic that has no
> publisher. The failure is uniform across arms (0 of 66 robot-logs), so it does
> **not** confound the contrast and nothing above needs revising; what it bounds
> is external validity. Claim the configuration measured, not the design as
> documented.

#### CORRECTION: `P(hybrid slower)` is not monotonic

The paragraph that stood here claimed the statistic "has moved 0.667 → 0.692 →
0.721 monotonically while the p-value oscillated", and offered that monotonicity
as a reason to prefer it. **The monotonicity was a three-point coincidence.** It
continued to 0.733 and then fell to **0.707** at n=25 before recovering to
**0.770** at the final 16/16. Three increases in a row is not a property of an
estimator; it is what a random walk does about a quarter of the time.

The *preference* survives, on the reason that was actually load-bearing:
`P(hybrid slower)` is a direct property of the two samples, whereas a p-value is
a tail probability that swings on the smaller arm's size. That argument never
depended on monotonicity, and it was careless to bolt the coincidence onto it.

**What the final value means, and why both numbers get reported.** At n=32,
`P(hybrid slower)` = **0.770**: of the 16 × 16 = 256 cross-arm pairs, hybrid is
the slower run in **197**, with no ties. Pick one run from each arm at random and
hybrid loses about **77%** of the time — so it wins roughly **one run in four**.
That is a materially different picture from "hybrid takes 30% longer", and both
belong in the write-up: the ratio is a statement about central tendency and says
nothing about how often hybrid actually loses, and the two come apart under a
heavy right tail, which is this distribution's exact shape. The ratio alone
invites a reader to infer hybrid loses nearly every run; `P(hybrid slower)`
alone hides how large the losses are when they happen.

This is a re-expression, **not** a second test on a completed campaign:
`P(hybrid slower)` is the Mann-Whitney *U* statistic rescaled, *U* = 0.770 × 256.
Its one-sided permutation p is 0.0044, and `advantage.py`'s two-sided MWU p is
0.009 ≈ 2 × 0.0044 — the arithmetic check, not new evidence. (`scratchpad/cles.py`.)

## 18. A biased random number generator, caught by validating the fallback

At 16/16 the arms give C(32,16) = **601,080,390** splits, so the exact
permutation test stops being enumerable exactly when pb2's headline number is
due. Both scripts needed a Monte Carlo fallback. Adding one is easy; the point
is that **the fallback was checked against the exact answer on the same data
before being trusted**, and it failed that check.

`permtest.py`'s hand-rolled generator returned **p=0.0224 where the exact value
is 0.0238** — off by 7× its own stated Monte Carlo error, i.e. biased, not
noisy. Cause: a linear congruential generator with a power-of-two modulus has
notoriously weak **low** bits, and `state % m` for a Fisher-Yates index reads
exactly those bits. Replaced with `random.Random` (Mersenne Twister, equally
reproducible from a fixed seed); it now returns 0.0239 against the exact 0.0238.

**`advantage.py` was checked too and is fine** — 0.0245 vs 0.0238, about 1.5
standard errors at its iteration count, consistent with sampling noise. Its
generator uses a 64-bit state and takes the **high** bits (`(s >> 33) % n`),
which is what saves it. Two hand-rolled LCGs in the same directory, one sound
and one biased, differing only in which bits they read.

Both scripts now hold the exact test to 20M splits and sample beyond it, so they
stay exact through the rest of pb2 and agree when they cannot.

**Rule: never introduce a sampling fallback without reproducing a known exact
result through it.** The bias here was small enough (0.0014) to change no
conclusion, and invisible without the comparison — at 16/16, where only the
sampled path exists, there would have been nothing to compare against.

---

## 19. Closing the tx-probe landmine before the probe fires

§10 recorded that every completed cell carries a **distinct `run_start.params`
signature** — something seed-specific lives in `params` — so the signature can
never separate one batch from another. It also recorded the consequence: when
`10_tx1.sh` (tx=5) and `20_env1.sh` (sparse forest) start writing cells, **the
tag prefix is the only guard**, because transmit power and scenario never enter
planner params. Nothing in the data will catch a mispool.

That was written as a warning to my future self. A warning is not a guard.

**What the analysis scripts actually did.** Every pooling script selected cells
identically and inline: any directory under `/tmp/hmr_campaign` whose name
contains `_seed`, whose arm substring matches, whose manifest says `all_done`.
Six scripts, six copies, and not one of them looked at the prefix —
`permtest.py`, `spread.py`, `advantage.py`, `power.py`, `distcost.py`,
`batch.py`. `10_tx1.sh` writes into that same root. The first `tx1_hybrid_seed1`
to finish would have been averaged into the tx=30 dense baseline by all six, and
the read-out would have looked entirely normal.

**Fixed:** one shared loader, `scratchpad/cells.py`, with an explicit
allowlist — `BASELINE = {mc1, pb1, pb2, rb1, rep1}`, all tx=30 dense, all on the
post-Aug-20 binary — and an `EXCLUDED` map that records *why* a prefix does not
pool. All six scripts now select through it.

> **CORRECTION (§23.3).** "All six scripts" was the wrong scope, not just the
> wrong count. Six were the scripts *this section had looked at*; a later audit
> counted **23** that walk the campaign root with no tag and no guard. Two were
> live and dangerous — `redundancy2.py`, which the tx probe had pre-declared for
> its gate-2 read-out, and `lever.py`, whose entire purpose is deciding whether
> lowering tx is worth a campaign. Both are guarded now. The error here was
> closing a landmine by fixing the instances I had already found and then
> reporting the *class* as closed.

**It raises rather than filters, and that is the whole point.** Silently
dropping unknown prefixes would have been worse than the bug: the read-out would
print a clean number while an entire probe sat unexamined beside it, and I would
not know the probe had landed. An unclassified prefix now aborts with the
offending prefix and an example cell named. Clearing it means opening the
harness script that produced those cells and adding one line to `BASELINE` or
`EXCLUDED` — which *is* the by-hand verification §10 demanded, now unskippable
instead of remembered.

**Validated both directions before wiring it in** (`scratchpad/test_cells.py`),
applying §18's rule to a filter instead of a sampler:

1. *Equivalence.* On today's 24 cells the loader reproduces the old inline
   selection **exactly** — same cells, same order, same `completion_s`, same
   `distance_m`. A guard that also moves the numbers is not a guard, and there
   would be no way to tell the two effects apart afterwards.
2. *Fail-loud.* Against a fake root containing a cell named `tx1_hybrid_seed1`,
   it aborts and names it. Test 2 fails the build if it ever returns quietly.

The fake root is built in the scratchpad from symlinks, never in
`/tmp/hmr_campaign` — a stray directory there would outlive the test and be read
as real data by every script in this directory.

Post-wiring, all six scripts reproduce their existing numbers unchanged
(time ratio 1.388, exact p=0.0238; distance 1.436, exact p=0.0079 at off 15 /
hybrid 9). The guard is a no-op today by construction. It stops being one the
moment the probe starts.

**Generalising §18:** validate a *filter* the way you validate a *sampler* —
against the result it is meant to reproduce, on data where you already know the
answer. Both bugs share a shape: they are invisible exactly when they matter,
because by then the uncontaminated comparison no longer exists.

---

## 20. Pre-registering the tail-drift test, before the tail exists

`pb2_off_seed14` landed at 658 s and the **off arm is now frozen at 16 cells**.
Everything left in the queue — `pb2_hybrid_seed9` through `seed14` — is hybrid,
six cells back-to-back over roughly 2.8 h with no off cell interleaved.

Two consequences, and the second is the dangerous one.

First, these are the high-value cells. Power runs on the harmonic mean of arm
sizes (§15), so at 16 off / 10 hybrid the effective n is 12.3, and each of the
six remaining hybrids buys far more than an off cell would have. The queue was
written to end this way on purpose.

Second, **time-order and arm are now perfectly confounded inside that block**.
If anything drifts across 2.8 h of continuous simulation — thermal throttling, a
slow leak in the sim, the disk filling — it lands entirely on the hybrid arm and
inflates the headline. And because the off arm can no longer change, *every*
remaining movement in the pooled ratio comes from those six cells.

That is precisely the situation where it is tempting to choose the test after
seeing which answer it gives. So the test is fixed now, in
`scratchpad/taildrift.py`, while the data does not exist:

- **Statistic:** exact permutation on the mean-log difference, complete
  enumeration of all C(14,6) = 3,003 splits.
- **Split:** `pb2_hybrid_seed9..14` (back-to-back) vs `pb2_hybrid_seed1..8`
  (interleaved). Only pb2 hybrids — `mc1_hybrid_seed20` and `pb1_hybrid_seed3`
  are excluded, because mixing batches would fold a batch effect into a
  within-batch time-order test.
- **p ≥ 0.05** → no detectable drift; the pooled 16/16 ratio stands as headline.
- **p < 0.05, tail slower** → the pooled hybrid median is inflated by time-order;
  recompute the headline from interleaved cells only and report the pooled
  figure as an upper bound.
- **p < 0.05, tail faster** → also a finding, and the more interesting one: the
  back-to-back block is not the hazard and the pooled ratio is conservative.
  Report it; do not quietly drop it for pointing the wrong way.

**The script refuses to run early.** With fewer than 6 tail cells it aborts
rather than printing a partial answer — an early run is an interim look, and
§17 is exactly about what interim looks do to a false-positive rate. Verified:
it currently aborts at 0 of 6.

**Stated in advance, so it cannot be quietly omitted later: this test is
blunt.** At 6 vs 8 cells against hybrid's log-sd of ~0.37 it detects only about
a 1.6× drift at 80% power. A null result rules out a drift large enough to
manufacture the 1.36 effect; it does not show the tail is clean. The p and the
detectable size get reported together or not at all.

Same discipline as §18 and §19, applied one step earlier: there, a sampler and a
filter were validated against a known answer *before* being trusted. Here there
is no known answer to validate against, so the substitute is committing to the
decision rule while the outcome is still invisible. If the rule turns out to be
wrong, that gets said in the write-up — the original stays in the file.

---

## 21. An outside variance analysis: one real defect, one inverted conclusion

A second analysis pass over the running campaign proposed four causes for the
spread in hybrid finish times and three actions. Checking it produced the most
useful planner finding of the week and, in the same pass, a clean example of a
correct mechanism pointed at the wrong remedy.

### 21.1 The candidate-starvation stall is real, and larger than reported

Claim: `pb2_hybrid_seed2`'s atlas spun in PLAN logging
`all 213 candidates rejected (close=1 map=79 unreach=132 blk=1 minpos=0)`
2039 times across "three back-to-back ~178 s bouts" (~613 s), each starting just
after a large fused-map load.

`stall.py` scans every completed cell's `planner_*.log`, groups retries into
bouts separated by >5 s, and reports the modal breakdown:

- The 2039 count and the message text are exact.
- It is **six** bouts totalling **739 s**, not three totalling 613 s.
- Modal breakdown is `close=1 map=74 unreach=194 blk=1 minpos=0` — 194 of 213
  candidates rejected as unreachable.
- It appears in **1 of 29 completed cells**. hybrid 1/11, off 0/16, pursuit 0,
  rendezvous 0.

The 0/16 in off looks decisive and is not: Fisher exact on 1/11 vs 0/16 is
p ≈ 0.41. One cell can never establish arm-specificity by frequency. That is
precisely why the mechanism, not the count, had to carry the argument.

### 21.2 Testing the mechanism instead of the frequency

The proposed mechanism — "the merged peer map floods the candidate set with
unreachable frontiers" — makes a sharp, falsifiable prediction: **every bout
must begin shortly after this robot ingests a peer map.** Bouts at random times
relative to merges would kill it. `stall_cause.py` checks all six:

| bout | window | stalled | preceding map/peer line |
|---|---|---|---|
| 1 | 406.7–622.4 s | 215.7 s | fused map load 0.3 s earlier, 921,969 voxels in ROI |
| 2 | 636.9–854.0 s | 217.1 s | fused map load 12.0 s earlier, 1,046,047 |
| 3 | 876.6–1091.9 s | 215.3 s | fused map load 8.1 s earlier, 1,209,354 |
| 4 | 1113.4–1119.2 s | 5.8 s | fused map load 8.3 s earlier, 1,212,072 |
| 5 | 1356.6–1441.8 s | 85.2 s | fused map load 11.2 s earlier, 1,215,376 |
| 6 | 1459.8–1460.1 s | 0.3 s | "Pursuit: team reconnected mid-chase (1/1) -> re-planning against merged map" 0.5 s earlier |

Six for six, within 0.3–12.0 s. The mechanism is confirmed.

### 21.3 Why confirming the mechanism *forbids* the proposed exclusion

The recommendation was: "worth a fix or at least an exclusion rule — it's a
stall, not exploration behavior, and it contaminates the arm mean."

The mechanism it rests on rules the exclusion out. The two possibilities point
opposite ways, and the test picked the one that closes the door:

- **Arm-independent bug that happened to land in a hybrid cell** → genuine
  contamination, and a pre-specified exclusion rule would be defensible.
- **Treatment-caused** → the stall is a *cost of reconnecting*, and it belongs
  in the hybrid arm's number.

> **CORRECTION, same day.** The first version of this section justified the
> second bullet with "an off cell cannot produce it, because off never merges a
> peer map." **That is false, and the logs say so plainly.** The dscovox map
> exchange runs in **both** arms whenever the link is up. Off cells load fused
> maps 96–303 times each — **2067 loads across the whole off arm** — reaching
> **1.68 M voxels in ROI**, *larger* than the 0.92–1.21 M loads that preceded
> seed2's six bouts. What is arm-specific is the reconnection *dispatch*, not
> the map sharing. The claim was never checked before being asserted; it took
> one `grep` to refute.
>
> The 2067 merges are worth more than the retraction, because they show
> **merging is necessary but nowhere near sufficient** — 2067 off-arm merges,
> zero stalls. The plausible treatment-specific ingredient is *displacement*:
> pursuit parks the robot in territory the peer has already covered, so an
> arriving merge swallows every nearby frontier at once (bout 6's own trigger
> line is "team reconnected mid-chase → re-planning against merged map"). An off
> robot sits in its own frontier basin when a merge lands, so starvation is rare.
> That is a hypothesis, not a verified mechanism, and it is flagged as such.
>
> **The conclusion survives on the weaker premise.** Exposure is
> treatment-**elevated**, not treatment-exclusive, and elevation is already
> enough: an exclusion whose trigger the treatment makes more likely deletes
> treatment-attributable harm asymmetrically. Corollary that the false version
> hid: the planner fix protects the **off** arm too, and a future off-cell stall
> would bias the ratio the *other* way.

The evidence says treatment-caused. Excluding the cell would remove
treatment-caused harm from the treated arm — deleting part of the very quantity
the campaign measures. The magnitude shows what the exclusion would buy:
dropping `seed2` moves the time ratio 1.342 → 1.274, roughly a fifth of the
effect, recovered by discarding a real instance of it.

**Fixing the planner is legitimate; excluding the cell is not.** The
relaxation/escape path for candidate starvation goes on the build-window list
(no builds while the campaign runs — all cells share one binary). Any fix
changes the binary and therefore starts a new campaign generation.

### 21.4 The two-regime efficiency claim fails, and fails informatively

Claim: hybrid splits into fast runs harvesting 7000–8200 voxels/m and slow runs
at 3500–4500, with nothing between, and "that alone produces most of the SD".

This is the bimodality claim in a new variable. The same split was asserted on
completion time at n=7, tested (largest-gap p=0.249 under one log-normal), and
then refuted empirically when seeds 6, 8 and 9 landed at 1104, 780 and 910 s —
inside the alleged gap. `voxgap.py` applies the identical test to vox/m:

| arm | n | range | largest gap (log) | p | reading |
|---|---|---|---|---|---|
| hybrid | 11 | 3449–8173 | 0.284 | **0.460** | unremarkable for one distribution |
| off | 16 | 3275–9009 | 0.606 | **0.002** | flags — one low outlier |

The arm that flags is **off**, not hybrid, and it flags on a single cell:
`rb1_off_seed3` at 3275 against a 6006–9009 body. That is an outlier, not two
regimes — and it is invisible to a pb2-only view, which is what the outside
analysis used.

**vox/m cannot be independent evidence in any case.** Total voxels barely moves
while distance swings:

| arm | voxels CV | distance CV |
|---|---|---|
| hybrid | 6.7% | 33.6% |
| off | 4.5% | 32.7% |

So vox/m ≈ k/distance, and the same analysis states finish time is proportional
to metres driven. "Two regimes in vox/m" and "two regimes in finish time" are
near-algebraically one statement about one set of runs, and the second already
failed twice.

### 21.5 Stratifying on voxels-per-metre is a post-treatment-variable trap

Independent of the gap test, the proposal to "stratify on voxels-per-metre as a
cheap per-run covariate" must be refused. vox/m is an **outcome affected by the
arm**: hybrid drives further for the same map, and that *is* the treatment
effect. Conditioning on a post-treatment variable biases the arm contrast toward
zero — it adjusts away the thing being estimated. The rule generalises: only
pre-treatment quantities are admissible as covariates here, and since arms share
seeds and configuration, there are essentially none.

### 21.6 The off-arm exclusion behind the headline

The headline was "noise almost entirely in the hybrid arm": hybrid 596–1671 s
(CV 36%) vs off 531–853 s (CV 18%). The off figure comes from restricting to
pb2, which drops `rb1_off_seed3` at **1828 s** — the slowest run in the entire
campaign, and an **off** run. Campaign-wide, off CV(log) is 28%, not 18%.

Hybrid's variance genuinely is higher, and the derived point is **conceded**: a
pooled-SD power estimate is optimistic for this arm, so the ~30-cells/arm figure
understates hybrid's requirement. The remedy is more hybrid cells, not covariate
adjustment.

Their times also run ~47–48 s below the ones in this document throughout,
because they read the `exploration_complete` clock where the analysis scripts
use `run_end`. Documented in §12; not a defect in either.

### 21.7 The endgame-churn recommendation was already audited

Reporting time-to-first-complete alongside run end was checked earlier: teardown
tail 123 s (off) vs 129 s (hybrid), and the ratio is 1.497 from `t_first`
against 1.589 from `t_end`. The conclusion is unchanged by the choice, so this
stands as a robustness note rather than a change of primary metric — and the
primary metric stays wall-clock time to finish, per the standing instruction.

### 21.8 What generalises

An outside pass found a real planner defect that nothing in the existing scripts
was looking for, and `stall.py` now checks it across every cell as a matter of
course. That is the value, and it was worth the check.

The failure mode worth naming is that **a correct mechanism was used to justify
the remedy it rules out**. Once "the merged peer map causes the stall" is
established, exclusion is off the table by the same stroke that made the
diagnosis credible — because the treatment raises exposure to the cause. The
habit that catches this: after confirming a mechanism, ask **which arm elevates
exposure to it** before deciding what to do about it.

The correction in §21.3 sharpens that habit rather than weakening it. The first
draft reached for the strongest available form — "the control arm *cannot*
produce this" — and asserted it without checking. Outright impossibility in the
control arm is rare, and reaching for it is a tell: it is the version of the
argument that needs no quantification, which is exactly why it is tempting and
exactly why it should be verified first. **Elevated exposure is the common case
and is already sufficient**, so nothing was gained by overclaiming and the whole
argument was staked on a `grep` that had not been run. Rule: when an argument
rests on a factual premise about the other arm's behaviour, check the other
arm's logs before writing the sentence — the check is cheaper than the
retraction.

It also cost a real finding. The true picture — 2067 off-arm merges producing
zero stalls — is *more* informative than the false one, because it establishes
that merging is necessary but not sufficient and points at displacement as the
missing ingredient. The overclaim did not just risk being wrong; it hid the
better result behind an argument that no longer needed it.

Second: two of the four causes rested on a sample that silently dropped the
campaign's most extreme cell, and it happened to be the one working *against*
the conclusion. Both the "noise is in hybrid" headline and the missing off-arm
outlier trace to the same restriction. Same lesson as §19 — state the cell
selection explicitly, and prefer a loader that refuses unknown prefixes over one
that quietly filters.

---

## 22. A fused planning map: the premise is right, and checking it found a dead nav planner

A follow-up proposal argued that the candidate starvation of §21.3 has a
structural cause — the exploration planner extracts frontiers from the *fused*
map but filters them against a *local* map — and that the fix is to have
dscovox publish a fused 2D planning map. Both halves of the premise are
correct. Checking them also turned up something nobody was looking for.

### 22.1 The domain mismatch, in the wiring

Two different maps, on purpose, and the seam is where the starvation lives:

| Use | Topic | Built from |
|---|---|---|
| Frontier extraction, EIG, scoring | `/<r>/dscovox_node/scovox` (`ScovoxMap`) | **fused** — every robot's voxels |
| Occupancy + reachability filter | `/<r>/scovox_node/global_planning_map` | **own sensors only** |

The fused subscription is `explo_planner_node.cpp:2203-2212`; the 2D one is
`explo_planner_node.cpp:2220-2226`, pointed at the scovox topic by
`run_explo_sim_rviz.sh:1156-1157`. So candidates are proposed in ground the
robot has only ever seen through a peer, and then rejected because the robot's
own grid has never observed a path to them. That is exactly the shape of the
seed-2 modal breakdown, `close=1 map=74 unreach=194 blk=1 minpos=0`: the
overwhelming majority fail *reachability*, not occupancy.

One worry I had about this story is closed. If the reachability flood were
radius-capped near 10 m, peer territory would be unreachable no matter what map
it ran on, and a fused grid would buy nothing. It is not: the harness sets
`cost_grid_radius_cap_m=500.0` (`run_explo_sim_rviz.sh:392`, 10×`ROI_HALF`,
deliberately past the 212 m grid diagonal), confirmed in the manifest of the
affected cell. The flood is unbounded within the grid. The limit really is the
map.

### 22.2 The thing that was not being looked for

The planner's *default* `planning_map_topic` is
`/<r>/dscovox_node/planning_map` (`explo_planner_node.cpp:2179-2181`), which the
harness overrides. That default is not a leftover — it is the stack's design
intent, and `simple_nav_3d` still follows it. In dscovox mode the nav **global**
planner is pointed at the same topic, with a comment saying it "reads dscovox's
merged planning_map directly" (`simple_nav_3d.launch.py:517-524`).

`dscovox_node.cpp` contains no planning_map publisher at all. It exposes a
`GetOccupancyGrid` *service* (`dscovox_node.cpp:242-244`, `802-850`) and nothing
else; a launch comment at `simple_nav_3d.launch.py:462` already records that the
planning_map params passed to it are "undeclared no-ops". Only `scovox_node`
publishes planning maps (`scovox_node.cpp:903-904`). The topic the nav global
planner subscribes to has no publisher anywhere in the stack.

**Consequence: the nav global planner has never planned a path, in any run of
any campaign.** `has_map_` is set only in the map callback
(`simple_nav_planner_node.cpp:441-443`) and gates the entire tick
(`:600-602`); everything downstream — path search, corridor emission, "goal
reached" — sits behind it.

The evidence needs care, because the original argument for this was unsound as
stated. It reasoned from *absence*: the global planner's log holds only a
startup line and `Goal changed` lines, and every post-gate message is missing.
But the node is launched with `--log-level simple_nav_global_planner:=warn`, and
every one of those messages is `RCLCPP_INFO` — under that flag the absence would
be expected whether or not the map arrived. Absence of an INFO line is not
evidence when INFO is suppressed.

What settles it is a **within-file control**. Both planners are the same binary
with `pipeline.role` flipped, fed the same goal stream and the same odom, and
both log to the same file. Their message inventories differ:

```
[atlas.simple_nav_global_planner]: Goal changed …                        33   (pre-gate, in the goal callback)
[atlas.simple_nav_global_planner]: planner node started: role=global …    1
                                   in_map=/atlas/dscovox_node/planning_map
[atlas.simple_nav_local_planner]:  planner cleared active goal: goal reached   32   (post-gate)
[atlas.simple_nav_local_planner]:  Goal changed …                          3
```

INFO is demonstrably *not* suppressed for the global planner — it emitted 33
INFO lines. It reached the post-gate path zero times while its twin reached it
32 times. `has_goal_` is true (that is what makes `Goal changed` fire) and odom
is shared, so the only gate that can differ is `has_map_`.

Swept across every cell in the campaign root — 66 robot-logs, all four arms,
mc1/pb1/pb2/rb1/rep1:

```
global-planner post-gate lines:  0
global-planner non-startup other: 0
local-planner  post-gate lines:  2002
```

Zero out of 66. Navigation has run this entire research programme on the local
planner alone: free A* over the 20 m rolling scovox window straight at the raw
goal, with the global→corridor→local design never once engaging (`corridor_active`
requires `has_global_path_`, `simple_nav_planner_node.cpp:642-643`, and no global
path is ever published).

### 22.3 What this does and does not mean for the campaign

**It does not confound the arm contrast.** The failure is uniform — 0 of 66
robot-logs, both arms, every prefix. Nothing about off-vs-hybrid is explained by
it, and no result in §1–§21 needs revising.

**It does bound external validity, and that has to be said in the write-up.**
Every number in this document characterises a stack whose global path planner
is inert. The reconnection penalty we are measuring is the penalty *under
greedy local navigation*. A repaired stack could move it in either direction,
and the honest claim is about the configuration measured, not about the design
as documented.

### 22.4 The fix, and why it is cheap

dscovox already computes exactly the required projection: `occupancyGridOnGrid`
does a 2D max-projection of fused `p_occ` over a z-band and returns a
`nav_msgs/OccupancyGrid` (`dscovox_node.cpp:802-850`). What is missing is a
periodic latched *publisher* wrapping it, with `scovox_node`'s world-fixed
semantics ported over — 0.40 m cells, 3×`ROI_HALF` side, origin at the world
centre, matching `scovox_node.cpp:461-472`.

Two properties make it a better input than what the planner uses today:

- **Frame.** `scovox_node` stamps its grid in `<robot>/odom`, which only works
  because `map→odom` is published as identity; the planner warns about it
  (`explo_planner_node.cpp:2235-2243`) and nothing enforces it. dscovox works in
  `map` natively, so the fused grid is stamped in the frame the planner actually
  indexes in. The fix removes a latent trap rather than adding one.
- **Graceful degradation.** With the link down the fused grid *is* the local
  grid, so the off arm and disconnected stretches behave as they do now.

### 22.5 Stage it, and publish under a different name

The moment `~/planning_map` exists on `dscovox_node`, the nav global planner
wakes up on its own — it is already subscribed, with transient-local QoS, so it
would latch the first message. That would land two behavioural changes in one
build and make the next campaign contrast unattributable.

So: **publish as `~/global_planning_map` on `dscovox_node`**, mirroring
`scovox_node`'s own world-fixed/rolling naming split. Nav stays exactly as
inert as it is today, and the exploration planner is repointed with a one-line
harness change (`run_explo_sim_rviz.sh:1157` →
`/$r/dscovox_node/global_planning_map`). Waking nav is then a separate,
separately-attributable decision.

### 22.6 Risks, including two the proposal did not raise

Carried over from the proposal, and agreed:

1. It does not replace the escape path. With zero genuinely reachable frontiers
   the planner still spins in PLAN. §21.3's fallback stays on the list.
2. Stale peer free space is safe in this static sim and is a real hazard on
   hardware. Note it now, solve it later.
3. It is a behavioural change to **both** arms and starts a new campaign
   generation, exactly like the starvation fix.

Two more, from the wiring above:

4. **The local-map filter may be acting as an accidental territorial
   partition.** Today a robot physically cannot pursue a frontier it has no
   own-sensor path to — which is most of the peer's ground. With the flood
   unbounded at 500 m (§22.1) and a fused grid underneath it, peer territory
   becomes reachable at *any* range. The filter that is starving seed 2 may also
   be the thing keeping the robots off each other's ground, and removing it
   could raise duplicated coverage — the very quantity §11 and the redundancy
   work are about. This is a hypothesis, not a prediction, but it is the
   plausible way the fix backfires and it should be measured, not assumed away.
5. **`map=` rejections could go up, not down.** The proposal expects both
   `unreach` and `map` to fall. `unreach` should: peer corridors become
   floodable. `map` is less obvious — peer-observed *trees* become known
   occupied where today they are merely unknown, so some candidates that pass
   today would be correctly rejected. Net direction is an empirical question.

### 22.7 Pre-registered predictions

Stating these before the build, so the fix can fail visibly (same discipline as
§20):

- On the seed-2 replay, `unreach` falls from ~194/213 to under a third of the
  candidate set. If it does not, the domain mismatch was not the mechanism.
- Total PLAN retries across a matched campaign fall; no cell shows a >100 s
  starvation bout.
- Completion-time ratio: **no directional prediction.** §22.6.4 says the sign is
  genuinely open. Predicting a win here would be the same error as §21's
  exclusion argument — reaching for the version that needs no measurement.

### 22.8 Timing

Nothing gets built. pb2 has one cell left and pb3 is queued behind it; all cells
share one binary, and building mid-campaign would silently split the generation.
This goes into the single build window with the other three items, and the
current 16/16 gets reported against the current planner — dead nav global
planner, seed-2 stall and all.

---

## 23. The tx probe: writing the gate before the data, and finding the gate was fake

The tx probe (`10_tx1.sh`, tx=5.0 vs the baseline's 30.0, 4 cells) pre-declared
its own read-out order back when it was queued:

1. **Gate 1 — did the manipulation happen?** `silent%` / `trig%` from
   `link_states.csv` must rise clearly above the tx=30 baseline. If a 25 dB cut
   does not move outage, nothing downstream means anything.
2. **Gate 2 — is tx a lever?** *Only if gate 1 passes*, `overlap%` must rise
   above the tx=30 off median. If gate 1 passes and gate 2 fails, tx is dead as
   a lever and the full campaign is not run.

`scratchpad/txgate.py` implements that, and was written **while the probe was
still on its first cell** — before any tx=5 number existed to tune against.

### 23.1 A negative control turned the gate into a rubber stamp

The first implementation read gate 1 as `probe_median > baseline_median`. To
check the code path before committing the probe to it, `PROBE` was pointed at
`rb1` — a batch **at the same tx=30** — which should show nothing.

It returned **PASS**, at silent **+4.6** and trig **+11.1** points.

Two identical-power batches cleared the gate on batch-to-batch noise alone. The
probe script had said "rise **clearly**"; `>` is not "clearly", and as written
the gate would have certified a disconnected knob as a working one — the exact
inversion of what a manipulation check is for. Any tx=5 result that followed
would have inherited the false assurance.

The bar is now the **baseline off-arm 90th percentile**: the probe's typical run
must be more silent than about nine in ten baseline runs. Re-running the same
negative control now returns **FAIL** (silent −9.4, trig −5.3 vs bar), which is
the correct answer for two batches at the same power.

**The general point.** A manipulation check needs a null calibration, not just a
direction. The data to calibrate it was already on disk — two same-config
batches sitting next to each other — and cost one command to use.

### 23.2 Gate 1 runs on the `off` arm only

`silent%` is not a property of the radio in the hybrid arm: hybrid reconnects on
purpose, so it **shortens its own outages**. Pooling arms would mix the
manipulation being checked with a treatment effect on the same number, and a
null could then mean either "the knob is dead" or "hybrid closed the gap it
opened". `off` is untouched by reconnection behaviour, so it isolates the radio.
Pooled values are printed for reference and are not the decision.

The thresholds are also **re-derived from the current baseline** rather than
hard-coded at the quoted 65.9 / 42.3 / 8.6, with drift reported explicitly.
Hard-coding them would bake in a stale constant — the same failure that had just
disarmed `test_cells.py`, where TEST 2 hard-coded `tx1` as its "unclassified"
fake and went quietly inert the moment `tx1` was classified.

### 23.3 Correction: the prefix guard did not cover what §19 claimed

§19 recorded the pooling landmine as closed. The working note behind it said
*all six pooling scripts* now select through `cells.py`. **That was wrong**, and
the tx read-out is where it would have bitten.

An audit (`scratchpad/_audit_pooling.py`) counted scripts that walk the whole
campaign root with no tag argument and no guard — the only genuinely dangerous
shape, since scripts taking a TAG and globbing `{TAG}_*` (`censor.py`) are safe
by construction and were wrongly swept up by a first, too-broad pass. The count
is **23**, not zero. Two matter now:

- **`redundancy2.py`** — the script the probe **pre-declared for gate 2**. It
  would have folded tx=5 cells into the tx=30 off median it is compared
  against, corrupting probe and baseline in one step.
- **`lever.py`** — the script whose whole purpose is deciding whether lowering
  tx is worth a campaign. It would have answered its own question using the
  contaminated output of the probe it motivated.

Both are now guarded and both still reproduce their pre-guard numbers (the guard
is a no-op until a tx cell completes). The remaining ~21 are historical or
single-use and are left unguarded **on record here** rather than silently
trusted. `txgate.py` does not depend on any of them: it separates the two
batches by prefix itself and computes `overlap%` inline, using definitions
copied verbatim from `lever.py` so probe and baseline are measured identically.

**The pattern worth keeping.** Both failures in this section are the same shape:
a check that had stopped checking. A test whose fake was classified, a gate
whose threshold was noise, an audit whose scope was asserted rather than
counted. None announced itself — each kept printing a confident answer. What
found all three was running them against a case where the answer was already
known.

## 24. Implementing the fused planning map: the change, and what it voids

§22 established the diagnosis and stopped short of the code on purpose. The
campaign was still running, and one binary has to hold across every cell of a
generation. On 2026-08-21 the campaign was stopped and this was implemented.

### 24.1 What was actually wrong

The exploration planner filtered candidates against a 2D map published by
**`scovox_node`** — the robot's own measurements — while planning over the
**fused** team map for everything else. Two different domains. Ground the
partner had surveyed was `unknown` on the reachability map, and the planner
treats unknown-and-unflooded as unreachable, so those candidates were rejected.
The planner starved in exactly the region where the experiment's whole
mechanism — map sharing on reconnect — was supposed to pay off.

There was a second, quieter version of the same bug available. `dscovox_node`
already answers a `GetOccupancyGrid` service, and the obvious implementation is
to publish that. It would have been wrong: `occupancyGridOnGrid` derives its
origin and extent from the **data** — a tight bounding box that moves every time
the map grows. `isCellOccupied` reads out-of-bounds as occupied, so a
shrink-wrapped grid reports the entire unexplored world as blocked. Correct for
an on-demand service, fatal for a planner. The new publisher shares none of that
code path; its envelope is a constant of the run.

### 24.2 The change

`dscovox_node` gains `publishGlobalPlanningMap()`, a world-fixed 2D projection
of the fused Beta grid, hung off the existing publish timer (which already holds
the shared lock) and rate-limited independently at 1 Hz. Same envelope, same
resolution and same inflation as `scovox_node`'s copy, so the two are comparable
cell-for-cell. Latched QoS matching the planner's subscriber exactly.

The three-state contract is the substance of the fix:

| value | meaning | source |
|---|---|---|
| `-1` | unknown | no voxel, **or** a prior-only voxel (`isPriorBeta`) |
| `0` | free | observed, `p_occ` below threshold |
| `100` | occupied | observed, `p_occ` at or above threshold; wins ties |

The prior-only case is the one that is easy to get wrong. A voxel can exist in
the grid carrying nothing but its Dirichlet prior — allocated, never measured by
anyone. Marking those `0` fabricates free space the robots would then plan
through; marking them `100` reproduces the starvation this fix exists to remove.
They must stay `-1`.

One incidental improvement: the new map is stamped in `map_frame_`. The planner
indexes these grids with raw world XY and applies **no** transform, so
`scovox_node`'s copy — stamped `<robot>/odom` — was only ever correct while
`map->odom` happened to be identity, which nothing enforces. The merger already
works in the map frame, so that latent hazard is simply absent here.

### 24.3 Why not `~/planning_map`

Because `simple_nav_3d` points its **nav global planner** at
`/<robot>/dscovox_node/planning_map`, transient-local, and that topic has never
had a publisher. §22 confirmed the planner is dead in 66 of 66 robot logs.
Publishing under that name would have silently started it — a second,
uncontrolled behavioural change riding along in the same build, confounded with
the one being tested. The topic is `~/global_planning_map`; the nav planner
stays exactly as inert as it was. Whether to wake it is a separate decision for
a later generation.

The launch file's dead `planning_map_*` block for `dscovox_node` was left in
place rather than removed, with a comment saying why. Deleting it would erase
the evidence of the trap.

### 24.4 What was verified

- Binary changed: `f7d021689a48ae1a` → `05075b034f33c67f`; all eleven new
  parameters present in the built artifact, and in the installed copy.
- The harness gate was **split in two**. It already waited on
  `scovox_node/global_planning_map`; it now additionally waits on the dscovox
  one. These fail for different reasons — the scovox map needs only this robot's
  integration, while the fused map also needs a `ScovoxMapBinary` to have
  arrived and been fused (the merger allocates its fused grid lazily on the
  first wire frame, and publishes nothing before that). Sharing one gate would
  have let a silent regression in the fused path pass on the strength of the
  local one.

**Live smoke run** (`smk2_off_seed41`, off arm, seed 41, flatforest_dense
2-robot lidar, 420 s). Both robots passed both gates and stepped throughout.
Sampling both grids off the wire at t≈210 s, with the two samples 0.2–0.4 s
apart:

| robot | grid | w×h | res | origin | frame | unknown | free | occupied |
|---|---|---|---|---|---|---|---|---|
| atlas | own (scovox) | 375×375 | 0.40 | (−75, −75) | `atlas/odom` | 121716 | 8732 | 10177 |
| atlas | fused (dscovox) | 375×375 | 0.40 | (−75, −75) | `map` | 109696 | 15232 | 15697 |
| bestla | own (scovox) | 375×375 | 0.40 | (−75, −75) | `bestla/odom` | 120667 | 9797 | 10161 |
| bestla | fused (dscovox) | 375×375 | 0.40 | (−75, −75) | `map` | 107660 | 16376 | 16589 |

Geometry is identical, so the planner's untransformed world-XY indexing lands on
the same cell in both. The fused state mix is non-degenerate (78%/11%/11% and
77%/12%/12%), so it is neither all-unknown — which would starve the planner
harder than before — nor fabricating free space out of prior-only voxels.

And it knows more. On `atlas`, **12124 cells known to the fused map are unknown
to the own-sensor map, against 104 in the other direction** (known: 30929 fused
vs 18909 own). On `bestla`, 13079 against 72 (32965 vs 19958). So the
reachability filter was blind to about **39% of what the team had measured**,
and the merge loses well under 1% of the robot's own cells in both directions.

> **Correction.** An earlier version of this section reported 31232 fused-only
> cells against 4, and "blind to 87%". **Those numbers were an artifact and are
> withdrawn.** The measuring script kept the *first* message per topic, and with
> `transient_local` durability the first message is the replay of the last
> latched sample. Both publishers stop publishing when they have no subscribers
> (`dscovox_node.cpp:887`, `scovox_node.cpp:2555`). The fused topic has a
> permanent subscriber — the planner — so its latch is always current; the
> own-sensor topic has **none** during a normal run, since only the harness
> startup gate touches it, via a `ros2 topic echo --once` that disconnects
> immediately. The own sample was therefore a snapshot from ~40 s into the run
> and the fused sample was current, and the script reported that ~260 s age gap
> as a knowledge gap. Most of the "peer's ground" was this robot's own later
> coverage. The numbers above come from a rerun that takes the *latest* sample,
> requires ≥2 messages per topic (the second is guaranteed live, because
> subscribing is what restarts publishing), and refuses to grade unless the two
> stamps are within 3 s of each other.

The **frame differs**, and in the right direction: the own map is stamped
`<robot>/odom`, the fused one `map`. This retires the latent frame trap the
planner warns about rather than introducing one. Note the cell-for-cell
comparison above assumes the two frames coincide; the script now prints that
assumption instead of hiding it.

**The pre-registered §24.6 checks, measured on this run:**

- *`unreach` below a third of candidates* — it is **zero**, on every logged
  evaluation of both robots, against a pre-fix seed-2 modal of
  `close=1 map=74 unreach=194` out of ~270. These tallies come from the
  `Step N: selected goal` lines, which are emitted on every selection rather
  than only on rejection, so this is the full population and not a subset that
  survived a filter.
  > **Corrected 2026-08-21 — this check was near-vacuous. See §25.6.** The
  > pre-fix base rate of `unreach>0` is **1 of 72 robot-runs**. Observing zero
  > post-fix is what 71 of 72 pre-fix robot-runs also did. This bullet is not
  > evidence of anything; the map comparison above is.
- *No >100 s starvation bout* — **zero barren evaluations** on both robots, on
  both runs: 12 evaluations → 12 selections on `atlas` and 17 → 18 on `bestla`
  in the first run, 8 → 8 and 9 → 9 in `smk2`, every one yielding a goal.
  Candidate pools ran 61–281.

Both checks read the planner's own logs and are **unaffected by the sampling
artifact corrected above**, which was confined to the off-the-wire grid
comparison.

One correction to method here, because the first cut of this check was wrong.
I initially measured *selection-to-selection gaps* and found a 109 s gap on
`atlas`, apparently breaching the 100 s threshold. It was not a starvation bout:
the log for that window shows the planner in NAVIGATE the whole time on a
103 m leg, loading fused maps every ~12 s, reaching the goal at +106 s and
selecting again 1.2 s later with 245 candidates and `unreach=0`. **A gap between
selections is dominated by travel time and is not a starvation metric.** The
metric above — an evaluation that produces no goal — is the one that means what
the pre-registration says.

That long leg is itself the pre-registered risk showing up: with a fused grid
under an unbounded flood, distant peer-surveyed ground is now reachable, which
is exactly the mechanism by which duplicated coverage may *rise*.

The verification script (`mapcmp.py`) went through **three** versions, and the
first two both passed while measuring nothing:

1. Scraped `ros2 topic echo` text, read zero cells from both topics — the shell
   was on the default domain, runs use `ROS_DOMAIN_ID=42` — and still printed
   `identical geometry PASS`, because two empty maps agree on their zero width
   and height.
2. Subscribed properly, but kept the *first* message per topic, which under
   `transient_local` is the stale latch. See the correction above.

Both are §23's failure mode — a check grading a case it never measured —
committed twice in the tool built to check for §23. The version of record takes
the latest sample, proves liveness by message count, gates on stamp skew, and
**exits nonzero** on failure rather than printing a warning: v2's own-data-loss
check was a `print` with no effect on exit status, so a merge dropping 100% of
this robot's data would have exited 0.

**Two behaviours found by adversarial review that are correct but worth stating,
because they are inherited from the scovox template and now apply to a larger
obstacle set:**

- *Inflation stamps `100` over unknown.* At `inflation_m=1.5` and `res=0.40` the
  radius is 4 cells = 1.6 m, a ~49-cell disc per occupied cell, written
  regardless of the cell's prior value. `CostGrid::build` treats unknown as
  traversable but `100` as blocked, so unknown corridor mouths narrower than
  ~3.2 m between known obstacles are sealed. This is byte-identical to the
  projection the planner already ran against, so it cannot recreate the
  map-domain starvation this change fixes — but the fused map has strictly more
  occupied cells than the own map did, so **strictly more unknown is walled off
  than before the repoint**. A candidate within 1.6 m of any occupied cell fails
  `isCellFree`.
- *"Free" is not evidence of freeness.* `min_occ_` is `occupancy_vis_threshold`,
  default 0.7 and unset in the launch, so a measured voxel at p_occ ∈ (0.5, 0.7)
  publishes as `0` and can host a goal. Same convention as the template; the
  three-state table above should be read with that in mind.

The review also **verified the claim in §24.3 rather than taking it on trust**:
`occupancyGridOnGrid` does derive origin and extent from the observed-voxel
bbox, returns an empty grid when there are none, and has no inflation; and the
planner's `plan_map_query.cpp:19-22` does return `kCellNoData` out of bounds,
which `isCellOccupied` reports as occupied. Reusing it would have failed on
three axes, not one. No unintended divergence from the template was found, and
attempts to break concurrency (no lock is taken, directly or transitively), QoS
matching, late-joiner latching, indexing conventions, and param-driven geometry
divergence all came back clean.

### 24.5 What this voids

**The change is behavioural, so it starts a new campaign generation.** Every
pre-fix number is now historical, on the same rule that retired the pre-Aug-20
archive. Specifically void as evidence about the current binary:

- the tx probe (2 of 4 cells had landed — both complete, neither comparable)
- `env1` and `pb3`, which were queued and never started

Those two scripts were moved out of the queue rather than left in it: their
pre-registrations were written against the old binary, and a restarted runner
would otherwise have drained them silently against the new one.

The **pb2 headline is not retracted** — 16/16, ratio 1.299, p=0.0161 — but its
scope narrows. It is now a statement about a planner that was filtering
candidates against the wrong map. That is a real configuration that really ran,
and the §22.3 caveat at the headline already says so; it is not a statement
about the fixed system.

### 24.6 Pre-registered, before any post-fix cell runs

Restated from §22.6.4 so it is on record next to the implementation:

1. `unreach` falls below a third of candidates.
   *(Retired 2026-08-21 as a check — it was already satisfied by 71 of 72
   pre-fix robot-runs. §25.6.)*
2. No starvation bout longer than 100 s.
3. **No directional prediction on completion time.** The fix removes a
   constraint; it does not follow that runs get shorter, and predicting a
   direction here would only manufacture a result to confirm.
4. **Duplicated coverage may RISE.** If the local-only map was accidentally
   partitioning the robots into territories, giving each of them the other's
   ground makes overlap *more* available, not less. A rise is consistent with
   the fix working and must not be read as a regression.

Point 4 is the one to hold on to. It is the same trap as
`treatment-caused-harm-stays-in`: a mechanism confirmed after the fact is not a
licence to reinterpret a metric that was defined before it.

## 25. Pre-registration: pb3g2, the post-fix headline re-run

Written **before any cell of this campaign ran**, against binary
`05075b034f33c67f`. This is the first campaign of the post-fix generation
(§24.5) and re-establishes the foundation every downstream probe was built on.

### 25.1 Why this campaign and not the parked ones

The tx probe, `env1` and `pb3` were all follow-ups to the pb2 headline. That
headline is void as a statement about the current binary, so running its
follow-ups first would build on it again. The pb2 *comparison* is repeated here
unchanged; only the binary differs.

The fix is not neutral between the arms, which is the substantive reason to
redo this one first. The planner filtered reachability against the robot's own
map while drawing candidates from the fused map, so **ground learned from a
peer was systematically unreachable**. That is precisely the payoff `hybrid`
exists to collect: a robot could complete a rendezvous, receive the partner's
map, and still be unable to plan into any of it. `hybrid` was paying the full
cost of the detour with its benefit disabled. `off` also ran with the bug, but
has far less peer-derived map to be denied. **A bug that suppresses the
treatment's mechanism is not a wash across arms.**

> **Qualified 2026-08-21 (§25.6).** "Systematically unreachable" describes the
> *code path*, which is real and directly measured (39% of team knowledge
> invisible to the filter). It does **not** describe the observed frequency of
> harm: only 1 of 72 pre-fix robot-runs ever logged a single unreachable
> reject. The mechanism was armed on every run and fired on one. That weakens
> the arm-asymmetry argument above considerably — it does not weaken the case
> for re-running, which rests on the binary having changed at all.

### 25.2 Design

| | |
|---|---|
| arms | `off`, `hybrid` |
| seeds | 1–30 |
| order | seed-major: `off:1, hybrid:1, off:2, hybrid:2, …` |
| scenario | `flatforest_dense_2robot_lidar.yaml` |
| tx | 30.0 dBm, `comms=1`, `expect_outage=1` |
| duration cap | 5400 s |
| endpoint | `done_unknown_fraction=0.60`, `done_coverage_source=scovox` |
| env | `FRONTIER_ONLY=1 PURSUIT_BUDGET_MAX=2400 UTIL_GAMMA=0.5 RECONNECT_MIN_SHARE_VOX=550000 MIDRUN_MIN_SILENCE=60 MIDRUN_MAX_SILENCE=240` |

**Seed-major order is for time-balance, not pairing.** Alternating the arms
means any drift in machine state is shared between them instead of loading onto
whichever arm ran second. It does **not** license a paired analysis: the sim is
nondeterministic run-to-run — the same seed has produced 1803 s and 856 s — so
a seed is a replicate, not a matched pair. The analysis below is unpaired.
`run_campaign.sh` builds this order by default and its inline comment still
claims a paired rationale; that comment is stale, the order is right anyway.

**Endpoint is unchanged by the fix.** Completion is measured on the own-sensor
scovox map, which this change does not touch, so the definition carries across
the generation boundary even though the values do not.

### 25.3 Analysis, fixed in advance

- **Primary:** completion time (`end_t_sim` at `all_done`), ratio of medians
  hybrid/off, **exact permutation test**, two-sided. Never the bootstrap.
- **No directional prediction.** Predicting a hybrid win because the fix
  restored its mechanism would be the same error as the exclusion argument in
  `treatment-caused-harm-stays-in`. It may still lose; the detour may simply
  cost more than the map is worth.
- **Fixed n = 30 per arm. No early stopping.** Interim looks are descriptive
  only and cannot stop or extend the campaign. This is stated now because the
  temptation to stop at a good-looking p is exactly what a pre-registration is
  for.
- **Exclusions, declared now:** only cells failing a run-time gate
  (leakage / relay_set / qos / overflow / odom) or not reaching `all_done`
  within the 5400 s cap. **No exclusion on any post-treatment quantity** —
  no dropping cells for long chases, high redundancy, or peer-loss counts.

### 25.4 Secondary, and one thing that may look like a regression

- Distance travelled, same test, reported alongside.
- **Duplicated coverage may RISE**, and that is the fix working, not a
  regression (§24.6). Peer-surveyed ground is now reachable at any range, so a
  robot can cross into it; a 103 m leg was already visible in smoke. Redundancy
  remains dead as a between-arm test at this n (~4451 cells/arm needed), so it
  is reported descriptively and tested on nothing.
- Inflation now walls off strictly more unknown than pre-repoint, since the
  fused map carries both robots' obstacles into the 1.6 m inflation disc
  (§24.4). Watch `blk` in the reject tallies; it is not an endpoint.

### 25.5 The pooling guard was protecting the wrong axis

Done at cell 2 of 60, with only one `off` cell finished and no `hybrid` cell
finished — **before any outcome existed that could have informed it.** Recorded
here because doing this at analysis time would have been the same decision made
with the answer visible.

`cells.py` refuses to pool a cell whose tag prefix is not hand-classified, and
on being handed the first `pb3g2` cells it fired exactly as designed. Then the
message it printed turned out to be wrong twice over, and the second one
mattered.

**It claimed the data could not decide.** Verbatim: *"Transmit power and
scenario are absent from `run_start.params`, so nothing in the data can decide
this."* True of `run_start.params`; over-generalised to all data.
`run_manifest.txt` carries `tx_power_dbm`, `scenario`, and per-repo git hashes
for **every** cell in the root — checked across all 38, not assumed
(`prov.py`). `tx1` could have been separated automatically all along.

**It guarded configuration, and what arrived was a generation.** `BASELINE`'s
comment read "post-2026-08-20 planner binary", as though that were one
generation for good. The map fix started a second one at *identical* tx and
scenario, so every config field agrees between `pb2` and `pb3g2` and they must
never pool. A hand allowlist cannot catch a distinction nobody thought to
encode — and `BASELINE` still listed the five pre-fix prefixes as poolable, so
adding `pb3g2` to it would have silently mixed the binaries.

Three changes, all calibrated against cases whose answers were known first
(`test_sig.py`, and see §23):

| change | positive control | result |
|---|---|---|
| `SIG_FIELDS` = tx, scenario, `git_scovox`, `git_explo_planner`, required to agree across everything pooled | `pb2`+`pb3g2` (24 cells, same config, different binary) | **RAISED**, citing `git_scovox` |
| | `tx1`+`pb3g2` (different tx) | **RAISED**, citing `tx_power_dbm` |
| | `pb3g2` alone (negative control) | passed — the check is not always-fail |
| the five pre-fix prefixes demoted to `EXCLUDED` | — | pool is now `pb3g2` only |
| `load_cells` raises on an **empty** pool | — | see below |

**The trap in choosing those fields.** `sha256_explo_planner_node` is
`b05e162ca74df23b` on **both** sides of the generation boundary: the fix was in
scovox/dscovox, so the planner binary is byte-identical and only the harness
moved. The one field that looks like binary identity is blind to this change,
and guarding on it would have pooled the two generations while appearing
rigorous. Confirmed by running it as a deliberate wrong signature (`test_sig.py`
TEST 4), rather than left as an assertion. In the other direction,
`git_hmr_explo` takes **13 distinct values inside `pb2` alone** — that repo
holds this document, so its dirty-hash tracks note-taking, not code.

**The empty-pool raise is the load-bearing part**, and demoting the prefixes is
what proved it. `test_cells.py` built its fixture from "the first two `all_done`
cells in the root", which quietly assumed the whole root was baseline. Once the
pre-fix prefixes were excluded that filler became unpoolable, and the test's
closing step ran on an empty pool and printed

    0 cells, none of them zzsentinel: True

followed by `Both tests passed.` — *"none of them are the sentinel"* is true of
an empty list. It failed only because `load_cells` had just been given a raise
on an empty result. That is instance five of §23's class, this time inside the
test written to police the class, and the general shape is now explicit: **a
fixture drawn from "whatever is lying around" inherits every reclassification.**
It draws from `BASELINE` explicitly and refuses to run if that is empty.

None of this touches the §25.3 analysis plan. It changes only which cells are
allowed to reach it — and the answer is now `pb3g2` and nothing else.

### 25.6 The unreach check was near-vacuous: the base rate is 1 in 72

Written during a routine health check of the running campaign, 2026-08-21. It
is a correction to §24.4 and §24.6, not to the fix.

The per-cell scan (`scratchpad/scan_pb3g2.py`) reported every finished `pb3g2`
cell clean: `unreach=0`, zero barren evaluations. Before believing that, I
pointed it at a pre-fix batch as a negative control — a generation the fix
is supposed to have repaired must light it up, or the scan is measuring
nothing (§23's rule). It reported `pb2_hybrid_seed3` **clean**.

Two things were wrong, in sequence.

**The scan's own defect.** It read only the *last* tally line per robot log —
one arbitrary evaluation, and by the end of a run the final evaluation is
usually trivial. Fixed to aggregate the peak and the prevalence of `unreach>0`
over every tally line in the run. The negative control now fires correctly on
`pb2_hybrid_seed2`: `unreach=271`, 59 barren evaluations, exit 1.

**The finding underneath.** `pb2_hybrid_seed3` was not a miscall. It was
genuinely clean — and so is almost everything else. Measured over the whole
pre-fix corpus (`scratchpad/baserate.py`), per **robot-run**, the unit the bug
acts on:

| generation | cells | robot-runs | robot-runs with ANY `unreach>0` |
|---|---|---|---|
| pre-fix (`mc1`,`pb1`,`pb2`,`rb1`,`rep1`,`tx1`) | 36 | 72 | **1 (1%)** |
| post-fix (`pb3g2`, in flight) | 7 | 12 | 0 (0%) |

The single affected robot-run is `pb2_hybrid_seed2/atlas`, and there the bug
was total: 2039 of 2055 tally lines had unreachable rejects, peaking at 271.
That one run is where the §22 diagnosis came from. **It is n=1.**

**What this invalidates.** §24.4 reported "`unreach` is zero on every logged
evaluation" as a pre-registered check passing. Zero is what 71 of 72 pre-fix
robot-runs also produced. The check could not have failed on a typical run of
*either* generation, so passing it says nothing — instance six of §23, and this
one is in the pre-registration itself, not in a script. Twelve post-fix
robot-runs cannot resolve a difference in the rate of a 1% event under any
statistical treatment; that check is retired rather than re-scoped.

**What this does not invalidate.**

- *The fix.* The map-domain mismatch is directly measured, not inferred from
  the symptom: on the same run at t≈210 s the fused grid knew 30929 cells the
  filter's own grid did not — 39% of team knowledge invisible to reachability.
  That is a property of the code path and holds on every run, whether or not it
  produced a logged rejection.
- *The re-run.* `pb3g2` is justified by the binary having changed at all
  (`archive-pre-2026-08-20-binary`), not by the size of the bug.

**What it weakens.** §25.1's arm-asymmetry argument — that the bug suppressed
`hybrid`'s mechanism specifically, so pb2's null was measured with the
treatment disabled. The mechanism was armed on all 72 robot-runs and fired on
one. It is suggestive that the one is a `hybrid` cell, and no more than that:
one event cannot carry an arm effect. If `pb3g2` reproduces pb2's null, "the
bug was hiding the effect" is not available as an explanation.

**The general shape.** A rare failure mode looks pervasive when you only ever
look at the run that made you notice it. The distance between "the code path is
always wrong" and "the run is always harmed" is a measurement, and I wrote the
first as if it were the second. The base rate cost one script and existed on
disk the entire time.

### 25.7 Re-derived power: 30/arm is the floor, not a margin

Asked mid-campaign whether 60 cells were necessary. Re-derived rather than
cited (`scratchpad/power_n.py`), because the recorded claim — "completion time
needs ~30 cells/arm and works" — turned out to be the optimistic half of a
range.

Method: resample the **observed** pb2 within-arm residuals on the log scale,
so the known heavy right tail survives into the estimate instead of being
smoothed into a lognormal. Simulated test is the pre-registered one: two-sided
permutation on the difference of log medians, `alpha=0.05`, 1000 datasets x 999
permutations. pb2 is used as a **variance donor only** — the spread is a
property of the simulator and the forest, not of the planner patch — and no
pb3g2 outcome was read, since using the running campaign's numbers to set its
own sample size is the early-stopping move §25.3 forbids.

Power at a 25% ratio effect:

| residual pool | 20 cells | 40 cells | **60 cells** |
|---|---|---|---|
| both arms pooled (sd=0.268) | 33% | 59% | **84%** |
| `hybrid` only (sd=0.305) | 24% | 42% | **58%** |

> **Corrected 2026-08-21 — BOTH rows were wrong. See §25.8.** The simulated
> test rejects only **3.6%** of the time when there is no effect at all (pooled
> donor: 4.0%), against its own nominal 5%: with 14 distinct donor values the
> resample median lands on a repeat constantly, and tied permutations are
> counted as "at least as extreme". Corrected at 30/arm: **`hybrid` 63%** (up
> from 58%) and **pooled 80%** (down from 84%). The two rows move in **opposite
> directions**, which is the tell that this is not one bias — see §25.8. The
> conclusion below is unaffected: 63% is still not 80%.

The two arms do not share a spread: `off` runs 1.56x min-to-max over 8 cells,
`hybrid` 2.67x over 14. Pooling lets the quieter arm flatter the estimate, so
the second row is the bound to plan against — and it says 30/arm delivers
**63%** (corrected), not 80%. `metric-power-and-fairness` is corrected
accordingly.

**Consequence for the queue.** Cutting to 40 cells costs the 1.25x detection
outright (42–59%). A null at that power is not evidence of no effect; it is no
evidence. If wall-clock ever forces a smaller design, the only legitimate move
is to fix the smaller n **blind, in advance**, and restate the detectable
effect as ~1.35x — never to stop when the interim numbers look convincing.

Two caveats kept in view: the residual pool is 22 cells (14 for the bound), so
the curve carries real uncertainty of its own — the non-monotonic entries at
25/arm are Monte Carlo noise, not structure. And pb2 is a prior generation;
the claim transported here is the simulator's noise, nothing else.

> The "Monte Carlo noise" reading of the flat entries was **wrong**, and §25.8
> is the correction. They were the signature of the defect.

### 25.8 `power_n.py` had no null calibration, and it was wrong in a predictable direction

Reviewed on request, because `power_n.py` is the script that drove two
decisions this session — "keep 60 cells" (§25.7) and "off vs hybrid stays the
sole confirmatory comparison" (§26.2). Verification code gets reviewed like
product code (§23).

**The defect.** The script never evaluates `ratio=1.0`. A power simulator whose
simulated test is broken still prints a smooth, plausible-looking curve; the
only thing that catches it is asking what it does when the truth is *no
difference*, where the answer must be α. `scratchpad/power_review.py` asks,
at 20,000 datasets per cell:

| n/arm | parity | raw, `hybrid` pool | raw, pooled donor | tie-broken |
|---|---|---|---|---|
| 10 | even | 0.043 | 0.041 | 0.048 |
| 15 | **odd** | **0.029** | **0.034** | 0.046 |
| 20 | even | 0.041 | 0.045 | 0.051 |
| 25 | **odd** | **0.029** | **0.036** | 0.050 |
| 30 | even | 0.037 | 0.044 | 0.051 |
| **pooled** | | **0.0358** | **0.0400** | **0.0483** |

Nominal is 0.05 and 2 SE is ±0.0014, so **neither raw column is close** — the
published test rejects a real 1.25x effect less often than advertised because it
rejects *everything* less often than advertised.

**The mechanism, and why the curve's shape was the tell.** The `hybrid` residual
pool has 14 distinct values. Resampling with replacement makes an **odd**-n
median exactly one of those 14 numbers, so identical medians recur across
permutations; the p-value counts ties as "at least as extreme"
(`stat >= obs - 1e-12`), so p is inflated. An **even**-n median averages two
order statistics and can land on ~100 values, so it suffers far less. That
predicts damage concentrated at odd n — and the null column shows exactly that
(0.029 at both odd n; 0.037–0.043 at the even ones). It is also why the
published curve went *flat* at n=15 and n=25, which §25.7 above dismissed as
noise. It was the defect, visible in the output, and read as nothing.

**Corrected figures** (`scratchpad/power_corrected.py`, 1.25x effect; the 63%
pinned separately at 20k sims in `reconcile30.py`, ±0.7 points, null 0.0476):

| residual pool | 20 cells | 40 cells | **60 cells** | α | vs published |
|---|---|---|---|---|---|
| `hybrid` only | 28% | 47% | **63%** | 0.05 | 58% → **up** |
| both pooled | 36% | 63% | **80%** | 0.05 | 84% → **down** |
| `hybrid` only | 14% | 30% | **~47–52%** | 0.0167 | 34% → up |

**Nothing that was decided changes.** 63% is still well short of 80%, so 30/arm
remains the floor and 60 cells were still necessary; the Bonferroni loss in
§26.2 is still severe; `off` vs `hybrid` still stands alone as confirmatory.
The correction moves the numbers, not the conclusions.

**Three things this review got wrong before it got them right**, all worth more
than the corrected numbers:

1. *The reviewer's own first version over-rejected* (null 0.065–0.081). A
   permutation test on exchangeable data is exact and **cannot** exceed α, so
   the impossible result was itself the diagnosis. Cause: the smoothing
   re-centred each arm on **its own** sample mean, which makes observations
   within an arm dependent and destroys the exchangeability the test needs.
   Centring on the pooled constant fixed it. The null calibration caught the
   bug in the tool built to run the null calibration.
2. *The two draws are different populations, not two estimates of one number —
   and I got this wrong once before getting it right.* On the 22-value pooled
   set at n=30 the raw test gives **more** power than the tie-broken one
   (84.7% vs 80%) while having a **lower** null rejection rate (0.0442 vs
   0.0476). A more conservative test that is also more powerful is not possible
   for two estimators of the same quantity, so they are not that: the raw draw
   samples the **discrete** empirical distribution, whose atoms make the sample
   median both stickier (more power) and tie-prone (conservative null), while
   the tie-broken draw samples a smoothed version of it. Real completion times
   are continuous and the atoms are an artifact of having 14 and 22 cells, so
   the smoothed figure is the better proxy — but it is a *modelling* choice with
   a bandwidth in it, not a bias correction, and the honest reading of 63% is
   "63% under the smoothed model", not "63%, exactly".

   **The first version of this bullet claimed the pooled raw test was "already
   calibrated (null 0.0493)" and that 84% therefore stood.** That 0.0493 came
   from 4,000 sims, where 2 SE is ±0.0069 — it could not distinguish 0.049 from
   0.044. At 20,000 sims it is **0.0442**, and the pooled-over-n figure is
   0.0400. The pooled row was never calibrated; it was less bad, and I read
   "less bad" as "fine" because the measurement was too blunt to say otherwise.
   **A number quoted to three decimals from 4,000 draws is quoted to two more
   decimals than it has.**
3. *A residual 0.0483 is an artifact, not a bug — and that was measured, not
   assumed.* `scratchpad/null_parametric.py` reruns the identical test on
   continuous i.i.d. draws, which removes resampling-with-replacement and
   nothing else; the null lands at 0.0489 (2 SE ±0.0022, in band). So the gap
   comes from two observations in one arm sometimes being jitters of the same
   donor — a correlation permutation mixes away — and it is **conservative**.
   The corrected figures are lower bounds. The same artifact is why the α=0.0167
   null reads 0.0147 and its power row is quoted as a range.

**The rule this is the second instance of today.** §25.6 retired a
pre-registered check that could not fail. This one is a check that was never
written: a power curve with no null calibration is the same failure one level
up — a number with nothing testing it. `power_n.py` now prints a `ratio=1.00`
table **first**, picks the raw or tie-broken column by which one lands on α,
labels the result a lower bound when the null is short, and **aborts** if the
chosen column over-rejects or falls more than 10% short. It cannot produce
another curve without first proving its own test is honest.

**The gate was calibrated in both directions, and needed three attempts.**

- *It must fire.* On the 14-value `hybrid` pool it reports 28% short and
  switches columns. ✅
- *It must not fire on a correct test.* First attempt at this arm used the
  22-value pooled set, on the assumption its raw null was calibrated — the
  assumption that turned out to be the 4,000-sim artifact above, so the control
  failed for a reason that had nothing to do with the gate. Replaced with a
  control whose answer is known **by construction**
  (`scratchpad/test_gate.py`): 4,000 *continuous* donors, no repeated values,
  so resample medians cannot tie and the raw test is exact by definition.
  Measured 0.0478 pooled, gate stays quiet. ✅
- *It must not abort on a usable answer.* The first gate ABORTed on the
  tie-broken column at 0.0483 — a value already proven to be the conservative
  with-replacement artifact. Refusing to report a lower bound is not caution;
  it is a check firing on a correct answer. The two directions are not
  symmetric and the gate now says so: **over-rejection aborts** (the figures
  would be inflated and nothing downstream survives), **under-rejection within
  10% reports a bound** (biased safe, which is the direction a sample-size
  decision can absorb). ✅

**A negative control drawn from real data is only as good as the estimate of
its ground truth.** The pooled-donor set *looked* like a case where the raw test
was fine, and it took a 5× longer run to find out it was not. Where a control's
answer can be fixed by construction instead of measured, construct it.

## 26. Design change: pb3g2 becomes a four-arm campaign

Made 2026-08-21 22:15, on request, with 25 of the 60 two-arm cells complete
(`off` seeds 1-13, `hybrid` seeds 1-12) and **no rendezvous or pursuit cell of
this generation in existence.**

### 26.1 What changed and what did not

`--arms off,hybrid` → `--arms off,rendezvous,pursuit,hybrid`, same 30 seeds,
120 cells. The harness cycles **seed-major** — every arm at seed *n* before any
arm at seed *n+1* — so an interruption leaves complete blocks rather than a
truncated last arm.

The 25 finished cells were **skipped, not re-run**: same tag, same root, same
binary, same config, and `run_campaign.sh` treats a cell with `run_end_reason=`
and a clean gates verdict as complete. The interrupted `hybrid_seed13` had
`run_gates_verdict=CLEAN` but **no** `run_end_reason=`, so it was correctly
cleared and re-queued rather than counted.

**`off` vs `hybrid` is untouched**: same arms, same 30 seeds, same test, same
pre-registration (§25.3). No n was changed after seeing data.

### 26.2 Confirmatory vs secondary, decided before the arms ran

Four arms means three comparisons against `off`. Controlling family-wise error
across them costs the primary test most of its power (`power_n.py`, `hybrid`
residuals, 1.25x effect):

| cells/arm | α=0.05 | α=0.0167 (Bonferroni, 3 comparisons) |
|---|---|---|
| 20 | 42% | 25% |
| 30 | 58% | **34%** |

> **Corrected 2026-08-21 (§25.8):** both columns were biased low by the missing
> null calibration. The corrected pair is **30/arm: 63% at α=0.05 → ~47–52% at
> α=0.0167**, and 20/arm: 28% → 14%. The decision below is unchanged and if
> anything better supported — correcting for three comparisons still costs the
> primary comparison roughly a quarter of its power in absolute terms, and it
> would be paid to buy significance verdicts for two arms this campaign is not
> powered to adjudicate anyway.

So: **`off` vs `hybrid` remains the single confirmatory comparison at α=0.05**,
exactly as §25.3 states. **`rendezvous` and `pursuit` are pre-registered but
secondary** — reported as ratio-of-medians effect sizes with intervals and the
exact permutation p as a descriptive quantity, with **no significance verdict
claimed**. No multiplicity correction is applied to the primary *because no
confirmatory claim is made for the other two*. Reading a secondary p as a
finding later would be exactly the error this paragraph exists to prevent.

### 26.3 No directional prediction, including for pursuit

The only four-arm data that exists is `mc1_*_seed20`, one cell per arm on the
**void** pre-fix binary (`git_scovox=5e30b56`):

| arm | completion | pair distance |
|---|---|---|
| off | 691 s | 442 m |
| rendezvous | 662 s | 431 m |
| pursuit | **1895 s** | **1195 m** |
| hybrid | 707 s | 457 m |

Pursuit at 2.7x `off` is the reason to expect trouble there — and it is **one
run of a dead binary**, against a within-arm spread that is itself 2.7x
min-to-max. It is recorded here so that a slow pursuit arm cannot later be
presented as a prediction confirmed, and a fast one cannot be presented as a
surprise. **No directional prediction is made for any arm.**

### 26.4 Disclosure: the operator was not blind

The driver log prints `t_sim=` per cell, so `off` and `hybrid` completion times
for the 25 finished cells **were visible** while this change was made, and some
were quoted in conversation. That is a real limitation and it is recorded rather
than glossed.

What protects the primary comparison is that **nothing about it was altered**:
its arms, its n, its test and its stopping rule were all fixed in §25.3 before
any cell ran, and adding two further arms changes none of them. No off-vs-hybrid
statistic has been computed, and none will be until all 30/30 are in. The change
originated in a request for the other two arms, not in the values seen.

The rule this does not escape: **had the decision been to change `off`/`hybrid`'s
sample size, having seen those values would have compromised it outright.** The
design was safe to touch only in the dimension that adds arms.

---

## 27. Pre-registration: `pb4d`, a denser forest as the second dose point

Written while `pb3g2` was at 116/120 and **before its confirmatory test was
run**. That ordering is deliberate and is the point of §27.6: the case for this
campaign rests on three mechanism measurements taken from the OFF arm alone, not
on the off-vs-hybrid contrast. If §27 had waited for the p-value it would be
impossible to tell afterwards which one drove the design.

### 27.1 The `pb3g2` null is not a delivery failure

A treatment that does nothing and a treatment that never happened produce the
same flat ratio. Two checks separate them, and both were run before any decision
about a next campaign.

**Was there anything to treat?** `link_diag.py`, 29 off cells, 124 855 link
samples:

| | median | range |
|---|---|---|
| share of run time disconnected | 55.5 % | 6.6 – 85.0 % |
| outages per run | 16 | 7 – 36 |

Outage duration p50 9.0 s, p90 64.2 s, max 522.6 s. There is no shortage of
disconnection. The arm spends more of the run with the radio down than up.

**Was the treatment delivered?** `manip_check.py`, counting a cell as fired if
either robot recorded a non-zero `reconnect_elapsed_sec`:

| arm | cells | fired | % |
|---|---|---|---|
| off | 29 | 0 | 0.0 % |
| rendezvous | 29 | 27 | 93.1 % |
| pursuit | 29 | 28 | 96.6 % |
| hybrid | 28 | 23 | 82.1 % |

**78 of 86 treated cells, 91 %.** The off row is a known-answer anchor: the off
arm has no reconnection behaviour, so a non-zero reading there would have meant
the column was not a treatment indicator and every other row was void. It read
zero.

So the treatment reached 91 % of the cells it was assigned to, in an environment
that is disconnected over half the time, and completion time did not improve.
Whatever `pb3g2` reports, it reports it about a treatment that actually ran.

### 27.2 A claim of mine that measurement overturned

Earlier in this campaign I asserted that in roughly half the treated runs the
treatment never fired, and I said so to the operator before measuring it. The
inference was: the silence gate is 60–240 s, the untreated outage distribution
has p90 = 64 s, therefore most runs contain no outage long enough to trigger.
`outage_reach.py` supports the premise — only 11.1 % of outages clear the 60 s
floor, and at the nominal 240 s gate 25 of 29 runs contain none at all.

The conclusion was still wrong, and §27.1 measures it wrong: 91 %, not ~50 %.
The premise fails because `reconnect_midrun_silence_sec` counts time since the
last successful map **share**, not time since the link dropped, and sharing is
gated by `reconnect_min_share_voxels = 550000`. A nine-second connected window
cannot move 550 k voxels, so silence accumulates *across* a chain of short
outages and reaches the gate without any single outage being long. Outage length
was never the binding variable.

This is recorded rather than quietly fixed because it changed a decision:
the same reasoning, left unchecked, would have justified building the next
campaign around lengthening outages — which is exactly what §27.3 goes on to
rule out on independent evidence.

### 27.3 Transmit power is the wrong lever, and this overrides a standing instruction

The standing fallback for a noisy result was recorded as *"if you think this
experiment is too noisy then do lower tx power first then other types of
forests."* Lower tx is **not** being done first, and the reason is measured
rather than argued.

`link_diag.py` reads the link budget off the data —
`snr_db = tx_dbm − path_loss_db + 101` — so a cut of D dB shifts every SNR
sample down by exactly D. But `connected` is not a threshold on SNR. It is
`bandwidth_mbps > 0` out of a tiered rate-adaptation state machine with
eight-sample hysteresis (`NextBandwidth`, `hmr_comms_sim_node.cpp:584`), and a
naive re-threshold predicted 28.7 % downtime where 55.5 % was measured. That
first sweep was discarded. `tx_replay.py` ports the state machine verbatim and
**refuses to print the sweep unless the D = 0 replay reproduces the logged
`connected` column**; it agrees on 100.000 % of 124 855 samples.

| tx (dBm) | downtime | outages | p50 | p90 | ≥63 s | ≥243 s |
|---|---|---|---|---|---|---|
| 30 | 55.5 % | 485 | 9 s | 64 s | 54 | 7 |
| 20 | 67.0 % | 471 | 8 s | 97 s | 66 | 12 |
| 10 | 75.0 % | 343 | 12 s | 167 s | 73 | 13 |

A **20 dB cut is a hundredfold reduction in radiated power** and buys 54 → 73
triggerable outages: +35 %. The outage count *falls* while downtime rises,
because at low tx the short gaps merge into fewer long ones. The reason the
lever is so weak is in the same file: **0.0 % of disconnected samples had clear
line of sight** (median 6 trunks, median 64 m), against 23.9 % clear among
connected samples. Outages here begin and end when a robot walks into and out of
a tree shadow. Their duration is set by walking speed and stand geometry, not by
link margin, and no amount of transmit power shortens a shadow.

The second ground is independent of any measurement and was already written down
at `run_campaign.sh:36`: *"Transmit power is fixed hardware and identical on both
robots; it is not an experimental variable. Severity belongs to the environment
(tree density, separation), never to the radio."* A result obtained by detuning
a radio below anything a field team would deploy is not transferable.

Density is the lever that survives both objections. It is physically what a real
deployment varies, and it moves the quantity the treatment acts on.

### 27.4 The world: `flatforest_dense2`, 400 stems/ha

Generated with the shipped `densify_forest.py`:

```
--stems-per-ha 400 -o flatforest_dense2.sdf --world-name flatforest_dense2
  walled area   : 104 x 104 m = 1.0816 ha
  final density : 400 stems/ha (433 stems, 484 placement attempts)
  trunks on a 50 m link (expected): 6.21
```

Progression: `flatforest` 80 stems (74/ha) → `flatforest_dense` 270 (250/ha) →
`flatforest_dense2` 433 (400/ha).

The choice of 400 is not "more, roughly." `flatforest_dense_2robot_lidar.yaml`
records that at 250 stems/ha a 50 m link carries **about 3.9 trunks against the
~3.8 needed to reach cutoff**. `pb3g2`'s world sits precisely on the cliff edge,
where whether the radio is up is decided by the AR(1) fade and by one trunk of
geometric luck. At 400 stems/ha the expected link carries 6.21 trunks — 2.3
trunks × 11.98 dB ≈ **28 dB past cutoff** — so connection becomes the exception
that needs favourable geometry rather than the coin flip. That is a change in
kind, not just degree, and it is the largest step the validated replay sweep
covers.

Structural validation before use: world name `flatforest_dense2`, 488 models,
353 added stems, 487/488 distinct poses, zero stems outside ±52 m,
`<world>`/`<model>` tags balanced, and only `sky_box` at the origin — identical
in that last respect to the known-good `flatforest_dense.sdf`.

### 27.5 Blocking gate: the criterion must be re-calibrated in this world

**No `pb4d` cell runs until this passes.** Runs terminate when
`unknown_fraction` falls to `done_unknown_fraction`, and that fraction does not
decay to zero — trunks shadow voxels no sensor can reach, so it decays to a
floor set by stand density. Both `_world_registry.py` and `densify_forest.py`
warn that the floor **rises** with occlusion, that the criterion "does not
transfer" between worlds, and that *"no criterion window exists any more"* is a
real possible outcome of densifying.

`pb3g2` used 0.60. `floor2.py pb3g2 0.60` puts the lowest unknown fraction any
off run reached at 0.5301 — a margin of **+0.0699**, with 29/29 off cells
terminating on coverage. 1.6× the trunks can plausibly consume all of that.

> **Superseded by §29.9, and the gate has moved.** The 0.5301 above was read
> from `*.events.jsonl`, which samples the same quantity 6–8× more coarsely than
> `planner_*.csv`. Because the floor is a minimum over samples, that reading is
> biased high *by construction*; the denser source gives **0.5011**, a margin of
> **+0.0989**. The 0.0290 difference is the whole of this script's 0.03 "thin
> margin" threshold. `floor2.py` now defaults to `csv` and prints both, with the
> corrected count of 30/30 off cells terminating on coverage. The paragraph above
> is left as written because §27.5 is what the gate was pre-registered against,
> and rewriting a pre-registration after the fact is exactly what it exists to
> prevent — but the number the gate now uses is 0.5011.
>
> **The decision rule itself is superseded by §29.12.** "Minimum floor, 0.03
> margin" is not runnable on a three-seed smoke: exact enumeration of all 4060
> 3-subsets of `pb3g2` shows a 3-cell minimum reads **+0.0510** high — larger
> than the 0.03 threshold it is compared against — and would falsely fail
> **11.2 %** of the time in a world that clears the criterion by +0.0989. The
> gate is now `gate2.py`'s C1/C2/C3, calibrated in both directions against
> `pb3g2` and three constructed bad worlds. The minimum still prints, as
> supporting information, but it is no longer the decision. **Run `gate2.py`,
> not `floor2.py`, to decide whether `pb4d` may launch.**

The two failure modes are silent in a completion-time table, which is why the
check runs first:

* **floor above criterion** — no run terminates, every cell burns to the 5400 s
  cap, and completion time measures the cap.
* **floor just below criterion** — time-to-cross becomes time to approach an
  asymptote, and completion time measures the stopping rule rather than the
  exploration policy.

Procedure: run smoke cells under a **distinct prefix** (so `cells.py`'s EXCLUDED
list keeps them out of every analysis), then `floor2.py <smoke-prefix> 0.60`.
Its VERDICT block fails on `best > CRIT` or on a margin under 0.03. If it fails,
the criterion is raised until it passes and the new value is recorded here
*before* the campaign, not chosen after seeing completion times.

### 27.6 Design

Two arms, **off** and **hybrid**, 30 cells each, 60 total. Seeds 1–30. World
`flatforest_dense2`, `tx_power_dbm = 30.0` unchanged, spawn poses unchanged
(0,0) and (0,3) — the robots start close together, and separation remains off
the table as a variable.

**Rendezvous and pursuit are deliberately dropped.** §25.7 put 30/arm at 63 %
power for a 1.25× effect at α = 0.05, and a four-arm campaign spends its
correction budget buying secondary arms that can only ever return effect sizes
too imprecise to carry a claim. The compute is better spent making the one
contrast that can carry a claim as clean as possible. `pb3g2` retains all four
arms and remains the record for how the strategies rank against each other.

Paired with `pb3g2`, this gives a **two-point dose–response on environment
severity** — 250 and 400 stems/ha, everything else identical — which is a
stronger object than either campaign alone.

### 27.7 Analysis, fixed in advance

* **Confirmatory:** off vs hybrid on completion time (max `run_end`
  `t_sim_sec` over both robots), exact permutation test, **two-sided**,
  α = 0.05, one test, no early stopping and no interim look at this contrast.
* **Effect measure:** ratio of medians, reported with P(hybrid slower than off).
* **No exclusions on outcome.** Cells enter as assigned. A hybrid run made
  slower by a chase it dispatched is the treatment working as built, and
  excluding it would condition on a post-treatment variable.
* **Intention to treat**, including cells where the treatment never fires.
* **Mandatory before interpreting any null:** `manip_check.py`, with the off-arm
  anchor required to read 0 %. A null with unverified delivery will not be
  reported as a null.
* **Redundancy is not a between-arm endpoint here** — §metric-power puts the
  required n in the thousands. It is descriptive only.

**`pb4d` must not pool with `pb3g2`, and the loader currently cannot express
that.** `cells.py` holds one flat `BASELINE` set, at present `{"pb3g2"}`, and
its `SIG_FIELDS` backstop includes `scenario`. Adding `pb4d` to that set would
therefore raise on a signature mismatch — correctly, since the two campaigns run
different worlds, but at analysis time and looking like a bug rather than a
design statement. The two worlds are separate experiments joined only as a
dose–response; nothing averages across them.

`cells.py` is **not** being changed to accommodate this yet. It sits on the path
of `pb3g2`'s pre-registered confirmatory test, and editing a loader before the
test it feeds is how an analysis choice gets made with the answer in view. Order
is: run `permtest.py` on `pb3g2` → then extend `cells.py` with an explicit
baseline-group argument → then analyse `pb4d`. The floor gate in §27.5 is
unaffected either way, because `floor2.py` globs prefixes directly and does not
import `cells.py`.

The same applies to the smoke cells: their throwaway prefix must be added to
`EXCLUDED` with its reason **before** any pooled read-out is run, or
`load_cells` will raise on an unclassified prefix. That raise is the module
working as designed — it refuses to silently drop a batch it has not been told
about — so it should be satisfied by classifying the prefix, never by relaxing
the check.

### 27.8 Directional prediction, and it is not the flattering one

§26.3 declined to predict a direction. Here there is a mechanism specific enough
to commit to, so it is committed to before the data exists.

At 250 stems/ha all three treatment arms sit **above** 0.5 on P(slower than
off): every reconnection strategy costs time. The mechanism on record is that a
chase ends on ground the partner has already covered, so post-chase metres yield
about 0.40× what ordinary metres yield. Density does not change that geometry —
it makes chases more frequent and longer. Meanwhile the benefit is capped:
roughly half of duplicated ground is swept with the radio *up*, so it is
sequential redundancy that no amount of reconnecting can recover.

**Prediction: hybrid does not beat off at 400 stems/ha either, and the ratio
moves further above 1.0, not below it.** The test stays two-sided and this
prediction has no weight in it; it is recorded so that a confirmation is not
mistaken afterwards for a hypothesis the data suggested.

If instead hybrid improves at 400/ha, that is a genuine dose–response and the
most interesting outcome available — it would locate a severity threshold below
which reconnection is not worth its cost, which is a more useful claim than a
flat "reconnection helps."

### 27.9 Outcomes that would make this campaign uninterpretable

Declared now so they cannot be renegotiated later:

1. **Floor gate fails** (§27.5) → the criterion is wrong for this world; fix and
   re-run the gate. Not a result.
2. **Delivery collapses** (manipulation check well below `pb3g2`'s 91 %) → the
   contrast is diluted, and the honest report is the ITT estimate *plus* the
   delivery rate, never the ITT estimate alone.
3. **A materially non-zero REDO/FAIL rate.** `pb3g2` ran its full grid with
   zero REDO, zero FAIL/ABORT and zero `.attempts` directories. That matters
   because re-rolls preferentially discard mild-outage realisations; at zero,
   that bias provably is not present. If `pb4d` re-rolls, the count and the
   discarded seeds get reported.

   **Do not audit this with a naive grep.** `pb3g2_driver.log` contains 25
   `[campaign] SKIP ... (already complete)` lines, and a pattern matching
   `FAIL|ABORT|REDO|SKIP` returns 25 hits that look like failures and are not.
   They are resume-skips: §26.1 extended `pb3g2` from two arms to four, and the
   relaunched driver stepped over the off and hybrid cells that had already
   finished rather than re-running them. Matching `FAIL|ABORT|REDO` alone
   returns 0, `SKIP` lines not carrying "already complete" number 0, and
   `.attempts` directories number 0. Three separate reads, because the one that
   was easiest to type was the one that was misleading.
4. **Cap saturation.** Any cell ending on the 5400 s cap rather than on
   coverage is reported separately; a table mixing the two is measuring two
   different quantities.

### 27.10 Disclosure

The operator was **not blind**. I had seen `pb3g2`'s interim completion-time
table at 115 cells before writing this section, and it informed the judgement
that more cells in that world would be low-yield.

What the design does *not* rest on is stated precisely so the distinction is
auditable: the world, the density, the arm selection and the directional
prediction in §27.8 follow from the OFF-arm mechanism measurements in §27.1 and
§27.3 — link exposure, delivery rate, occlusion gating, and the calibrated tx
replay. None of those touches the off-vs-hybrid contrast. The `pb3g2`
confirmatory test had not been run when this was written, so no part of §27
could have been tuned to its p-value.

One further trap, recorded because it nearly cost the manipulation check: `phase`
reads `explore` in all 116 `pb3g2` cells across all four arms, **including
treated cells that demonstrably fired**. Unlike `target_id` this is not an
instrumentation gap — the §25 dead-column audit already classified `phase` as a
constant by configuration and marked it "do not fix" — but the practical
consequence is the same either way. Firing is detectable only via
`reconnect_elapsed_sec > 0`, and `manip_check.py` would have reported 0 %
delivery in every arm had it trusted `phase` alone. It reads both, which is why
the off-arm anchor is load-bearing rather than decorative: a check keyed on
`phase` would have anchored at 0 % correctly and then reported 0 % everywhere
else just as confidently.

## 28. Result: `pb3g2` at 120/120 — the pre-registered endpoint is a null

The grid finished 120/120 on 2026-08-23: 30 cells in each of `off`,
`rendezvous`, `pursuit`, `hybrid`, every one `run_end_reason=all_done`, zero
FAIL/ABORT/REDO, zero `.attempts` directories, one configuration signature
across all 120 (`preflight.py`). §27 was already written when the test ran, so
nothing in the next campaign's design could have been tuned to what follows.

### 28.1 The confirmatory answer

Primary endpoint, fixed in §25.3: **completion time**, `off` vs `hybrid`, exact
two-sided permutation test, α = 0.05, n = 30 per arm, no early stopping.

| | ratio of medians | geometric-mean ratio | p | verdict |
|---|---|---|---|---|
| completion time, hybrid/off | **1.026×** | 1.061× | **0.3953** ± 0.0007 | **not significant** |

`P(a random hybrid cell is slower than a random off cell) = 0.574`. Medians:
`off` 804 s (IQR 656–955), `hybrid` 825 s (IQR 720–1094).

**This is a null, and by §27.1 it is a real one rather than a delivery
failure.** 91 % of treated cells fired, the `off` arm sat disconnected 55.5 % of
run time, and 0 % of dropouts had clear line of sight. The treatment was
delivered into a world with plenty to act on, and it did not buy time back.

Leave-one-out over the 60 off/hybrid cells moves the geometric-mean ratio
between 1.036 and 1.085 — no single cell carries the result, and no single cell
rescues it either.

### 28.2 An effect-size substitution I nearly made without noticing

§25.3 pre-registers the effect size as a **ratio of medians**. `permtest.py`'s
test statistic is the difference of log means, whose exponential is a **ratio of
geometric means**. Those are different numbers, and the script printed only the
second under the heading the first was promised as.

The p-value is unaffected — a permutation test is exact under label exchange for
*any* statistic, so 0.3953 stands as computed. What was wrong was the reported
magnitude, and the gap is not cosmetic: on completion time the pre-registered
statistic says 1.026× where the computed one says 1.061×, and across the
secondary arms the two even **reorder which arm is worst** (medians: rendezvous
1.175× > pursuit 1.068×; geometric means: pursuit 1.116× > rendezvous 1.099×).
Reporting whichever of the two came out of the script, and calling it the
pre-registered quantity, is exactly the class of slippage §17 and §23 exist to
catch — small and defensible one number at a time, unfalsifiable in aggregate.

Both are now printed side by side, with the pre-registered one named. The edit
was made **after** seeing the result, which is disclosed here rather than
smoothed over; it adds a column and changes no computation, and the direction of
the correction happens to make the headline effect *smaller*, not larger.

### 28.3 Distance is significant — and it is a secondary endpoint

| | ratio of medians | geometric-mean ratio | p | tier |
|---|---|---|---|---|
| distance travelled, hybrid/off | **1.094×** | 1.159× | **0.0130** ± 0.0002 | secondary |

`permtest.py` printed this under a banner reading `CONFIRMATORY`. That banner
was wrong. §25.3 names completion time as primary and §25.4 opens with
"Distance travelled, same test, reported alongside" — distance has been
secondary since before the first cell ran. §26.2's phrase "the single
confirmatory comparison" was about **arms**, not metrics, and it does not
promote distance; the metric tiering was already fixed one section earlier.

So the honest report is: distance is **higher for hybrid by 9.4 % (medians) with
a descriptive p of 0.0130**, and no confirmatory significance is claimed for it.
It corroborates; it does not establish.

Stating the obvious counterfactual so the tiering cannot look like it is doing
work it should not: had distance been co-primary, a two-metric Bonferroni
threshold of 0.025 would still have passed it. Nothing is suppressed by calling
it secondary — and nothing is rescued by it either, because the significant
result runs in the direction of **more cost**, not less. A secondary endpoint
that confirmed a win would deserve far more suspicion than one confirming a
price.

### 28.4 The extra distance is not just extra seconds

A slower run covers more ground merely by lasting longer, so a raw distance gap
can be the completion-time gap restated. It is not:

| arm | median m per s of run time | ratio vs off |
|---|---|---|
| off | 0.572 | — |
| rendezvous | 0.604 | 1.055× |
| pursuit | 0.620 | 1.083× |
| hybrid | 0.616 | 1.077× |

Treated pairs drive **more metres per second of run time**, not just for more
seconds. That is what a chase looks like from the odometer's side: a long,
mostly-unobstructed transit leg with little frontier evaluation along it,
arriving on ground the partner has already covered — the 0.40× post-chase yield
in `chase-cost-is-yield-not-distance`, seen from the other end.

This is a ratio of two post-treatment quantities and is therefore mechanism
description, never an endpoint. It cannot be tested between arms without
conditioning on exactly the thing the treatment changes
(`treatment-caused-harm-stays-in`).

### 28.5 Twelve ratios, all above 1.0 — and why that is one fact, not twelve

Three treatment arms × two metrics × two location statistics = twelve ratios
against `off`. **Every one exceeds 1.0.** Every `P(slower than off)` exceeds 0.5,
spanning 0.574 to 0.721. There is no arm, metric, or statistic under which any
reconnection strategy comes out ahead.

That consistency is worth stating and worth not overselling: all twelve share
the same 30 `off` cells, so they are heavily dependent and this is **one
coherent picture, not twelve confirmations**. An unlucky `off` draw would tilt
all of them together. It raises confidence in the *sign*; it adds nothing to the
p-value, which is why no combined test is computed here.

### 28.6 What this null licenses, and what it does not

It does **not** say reconnection logic is worthless. It says that in *this*
world — 250 stems/ha, 55.5 % downtime, 91 % delivery, both robots spawned 3 m
apart — none of the three strategies pays for itself on the metric that matters
operationally, and all three cost distance.

It is also not a "we found nothing because n was small" null, but neither is it
an unlimited one. `metric-power-and-fairness` puts 30 cells/arm at 58–84 % power
for a 25 % ratio effect on completion time — the floor, not a comfortable
margin. So a **large hybrid win is excluded**; a few-percent one is not, and
1.026× is not distinguishable from 1.000× at this n. The correct summary is *no
useful gain*, not *provably exactly zero*.

The follow-up is therefore a change of world rather than more cells in this one,
which is what §27 pre-registers: 400 stems/ha as a second dose point on the
density axis, `off` + `hybrid` only. That choice was written down before this
p-value existed, and the reasoning behind it — occlusion gating, the calibrated
tx replay, the delivery rate — comes entirely from `off`-arm measurements that
never touch this contrast.

### 28.7 Retractions closed out by the final grid

- **"pursuit is ~1.6× slower"** rested on 2 cells. Final: 1.068× on medians,
  1.116× on geometric means. The sign survived; the magnitude was a small-sample
  artefact and the claim should never have been said aloud at n = 2.
- **"the treatment only fires in ~50 % of runs"** — retracted in §27.2 on direct
  measurement (91 %), before this result was known.
- The **interim** table at 115 cells gave `off` 813 s and `hybrid` 818 s
  (1.01×). The final 120 give 804 s and 825 s (1.026×). Five cells moved the
  ratio by 1.6 points — small, but a live demonstration of why §25.3 forbids
  early stopping: a look that lands near a threshold moves more than intuition
  says it can.

---

## 29. Building `pb4d`: the world, the gate, and the loader partition

Written 2026-08-23, after §28 and before any `pb4d` cell exists. Everything
here is machinery, not evidence: §27 pre-registered the campaign, §28 closed the
one before it, and this section records what had to be built between them and
what broke while building it. The floor gate demanded by §27.5 is **running as
this is written and its verdict is not in**, which is deliberate — a section
written after the gate reports would be free to describe whatever the gate said
as what I had planned for.

### 29.1 The world is registered, and the registry entry carries the argument

`flatforest_dense2` (433 stems, 400/ha) is now in
`hmr_sim/launch/_world_registry.py`, with the same four spawn points as
`flatforest_dense`. The entry carries the reasoning in-line rather than pointing
at this document, because the next person to change a world will be reading the
registry, not §27:

- **Why 400/ha and not more transmit power.** 250/ha puts ≈ 3.9 trunks in the
  Fresnel corridor of a 50 m link against the ≈ 3.8 needed to reach the 2 dB
  cutoff — the world sits *on* the cliff edge, where a link is decided by which
  side of one trunk a robot passes. 400/ha gives 6.21 trunks, ≈ 28 dB past
  cutoff. §27.3 records the measurement that rejected transmit power as the
  lever (a 20 dB cut buys only +35 % triggerable outages) and that this
  overrides a standing instruction to try tx first.
- **Spawn clearances are unchanged where it matters.** The two poses the
  campaign actually uses clear 2.75 m and 2.13 m — identical to
  `flatforest_dense`, which ran 120 cells without a spawn failure. Only the
  unused `(0, −3)` tightened, 5.35 → 3.70 m. §27.6 holds spawns fixed and this
  is the check that it held.
- **The floor warning, repeated where it will be seen.** `pb3g2` cleared its
  `done_unknown_fraction = 0.60` by only +0.0699. 1.6× the trunks shadow more
  voxels permanently and can eat that margin outright.

### 29.2 A checker that failed loudly on the wrong thing

`check_world_reg.py` verifies two properties per registered world: the SDF
exists, and the SDF's internal `<world name="…">` equals the registry key. The
second is the one worth automating. `robot_sim.launch.py` builds
`/world/<short_name>/create` and `/world/<short_name>/set_pose` from the
**registry key**, while Gazebo advertises those services under the name **inside
the SDF**. A mismatch does not error: the spawn call goes to a path nobody
serves, and the run starts with no robots and no stated cause. That is a failure
I would rather catch in a second than at cell 1 of 60.

Its first version reported five worlds broken — all five `cmu_*` — as
`<NO <world name=> TAG>`. That is a sweeping enough claim to be worth
disbelieving. `grep -b -m1 -o '<world name='` found the tag at byte 45, so it
plainly existed; `od -c` showed `<world name='cmu_forest'>`. Single quotes.
Equally valid XML, and the regex only accepted double.

Worth stating plainly because it cuts against the way I usually justify these
guards: **failing loudly is not the same as failing correctly.** A false alarm
here would have blocked a legitimate launch, and the natural response to a
checker that blocks a launch you believe in is to bypass the checker. Fixed to
accept both quote styles rather than special-cased. Final:
`OK: 17 worlds, every SDF present and every internal world name matches its key`,
plus an explicit assertion that `flatforest_dense2`'s first two spawns match
`flatforest_dense`'s.

### 29.3 The floor gate: `fd2s`, deliberately mis-configured

§27.5 makes re-calibrating `done_unknown_fraction` a **blocking** precondition,
because if the coverage floor in the denser world sits above 0.60 no run can
terminate, and if it sits just below, completion time stops measuring
exploration and starts measuring the stopping rule.

The smoke is three `off` cells at `DONE_UNKNOWN=0.30` — a value no run is
expected to reach, and that is the point. **A smoke at 0.60 would terminate at
0.60 and tell me only that 0.60 is reachable, not how far below it the floor
sits, which is the entire question.** So the runs burn their 2400 s and trace
the coverage asymptote instead of stopping on it. Consequences, written down
*before* the runs finished so they cannot be misread afterwards as a failed
smoke:

> `run_end_reason` will be the duration cap, not `all_done`. That is SUCCESS.
> Their completion times mean nothing — they are the cap. `floor2.py` will print
> "0/3 off runs reached all_done". Expected.

`off` only, because coverage progress in a treated arm is post-treatment. Three
seeds because the floor estimate is a minimum over runs and the sim is
nondeterministic run-to-run. Every other parameter was copied field-by-field out
of `pb3g2_off_seed1/run_manifest.txt` — checked against the harness defaults,
not assumed — so the smoke differs from the campaign that set 0.60 in exactly
two ways: the world and the stopping rule.

The gate's decision rule is fixed in advance and is `floor2.py`'s, not mine:
floor above the criterion → nothing can terminate; margin `0.60 − best < 0.03` →
the endpoint is compromised and the criterion must be raised before `pb4d` runs.
§27.9 already lists a compromised endpoint among the outcomes that would make
the campaign uninterpretable.

### 29.4 `fd2s` was excluded before it existed

The prefix went into `cells.EXCLUDED` **before the first cell was launched**,
with the reason written out: different world, deliberately unreachable stopping
rule, `off` arm only. Anything that pooled it would be pooling all three at
once, and its completion times are caps rather than measurements.

This is the same discipline as §25.5 but applied one step earlier. Classifying a
prefix after its cells are on disk is a decision made with data in view;
classifying it before there are any is a decision made on design alone.

### 29.5 `BASELINE` could not express "both current, must never pool"

The pooling guard held one flat set. That set could say *these pool* and *these
are void*, but not *these are both live and belong to different experiments*.
`pb4d` runs the same binary and the same transmit power as `pb3g2` in a
different world, so it is neither poolable nor excludable-as-void. Under the flat
set the only way to analyse `pb4d` was to add its prefix alongside `pb3g2`,
where `SIG_FIELDS` would then raise on `scenario` — correct, but at analysis
time, and looking exactly like a bug.

So the loader now holds **groups**, and a group is the unit that pools:

- `SIG_FIELDS` is still enforced **within** the selected group, so nothing is
  loosened. This names the partition that was always intended instead of
  discovering it from a signature clash.
- A prefix belonging to *another* group is **classified**: skipped, counted as
  `other_group`, and specifically not an error. Only a prefix in no group and no
  `EXCLUDED` entry is unknown, which remains the loud failure.
- A typo'd group name raises rather than falling back to the default. A silent
  default here would analyse `pb3g2` while the caller believed it was reading
  `pb4d` — the worst available failure, since both are real campaigns and both
  produce plausible numbers.
- `cells.BASELINE` survives as an alias for the default group, so no existing
  read-out changes meaning.

`pb4d` is registered in `GROUPS` now, before its first cell exists, for the
reason in §29.4.

The new partition has exactly one dangerous failure mode and it is silent in
both directions: a foreign group's prefix falling through to *unknown* makes
every read-out raise spuriously the day `pb4d` starts writing, and falling
through to *poolable* merges two worlds into one number. The existing tests
could not see either — their sentinel prefix is in no group at all — so a third
test asserts all four properties directly (foreign group not raised, not pooled,
selecting it returns exactly its own disjoint cells, bad name raises). Test 1
still reports `IDENTICAL` on the `pb3g2` pool, which is the guarantee that
matters: **the refactor changed which cells are selectable, and changed nothing
about which cells `pb3g2` selects.**

### 29.6 The pre-flight checked the wrong scope for the world that is coming

`preflight.py` enumerates every prefix on disk and checks signature agreement
before the confirmatory test runs. It read `cells.BASELINE` directly and checked
signatures over the union of everything pooled.

That was right for one group and would have gone quietly wrong for two.
Signature agreement across the union of `pb3g2` and `pb4d` is **guaranteed to
fail** — `scenario` differs by construction — and the failure would have looked
identical to the mixed-generation bug the check exists to catch. A check that
cries wolf on a designed difference gets relaxed, and then it is not a check.
It now reports the group name per prefix and checks `SIG_FIELDS` **within each
group separately**, refuses a prefix listed in two groups, and prints an
explicit "no cells on disk yet" for a group registered ahead of its first run
rather than a silent zero that would read as "checked and fine".

Current output: 8 prefixes, all classified; `pb3g2` holds 120 cells at one
signature; `pb4d` empty and correctly so.

### 29.7 The campaign root now holds only live data

`/tmp/hmr_campaign` held 8 prefixes, 6 of which must never be analysed. They
have been moved to `~/hmr_campaign_archive`, 78 entries in total (cells, their
`.attempts` siblings, and the per-prefix driver logs). Both paths are on the
same filesystem, so every move is a rename: no copying, no disk freed, and safe
to run with a sim live — which is the only reason it was done now rather than
between campaigns.

Two details worth recording:

- **`tx1` is filed separately, under `tx_probe_5dbm/`, not with the void
  generation.** The five pre-fix prefixes are void: they ran a planner filtering
  candidates against the wrong map, so their numbers are not about the current
  system at all. `tx1` ran the same binary correctly at 5 dBm — it is a
  different *experiment*, and §27.3 leans on it for a live conclusion. Filing it
  under `pre_fusedmap_fix` would, in six months, read as "voided along with the
  rest", and a directory name is the only label an archive really has. Each
  subdirectory also carries a `WHY_ARCHIVED.txt` written from the same table
  that drives the move, so the two cannot drift.
- **The `EXCLUDED` entries stay in `cells.py`.** Deleting them because the
  directories moved would mean that if a prefix ever came back it would return
  as *unclassified-then-approved* rather than as an already-decided exclusion —
  a second adjudication, made the second time with the answer in view.

Verified afterwards: the pooled `pb3g2` set is still 120 cells and every number
in the results table is byte-identical. The only lines that moved were the
`skipped: excluded=36 → 1` counter and the pooling header — cosmetic, and both
expected.

### 29.8 The gate's first launch was wrong, and it was wrong in the dangerous direction

The smoke ran for about 390 s of sim time before I stopped it. Not because the
run misbehaved — because of how I had launched it.

**The error.** `launch_fd2s.sh` passed `--duration 2400`. `pb3g2` ran
`duration_s=5400`. Since `DONE_UNKNOWN` is deliberately set unreachable, the
duration cap is the **only** terminator, so the cap alone decides how far down
the coverage curve the smoke gets to look. A 2400 s smoke sees 44 % of the
campaign's time budget.

**Why that is not a conservative error.** A short smoke reports a *lowest
unknown fraction reached* that is too **high** — the run simply had not finished
descending. Too high a floor fails the gate, and §27.5's response to a failed
gate is to **raise the criterion**. Raising the criterion means stopping runs
earlier, closer to the point where completion time measures the stopping rule
instead of the exploration policy. So the mistake would not have blocked the
campaign safely; it would have quietly degraded the endpoint the gate exists to
protect, and every downstream number would have looked fine.

**Where it came from.** `2400` is the value of `pursuit_budget_max_sec`, which
is passed on the same command line. It is a plausible slip. The part worth
recording is not the slip but the sentence in the script header: *"checked field
by field against that manifest, not assumed."* That claim is what stopped me
re-checking, and it was false for exactly one field. A written assurance that
verification happened is not verification, and it is worse than no comment at
all, because it terminates the search. Now re-verified with the manifest values
quoted in the header: `duration_s=5400`, `tx_power_dbm=30.0`,
`reconnect_min_share_voxels=550000`, `pursuit_budget_max_sec=2400`,
`exploitation_enabled=false`, `record=0` — all matching, `done_unknown_fraction`
deliberately not.

**What the aborted run bought.** It is kept, not deleted, in
`~/hmr_campaign_archive/fd2s_aborted_2400s/`, because it settled a question the
gate itself would not have answered until much later. At the same sim time
(392 s), against five `pb3g2` off cells:

| | fd2s (400/ha) | pb3g2 (250/ha) |
|---|---|---|
| distance travelled | 64 m | 99–137 m |
| map covered | 11.5 % | 26–37 % |
| voxels observed | 324 k | 970 k–1270 k |
| candidates rejected as unreachable | 0 | 0 |

The robots move, the sensor returns, frontiers exist, and **nothing is rejected
as unreachable**. `flatforest_dense2` is *dense, not impassable* — which was the
real risk in a 1.6× densification and is now ruled out on measurement rather
than on the spawn-clearance argument in §29.1, which only ever covered the two
spawn points.

It also produces a number that changes what `pb4d` costs: coverage accrues
roughly **2.5× slower per second** than in `flatforest_dense`. §27.6 sized the
campaign at 60 cells without pricing that, and 60 cells at up to 5400 s is
plausibly a multi-day run. The smoke will settle it properly — the sim time at
which an off run's `unknown_fraction` first crosses 0.60 *is* the completion
time a real `pb4d` off cell would have recorded, so the calibration run prices
the campaign as a side effect. That estimate is for scheduling only; those
crossing times are `fd2s` cells under a different stopping rule and an
`EXCLUDED` prefix, and they never enter an arm comparison.

### 29.9 The gate was reading its own measurement through a 6× coarser grid

While the smoke ran I audited the gate script itself rather than waiting for its
input. `floor2.py` read `unknown_fraction` from `*.events.jsonl`; every
diagnostic I had written that week (`diag_dense2.py`, `crossing.py`) read the
same quantity from `planner_*.csv`. Two sources for one number is worth ten
minutes even when both look fine.

They are not two measurements. `src_agree.py` compares them at shared
timestamps and finds a worst-case disagreement of **0.00e+00** — it is one
quantity, logged twice. What differs is how often: **1,385 event samples against
11,339 CSV rows** over the same 30 `pb3g2` cells, 6–8× per robot.

That is fatal for this particular statistic and harmless for most others,
because the floor is **a minimum over samples**. A minimum over a sparse grid
can only be greater than or equal to the minimum over a dense one — the sparse
reading is biased *high by construction*, not by chance. `floor_source.py`
sized it on `pb3g2`, where both sources are complete:

| | events | csv |
|---|---|---|
| floor (min over 30 cells) | 0.5301 | 0.5011 |
| margin under the 0.60 criterion | +0.0699 | +0.0989 |

Per cell the bias is nothing — median **+0.0001**. But the gate does not use a
typical cell, it uses the extreme one, and across 30 cells the minimum moves
**+0.0290**. The script's own "margin is thin" threshold is **0.03**. The
artifact was the entire decision rule: `fd2s` could have failed the gate on
logging cadence and never on the world.

And a failed gate's remedy is *to raise the criterion*, which weakens the
endpoint. This is the **same direction as the duration error in §29.8** — the
second instrument in two days whose failure mode was to look conservative while
quietly making the experiment easier to pass. Fail-safe is a property you check
per instrument; it is not a property of being careful.

**The fix, and its awkward direction.** `floor2.py` now takes a source argument
and defaults to `csv`. I am stating plainly that this moves the gate the way I
would want it moved: a lower floor is a larger margin, so the correction makes
the gate *more likely to pass* and `pb4d` more likely to launch. Two things make
it defensible anyway, and neither is "I checked my motives":

1. The bias is one-directional **by construction**. This is not a judgement that
   one estimate is nicer — min-over-subset ≥ min-over-superset is arithmetic,
   and the two sources are proven to carry identical values where they overlap.
2. It was decided **before the data existed**. At the time of the change the
   only `fd2s` cell had 35 planner rows and `unknown_fraction` was still 0.893 —
   nowhere near the floor. There was no answer available to steer toward.

Both floors now print on **every** run, so §27.5's recorded 0.5301 stays
reproducible and the size of the correction stays visible instead of being
silently retired.

**A guard that would have caught it.** The script now also computes the verdict
under *both* sources and shouts if they differ:

```
!! THE TWO SOURCES DISAGREE ON THE VERDICT: events=FAIL-thin, csv=OK
!! The gate would be decided by sampling density, not by the world.
```

Per the "checks that stopped checking" lesson, a new guard is worthless until it
has been seen to fire, so it was calibrated against a constructed straddle case
(criterion 0.55, where the two floors genuinely fall on opposite sides of the
0.03 threshold) — the output above is that test, not a hypothetical. On the real
gate it is a no-op, which is what it should be, and now that is a *demonstrated*
no-op rather than an assumed one.

Re-running the corrected script on `pb3g2` reproduces **0.5301** and **0.5011**
exactly, matching `floor_source.py`'s independent measurement. The band table
also shows why `pb3g2`'s 0.60 was genuinely safe and not luckily safe: marginal
cost runs 3, 4, 5, 7, 8, 6, 8, 9 metres per 0.01 straight through the criterion
band with no explosion. The thinning `n` below 0.59 is runs *terminating* at
0.60, not runs hitting a floor — a distinction the gate has to keep straight,
since both look like "few samples down here".

### 29.10 Pre-registration: what happens to a cell that hits the duration cap

**Written 2026-08-23, while `pb4d` had zero cells on disk.** This rule exists to
be fixed before the data can influence it.

`flatforest_dense2` explores ~2.5× slower against the same 5400 s cap, so some
`pb4d` cells will end on the cap rather than on coverage. `cells.load_cells`
handled that case already, and handled it wrongly: `require_done=True` **drops**
every cell that did not reach `all_done`.

That is a filter on the outcome. A cell hits the cap *because it was slow*, and
slow is the thing being measured, so dropping it conditions on a post-treatment
variable — the identical error §24 refused when it declined to exclude
treatment-caused harm. The consequence here is worse than losing n, because the
loss is not symmetric: if one arm is genuinely slower it caps more often, loses
more cells, and the median of what *survives* is pulled down toward the fast
arm. **The filter manufactures nulls out of real differences.**

First question was whether this had already happened. `dropped.py` says no —
`pb3g2` is **120/120 `all_done`, zero cells dropped in any of the four arms**, so
§28's null is unaffected. The problem is entirely ahead of us.

**The rule.** `pb4d` is analysed with `cap_s=5400`: a capped cell is **kept**,
with `completion_s = max(observed, 5400)` and `censored=True`. That is
min(*T*, cap) — ordinary right-censoring — and it is the correct treatment for
this endpoint for three specific reasons, not as a general preference:

1. **The median survives it exactly.** The median depends only on ordering, and
   every censored cell is known to lie above every uncensored one. As long as
   fewer than half an arm's cells are censored, the arm's median is a genuine
   observed time, not an imputed one. The pre-registered effect size is a ratio
   of medians, so the endpoint is undamaged.
2. **The exact permutation test stays valid.** min(*T*, cap) with one common cap
   applied identically to both arms is a deterministic function of the
   underlying times, so under H₀ the arm labels remain exchangeable and the
   permutation null is still exact. No adjustment, no survival model.
3. **It errs toward the null.** Imputing the cap understates a slow arm's true
   time, shrinking a real difference toward 1.0. Unlike the previous two
   instrument bugs, this one fails in the safe direction *and* that has been
   checked rather than assumed.

**The abort condition, also fixed now.** Past **50 % censoring in either arm**
the median *is* the cap and the ratio is an artifact of the cap, not a
measurement. `cells.censoring_report()` refuses at that point, and the campaign
is reported as **failed on the duration cap**. Raising the cap and re-running is
then a new campaign, not a re-analysis — this is exactly the lever that would
let a disappointing result be re-run until it cooperated.

**Distance is not covered by this.** A censored cell's distance is not
min(*D*, anything) — the run stopped mid-flight, so its distance is simply
incomplete. Cap-imputation launders time and cannot launder distance. Censored
rows carry the flag; the secondary distance readout must exclude them and say
how many it excluded.

`cap_s` defaults to `None`, so every existing call behaves exactly as before.
Verified: after the change `permtest.py` on `pb3g2` still returns completion
**1.026×, p = 0.3953** and distance **1.094×, p = 0.0130** — the whole output
diffs clean against the pre-change run.

**None of the above had ever executed.** `pb3g2` has nothing to censor, so every
line of this machinery was dead code with an argument attached — the exact
shape of the six inert guards found earlier. `test_censor.py` calibrates it on
synthetic rows with answers known in advance, and TEST 4 is the one worth
quoting, because it stops the bias being a claim and makes it a number.

Thirty `off` cells drawn uniformly from 1500–4200 s; thirty `hybrid` cells at a
**true ratio of exactly 1.500×**; cap at 5400 s. The censoring that results is
asymmetric on its own, with no help — **3 hybrid cells capped, 0 off cells**,
because only the slower arm reaches the cap at all:

| | ratio of medians | error against the truth |
|---|---|---|
| true, uncapped | 1.500 | — |
| cap-imputed (the rule) | **1.500** | +0.000 |
| dropped (the old behaviour) | **1.403** | −0.097 |

Three cells out of sixty, and dropping them erases a fifth of a 50 % effect.
Scale that to a world where capping is common and the endpoint stops reporting
what happened. Cap-imputation recovers the ratio *exactly* here, and TEST 5
shows why that is not luck: at 10 % censoring the hybrid median is the same
number to full precision whether computed on the true times or the capped ones,
because the cells that moved were all above it.

TEST 4c also pins the direction — cap-imputation is constrained to land in
[1.0, truth], so it can understate a real difference and cannot invent one.
TEST 3 checks the refusal is **per-arm**: 5 of 10 censored in one arm is 50 %
and must refuse, while the *pooled* fraction is 25 % and would sail through.
A pooled check would have been the natural thing to write.

### 29.11 Archiving the void data disarmed the check that the void data calibrated

Running the full test suite after the `cells.py` change turned up two failures
that had nothing to do with it:

```
FAILED:
  TEST 1  pb2 + pb3g2 (same config, different binary): DID NOT RAISE on 120 mixed cells
  TEST 2  tx1 + pb3g2 (different tx power): DID NOT RAISE on 120 mixed cells
```

`test_sig.py`'s positive controls prove `check_signatures()` can fail, by mixing
`pb3g2` with a generation that genuinely differs. **§29.7 moved those
generations out of the campaign root.** The controls then had nothing to mix, so
the check found one uniform signature and correctly did not raise — and the test
recorded that as the check being inert.

The archive was right and the tests were right. The coupling between them is
what nobody wrote down: *the tests took the void data as their fixture, and the
cleanup treated it as garbage.*

**The part worth keeping is why the guard against this missed.** `test_sig.py`
already had an anti-vacuity check — it refused to pass on an empty row list. It
did not fire, because the list was not empty: `pb3g2`'s 120 cells were still
there. The guard asked *"are there any cells?"* when the question it needed was
*"are there cells from **each** prefix this control names?"* Those coincide for
a one-prefix test and come apart for exactly the two-prefix positive controls
that matter. **A vacuity check has to be vacuity-checked against the specific
way its own test can go hollow** — the generic version passed while the test it
guarded tested nothing, which is the failure mode of §29.2 and of the six inert
guards in the earlier audit, arriving by a third route.

Both are fixed: `rows_for()` now searches `~/hmr_campaign_archive` alongside the
live root and carries an explicit per-row `dir`, and the vacuity check now
requires at least one cell from every prefix named. `check_signatures()` honours
a `dir` key if present, which `load_cells` never sets, so the production path is
untouched. TESTS 1 and 2 raise again, now over **142** and **122** cells — the
counts are the evidence that the archive is really in scope, since 120 would
mean `pb3g2` alone and the controls would be hollow once more.

There is a standing consequence for the archive: **it is not cold storage.** It
holds the only known-answer fixtures for the provenance check, and deleting it
to reclaim disk would silently return `check_signatures()` to being untestable.

### 29.12 A gate whose statistic had no scale, replaced twice before it worked

§27.5's floor gate is a **minimum over cells** compared against a fixed 0.03
margin. pb3g2's minimum is taken over 30 cells; fd2s's will be taken over 3,
because the smoke runs three seeds. A minimum over fewer draws can only read
higher, so the gate was about to compare a 3-cell estimate against a threshold
reasoned about from a 30-cell one.

`floor_n.py` enumerates all **C(30,3) = 4060** 3-subsets of pb3g2's off arm
exactly — no bootstrap, for the same reason the endpoint uses an exact
permutation test — and prices that mismatch:

| cells | subsets | median min | median bias | P(fails the gate) |
|---|---|---|---|---|
| 1 | 30 | 0.5697 | +0.0687 | 50.0 % |
| 2 | 435 | 0.5624 | +0.0614 | 24.1 % |
| **3** | **4,060** | **0.5521** | **+0.0510** | **11.2 %** |
| 4 | 27,405 | 0.5342 | +0.0332 | 5.0 % |
| 30 | 1 | 0.5011 | +0.0000 | 0.0 % |

A 3-cell floor reads **+0.0510** high against a **0.03** threshold, so the bias
is larger than the quantity being tested, and the gate would falsely fail
**11.2 %** of the time in a world that clears the criterion by +0.0989. The
remedy for a failed gate is to raise the criterion, which weakens the endpoint.
That is the **third instrument this week whose error pointed the same way**:
§29.8 was a duration reading the floor high, §29.9 a sampling grid reading it
high, and this is a cell count doing the same thing. Three independent
mechanisms, one direction, and none of them announced itself.

**The replacement.** `gate2.py` drops the minimum for three conditions that
three cells can actually carry, all directly observable because the smoke is
built to run *past* the criterion (`DONE_UNKNOWN=0.30` is unreachable, so each
cell drives through 0.60 and keeps going):

- **C1** — every cell crosses the criterion. Strictly stronger than "min floor
  below criterion": crossing implies the floor is below it *and* that a run got
  there.
- **C2** — the criterion is not on the asymptote, measured as
  `tail_frac = (t@CRIT − t@CRIT+0.05) / t@CRIT`, the share of the run spent on
  the final 0.05 of unknown. **Median** over cells must be ≤ **0.50**.
- **C3** — every cell crosses at or before 60 % of the duration cap. This is
  §29.10's censoring condition pulled forward: if the calibration cells only
  just make it, pb4d's slower arm will cap, and a capped campaign is unreadable
  whatever the floor was.

**C2 took three attempts, and the first two were the wrong quantity rather than
the wrong number.** Both were caught by running pb3g2 as a known-answer case,
which is the only reason either was found:

- **v1** compared the cost band at the criterion against the median of all
  bands above it. It failed pb3g2 **2/30**. Marginal cost rises monotonically
  through *any* healthy run — early metres sweep open space, late metres work
  the gaps between trunks — so the ratio is ≈2× for every cell everywhere. It
  measured "is this late in the run", which was never in question.
- **v2** compared it against the band immediately above, a *local*
  acceleration. On pb3g2 that spans **0.15× to 35.75×** (median 0.96×, p90
  2.74×). A world already agreed to be healthy routinely produces 35×, because
  the denominator is a distance difference that approaches zero whenever a cell
  descends quickly through a band. No threshold separates healthy from
  asymptotic when the statistic has no stable scale.
- **v3**, `tail_frac`, has a denominator that is the whole run and therefore
  never small, is bounded in [0, 1), and states the concern in the endpoint's
  own unit. On pb3g2 it spans **[0.042, 0.452]**, median **0.191**.

**Why C2 is a median while C1 and C3 are per-cell.** C1 and C3 test the thing
itself, not an estimate of it — a cell that fails to cross *is* a censored cell
in pb4d — so "every cell" is the right quantifier and getting stricter with more
cells is correct. C2 estimates a curve shape from noisy per-cell draws, so an
all-must-pass quantifier turns it into a *maximum*, which is the same
small-sample trap as the minimum it replaces. `tailfrac.py` prices both in the
known-good world at a 0.30 threshold:

| rule | false-failure rate on pb3g2 |
|---|---|
| median-of-3 exceeds | **3.94 %** |
| any-of-3 exceeds | **34.90 %** |

Across all 4060 3-subsets the median-of-3 never exceeds **0.333**, so the 0.50
threshold clears the entire known-good world by 0.167. It is also a
*definition* rather than a fitted value: over half the run going on the final
0.05 is what sitting on an asymptote means.

**Calibrated in both directions.** The earlier C2s were only ever tested against
"must pass"; neither was shown capable of failing. `test_gate2.py` runs both
halves, and all six cases behave as predicted:

| case | expected | result |
|---|---|---|
| pb3g2, 30 cells | PASS | C1 30/30, C2 0.191, C3 30/30 |
| pb3g2, every disjoint 3-subset | PASS | 10/10 |
| asymptote (floor 0.005 *below* criterion) | fail **C2** | median tail **0.562**, C1 passed 3/3 |
| floor above criterion | fail **C1** | 3/3 never crossed |
| crosses in open water at 82 % of cap | fail **C3 only** | tail 0.188, C1 clean |
| no cell reaches the band | fail **C2** | not passed by vacuity |

The asymptote case is the one that matters: it **satisfies C1** — it does cross
— and is caught by C2 alone, so C2 detects something no crossing test can. The
late-crossing case must fail C3 *and nothing else*, because the two diagnoses
have opposite remedies: "raise the criterion" weakens the endpoint, while
"raise the cap" costs wall time and no validity. A gate that cannot tell them
apart would spend the endpoint to buy compute.

One fixture bug is worth recording, since a bad fixture and a bad gate look
identical in the output: the late-crossing world was first built with a decay
constant that crossed at 37 % of the cap, so C3 had nothing to catch and the
test failed. The gate was right and the test was wrong.

**Timing, so the rule cannot have been fitted to the answer.** All of the above
was fixed at 04:59 on 2026-08-23, when the smoke's only started cell had its
leading robot at unknown **0.6580** — above the 0.65 edge where C2's tail band
begins. Neither endpoint of the tail band existed for any fd2s cell, so no fd2s
`tail_frac` was available to tune against. The margin was **0.008**, which is
thin enough to state as a number rather than to wave at.

The minimum floor is still printed as supporting information, with its
k-dependence stated alongside it. It is no longer the decision.

### 29.13 A premise check that used the wrong statistic, and nearly killed a good world

Before committing to `pb4d` there were two questions worth asking, and only one
of them had ever been written down. §27.5's gate asks **is the criterion
measurable in this world**. Nothing asked **is this world worth measuring** —
whether `flatforest_dense2` differs from the world that already returned a null
in the way the experiment needs.

**What pb4d costs.** `run_campaign.sh` is strictly sequential — one cell, then
`sleep 10`, then the next (`run_campaign.sh:104-174`). No job-count knob, no
`&`. So 60 cells run back to back and the campaign's wall time is the sum.
`price.py` measures wall/sim from the live smoke at **1.74×** and projects
`t_cross(0.60)` by exponential fit. That fit is not trusted on its own:
`price_calib.py` truncates all 30 pb3g2 cells at the unknown level fd2s is
currently at, predicts from the surviving prefix only, and compares against the
crossing each cell actually achieved — median `pred/actual` **0.95×** (range
0.61–1.42, 60 % under-predicting), so the correction is a modest **1.06×**.

| scenario | per cell | 60 cells |
|---|---|---|
| point estimate | 0.72 h | 43.0 h (1.8 d) |
| 2× the estimate | 1.41 h | 84.4 h (3.5 d) |
| every cell hits the 5400 s cap | 2.64 h | 158.7 h (6.6 d) |

The bottom row is a hard ceiling — no cell can exceed the duration cap — so the
campaign cannot cost more than 6.6 days however wrong the fit is. Plan against
the middle row.

**The premise check, first attempt, which was wrong.** `world_compare.py`
compared the **disconnected fraction** over a matched sim-time window (matching
is not optional: pb3g2 cells stop at the criterion around 558 s while the smoke
runs to the cap, and robots separate as exploration proceeds, so an unmatched
comparison measures run length):

| | pb3g2 | fd2s | ratio |
|---|---|---|---|
| mean `trees_on_link` | 3.78 | 4.45 | 1.18× |
| disconnected fraction | **51.4 %** | 54.1 % | **1.05×** |

That printed a warning that `flatforest_dense2` buys nothing and pb4d should be
reconsidered. **The warning was wrong, and the statistic was the reason.**
Disconnected fraction counts seconds without asking how they are arranged, and
the arrangement is the entire mechanism:

- a link down 4 s in every 8 s is 50 % disconnected and costs almost nothing —
  the partner's map arrives in the next window, whatever policy is running;
- a link down once for 300 s of a 560 s run is 54 % disconnected and genuinely
  partitions the team.

Identical in that column, opposite for this experiment.

**Decomposing the seconds by the length of the outage they belong to.** A
second error surfaced on the way: an intermediate pass reported pb3g2's *median
outage* as 8.9 s and concluded the world merely flickers. But 802 s of run at
51.4 % disconnected is ~412 s spread over ~16 outages — a mean of ~26 s against
that 8.9 s median. The **count** is dominated by short drops; the **seconds**
are dominated by long ones, and only the seconds matter here.

| outage length | pb3g2 % of disconnected time | fd2s % |
|---|---|---|
| 0–5 s | 3.1 % | 2.2 % |
| 5–20 s | 13.4 % | 7.5 % |
| 20–60 s | 22.1 % | 0.0 % |
| 60–180 s | 31.0 % | 0.0 % |
| **180 s+** | **30.4 %** | **90.3 %** |

pb3g2 does not flicker: **61.4 %** of its disconnected time sits in outages
≥60 s. And on the statistic that can see the mechanism the two worlds differ by
roughly **2.6×**, not 1.05× — fd2s's cell spends 964 s of a 1361 s run inside
outages ≥180 s. A premise check that had been believed would have discarded a
promising world on a number structurally incapable of seeing the difference.

**Pre-registered launch licence** (`partition.py`, fixed 2026-08-23 ~05:15 with
**one** fd2s cell on disk and two still to run):

> `partition_share` = seconds inside outages ≥60 s, as a share of the **run**
> — a share of the run rather than of disconnected time, because what competes
> with reconnect behaviour is the exploring a robot does while cut off.
>
> pb4d launches only if the **median** `partition_share` over the fd2s cells is
> at least **1.5×** pb3g2's 30-cell median of **30.0 %**, i.e. **≥ 45.0 %**.

A median over cells, not all-must-pass, for the reason §29.12 settled. **Why
1.5×**: pb3g2 is a null at 30 cells per arm, so a world that partitions only
slightly harder would be a null too, and 3.5 days is too much to spend
confirming that at a resolution the design cannot resolve. The threshold is
stated before the data, which is the only property that makes it a threshold
rather than a rationalisation. Cell 1 sits at **68.9 %** — the rule is
deliberately set well below what has already been seen, so it binds on cells 2
and 3 rather than on cell 1.

**One cell is not a verdict.** fd2s's cell sits at pb3g2's 93rd percentile on
this statistic, and at the 80th on longest-outage-share — 6 of 30 pb3g2 cells
do as well or better on the latter. A single draw from a distribution that
contains it proves nothing. The licence is re-evaluated at 3 cells when the
smoke finishes.

**If the licence fails**, the finding is that `flatforest_dense2` partitions
like `flatforest_dense` — pb3g2's world, per its manifest, already a *dense*
forest rather than the plain `flatforest`, which is what makes "denser still"
a modest step rather than an obvious one — and the lever is elsewhere: world
**size** rather than tree density, since spawn separation is off the table by
standing instruction. That is much cheaper to learn now than after 3.5 days.

**Both gates must pass before pb4d launches**, and they ask different things:
`gate2.py` whether the criterion is measurable here, `partition.py` whether
this world is different enough from the null world to be worth the compute.

### 29.14 A knob with no exogenous window: why the dose analysis cannot run

§29.13's licence rests on a premise asserted throughout this document and tested
nowhere — that a world which partitions the team harder gives reconnect
behaviour more to bite on. pb3g2 has 120 cells with full link traces, so the
premise was checkable for free while the fd2s smoke ran. It did not survive the
check, though not in the way expected, and the reason it did not is more useful
than the answer would have been.

**The question, narrowly.** Not a treatment effect. **Within a single arm**, do
the cells that got partitioned harder finish later? Arms are analysed separately
and never pooled: pooling would make `partition_share` post-treatment and turn
this into exactly the adjustment §"treatment-caused harm stays in" forbids.

**First reading, window `[0, 425 s]`** (the earliest crossing anywhere, so no
cell has finished inside it):

| arm | `partition_share` ρ | p | `disconnected_frac` ρ | p |
|---|---|---|---|---|
| off | **−0.50** | 0.0055 | **−0.64** | 0.0002 |
| hybrid | −0.08 | 0.6637 | −0.43 | 0.0187 |
| pursuit | −0.22 | 0.2335 | −0.49 | 0.0072 |
| rendezvous | −0.27 | 0.1500 | −0.52 | 0.0037 |

Every sign is **negative** — more early disconnection, *faster* completion — and
`disconnected_frac` is significant in 4/4 arms. Taken at face value that
contradicts the premise outright.

**It should not be taken at face value.** `off` never enters a reconnect state
in any of its 30 cells (verified from the events, not assumed), so it has no
treatment to leak at any window. Sweeping the window **inside `off` alone**:

| window | `partition_share` ρ | `disconnected_frac` ρ | p |
|---|---|---|---|
| 143 s | *identically zero* | −0.33 | 0.0764 |
| 175 s | *identically zero* | **−0.55** | 0.0018 |
| 250 s | −0.15 | −0.62 | 0.0004 |
| 350 s | −0.59 | −0.66 | 0.0001 |
| 425 s | −0.50 | −0.64 | 0.0002 |

143 s is the **earliest entry into any reconnect state anywhere in pb3g2** — the
last moment at which all four arms are still running the identical base planner.
The correlation is absent at that boundary and significant 32 s past it, inside
an arm that is never treated. So it is not treatment contamination. It is the
covariate needing the robots to **separate** before it carries any signal, and
separation drives both columns: robots far apart disconnect more *and* cover
more distinct ground. The negative sign is a confound, not a mechanism by which
losing the radio helps.

**Both uses of the covariate die on the same fact.**

*As a premise test* — in the only window where the covariate is exogenous it is
silent. `partition_share` is **identically zero in every cell of every arm** at
143 s: no outage reaches 60 s that early, so pb3g2's long partitions are a
late-run phenomenon by construction. There is no window that is both exogenous
and informative, so this design cannot test the premise. That is a property of
the experiment, not a verdict on the hypothesis.

*As variance reduction* — this was the more valuable prize and is worth
recording as closed rather than left to be re-proposed. A genuine baseline
covariate is legitimate to adjust on precisely because it is fixed before the
arms diverge, and at ρ ≈ −0.5 it would cut outcome variance ~25 %, the cheapest
power available on a campaign costing 1.8–3.5 days. At the boundary the estimate
is **ρ = −0.33, p = 0.076** — worth ~11 % at best, and below this test's
resolution anyway. **Not built.**

**Stated honestly, because the tempting overstatement is right there:** −0.33 is
not nothing, and at n = 30 this test has 80 % power only for |ρ| ≥ 0.52
(calibrated by simulation, not assumed). So the correct claim is *"no baseline
predictor was established"*, never *"there is no baseline predictor"*. The
practical decision is the same either way — an unestablished covariate worth
~11 % is not worth the risk of adjusting on something partly post-treatment —
but the two statements are not interchangeable and only one of them is true.

**Effect on the launch decision: none.** The negative correlation is explained
and is neither a reason to hold pb4d nor support for it. The licence in §29.13
stands on its own footing. What changes is that the premise behind it is now
**explicitly an assumption rather than a finding**, and pb4d is the thing that
tests it.

**The statistics were calibrated before being believed** (`test_dose.py`, 8/8):
ρ matched against `scipy` to 1.1e-16 across 200 tied datasets; the permutation
null rejects at 4.8 % against a nominal 5 %; the claimed 80 %-power ρ delivers
80 % and a smaller ρ is genuinely missed at 28 %; window clipping verified to
count only the seconds inside the window. The rank correlation and the
permutation p were both hand-written, and an LCG's low bits have already
produced a biased permutation p once in this project.

### 29.15 The analysis code was pre-registered too, and two of its defaults were wrong

Everything in §25–§29 pre-registers what will be *computed*. None of it
pre-registers the *code that computes it*, and while pb4d was still gated on a
smoke — i.e. while no pb4d cell existed and no choice here could be shaped by
an outcome — the read-out path was run against the world it is about to face.
Two defaults did not survive.

**Both were silent.** Neither would have raised, printed a warning, or produced
a number that looked wrong.

**1. `final_table.py` would have analysed the wrong campaign.** It called
`load_cells(arms=cells.ARMS, require_done=True)` with no `group`. `group`
defaults to `DEFAULT_GROUP`, which is `pb3g2`. Run against a finished pb4d it
would have printed a complete, correctly formatted, entirely plausible table —
of pb3g2. Nothing in the output named the group, so there was no line a reader
could have checked. This is the same class of error as §29.11: the `GROUPS`
partition added in §29.9 did its job perfectly at the loader and was then
bypassed by a caller that never passed the argument.

**2. It would have dropped exactly the cells §29.10 exists to keep.**
`require_done=True` discards every cell that did not reach `all_done`. §29.10
pre-registers the opposite — keep them at `min(T, cap)` with `censored=True` —
because dropping conditions on a post-treatment variable and does so
asymmetrically: the slower arm loses more cells, its surviving median is pulled
down, and the ratio is dragged toward 1.0. **The filter manufactures nulls.**
On pb3g2 this cost nothing, and that is precisely why it survived: pb3g2 was
120/120 `all_done`, so the flag never bound.

Both now come from the command line, spelled exactly as `permtest.py` spells
them (`--group`, `--cap`), so the descriptive table and the inferential test
cannot drift apart; and the group and censoring policy are printed in a banner,
so analysing the wrong campaign became something the output says out loud.
Back-compat was verified against the published §28 numbers — no arguments still
reproduces 1.026× completion and 1.094× distance, exactly.

**A branch that has never executed is not tested by the campaign that passed.**

The censoring path had *never run on real data*. `censored` was `False` for
every cell ever analysed, so every line of `cap_s` handling was dead code that
sat in the loader looking implemented. pb4d explores ~2.5× slower against the
same 5400 s cap, so pb4d is the first campaign to enter that branch — directly
underneath the headline number. "pb3g2 passed" is evidence only that the branch
was never entered.

`test_censoring_path.py` (14/14) therefore builds a fixture campaign with
censoring known by construction — synthesised rather than symlinked from real
cells, because the property under test *is* the `end_reason`/cap interaction and
no real cell can express it. What it pins:

| | check | why it would be silent |
|---|---|---|
| **A** | dropping censored cells turns a real **1.100×** into a perfect **1.000×** | a manufactured null reads as "no effect found" — publishable-looking and wrong |
| **B** | censored times impute to the **cap**, not their own `t_sim` | a run's last step lands ~40 s under the cap, making a cell that *failed* to beat it look marginally faster; same direction every time |
| **C** | distance **excludes** censored rows, completion keeps them | the fixture's medians are identical either way — only the **count** exposes it |
| **D** | the ≥50 % abort fires at 60 %, not at 40 %, **and at exactly 50 %** | a boundary silently reading as strict `>` passes both ordinary cases and fails only the one it was written for |

Row **C** is the one worth keeping. Including truncated distances did not move
the fixture's median at all — the check that would have caught it in a table is
the pool size, not the statistic. A test asserting on the median alone would
have passed while the defect sat in the output.

**What this does not claim.** These are two defects in read-out code, found and
fixed before the data existed. They say nothing about whether pb4d's world is
the right one — that is what the gates in §29.12 and §29.13 decide, and they are
still the thing standing between here and a 2.3-day campaign.

### 29.16 Censoring costs almost no power. The abort rule is what binds.

§29.10 keeps capped cells rather than dropping them, which is correct but not
free: imputing the cap **understates** a slow arm's true time, so it shrinks a
real difference toward 1.0. pb4d runs ~2.3× slower against the *same* 5400 s
cap pb3g2 used, and n=30 was already the floor — 58–84 % power on pb3g2, never
a clean 80 %. If censoring pushed that down another twenty points, pb4d would
most likely spend 2.3 days returning a second ambiguous null, and the right
response would be a **longer cap**, not more cells. That has to be settled
before launching, so `censor_power.py` settles it.

The simulation resamples pb3g2's own 30 `off` times with replacement (not a
fitted lognormal — the tail is what censors, and a parametric fit smooths away
exactly the thing under test), scales the whole distribution by a slowdown `s`,
applies a multiplicative effect `r`, caps at 5400 s, and runs the same
mean-log-difference permutation statistic `permtest.py` uses.

**The null was calibrated first, because `power_n.py` shipped without one and
ran ~30 % low.** A power simulation that cannot reproduce α under H₀ is not
measuring power, and it fails in the flattering direction. Two calibrations
gate the output: the vectorised permutation p matches the *enumerated* exact p
to 0.0018, and the null rejects at 3.8 % against a nominal 5 % **with censoring
active** — a cap applied identically to both arms must not disturb
exchangeability, and this checks that rather than asserting it.

| slowdown | proj. median | % censored | r=1.00 | r=1.15 | r=1.25 | r=1.40 |
|---|---|---|---|---|---|---|
| 1.00× | 804 s | 0.0 % | 3 % | 46 % | 90 % | 100 % |
| 2.30× | 1849 s | 0.0 % | 5 % | 47 % | 88 % | 100 % |
| 3.25× | 2613 s | 3.4 % | 4 % | 50 % | 84 % | 100 % |
| 3.75× | 3015 s | 9.1 % | 6 % | 51 % | 88 % | 100 % |
| 4.50× | 3618 s | 18.7 % | 5 % | 53 % | 89 % | **100 % [abort 20 %]** |

**Power is flat.** It does not degrade with censoring even at 18.7 % censored —
because `min(T, cap)` compresses the effect and the spread *together*, so the
test survives truncation far better than intuition suggests. Power here is set
by the effect size and n, not by the cap.

**What does not survive is the pre-registered rule.** Once an arm passes 50 %
censored its median *is* the cap, §29.10 declares the campaign failed, and the
significance of the statistic is beside the point because the result is not
reported. That is the `[abort 20 %]` cell: at 4.5× slowdown with a large true
effect, one campaign in five destroys itself by its own rule while the test is
still rejecting. **The binding constraint on the 5400 s cap is the abort
threshold, not statistical power** — which is the opposite of the intuition
that motivated the check.

**At the measured slowdown pb4d is clear.** fd2s cell 1 crossed at 1815 s
against pb3g2's off median of 804 s, i.e. 2.26×, where censoring is 0.0 % and
the abort never fires. The cap is not a threat at the density actually
measured; the safe region extends to ~3.75× and only the 4.5× corner is
dangerous. So pb4d's real limit is the one it always had: it can detect a 25 %
effect reliably (~88 %) and a 15 % effect barely (~47 %).

**Assumption, stated rather than buried.** The multiplicative-shift model says
the treatment *scales* completion times rather than adding a constant. If
reconnection instead costs a roughly fixed detour, the tail censors less than
modelled and these numbers are pessimistic. The slowdown is also swept rather
than trusted, because 2.26× rests on a single fd2s cell — the table exists so
the finished smoke can be read against a pre-computed threshold instead of one
derived after the answer is visible.

### 29.17 In dense2 the criterion is crossed by a merge, not by driving

The smoke was launched to answer gate2's question — is 0.60 reachable in this
world — and it is answering a second one that was not asked. Cell 1's descent
to the criterion is not a descent. It is a staircase.

```
  criterion   t_cross (atlas)   note
    0.70            628s
    0.65           1312s
    0.62           1838s
    0.60           1838s        SAME STEP as 0.62 — one jump spans both
    0.58           1853s
    0.55           1853s        SAME STEP as 0.58
    0.52             --         NOT REACHED (floor 0.5430 at 2703s)
```

Between 1351 s and 1802 s the map moved −0.0051 while the robot drove 82 m. In
the next window it moved −0.0868. In the window after that, −0.0002 across
101 m. Long flats, then a cliff.

**What produces the cliffs.** `coverage_source` cannot say — it is constant
`scovox` for all 605 steps, naming the pipeline rather than the provenance of a
voxel, so add it to the list of columns that look diagnostic and are not.
`total_observed_voxels` says it plainly:

| step | Δ observed voxels | Δ distance |
| --- | --- | --- |
| typical | **+7 to +17** | ~2 m |
| atlas 1833.76 → 1837.50 s | **+140,348** | **0 m** |
| bestla 1805.02 → 1810.01 s | **+113,542** | 2 m |
| bestla 1810.01 → 1815.04 s | **+153,185** | 2 m |

Four orders of magnitude above a normal step, with the robot stationary. A
LiDAR sweep that has been adding ten voxels a step does not add a hundred and
forty thousand because the robot stood still. And the two robots' totals
converge across the event — atlas 1.28 M → 1.62 M, bestla 1.26 M → 1.63 M —
which is what a mutual full-map exchange looks like and what nothing else does.

Aggregated over the cell, the jump steps deliver map at **0.3 m per 0.01 of
unknown_fraction on atlas and 0.9 m on bestla, against 20.2 m and 29.8 m in
every other step**. Twenty to sixty times cheaper per point. Twenty per cent
(atlas) and thirty-three per cent (bestla) of the entire descent arrives in
about one per cent of the steps.

**The link association is the weakest evidence here, and it is reported as
such.** All nine jumps had the radio up within 60 s. That looks damning until
the base rate is computed: 51–52 % of *all* steps had the radio up within 60 s,
because the link is up 25.4 % of the time and a 60 s window is generous. Worse,
the jumps are not nine independent events — they cluster into roughly three
time windows (~230–335 s, ~1045 s, ~1810–1853 s) and the two robots share one
radio, so the honest count is about three. At a 51 % base rate that is
p ≈ 0.13. **The link column does not carry this finding.** The voxel counts do.

**Why this matters more than the gate it interrupted.** pb4d's premise has been
that a denser world gives the reconnection treatment more to work with. This is
the first direct evidence for a *mechanism*: in dense2, progress toward the
criterion is merge-gated, and the step that crosses 0.60 is itself a merge. A
method that engineers reconnections is then acting on the rate-limiting step
rather than on a second-order cost. pb3g2 returned 1.026× and a p of 0.3953;
this is the first concrete account of what was different there that could make
dense2 different.

**And it undermines the power analysis written two hours ago.** §29.16 priced
pb4d by resampling pb3g2's thirty `off` completion times and multiplying by a
slowdown — a model that says dense2 is pb3g2 played slowly: same shape, longer
clock. Under it, censoring at the measured 2.26× is 0.0 % and the §29.10 abort
never fires. But if crossing the criterion waits on a chance encounter, t_cross
inherits the encounter's timing, and encounter-gated waits have a long right
tail that a scaled pb3g2 does not have. A fatter right tail against a fixed cap
means more censoring than predicted, and §29.10 aborts an arm at 50 % censored.
That is 2.3 days spent to be told the endpoint was unreadable.

§29.16's conclusion is not withdrawn — its sweep runs to 4.5× precisely because
the slowdown was uncertain — but its *shape* assumption now has a named reason
to be wrong, which it did not have when it was written.

**Pre-registered response, armed while two of the three cells do not yet
exist.** `shape_tripwire.py`, threshold fixed from pb3g2 alone:

```
  statistic   max(t_cross)/min(t_cross) over the 3 fd2s cells.
              Scale-free, so the unknown slowdown cancels exactly — which is
              the point, since 2.26× rests on one cell.
  reference   the same statistic for 3 draws from pb3g2's 30 off cells
              (200 000 resamples):  p50 1.53×  p75 1.88×  p90 2.36×  p95 2.56×
  THRESHOLD   2.36×  — trips if fd2s exceeds it; re-price before launching.
```

The threshold is deliberately loose. At n=3 a tight one fires on noise, and a
tripwire that cries wolf gets ignored, which is worse than not having one.

**What a non-trip will and will not mean.** Three cells cannot estimate a tail.
Nothing with n=3 can. So this instrument can only detect a spread so wide that
a scaled pb3g2 would rarely produce it; it cannot certify that the shape is
fine. Recording that now, before the result, because "the tripwire did not
fire" reads as clearance in a way the code cannot prevent and this paragraph
can. If it does not trip, the censoring risk stays on the books as a known
unknown and pb4d's own censoring rate gets watched as it accrues — the campaign
is resumable, so noticing at cell 20 costs one cell, not the campaign.

One further consequence for pricing, independent of the tail. Because the
criterion sits on a cliff, `t_cross` is a step function of the criterion:
atlas crossed 0.65 at 1312 s and 0.60 at 1838 s, a 40 % swing in cost for a
0.05 change in a threshold. 0.60 is safely *inside* a cliff rather than on its
edge, which is the good case. But it also means cell-to-cell variation in
t_cross will be driven by *which* cliff crosses the line, not by smooth
variation in exploration speed — so the three smoke cells are not three
measurements of one number, and the spread between them is the thing to read.

### 29.18 What the endpoint actually measures, and a free measurement of map overlap

§29.17 established that in dense2 the criterion is crossed by a merge. That
forces a question about the endpoint that has never been asked in writing:
**whose map is the criterion evaluated on?**

`run_explo_sim_rviz.sh:1294` answers it. `STOP_ON_DONE=1` "ends the run once
EVERY planner is in DONE", and a planner enters DONE on its **own**
`unknown_fraction` falling to `done_unknown_fraction`. So the endpoint is
per-robot map coverage, required of *both* robots. (This also confirms
`shape_tripwire.py` is right to take `max` over robots rather than `min`; a
per-robot crossing would price a cell shorter than the campaign runs it.)

**The threat to validity, named before the result rather than after.** If
progress to the criterion is merge-gated and the criterion is per-robot, then a
method that engineers reconnections is partly being rewarded for making the
*stopping rule* fire, not for exploring. A reader will raise this, and it is
much cheaper to answer now.

The answer is that a merge cannot manufacture coverage — it can only reveal it.
The voxels bestla hands atlas at 1805 s are ground bestla had already driven.
Crossing 0.60 still requires the *union* to have covered 40 % of the ROI, which
no amount of radio achieves on its own. What the merge changes is the lag
between ground being covered and the robot that must act on it *knowing* it was
covered. Reducing that lag is not an artifact of the stopping rule; it is the
entire operational point of a reconnection method, and it is the mechanism
behind the redundancy already on record — a robot that does not know its
partner swept a region goes and sweeps it again.

So the endpoint stands, restated precisely: **time until joint exploration
becomes common knowledge.** Both arms are measured identically against it.

**A measurement that turns out to be free.** The same identity that explains the
jumps also measures map overlap, with no code change:

    delta |A| at a merge = |B - A|
    |A union B|     = |A| + |B - A|
    |A intersect B| = |B| - |B - A|

`total_observed_voxels` already logs this, in every campaign ever run,
retroactively. "Per-voxel merge transfer logging" has been on the deferred list
as a planner change; for the aggregate it was mainly wanted for, it is
unnecessary. (It is still worth doing for per-voxel detail.)

Because both robots merge, one event gives **two independent estimates of the
same quantity, from different columns of different files.** They must agree.
Result on fd2s cell 1 at the 1805 s event:

| from | gain = other − me | \|atlas\| | \|bestla\| | union | overlap | of partner |
| --- | --- | --- | --- | --- | --- | --- |
| atlas | 317,386 | 1,284,745 | 1,264,453 | 1,602,131 | 947,067 | **74.9 %** |
| bestla | 327,645 | 1,284,745 | 1,264,453 | 1,592,098 | 957,100 | **74.5 %** |

Two independent estimates differing by 1.0 %, with the union identity closing
against each robot's own post-merge count to under 1 %.

**The cross-check earned its keep immediately — it caught two bugs that each
produced a confident wrong number.**

1. *Reading the partner's "before" count at my own merge time.* The merges are
   mutual and near-simultaneous, so by the time atlas merges, bestla's count
   already includes atlas's contribution, double-counting the shared volume. It
   read 77.7 % from atlas against 71.5 % from bestla — plausible on their own,
   but the *absolute* overlaps differed by 300,000 voxels, which is what
   exposed it. Fixed by pairing events across robots and taking one reference
   time before either merged.
2. *An absolute voxel threshold for "this is a merge".* Sensing yield decays as
   the world saturates: atlas's first sweep adds 127,963 voxels and the next
   step adds 35,242 for 2.5 m driven, while a late-run sensing step adds ~10. A
   fixed threshold therefore flags the opening sweeps as a merge and reports
   **negative overlap** — the tell that saved it. Fixed by comparing each step
   against that robot's *own trailing voxels-per-metre*, so the detector
   follows the saturation curve instead of fighting it.

Neither bug would have been caught by a single-sided calculation. This is the
same lesson as §"checks that stopped checking", in the constructive direction:
the redundant estimate was not ceremony, it was the only thing that failed.

**What the 75 % is and is not.** It is MAP overlap: volume both robots have
observed. It is **not** redundant driving. The robots start close together by
standing instruction and a LiDAR sees far, so a large shared volume is sensed
from the first sweep without either robot re-covering the other's ground. Treat
it as an upper bound on wasted travel, consistent with — not a replacement for
— the existing finding that redundancy is sequential rather than comms-driven.

The single-sided events at 300 s and 1020 s report 87 % and 89 % but have no
partner event to check against and their union identity closes only to 2.7–4.5 %.
They are printed with that flag and should not be quoted.

---

## §29.19 The link column cannot tell these two worlds apart — a retraction before publication

§29.18 left an open question worth chasing: `overlap_campaign.py` validated only
**8 of 30** pb3g2 `off` cells (27 %). The tempting read is that the detector is
badly tuned. The alternative is that the low acceptance **is the finding** — if
pb3g2's radio is up most of the time, the maps stay synchronised incrementally
and there is no discrete merge to detect. That alternative makes a prediction
about a quantity involving no detector, no threshold and no inference at all:
the fraction of `link_states.csv` samples with `connected=1`.

It did not survive. Recording the whole path because the intermediate results
were persuasive and wrong, and two of them nearly reached this document.

### The first answer, which was a duration artifact

Computed over each cell's own logged span, `link_duty.py` gave:

| world / arm | n | median duty | median longest gap |
|---|---|---|---|
| pb3g2 / off | 30 | 43.7 % | 126 s |
| fd2s / off | 1 | 22.4 % | 596 s |

Duty halved and the longest outage nearly quintupled — exactly the predicted
shape, and a clean mechanistic story for pb3g2's 1.026× null.

It is almost entirely an artifact of **span**. pb3g2's `off` cells have a median
span of 802 s; the fd2s cell had run 3100 s. Both statistics are confounded by
duration, in opposite directions. Longest-gap rewards duration outright — a
longer run has strictly more chances to contain a long outage. Duty is
confounded the other way: the robots start together by standing instruction, so
early duty is high, and a cell observed only early looks better connected than
one observed throughout.

### The second answer, after matching the window

| window | pb3g2 duty | fd2s duty | pb3g2 median gap | fd2s gap |
|---|---|---|---|---|
| 567 s | 48.1 % | 46.3 % | 101 s | 232 s |
| 800 s | 45.4 % | 33.7 % | 126 s | 271 s |

The duty difference **collapses** — 48 % against 46 % is the same number. The
gap difference survives and widens. The claim I was one step from writing was
that dense2 is not *less*-connected but *lumpier*-connected: comparable total
connectivity delivered in longer individual outages, which is precisely what
would make merges discrete in dense2 and continuous in pb3g2.

### The third answer, which retracts the second

With one fd2s cell, the only honest form is a percentile against pb3g2's 30
untreated cells — descriptive, not a test, since n=1 admits none:

| window | pb3g2 median gap | pb3g2 p90 | fd2s gap | **percentile** | pool |
|---|---|---|---|---|---|
| 400 s | 63 s | 146 s | 65 s | **53 %** | 30/30 |
| 567 s | 101 s | 179 s | 232 s | **93 %** | 30/30 |
| 800 s | 126 s | 282 s | 271 s | **82 %** | 17/30 |
| 1200 s | 357 s | 487 s | 397 s | **50 %** | 4/30 |

53 → 93 → 82 → 50. A statistic that reads "utterly ordinary" at 400 s,
"striking" at 567 s and "ordinary" again at 1200 s is not measuring a property
of the world — **the window is choosing the answer**. Both windows with the full
30-cell pool are available and they disagree with each other.

**Survivorship makes the long windows worse than merely underpowered.** Only
pb3g2 cells that *ran* that long can be compared at 800 s and 1200 s, and those
are the slow cells. If slow cells are also the badly-connected ones, the pool is
selected toward long gaps exactly where fd2s stops looking unusual — so the two
windows that soften the result are the two that cannot be trusted to soften it
honestly. The bias points the convenient way, which is a reason to distrust it,
not to lean on it.

And the one striking window rests on one event: between 400 s and 567 s this
single cell opened a single long outage. Three smoke cells will not repair a
statistic whose value depends on where the window is cut.

### What this means

**The 27 % acceptance rate in §29.18 remains unexplained.** Tool defect and
world property are both still live; this test was supposed to separate them and
could not. The conditional framing there — "in cells with a clean merge", not
"in this arm" — stands and should not be relaxed.

**The link column has now failed to carry a claim three times**: §29.17's jumps
looked decisive at 9/9 until the base rate came out at 51–52 % over ~3
independent windows (p ≈ 0.13); the standing note that the link must be read
from `link_states.csv` rather than from `peer_lost` events; and now this. The
pattern is consistent enough to treat as a prior: *link-derived statistics in
this project are low-information and easy to over-read.* The evidence that
dense2 differs from pb3g2 is the **voxel-jump mechanism** of §29.17 — a merge
adding +140,348 voxels in one step against a typical +7 to +17, with the robot
stationary. That is not a link statistic, and it is not weakened by anything
here.

**No gate changes.** `partition.py` (gate 2) tests dense2's partitioning against
its own pre-registered ≥1.5× threshold on the finished 3-cell smoke, and is not
affected by a percentile computed on 1 cell. Nothing in `gate_and_launch.sh` is
touched.

**Method note.** The failure mode that produced the first two answers is the
same one both times: a comparison between groups that differ in *how long they
were observed*. It was caught by asking for a matched window, and then the
matched window was itself caught by asking whether the answer depended on which
window. The second check is the one that mattered, and it is the one that is
easy to skip after the first check has already "fixed" the problem.

---

## §29.20 Breaking the circularity: overlap is ~88 %, and the 27 % was the measurement, not the world

§29.19 failed to explain why `overlap_campaign.py` validated only 8 of 30 pb3g2
`off` cells. This does, by fixing a defect in the tool that §29.19 could not
have found because it was looking at the wrong column entirely.

### The defect

`overlap.py` **detects** merges by looking for voxel jumps, then **accepts**
them by checking a voxel identity. Detector and acceptance test read the same
column, so a cell can only fail in ways the detector was already blind to. A
73 % failure rate under those conditions is uninterpretable: it cannot be
attributed to the world, because the tool never gave the world a chance to
disagree with it.

`overlap_link.py` triggers on the **rising edge of `link_states.csv`** instead —
link down ≥20 s, then up ≥5 s, debounced at both ends. The link says *when* to
look; the planner CSV then has to close the identity on its own. Failure now
means something.

### Known-answer calibration first

Per §"checks that stopped checking", a new detector is validated against a case
with a known answer before it is pointed at an open question. On fd2s cell 1 the
link trigger independently landed on **t = 1805 s** — the same event the voxel
trigger found — and returned **72.2 %** against `overlap.py`'s 74.9 %/74.5 %.
Two unrelated triggers, ~3 points apart. The residual is expected and has the
right sign: the 60 s settle window includes 15–24 m of genuine driving, which
enlarges the union and lowers the ratio.

### The answer

| | pb3g2 / off | fd2s / off |
|---|---|---|
| cells validated | **13/30 (43 %)** | 1/1 |
| reconnections examined | 58 | 2 |
| **overlap, share of partner's map** | **median 88.0 %** (74.4–97.7) | 72.2 % |
| Jaccard | median ~79 % | 56.4 % |
| gain explainable by own sensing | median 26 % | 1 % |

Acceptance rose 27 % → 43 % purely from removing the circularity. More
importantly, **not one cell failed as "maps already synced"**. The §29.19
hypothesis — that pb3g2's link is up so much that transfers trickle and there is
no discrete merge — is **false**. Transfers happen at pb3g2 reconnections. The
17 failures are 11 *unidentifiable* (sensing mixed into the same window), 4
*robots disagree*, 1 *no qualifying reconnection*.

So the low acceptance is a **limit of the measurement, not a property of the
world**: pb3g2's robots keep driving through reconnection, and a voxel count
cannot separate "I sensed this" from "you sent me this" when both happen at
once. 41 % of robot-windows exceed the 35 % refusal threshold. No detector
tuning fixes that, and a tool reporting a confident number there would be
inventing one.

### The two confounds, tested rather than assumed

**Sensing-subtraction bias.** `transfer = gain − explainable`, so over-subtracting
shrinks the transfer and *inflates* overlap — and pb3g2 carries 26 % explainable
sensing against fd2s's 1 %. Tested: split at the median sens%, overlap is 88.0 %
vs 88.2 %; r(overlap, sens%) = **−0.271**; and the five least-contaminated cells
(sens% ≤ 5 %) sit **highest** at 93.8 %. Every direction is wrong for the
artifact. With n=13 this test could not detect a small bias, but a large one in
the feared direction is excluded.

**Event time.** pb3g2's validated events fall at 230–832 s; fd2s's is at 1805 s.
This is precisely the trap of §29.19, so it was checked rather than waved at:
r(overlap, event time) = **+0.583** (n=13, critical 0.553), with 80.5 % median
early against 92.6 % late. Overlap **rises** with time — so comparing fd2s's late
event against pb3g2's early ones is **conservative** for the claim that dense2
has lower overlap. Unlike §29.19's survivorship, the bias here points *against*
the finding, which is the one case where it can be leaned on.

### What this is worth

It answers the standing question *"can we check the redundant exploration by
robots when reconnect methods are off"*: **yes — in pb3g2's untreated arm, in
cells where the transfer is separable, ~88 % of the partner's map is volume the
robot has already observed.** Still conditional on 43 %, still selected on
separability, and still to be read as "in cells with a clean merge".

And it is **MAP overlap, not redundant DRIVING**. Robots start close together by
standing instruction and a LiDAR sees far, so a large shared volume is sensed
from the first sweep without either re-covering the other's ground. This is an
upper bound on wasted travel, consistent with — not a replacement for — the
finding that redundancy is sequential rather than comms-driven.

**A second, independent argument for dense2.** fd2s's 72.2 % against pb3g2's
88.0 %, at a later time when pb3g2's trend says it should be *higher*, says
denser occlusion leaves more genuinely-divergent map, so sharing has more to
deliver. That is exactly pb4d's premise, arrived at from a different column than
§29.17's voxel jumps — and unlike §29.19 it is not a link statistic, so it does
not inherit that section's caveat. **fd2s is n=1**; re-run when the smoke lands.
This changes no gate: `partition.py` tests its own pre-registered threshold.

---

## §29.21 The settle window is a benign knob — checked, because §29.19 says check

`overlap_link.py` has a tunable: how long after reconnection to wait before
reading the post-transfer state. §29.19 was a retraction caused by quoting a
statistic at the window that flattered it, so the knob gets audited before its
number stays in the document.

### The raw sweep, which looks alarming

| settle | fd2s (known answer ≈72–75 %) | pb3g2 overlap | pb3g2 acceptance | unidentifiable |
|---|---|---|---|---|
| 10 s | **no measurement at all** | 95.8 % | 7/30 (23 %) | 28 % |
| 20 s | **no measurement at all** | 90.8 % | 18/30 (60 %) | 21 % |
| 30 s | **no measurement at all** | 87.8 % | 18/30 (60 %) | 26 % |
| **60 s** | **72.2 %** ✓ | **88.0 %** | 13/30 (43 %) | 41 % |
| 90 s | 72.1 % ✓ | 89.3 % | 10/30 (33 %) | 44 % |
| 120 s | 72.1 % ✓ | 90.6 % | 10/30 (33 %) | 47 % |

Acceptance swings 23 %→60 % and the overlap median moves 87.8–95.8 %. Taken at
face value that is a knob choosing the answer, and the tempting move — pick 20 s,
report 60 % acceptance — is precisely the §29.19 error.

### Why short windows are biased, not merely different

fd2s settles the question. Its transfer is not complete until ~60 s: at 10, 20
and 30 s the calibration cell yields **nothing**. A window shorter than the
transfer measures a *partial* gain, and since intersection = partner's map −
transfer, a partial gain **inflates** overlap. That is the direction the sweep
shows, with the shortest window highest at 95.8 %. So 60 s is not a preference,
it is the shortest window that reproduces a known answer — and it was fixed as
the default before this sweep ran.

### The test that settles it: hold the cell set fixed

Comparing medians across settle values compares different cells — 13 validate at
60 s, 18 at 20 s. A moving median could be the estimate moving or the membership
changing, which have opposite implications. On the **8 cells that validate at
every setting**:

| cell | 20 s | 30 s | 60 s | 90 s | 120 s | spread |
|---|---|---|---|---|---|---|
| seed10 | 97.7 | 97.7 | 97.7 | 97.7 | 97.7 | 0.0 pp |
| seed14 | 90.6 | 85.0 | 84.7 | 84.3 | 84.3 | 6.3 pp |
| seed17 | 96.5 | 96.5 | 96.6 | 96.6 | 96.5 | 0.1 pp |
| seed19 | 88.9 | 87.9 | 88.0 | 88.0 | 88.0 | 1.0 pp |
| seed2 | 94.2 | 93.4 | 90.4 | 90.5 | 90.5 | 3.8 pp |
| seed20 | 92.7 | 92.7 | 92.6 | 92.5 | 92.5 | 0.2 pp |
| seed25 | 76.8 | 76.0 | 76.7 | 75.3 | 74.5 | 2.3 pp |
| seed8 | 93.8 | 93.8 | 93.8 | 93.8 | 93.8 | 0.0 pp |
| **median** | **93.2** | **93.1** | **91.5** | **91.5** | **91.5** | — |

Median per-cell spread **0.6 pp**, max 6.3 pp; the fixed-set median moves
**1.8 pp** across a 6× change in the window. The knob decides **how many** cells
can be measured, not **what** they measure. That is the benign form of a tunable
parameter and the opposite of §29.19, where the window drove the value itself.

### One thing this does not license

The fixed-set median (91.5–93.2 %) is **higher** than the 13-cell median at the
calibrated window (88.0 %). That is selection, not disagreement: cells
measurable at *every* setting are the easy ones, and the easy ones have higher
overlap. So **88.0 % remains the number**, and this exercise is a robustness
check on the estimator rather than a competing estimate. Quoting 93 % because it
came from the more carefully controlled comparison would be reintroducing the
selection the control was built to expose.

---

## §29.22 Two pre-launch audits: one clean, one that found a live hazard

Both were run while the smoke was still burning, on the principle that the
moment the gates pass is the worst possible time to discover a typo. Recorded
including the one that found nothing, because §"checks that stopped checking"
is about guards that were never *exercised*, not guards that failed.

### Audit 1 — does `overlap_link.py` assume "no sensing" when it means "unknown"?

The suspect line computed the share of a gain attributable to the robot's own
sensing as `0.0` whenever the trailing voxels-per-metre baseline was
unavailable. But an unavailable baseline means *unknown* sensing, not *zero*
sensing: if the robot drove during the settle window, real sensing would be
credited as received map. Direction of the error is transfer too large →
intersection too small → **overlap too low**, so it would have deflated the
88 %, not inflated it. Safe direction, still wrong.

`audit_baseline.py` over pb3g2's 116 robot-windows: **7** had an unknown
baseline, and in **all 7** the robot had not moved — where crediting the whole
gain as transfer is correct. **0 defective windows**; fd2s had 0 unknown
baselines at all. The 88 % is untouched.

The path was closed anyway, since pb4d runs in a world whose driving behaviour
is not yet observed and the fix costs nothing: unknown baseline **with**
movement now returns unidentifiable rather than zero. Re-running afterwards
reproduced 43 % / 88.0 % / 72.2 % exactly, confirming a no-op on today's data.

### Audit 2 — would `launch_pb4d.sh` actually launch 60 cells?

`run_campaign.sh:66` makes an unknown flag **fatal**, so a typo surfaces only at
launch. All ten flags `launch_pb4d.sh` passes were diffed against the parser's
accepted set (lines 45–70): **all ten parse**.

The seed expansion was the real find. `run_campaign.sh:77` is

```bash
IFS=',' read -ra _S <<< "$SEEDS"
```

which splits on **commas only**. The script is correct — it passes
`SEEDS=$(seq -s, 1 30)`, verified to expand to 30 seeds × 2 arms = **60 cells**.
But its own header comment documented the flag as `--seeds 1..30`, and that form
parses as a *single* seed literally named `1..30`:

| form | seeds parsed | cells |
|---|---|---|
| `$(seq -s, 1 30)` | 30 | **60** ✓ |
| `1..30` | 1 (the string `"1..30"`) | **2** |

A future reader "simplifying" the code to match the comment would get a
two-cell campaign that looks like a normal one which merely finished early —
and against a 2.3-day expectation, finishing early is the failure mode least
likely to be questioned. The comment now carries the parser line number and an
explicit instruction not to simplify it.

**Neither audit changed a result.** One closed a latent path before new data
could reach it; the other removed a hazard that lived in a comment rather than
in code, which is where the checks that stopped checking tend to live.

### 29.23 The launch script's PASS path had never run, and the first attempt to test it tested the wrong branch

`gate_and_launch.sh` decides whether to spend 2.3 days on pb4d. Its refusal
path was tested at 0/3 cells (exit 3). The path that actually **launches** had
never once executed, and it was going to execute for the first time unattended,
against a smoke that exists once. That is exactly the gap `test_censoring_path.py`
was built to close for §29.10, and §"checks that stopped checking" is the record
of what happens when it is left open.

Testing it needed a fixture, and a fixture needed the gates to be pointed
somewhere other than the live campaign. `gate2.py`, `partition.py` and
`shape_tripwire.py` each hardcoded `/tmp/hmr_campaign`; all three now read
`HMR_CAMPAIGN_ROOT` with that path as the default, and the wrapper exports it so
a wrapper and a gate can never disagree about where the data is. Re-run against
the real root afterwards, every verdict was unchanged.

**The first attempt to test the tripwire branch did not test it.** The obvious
fixture stretches one cell until `max/min` exceeds the armed 2.36x. It does trip
gate 3 — and it also pushes that cell's crossing past gate 2's "crosses by 60 %
of cap" condition, so **gate 2 fails first** and the script prints gate 2's
instructions. The verdict line still read `tripwire=TRIPPED`, and both paths
exited 1:

```
VERDICT   gate2=FAIL   partition=LICENSED   tripwire=TRIPPED
gate2 FAILED. Pre-registered response (§29.12): raise the criterion ...
```

Read on the real run, that output invites the wrong pre-registered response —
re-pricing `censor_power.py` when the actual fix is raising the cap. Neither the
exit code nor the summary line distinguished the two. Isolating gate 3 required
scaling **all three** cells (0.70/1.20/1.70): ratio 2.43x with the slowest cell
still crossing inside 3240 s. Two fixes followed: each failure block now prints
all three gate verdicts, and the exit codes are distinct — 1 gate2, 5 partition,
6 tripwire, 4 pb4d-already-exists, 3 smoke-incomplete.

A second case passed for the wrong reason. The NOT-LICENSED fixture deleted the
pb3g2 reference, so `partition.py` bailed with `need cells for both (0, 3)`
before computing anything — same exit code, different code path, and the branch
that fires in the real world stayed untested. Replaced with a fd2s link column
forced permanently connected, so the gate refuses on its own arithmetic
(median 0.0 % against a required 45.0 %).

`test_gate_launch.sh` now covers all nine paths. It ends with a **negative
control**: the pass fixture asserted to exit 3, which must be reported as FAIL.
Ten green lines on first run is also what a harness that cannot fail looks like,
and this project has already shipped six guards that printed PASSes after going
inert.

```
  PASS  pass / incomplete / invalid / gate2_cap / tripwire
  PASS  partition / guard_off / guard_hybrid / clears_again
  FAIL  NEGCTL_expect_fail     exit 0 (want 3)   <- expected
  10 passed, 0 failed
```

**The suite found no bug in the launch logic itself.** What it found was in the
*reporting*: two distinct failures that a reader would have had to disambiguate
by eye, at the one moment when the cost of getting it wrong is 2.3 days. The
fixture that was supposed to prove a branch worked instead proved the branch had
never run — which is the argument for building it.
