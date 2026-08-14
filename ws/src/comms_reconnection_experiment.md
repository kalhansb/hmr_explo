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
