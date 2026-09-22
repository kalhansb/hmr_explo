# Generation 33 design — positive connection, observable map exchange, peer mode

**Status:** for review. No code written. No file under `hmr_explo_ws` touched.
**Baseline:** gen 32, `explo_planner_node` sha `751d8344bd4dd21f`, commit `7a7e387`.
**Evidence base:** `ts4_32_n2`, 40 cells, 10 seeds × 4 arms, all on the gen-32 binary.
**Scope decision (Kalhan, 2026-09-22):** no comparison against earlier generations is
required. The goal is a system that is right in principle, not one that is
comparable to the banked cells. That removes the provenance argument from every
decision below; it does **not** remove the correctness arguments.

---

## 1. What was asked

> We should know when the maps exchanged.
> And then robots leave for exploration or homing.
> Also other robot should know if there are any other robots that do homing or
> exploration, that way in rendezvous or pursuit they know if any re-connection
> is profitable.

and, after the first review round:

> we should be able to positively determine if the robots connected … no guess work

Four requirements. They are numbered **R1–R4** below and traced through the rest
of the document.

| | requirement |
|---|---|
| **R1** | Know, positively, when two robots are connected |
| **R2** | Know, positively, when the map exchange has completed |
| **R3** | Depart for exploration or homing on that signal |
| **R4** | Know each peer's mode, so reconnection profitability can be priced |

---

## 2. Measurement

Everything in §3 and §4 rests on these numbers. Method and denominators are
stated so that a zero can be distinguished from a check that did not run.

**Tool:** `connect_and_drain.py` (scratchpad). It does **not** re-implement the
detector. `team_exchange` events already carry the node's own verdict
(`direct` / `one_way` / `via_relay` / `last_direct_age_sec`) on every received
`TeamWorld`. Between receipts the onboard state is fully determined by the TTL
rule, so the belief is reconstructed as

```
belief(t) = last_row.direct AND (t - last_row.t) <= direct_ttl_sec   (5.0 s)
```

which requires no model of the handshake — only the verdict the node wrote down.
Ground truth is `link_states.csv` (`connected`, per pair, 5 Hz), which the robot
cannot see.

### 2.1 Oracle integrity

All 40 cells: `mask_verdict=OK`, `masked=0`, `masked_by_valid=0`,
`masked_by_path_loss=0`. No rows with `valid=0`. The survivors-only hazard does
not bite on this campaign — the denominator is the full record.

### 2.2 Is `direct` a positive determination of connection? (R1)

Scored: **471,826 grid points = 94,365 robot-seconds** of link time.
9,237 further points fell before the first packet and are reported, not scored.

| outcome | count | rate |
|---|---:|---:|
| agree | 422,956 | **89.642 %** |
| **false positive** — believed direct, oracle DOWN | 13,908 | **2.948 %** |
| false negative (stale) — oracle UP, no packet inside TTL | 31,231 | 6.619 % |
| false negative (one-way) — oracle UP, fresh packet, mask did not name us | 3,731 | 0.791 % |

Edge latencies:

| edge | n | p50 | p90 | p99 | max | never |
|---|---:|---:|---:|---:|---:|---:|
| oracle up → believed | 928 | **10.20 s** | **121.32 s** | 553.74 s | 729.60 s | 0 |
| oracle down → dropped | 926 | 4.40 s | 5.00 s | 5.55 s | 26.40 s | 2 |

**The down edge is exactly as designed.** p90 = 5.00 s is `direct_ttl_sec` to two
decimal places. Loss detection is sound, bounded, and needs nothing.

**The up edge is not.** p50 = 10.20 s.

### 2.3 How long is a connection actually available?

| interval | n | p10 | p25 | p50 | p90 | max |
|---|---:|---:|---:|---:|---:|---:|
| connected | 466 | 2.0 s | 4.8 s | **11.5 s** | 91.2 s | 239.0 s |
| disconnected | 465 | 5.2 s | 6.8 s | 14.0 s | 229.3 s | 757.8 s |

**Median connected window 11.5 s; median time to believe it 10.2 s.** At the
median the robot accepts the link with ~1.3 s of the window left.

### 2.4 Eliminating the benign explanations for the up-edge

| candidate | measurement | verdict |
|---|---|---|
| Micro-blips too short to carry a 1 Hz packet | only **8.6 %** of connected intervals are < 2 s | **rejected** |
| Emulator queueing | `queue_age_sec` p50 **0.08 s**, p90 0.08 s | **rejected** |
| Slow delivery | receipt gap while delivering p50 **1.00 s**, p90 1.20 s — exactly `team_world_hz = 1.0` | **rejected** |
| Circular handshake | `directMask()` publishes `direct \|\| heard_one_way` — "I receive from them" — explicitly to avoid the deadlock. Cold start needs ~3–4 s | **rejected** |

Every benign explanation is eliminated by measurement. **There is an unexplained
~6–7 s between the oracle calling a link up and the first `TeamWorld` actually
arriving.** See §5.1 — this is the one question that must close before any code.

### 2.5 When does a map exchange actually happen? (R2)

| quantity | value |
|---|---:|
| `team_exchange` rows | 25,484 |
| rows with `applied > 0` | **232 (0.9 %)** |
| `RETURN_SYNC` visits (robot standing at the meeting) | 118 |
| …that applied **nothing at all** | **110 (93.2 %)** |
| inter-arrival of merges *within a single meeting* | **n = 0** |
| silence from entering `RETURN_SYNC` to first merge | n=8, p50 **55.9 s**, p90 195.9 s, max 258.3 s |

Three facts, each decisive:

1. **93.2 % of meetings exchange no census at all.**
2. **A meeting never contains two merges.** There is no "merges stopped arriving"
   event anywhere in 40 cells, because merges never arrive more than once.
3. **Where a merge does arrive, the median wait is 55.9 s — against a 30 s
   settle.** Today's meetings release before the exchange they are waiting for.

### 2.6 Relay

`via_relay = 1` on **0 of 25,484 rows**. Expected at N=2 — there is no third
robot to bridge through. Answered at N≥3 in §2.8.

`one_way = 1` on 723 rows (2.84 %). The §10 one-way failure is real but small.

### 2.7 Why every empty meeting is empty — the decomposition

"Merged nothing" has an innocent cause that must be separated before it can be
called a defect. Each `RETURN_SYNC` visit is scored on three facts: seconds the
peer was **physically** in range (oracle), seconds the robot **believed** it had
a direct peer (the node's own logged verdict + TTL), and whether anything
applied. Both timelines are sampled on the oracle's own 5 Hz grid so they share
a time base exactly.

| bucket | meaning |
|---|---|
| **ABSENT** | nobody there — empty is **correct** |
| **BLIND** | peer there, never believed — **the detector cost the exchange** |
| **SILENT** | there, believed, still nothing applied |

**Empty meetings, by rung:**

| | N=2 (gen 32, 40 cells) | N≥3 (gen `e7c185b`, 251 cells) |
|---|---:|---:|
| visits scored | 118 | 364 |
| merged something | 8 (6.8 %) | 167 (45.9 %) |
| merged nothing | 110 (93.2 %) | 197 (54.1 %) |
| → ABSENT | 17 (15.5 %) | 36 (18.3 %) |
| → **BLIND** | **1 (0.9 %)** | **72 (36.5 %)** |
| → SILENT | 92 (83.6 %) | 89 (45.2 %) |

**Believed ÷ connected during empty visits:** N=2 **p50 1.00**; N≥3 **p50 0.14**
(oracle-connected p50 28.4 s, believed p50 **0.0 s**).

*Sanity check:* ABSENT visits merged 0 of 36. A visit that merges cannot have
had nobody there, and an earlier run of this decomposition failed exactly that
check — see §2.9.

### 2.8 Relay at N≥3 — it is real and it carries content

402,279 `team_exchange` rows across 251 cells:

| | count | rate |
|---|---:|---:|
| `via_relay = 1` | 1,030 | 0.256 % of rows |
| …**of which applied a merge** | **160** | **15.5 % of relay rows** |
| `one_way = 1` | 16,092 | 4.00 % (vs 2.84 % at N=2) |

Relay rows are rare but **far more productive than average**: 15.5 % of them
carry an applied merge. Consistent with this, 10 of the 82 BLIND visits merged
something — **content demonstrably arrives without `direct` ever being true.**

> **Design consequence.** Requiring `direct` to open the exchange hold would
> discard a channel that measurably delivers. §4.1 already survives this
> (`direct` gates the **start**, arrival governs **staying**), but the rule must
> never be tightened to require `direct` throughout.

### 2.9 Methods note — a schema vintage that fakes a measured zero

`link_states.csv` gained its `valid` column part way through the corpus;
**ts1b and ts1d predate it** (216 of the 251 cells). Reading a missing column as
"not valid" drops every row, empties the oracle, and lands every visit in
ABSENT — a clean-looking zero that is pure artifact. The first run of §2.7 did
exactly this and reported 88.7 % ABSENT with 152 of those visits merging
something, which is impossible. The tool now detects the schema, treats an
absent column as unfiltered, and refuses to score a cell whose oracle is empty
rather than counting it as ABSENT. Re-running on the 40 cells that **do** have
the column reproduced their numbers exactly — a correct fix is inert where the
old path was already right.

### 2.10 Co-sensing flux at the meeting — does a hold ever drain?

Raised in review: if both robots keep sensing while parked, each one's new
observations stream to the peer, the counter never stops moving, the hold never
releases, and every meeting runs to the 3000 s cap. The objection rests on an
empirical premise — that a **parked** robot keeps observing new voxels at a rate
that matters — and a stationary lidar re-observes the same space, so it is not
obvious either way. Measured from `planner_*.csv` (col 1 `sim_time_sec`, col 2
`total_observed_voxels`, col 26 `state`):

| voxels/sec | N=2 (gen 32, 40 cells) | N≥3 (gen `e7c185b`, 235 cells) |
|---|---:|---:|
| while EXPLORING, p50 | 616.1 | 847.2 |
| while RETURN_SYNC, p50 | **0.4** | **0.0** |
| **last 30 s of meetings > 60 s**, p50 | **0.3** | **2.7** |
| …same, p75 | **1.8** | **1636.2** |
| …still gaining > 1 vox/s at the end | **10 / 31 (32.3 %)** | **147 / 282 (52.1 %)** |

**The premise is false at the median.** A parked robot's flux collapses by three
orders of magnitude, and the median long meeting ends at 0.3–2.7 vox/s. A
drained state exists and is detectable — Part 2 is not structurally doomed.

**But the objection is real in the tail.** A third of long meetings at N=2, and
**half** at N≥3, are still gaining when the robot leaves. A release rule that
waits for the counter to reach *zero* would over-hold exactly there. **The rule
must be a rate threshold, not a zero test** — recorded as a design requirement
in Part 2.

**The measurement cannot say which flux this is, and that is the point.**
`total_observed_voxels` is written from `stats.total_voxels`
(`explo_planner_node.cpp:16731`) — the same fused ROI grid that §3.1 disqualified
— so it sums own sensing and peer fusion. End-of-meeting flux is therefore
either co-sensing (the review's worry) **or a peer merge still landing as the
robot departs**, which is the §2.5 truncation defect and argues for *longer*
holds, not shorter.

One discriminator is available: **own-sensing flux should not scale with team
size; peer-merge flux should.** It scales steeply — 32.3 % → 52.1 % of meetings
still gaining, p75 1.8 → 1636.2 vox/s. That points at peer merge arrival as the
dominant component, i.e. at truncation. It is not conclusive (the rungs are
different generations, and larger teams explore more space), and separating the
two cleanly is **exactly what Part 1's per-peer counter is for**.

---

## 3. What the measurements kill

### 3.1 The drain design, as previously specified, is inapplicable

The earlier proposal was: hold until the exchange deltas go quiet for K seconds.
§2.5 removes its ground. With **n = 0** within-meeting inter-arrivals there is no
"quiet after flow" state to detect. The rule would collapse onto its
never-moved branch in 93.2 % of meetings — i.e. onto a plain timer, which is what
it was meant to replace.

Additionally, two of the three signals originally proposed are contaminated:

- `latest_map_voxels_` is fed from `map_cache_->computeStats().total_voxels`
  (`explo_planner_node.cpp:16735`) — the **already-fused** grid, own sensing
  included. It moves while the robot stands still.
- `sharedHash()` (`cell_world.cpp:334`) is FNV over **every** cell status; a cell
  this robot covers itself changes it.

Only `team_merge_applied_total_` is peer-attributable, and §2.5 shows it is zero
in 93 % of meetings.

### 3.2 "The maps exchanged" is not currently observable at all

This is the finding that reshapes the design.

`team_merge_applied_total_` measures the **TeamWorld cell census** — coordination
state, 100 cells. It is not the map.

The **map** is the dscovox voxel fusion. dscovox merges peer voxels *before* the
planner sees anything, and the planner receives only
`ScovoxMap → computeStats().total_voxels`: an aggregate with no peer attribution.
`peer_voxels` (`explo_planner_node.cpp:6503`) is the peer's **self-reported**
count from its `RobotIntent` beacon — a comparison input for the info-gate, not
a record of anything received.

> **There is no signal anywhere in the planner that reports peer-attributable
> map arrival.** R2 cannot be satisfied by rearranging existing code. The signal
> must be created, and it must be created in dscovox, which is the component
> that actually receives and fuses peer voxels.

### 3.3 "Skip homing peers" is the wrong rule for pursuit

Profitability is not only future coverage. A homing robot still carries a full
map that has never been merged, and it is the **cheapest possible intercept** —
its destination is fixed and known, unlike an exploring robot whose tour must be
predicted. A `continue` in the gate discards the one peer whose position is
perfectly predictable. R4 wants a **re-price**, not a skip.

---

## 4. Design

The measurements reorder the work. R1 is not one requirement of four — it is
underneath the other three, and there is a strong hypothesis that it *causes* the
R2 symptom.

> **Leading hypothesis (H1).** The 93.2 % empty-meeting rate and the 10.2 s
> acquisition latency are the same defect. If a robot only believes a link for
> ~1.3 s of a median 11.5 s window, then for most of every contact opportunity
> it is not exchanging, not holding, and not counting the peer as present.
> Nothing downstream can work correctly on top of that.
>
> **TESTED (§2.7). The answer depends on team size, and that is the single most
> consequential result in this document.**
>
> At **N=2 H1 is refuted**: only **1 of 110** empty meetings was BLIND, and
> during empty visits the robot believed the link for **100 %** of the time the
> peer was actually there. At the meeting both robots park in range, the link is
> stable, and a 5 s handshake amortises over a 43 s visit — acquisition does not
> bite where the exchange happens.
>
> At **N≥3 H1 is strongly supported**: **72 of 197** empty meetings (36.5 %)
> were BLIND, and during empty visits the robot believed the link only **14 %**
> of the time the peer was present (believed p50 **0.0 s** against
> oracle-connected p50 **28.4 s**).
>
> **The confound is stated plainly: the two rungs are different generations**
> (gen 32 vs `e7c185b`). The split may be team size, or it may be the
> generation. **The run that separates them is the gen-32 N=3/4 rung, which
> never executed** — `ts4_chain32` aborted after n2 returned `ok=39 fail=1` on a
> gate false positive and refused to start n3. Fixing that gate needs no rebuild
> and no new generation.

Two readings survive, and they imply opposite priorities:

- **if team size** — Part 0 is the highest-value work here, because every real
  deployment is N≥3 and a third of its meetings are lost to a detector that will
  not admit a peer standing next to it;
- **if generation** — gen 32 already fixed it, Part 0 is nearly done, and the
  effort belongs in Part 1.

Nothing else can be correctly prioritised until that resolves, and it resolves
with one campaign that is already written.

### Part 0 — R1: make connection acquisition fast and provable

The detector itself is correct and needs no redesign. `TeamModel::Peer::direct`
is a genuine two-way handshake — a packet inside `direct_ttl_sec` **and** that
packet's own `in_range_mask` naming us (`team_model.cpp:238-245`). The header is
right that this is *"the only thing that makes a peer a direct contact"*, and the
loss edge (§2.2) proves it behaves exactly as specified.

What must change is **acquisition**, and the change depends on §5.1's answer. The
candidate levers, in order of preference:

1. **Close the ~6–7 s gap at its source** once §5.1 identifies it. If the
   emulator's forwarding gate differs from the logged `connected`, that is a
   fidelity bug in the harness, not a planner change at all — the cheapest
   possible fix and it touches no binary.
2. **Raise `team_world_hz` during acquisition only.** The handshake costs ~3
   packets; at 1 Hz that is a 3 s floor against an 11.5 s median window. A
   faster beacon while a peer is `heard_one_way` (i.e. we are mid-handshake)
   shortens acquisition without raising steady-state airtime. Note the shared
   airtime bucket — this must be measured, not assumed free.
3. **Do not widen `direct`.** Accepting one-way contact would destroy the
   property that makes it a positive determination. §2.2's one-way false
   negative is only 0.791 %; it is not where the loss is.

**Instrumentation (required, not optional).** Emit the acquisition latency
directly: on every `heard_one_way → direct` transition, log the elapsed time
since the first one-way packet. Today this quantity exists only by offline
reconstruction against an oracle that does not exist in the field.

### Part 1 — R2: make the map exchange observable

dscovox gains a **monotone, per-peer counter of voxels ingested from that peer**,
published alongside the map it already publishes. The planner differences it
exactly as it differences `team_merge_applied_total_` today, and
`MapExchangeBaseline` (`explo_planner_node.cpp:1441`) extends by one field.

This is the only way to satisfy R2 as asked. It is also the largest piece of work
in this document and the only one that leaves `explo_planner`.

**Once the signal exists**, the hold rule follows from §2.5 rather than from
guesswork — but the *shape* of the rule must be re-derived from the new signal's
own arrival distribution, measured the same way §2.5 measured the census. The
census distribution does not transfer: voxels are bulk data and will almost
certainly arrive in streams where census merges arrive as single events.

**Explicitly deferred:** the quiet-window length. It is a free parameter until
the voxel arrival distribution is measured, and §3.1 is what guessing it looks
like.

### Part 2 — R3: depart on the signal

Smallest piece. The ladder at `finishOrRendezvous`
(`explo_planner_node.cpp:10438`) already routes correctly — re-plan if frontier
remains, else home, else done. Only the **trigger** moves: from the fixed settle
to Part 1's drain signal. No new routing logic.

**The release predicate.** Raised in review: co-sensing could keep the counter
moving forever and pin every meeting to the 3000 s cap, converting a timing
defect into a censoring defect. §2.10 measures it. The predicate is therefore
specified now, with two properties the measurement forces:

> Release when, for every peer believed present, the **per-peer ingest counter**
> (Part 1 — *not* `total_observed_voxels`) has gained **less than `R` voxels/s
> averaged over a trailing window `W`**, with a hard floor of `W` seconds held
> regardless and the existing duration cap unchanged as a backstop.

1. **A rate threshold, not a zero test.** 32.3 % (N=2) and 52.1 % (N≥3) of long
   meetings are still gaining >1 vox/s when the robot currently departs. A
   "counter has stopped" rule over-holds in a third to a half of cases; a rate
   rule does not.
2. **Per-peer, not fused.** The quantity being thresholded must be *merge
   arrival only*. `total_observed_voxels` sums own sensing and fusion
   (`:16731`), so a rate rule over it is exactly the co-sensing trap the review
   describes. Part 1 is therefore a **hard prerequisite for Part 2**, not a
   parallel workstream — ordering already stated in §5.
3. **The cap stays.** It is the backstop that makes a mis-set `R` a timing loss
   rather than an unbounded hold.

`R` and `W` are **not chosen here.** Choosing them off `total_observed_voxels`
would be fitting a threshold to the contaminated signal. They get fixed from the
per-peer distribution once Part 1 logs it — the first offline read after Part 1
lands, before Part 2 is enabled.

### Part 3 — R4: peer mode on the wire

`TeamWorld` gains `uint8 mode` (own) and `uint8[] robot_mode` (relayed), with
`EXPLORING < HOMING < DONE`, merged by max, no TTL — the exact semantics
`robot_finished[]` already uses and already justifies: *"A monotonic fact has no
freshness to check."*

**The monotonicity precondition is verified.** `doReturnHome`
(`explo_planner_node.cpp:14306-14613`) contains **zero** `transitionTo` calls,
and `startReturnHome` carries two re-entry guards that *"refuse every later
request"*. Homing never reverts to exploring, so a sticky bit cannot become a
permanent lie. **This must be written down at the declaration** — if anyone later
makes homing resumable, there is no TTL to rescue the encoding.

Consumers:

- **rendezvous / hybrid barrier** — widen `accountedPeerCount` channel 3 from
  `finished` to `finished || homing`. A homing peer is not coming to the meeting.
  This closes the window between "left for home" and "arrived and published
  finished", which is exactly where a partner burns its wait.
- **pursuit gate** — `HOMING` is a **re-price**, not a skip (§3.3): near-zero
  future-coverage value, full map-merge value, and the intercept point is
  **home**, not the predicted trail.

#### The `finished`-consumer decision table

Raised in review: widening only `accountedPeerCount` leaves the other
`finished` consumers on the old predicate, so the fleet holds two
inconsistent answers to "is this peer participating?". Correct, and the
consumer set is **larger than four**. Every site that reads a *peer's*
`finished`, with an explicit decision:

| # | site | question it asks | widen to `\|\| homing`? |
|---|---|---|---|
| 1 | `explo_planner_node.cpp:10762` `peerAccounted` | should I stop waiting? | **YES** — the barrier. This is the headline fix |
| 2 | `:10806` `reachablePeerCount` | is the appointment releasable? | **YES** — same question, ORs into `teamComplete` |
| 3 | `:10848` `peerReportsTeamBreak` | should the team *arm* a new appointment? | **YES** — the existing comment's own reasoning applies verbatim: arming for a robot "never coming" is the censoring the exemption exists to stop |
| 4 | `global_allocator.cpp:245` | who gets frontier cells? | **YES** — a homing robot explores no more cells; leaving it in lets makespan balance reserve work for it |
| 5 | `rendezvous_scheduler.cpp:29` | who is the meeting for? | **YES, and it is not optional** — `:20-23` states a contract that this filter *must* match the allocator's, pinned by test, not by compilation. Widening #4 without #5 breaks a documented invariant silently |
| 6 | `:9155` `robots_in_problem` | diagnostic count of the problem | **YES** — it counts #4's set. Not widening it makes the log describe a set the solver never saw |
| 7 | `:7885` `all_in_comms` | is every unfinished peer in radio contact? | **NO** — a *radio* statement, consumed only at `:9152` as a log field. A homing robot out of comms is a real outage; widening hides it from diagnosis |
| 8 | `:7920` separation anchor skip | should this peer still repel candidates? | **NO** — the comment's own justification is that the term "repels from somewhere a robot genuinely is". A homing robot genuinely is there *and is moving*. Skipping it removes a valid force |
| 9 | `:8605` → `reconnect_gate.cpp:21`, `:102` | is reconnecting to this peer profitable? | **NO — re-price instead** (§3.3). Near-zero future-coverage value, full map-merge value, intercept at **home** |
| 10 | `:10994` | reset of the local vehicle list | n/a — not a peer read |
| 11 | `team_model.cpp:225` | gossip OR | n/a — encoding; `mode` gets its own max-merge |

The split is one rule: **widen where the question is "will this peer
participate?" (a mode question); do not widen where it is "can I reach this
peer?" (a radio question).** Sites 7 and 8 are radio/geometry statements that
merely *borrowed* `finished` as a proxy for "parked"; homing is not parked.

Sites 4–6 are a **single atomic change** — they are three views of one set.

**Honest scope limit:** the relay needs a third robot. At N=2 this degrades to
last-contact-only, which is information the robot already has. Part 3's value is
concentrated at N=3/4 and §2.6 could not measure it at all. A flat N=2 result is
not evidence the mechanism failed.

### 4.1 Separation of concerns

The exchange asks a different question from the barrier, and conflating them is
the root of the original A3 defect:

| question | correct signal | why |
|---|---|---|
| "Should I stop waiting?" | `accountedPeerCount` — **keep channel 3** | A finished peer never arrives; without it the barrier hangs to the duration cap |
| "Is someone here to trade maps with?" | `p.direct \|\| p.via_relay` | Must be a radio statement. Channel 1 is state-gated; channel 3 may be third-hand *"with no contact of any kind"* |

Two corrections to an earlier draft of that second row, which read
`p.direct && !p.via_relay`:

- `!p.via_relay` is **dead code**. The closure loop opens with
  `if (p.direct) continue;` (`team_model.cpp:274-288`), so a direct peer can
  never carry `via_relay`. The conjunct can never be false when the first is
  true.
- Worse, it stated an intent that §2.8 refutes. Relayed rows applied a merge
  **15.5 %** of the time against 0.9 % overall, and 10 BLIND visits merged with
  `direct` never true. Relay is the **most** productive exchange channel per
  row. Excluding it would discard content that demonstrably arrives.

Presence gates whether the hold **starts**; arrival gates whether it **stays**.
They cover each other: if a peer reaches us some way `direct` cannot see, the
arrival counter moves anyway and the hold persists regardless.

---

## 5. Open questions — must close before code

### 5.1 The ~6–7 s acquisition gap **(blocking)**

Blips, queueing, delivery rate and handshake circularity are all eliminated
(§2.4). Remaining candidates:

- the comms emulator's forwarding gate uses a different or hysteretic criterion
  than the `connected` column it logs;
- `connected` is sampled at 5 Hz but forwarding is re-evaluated more slowly;
- publish-timer phase interacts with gate transitions.

**Test:** instrument the emulator to log the instant it begins forwarding between
a pair, and diff against `link_states.csv`'s up-edge on the same run. One
instrumented cell answers it. **No binary change, no campaign.**

Until this closes, we do not know which of two disagreeing signals is the truth,
and *"positively determine if the robots connected"* is not satisfied by either.

### 5.2 Does H1 explain the empty meetings? — **ANSWERED (§2.7)**

Refuted at N=2 (1 of 110 BLIND, believed/connected p50 1.00); strongly supported
at N≥3 (72 of 197 BLIND, believed/connected p50 0.14). **Confounded with
generation**, which promotes §5.5 to the blocking item.

### 5.3 Does the relay carry content? — **ANSWERED (§2.8)**

Yes. 1,030 relayed rows at N≥3, **160 of them applied a merge** — a 15.5 % hit
rate against 0.9 % for rows overall. 10 of 82 BLIND visits merged something, so
content arrives with `direct` never true. `via_relay` peers **belong** in the
exchange partner set; §4.1's split (direct gates the start, arrival governs
staying) is confirmed and must not be tightened.

### 5.4 Is the SILENT bucket correct behaviour? **(blocking Part 1's size)**

SILENT — connected, believed, nothing applied — is **92 of 110** empty meetings
at N=2 and **89 of 197** at N≥3. It is now the dominant unexplained bucket, and
§3.2 predicts the answer: `applied` counts the **100-cell census**, which
converges early and then legitimately has nothing to say, while the **voxel map**
underneath is still diverging. A sampled row is consistent with this
(`known_by_only: 82` of 100, `refused_guard: 13`, `refused_local: 5`).

If that holds, SILENT is not a defect at all — it is `applied` being the wrong
instrument, which is precisely why Part 1 exists. One offline pass over the
existing refusal counters, cross-checked against per-robot
`total_observed_voxels` divergence across the same visits.

### 5.5 Is the N=2/N≥3 split team size or generation? **(now the blocking item)**

It decides whether Part 0 is the most valuable work in this document or nearly
finished (§4). **Test:** fix the gate-scope false positive that aborted
`ts4_chain32`, then run the n3 and n4 rungs on the gen-32 binary already pinned
at `751d8344bd4dd21f`. No rebuild, no new generation, and the campaign script
exists. Re-run §2.7 on the result and compare the BLIND rate against the 36.5 %
measured at `e7c185b`.

---

## 6. Risks

| risk | mitigation |
|---|---|
| **`TeamWorld` field addition (Part 3).** On Humble without XTypes, a consumer built with a new field cannot deserialise a producer built without it: the producer becomes *silently and permanently invisible* for everything, claims included. **24 files reference `TeamWorld`**, including `pursuit_predictor.hpp`, `team_model.hpp`, `coordination.hpp`, `separation.hpp`, `reconnect_gate.cpp` and four gen-2x gtests. | Full `--packages-select explo_planner_msgs explo_planner` rebuild. Re-run the four gtests rather than assume. No mixed-binary cell, ever. |
| **Part 1 leaves `explo_planner`.** A dscovox change widens the blast radius and the rebuild surface. | Scope and review it as its own change; do not bundle its build with the planner's. |
| **Analysis plumbing.** `event_log.py` has `READER_SCHEMA = 9`, `MIN_SCHEMA = 9`; the writer is at 12, and the reader is already known to void N≥3 analysis. | Read the JSONL directly for anything this design adds. Do not route new instrumentation through it. |
| **Quiet window as truncator.** A window shorter than real arrival gaps silently truncates live merges and looks like a speed-up. | The window stays unspecified until the voxel distribution is measured (§4 Part 1). |
| **Censoring.** Longer meetings increase exposure to the 3000 s cap; hybrid seed 3 already censored in gen 32. Censoring is exposure, not severity. | Any extension gates on live arrival evidence, never on a flat larger timer. |
| **Acquisition beacon raises airtime.** One shared bucket; N=2 cells already run 8–88 %. | Gate the faster beacon to the mid-handshake window only, and measure the bucket. |

---

## 7. Test plan

**Before any code**

1. §5.1 emulator forwarding instrumentation — one cell.
2. §5.2 believed-time vs merge outcome over the 118 visits — offline.
3. §5.4 refusal-counter pass — offline.

**Unit (gtest, no colcon; `make <target> -j4` in `build/explo_planner` does not
relink the node)**

4. Mode monotonicity: `EXPLORING → HOMING → DONE` raises only; max-merge is
   order-independent; a `DONE` relay never downgrades a first-hand `HOMING`.
5. Barrier: a peer reporting `HOMING` releases the appointment barrier; a peer
   reporting `EXPLORING` does not.
6. Pursuit re-price: a `HOMING` peer is scored, not skipped, and its intercept
   is home rather than the predicted trail.
7. Exchange presence: a `finished` peer with no fresh `direct` does **not** open
   a hold (the A3 regression).

**Mutation** — every test above must be shown to fail against a deliberately
broken implementation before it is believed. A green suite can be blind by
construction.

**Campaign** — fresh root. CLEAN cells are skipped, not re-run, so a relaunch of
an existing root would silently keep pre-fix cells.

---

## 8. Build and provenance constraints

- All edits between campaigns. Editing a tracked file *or* creating an untracked
  one under `hmr_explo_ws` changes the manifest's `-dirty.<hash>` suffix.
- Read the sha **after** the final build. `__TIME__` makes the build
  non-reproducible; a rebuild to "check" invalidates the pin.
- Restore the NV shim block (`HMR_NV_SHIM=580.173.02`) after any colcon build —
  it is dropped every time. Verify by sourcing, not by grepping: the block
  exports a computed variable.
- colcon: `source /opt/ros/humble/setup.bash` and `PATH="/usr/bin:$PATH"`.
  Never `--packages-up-to explo_planner`.
- `PYTHONDONTWRITEBYTECODE=1` on every python invocation — a stray `__pycache__`
  entry is an untracked file under the workspace.
- **This file lives in the `hmr_explo` superproject on purpose. Do not move it
  into `ws/src/explo_planner`.** The comparability key's first field is
  `git_explo_planner` (`build_index.py:245`), and the manifest re-reads that rev
  per cell. Adding *any* file to that submodule — even a document — changes the
  recorded value and splits every subsequent cell from the banked `ts4_32_n2`
  corpus, which is precisely the N=2 baseline §5.5 must compare against.
  `git_hmr_explo` is harvested (`harvest.py:7`) but is **not** in the key, so
  the superproject is the safe home for campaign documents.

---

## 9. Summary for the reviewer

- **R1 is not satisfied today**, and the reason is measured, not asserted: two
  available signals disagree by ~6–7 s and every benign explanation is
  eliminated. §5.1 is blocking and costs one instrumented cell.
- **R2 is not satisfiable with existing signals.** The quantity that would
  answer it does not exist in the planner; it must be created in dscovox.
- **R3 is small** and rides on whatever R2 produces. Its release predicate is
  now specified as a **rate threshold, not a zero test** (Part 2), because
  §2.10 measures 32–52 % of long meetings still gaining voxels at departure.
  The threshold constants are deliberately left unset until Part 1 exposes an
  uncontaminated per-peer rate to set them from.
- **R4 is ready to build**, with its monotonicity precondition verified — but
  `HOMING` must be a re-price, not a skip, and its value cannot be demonstrated
  at N=2.
- **The previous drain proposal is withdrawn**, on its own evidence.
- **H1 was tested and split by team size** (§2.7): refuted at N=2 (1 of 110
  empty meetings BLIND, believed/connected p50 **1.00**), strongly supported at
  N≥3 (72 of 197 BLIND, believed/connected p50 **0.14**). Acquisition latency
  does **not** cost exchanges at the meeting in a pair; at N≥3 it appears to
  cost a third of them.
- **That split is confounded with generation**, and the run that separates them
  never executed: `ts4_chain32` aborted after the n2 rung returned
  `ok=39 fail=1` on a gate false positive. **§5.5 is now the blocking item**,
  and it costs a gate fix plus a campaign that is already written — no rebuild,
  no new generation.
- **Relay is real and productive** (§2.8): 160 of 1,030 relayed rows carried a
  merge, a 15.5 % hit rate against 0.9 % overall, and content arrives with
  `direct` never true. `via_relay` peers belong in the exchange partner set.
- **SILENT is now the dominant unexplained bucket** and §3.2 predicts why: at
  the meeting the robots are connected, believed, and have nothing left to say
  *about the census* — while the map underneath may still be diverging. That is
  the strongest argument in this document for Part 1, and §5.4 confirms or kills
  it offline.
