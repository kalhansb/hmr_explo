# Generation 33 design — positive connection, observable map exchange, peer mode

**Status:** reviewed twice, revised, campaign stopped — cleared to implement.
**Baseline:** gen 32, `explo_planner_node` sha `751d8344bd4dd21f`, commit `7a7e387`.
**Evidence base:** `ts4_32_n2`, 40 cells, 10 seeds × 4 arms, all on the gen-32
binary; plus **2 completed cells** of the stopped `ts4_32_n3` rung. Historical
blackout figures come from `ts1b`/`ts1d` (superseded binary, pre-2026-09 radio
regime) and are labelled as such wherever used.
**Scope decision (Kalhan, 2026-09-22):** no comparison against earlier generations is
required. The goal is a system that is right in principle, not one that is
comparable to the banked cells. That removes the provenance argument from every
decision below; it does **not** remove the correctness arguments.

> **What the waiver does and does not cover.** It waives *performance*
> comparison: gen 33 owes no speedup against a banked generation, and no result
> below is expressed as a delta to one. It does not waive *validity*. §5.5 and
> §2.10 both block on a cross-generation comparison, and neither is asking
> whether gen 32 beat `e7c185b` — each is asking whether a defect **measured on
> an old binary still exists on the current one**, which is the question of
> whether the evidence for a design decision is about this system at all. That
> is answered by re-measuring on one binary, not by comparing two. Where this
> doc cites a number spanning generations, it now says so and treats it as
> unmeasured rather than as evidence.

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

> **Does the node actually latch this way?** Raised in review: if `direct` is
> recomputed from a fresh `in_range_mask` on every arriving packet rather than
> held, the TTL term above is fiction and every number in §2 inherits the error.
> Checked against the source, and the reconstruction is exact rather than
> approximate.
>
> `TeamModel::tick(double now_sec)` (`team_model.cpp:238-244`) recomputes every
> peer on the **clock**, not per message:
>
> ```cpp
> const double at = reported_at_sec_[i];
> const bool receiving = at >= 0.0 && (now_sec - at) <= cfg_.direct_ttl_sec;
> const bool mutual    = receiving && maskHas(reported_mask_[i], self_id_);
> p.direct = mutual;
> ```
>
> `at` is the arrival stamp of the last packet, so at the moment a row is
> written `at == last_row.t` and `last_row.direct == mutual`. For any later `t`
> with no new packet, `receiving` is `(t - last_row.t) <= direct_ttl_sec` and
> `mutual` cannot become true again without one. The reconstruction is
> therefore **algebraically identical** to what the node computes, not a model
> of it.
>
> There is one call site (`explo_planner_node.cpp:19435`) and it fires
> unconditionally, including on empty batches — the codebase anticipates this
> exact objection in the comment above it: *"TTL expiry is a function of time,
> not of arrivals: a model that only advanced when a message came in could
> never notice that they stopped, which is the one thing it exists to notice."*
> `team_model.hpp:232-236` gives the same reason for separating `tick()` from
> `observe()`.
>
> **Residual bias, stated rather than dismissed:** rows are stamped at *drain*,
> not at arrival, so `last_row.t` runs late by `queue_age_sec` — p50 **0.08 s**,
> or 1.6 % of the 5 s TTL. `coalesced` is non-zero on one row in the whole
> corpus. The effect is to make the belief look very slightly *stale*, which is
> conservative for the up-edge finding in §2.2 and cannot manufacture it.

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

> **Why 28.4 s here and 11.5 s in §2.3.** Raised in review as an unexplained
> discrepancy — a meeting window apparently offering 2.5× the contact of a
> typical link — with the worry that meetings are a selected, unusually
> favourable subset and the decomposition therefore over-states what a normal
> reconnection gets. Two things, and the first is the whole of it:
>
> 1. **The two numbers do not measure the same quantity.** §2.3's 11.5 s is the
>    duration of a single connected **interval**. The 28.4 s is **total
>    connected seconds summed across a visit**, which may contain several
>    intervals separated by drops. Summing a quantity and taking its median is
>    not comparable to taking the median of its parts, so no selection effect is
>    required to produce the gap.
> 2. **Selection exists but is small.** Holding generation and team size fixed
>    and varying only the RETURN_SYNC conditioning, meeting windows are
>    **+11–14 %** on connected time, not +150 % (N=3: 15.2 → 17.4 s; N=4:
>    15.8 → 17.6 s). Robots that have converged on a point are slightly better
>    connected than average, which is expected and does not distort the BLIND
>    finding — a window that is 12 % more favourable still delivered **zero**
>    rows.

That N=2 `p50 1.00` is **exact, not rounded**, and the distribution says so
rather than the percentile: **56 of 101** scorable visits (55.4 %) sit at
exactly 1.0 and **none** fall in [0.995, 1.0). At N=2 the detector believes the
peer for the entire time the peer is there.

*Sanity check:* ABSENT visits merged 0 of 36. A visit that merges cannot have
had nobody there, and an earlier run of this decomposition failed exactly that
check — see §2.9.

**BLIND is not slow acquisition — it is total silence.** Scoring what actually
*arrived* during each BLIND visit, rather than only what was believed:

| | N=3 | N=4 | both |
|---|---:|---:|---:|
| BLIND visits | 41 | 31 | 72 |
| …that received **zero** `team_exchange` rows | 40 | 31 | **71** |
| …that ever saw a `direct` row | 0 | 0 | **0** |
| oracle-connected p50 | 16.6 s | 26.0 s | — |
| oracle-connected p90 | 37.3 s | 64.8 s | — |

Longest such visit 103.8 s; 63 of the 71 exceed 5 s; the unheard peer was alive
and still logging in 70 of 71. At 1 Hz the median BLIND visit should have
carried 16–26 messages and carried **none**.

Dumping the three longest individually shows the silence is **fleet-wide and
bidirectional**, not a per-link acquisition failure. In `ts1d_n4_…_seed2`, husky
held `RETURN_SYNC` for 156.6 s with skadi connected 103.8 s of it and received
**0 rows from atlas, bestla *and* skadi** — while skadi, logging 82 events and
cycling `NAVIGATE→INTEGRATE→LOG_STEP→PLAN` normally, logged **0 rows from
anyone**. In all three the first row after the blackout carries
`direct=False, one_way=True`: the handshake restarting from scratch, not
resuming.

> **This kills H1 as stated, and redirects Part 0.** Acquisition latency cannot
> explain a window in which nothing is acquired, in either direction, across
> every pair at once. The operative hypothesis is contention or shedding in the
> comms emulator during **convergence** — when every pair goes connected
> simultaneously against one shared `airtime_capacity: 0.6` bucket — which is a
> harness property, not a detector property. §5.1 is reframed accordingly.

**Scope limit, and it is load-bearing.** Every number in this subsection comes
from `ts1b`/`ts1d`: a **superseded binary** and the **pre-2026-09 radio
regime** (this corpus predates 70 dB trunks + 30 m horizon). Whether the
blackout survives into the current binary is §5.5, and it now decides whether
Part 0 and Part 1 are needed **at all**.

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

(The 82 is BLIND over *all* visits; §2.7's 72 is BLIND among visits that merged
nothing. 72 + 10 = 82.)

**`direct` also churns, and the churn scales with team size.** Onset latency is
only half the question — a verdict that flickers is as costly as one that
arrives late. Re-scored on a connected-time denominator, counting `direct`
**drops** while the oracle still says connected:

| | N=2 | N=3 | N=4 |
|---|---:|---:|---:|
| `one_way` rows while connected | 2.84 % | 3.95 % | 4.03 % |
| `direct` drops per pair-minute | **0.00** | 0.15 | 0.34 |
| pairs that ever dropped `direct` | **0 of 80** | 404 of 930 (43 %) | 804 of 1152 (70 %) |

At N=2 the verdict is **perfectly stable — not one drop in 80 pairs.** At N=4 it
is unstable for 70 % of pairs, but at ~one drop per three pair-minutes it is
churn, not cycling; re-acquires exceed drops several-fold (7,831 vs 2,559 at
N=4), so most are span-onset acquisition rather than recovery from a flicker.

> **Design consequence.** Part 0's instrumentation must report the **drop rate**
> alongside acquisition latency. A design that only measures onset would score
> N=2 and N=4 identically on a property where they differ by infinity.

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

One discriminator suggests itself: **own-sensing flux should not scale with team
size; peer-merge flux should.** Across the table it scales steeply — 32.3 % →
52.1 % still gaining, p75 1.8 → 1636.2 vox/s — which would point at peer merge
arrival as the dominant component, i.e. at truncation.

> **That reading is withdrawn.** It is the same generation × team-size confound
> §5.5 calls blocking, and it cannot be waved through with a parenthetical here
> while blocking a design decision there. The two columns differ in **binary**
> as well as in N: the N=2 column is gen 32, the N≥3 column is `e7c185b`. So
> the "scale-up" is equally well explained by the generation change, and this
> table cannot distinguish the two.
>
> Re-run with the binary **held fixed** — gen 32 on both sides, the only
> comparison that tests the stated premise — the discriminator has no power at
> all:
>
> | still gaining > 1 vox/s at the end | N=2 (gen 32) | N=3 (gen 32) |
> |---|---:|---:|
> | meetings > 60 s scored | 31 | **3** |
> | still gaining | 10 (32.3 %) | 0 (0.0 %) |
> | p75 vox/s | 1.6 | 0.0 |
>
> Three meetings. The point estimate moves the *opposite* way and means nothing
> at that n. The honest statement is that **the component split is unmeasured**,
> not that it favours truncation.

This changes no decision in Part 2 — the rate-threshold requirement above rests
on the N=2 tail (10 of 31), which is within one generation and stands on its
own. What it removes is the claim about *which* flux that tail is. Separating
the two cleanly is **exactly what Part 1's per-peer counter is for**. The
same-generation N=4 arm that would have given this table power was forfeited
when the gen-32 campaign was stopped (§5.5), so Part 1's counter is now the
**only** route to the split rather than one of two.

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

What must change is **acquisition**, and §2.7 has since narrowed the levers from
three to one:

1. **Close the gap at its source in the harness.** §2.7 shows the BLIND windows
   are fleet-wide TeamWorld blackouts, not per-link handshake latency, so the
   defect is in delivery rather than detection. This is a fidelity bug in the
   emulator — **it touches no binary and is the whole of Part 0's value.**
   §5.1 specifies the instrument.
2. ~~**Raise `team_world_hz` during acquisition only.**~~ **Withdrawn — refuted
   by §2.7.** The lever assumed a 3-packet handshake against a 1 Hz beacon, i.e.
   a 3 s floor worth attacking. But **71 of 72** BLIND visits received *zero*
   packets across a median 16–26 s of contact. A 5 Hz beacon multiplies zero. It
   would spend airtime — on the same contended bucket now suspected of causing
   the blackout — to send more copies of a message that is not being delivered.
   > A second, independent reason it would not have worked as specified, raised
   > in review: the lever is **not unilateral**. `direct` requires a packet from
   > the peer *and* that packet's `in_range_mask` naming us, so the acquisition
   > time is set by the **slower** of the two beacons. A robot that raises its
   > own rate to 5 Hz while the peer stays at 1 Hz still waits on the peer's
   > 1 Hz mask — it buys nothing and spends airtime to do it. "Raise the rate
   > during acquisition" therefore needs a fleet-wide coherent trigger, which
   > a robot that has not yet acquired the peer cannot coordinate. Even had
   > §2.7 not refuted the premise, the lever was underspecified.
3. **Do not widen `direct`.** Unchanged and now better supported. Accepting
   one-way contact would destroy the property that makes it a positive
   determination, and §2.8 shows the one-way rows are not where the loss is
   (4 % of rows, against visits losing 100 % of their traffic).

**Instrumentation (required, not optional).** Emit **both** halves of the
verdict's behaviour, because §2.8 shows they diverge:

- on every `heard_one_way → direct` transition, the elapsed time since the first
  one-way packet — acquisition **latency**;
- on every `direct → ¬direct` transition while still receiving, the duration
  held — acquisition **stability**.

Today both exist only by offline reconstruction against an oracle that does not
exist in the field.

### Part 1 — R2: make the map exchange observable

dscovox gains a **monotone, per-peer counter of voxels ingested from that peer**,
published alongside the map it already publishes. The planner differences it
exactly as it differences `team_merge_applied_total_` today, and
`MapExchangeBaseline` (`explo_planner_node.cpp:1441`) extends by one field.

This is the only way to satisfy R2 as asked. It is also the largest piece of work
in this document and the only one that leaves `explo_planner`.

#### Where the counter comes from — the quantity already exists

Raised in review: this was hand-waved, and a per-peer counter could have meant
building a shadow grid. It does not. **dscovox already keeps peers separate.**

- `dscovox_node.cpp:1046-1047` — `std::unordered_map<std::string, SourceGrid>
  sources_`, commented *"One source grid per robot, keyed by header.frame_id of
  incoming binaries."* Peers are **not** fused into an anonymous common grid;
  they are folded at query time (`:559-582`).
- `:405` keys each incoming binary by `msg->header.frame_id`; `:485-497`
  finds-or-creates that source's grid.
- `:437-438`, `:587-588` — `n_touched_beta` / `n_touched_dir` already count
  *"fused occupancy/semantic cells this frame changed"*, per frame, per source.

So Part 1 is **a per-source running sum of a number dscovox already computes**,
not new map machinery. Three consequences, each answering a review question:

| question | answer |
|---|---|
| Shadow grid needed? | **No.** Per-peer separation is native to `sources_`. |
| Monotone across a dscovox restart? | **No — and it must not be relied on.** The accumulator is node-local and resets. Part 2 differences *increase since the hold began*, so the planner captures a baseline at hold start; a restart appears as a negative delta and is treated as **re-baseline**, never as arrival. This is what `MapExchangeBaseline` already does for the census. |
| Under relay, who is credited? | **The originator, and the emulator cannot do otherwise.** `hmr_comms_sim_node` forwards with `create_generic_subscription` / `create_generic_publisher` over `rclcpp::SerializedMessage` (`:774-782`, `:911-912`) — it never deserializes, and the string `frame_id` does not occur in the file. The payload crosses byte-for-byte, so `header.frame_id` is structurally preserved and voxels count against the peer that sensed them, not the relayer. Consistent with §4.1: relay is a delivery path, not a different partner. |

**Cost of being wrong here is bounded:** if `n_touched_*` proves too coarse (it
counts cells changed, not voxels ingested, and a re-observation of a known cell
touches nothing), the fallback is the per-source grid's own active-voxel count,
already available from the same map. Both are per-peer; neither needs a new grid.

#### The counter must be a **pair**, and it must be published unconditionally

**Found by adversarial review of this document, 2026-09-22. This is the one
finding that changes what gets built.**

A novelty counter alone cannot distinguish the two states Part 2 must tell
apart, because **both read zero**:

| peer state, peer **believed present** throughout | `deltas_received` (arrival) | `cells_touched` | correct action |
|---|---:|---:|---|
| stream tailed off — **genuinely drained** | rose earlier in the visit, now flat | flat | **release** |
| **map blackout** — nothing arriving at all | **never moved this visit** | **never moved** | **do not release** |

As Part 1 was written — one counter, of novelty — Part 2 releases in both rows.
That means the release predicate fires **fastest in exactly the failure mode
Parts 0 and 1 exist to fix**, and the robot departs having exchanged nothing
while its own logs record a clean drain. This is the same error §2.7 had to
build an oracle-based decomposition to escape — *silence read as completion* —
reproduced one layer down, at the voxel layer, where there is no oracle.

> **Corrected during implementation, and the correction sharpens the fix rather
> than softening it.** The first draft of this table claimed the two rows
> separate as *arrived > 0, novel → 0* against *both 0* — a drained peer still
> pushing redundant voxels. **They do not separate that way, because neither
> end works like that.** The producer delta-encodes: `publishBinaryMap` ships
> only what `drainTouchedBeta()` / `drainTouchedDir()` hand it
> (`scovox_node.cpp:2094-2100`), so a drained peer sends *fewer deltas*, not
> the same deltas again. And the receiver's `touched_*` sets are inserted into
> **unconditionally** — `*v = d.data; touched_beta.insert(mc);`
> (`dscovox_node.cpp:536-539`, `:547-551`) — with no compare against the
> previous value, so `cells_touched` counts *cells written*, not *cells
> changed*. The two counts therefore rise and fall **together**, and no
> instantaneous pair of rates tells the rows apart.
>
> What separates them is **whether the arrival counter moved at all during this
> visit**. That is why the arrival term must be differenced against a
> hold-start baseline — the same thing `MapExchangeBaseline` already does for
> the census — and not merely rate-thresholded. It also means
> `cells_touched` is not load-bearing for the release decision. It stays
> because it is free, it is the number that answers R2's actual question
> (*"we should know when the maps exchanged"* — did the fused map move, or did
> bytes cross and change nothing?), and it is the fallback if `deltas_received`
> turns out to be too coarse.

Two code facts make the failure concrete rather than theoretical:

1. **The arrival quantity already exists and is already computed in the same
   scope.** `dscovox_node.cpp:437-438` computes `n_beta_deltas` /
   `n_dir_deltas` — *"how much arrived"* — right beside `n_touched_beta` /
   `n_touched_dir` — *"how much it touched"*. The per-source log at `:601-604`
   already prints all four. Nothing new is measured; one of two existing
   numbers was simply dropped from the design.
2. **Publishing "alongside the map" would itself be silent during a blackout.**
   `publishFusedMap()` returns early unless the map is dirty (`:850`
   `if (!fused_dirty_.exchange(false)) return;`), and `fused_dirty_` is set only
   at `:609`, reached only once non-empty deltas have been fused. So when
   nothing arrives, **nothing is published**, and the planner sees the counter's
   last value persist — indistinguishable from "arrived, added nothing".
   (`:848` returns early on zero subscribers too.)

**Required, and both parts are cheap:**

- dscovox publishes **two** monotone per-source running sums — `deltas_received`
  (arrival) and `cells_touched` (fused cells written) — not one.
- They publish on their **own timer, independent of `fused_dirty_`**, so that
  "nothing arrived" appears as a *fresh sample with an unchanged arrival count*
  rather than as an absent sample. A counter whose silence is ambiguous is not
  an observation.

This is additive: it changes no fusion logic, adds no grid, and reuses both
quantities verbatim. It does mean Part 1's deliverable is a small message, not
a single integer.

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

> Release when, for **every** peer believed present, both hold:
>
> - **it has spoken at all this visit** — its Part 1 `deltas_received` has
>   risen strictly above the baseline stamped at hold start; **and**
> - **it has stopped** — its `deltas_received` rate over the trailing window
>   `W` has fallen **below `R` per second**.
>
> A hard floor of `W` seconds is held regardless, and the existing duration cap
> is unchanged as a backstop.
>
> **The first clause is not redundant and must not be optimised away.** Without
> it the predicate reads a map blackout as a drain and releases on the floor —
> see *"The counter must be a pair"* in Part 1. A peer that has delivered
> nothing since the hold began is **not drained**, however long its rate has
> been zero; that visit falls through to the duration cap and must be **logged
> as an unfinished exchange**, not as a release. The two outcomes are different
> events and the logs must name them differently, or the same ambiguity returns
> as a reporting bug instead of a control bug.
>
> **It is a level test, not a rate test, and that is deliberate.** The rate
> over `W` is zero in both rows at the moment of the decision; only the
> difference against the hold-start baseline separates "went quiet after
> talking" from "never started".

1. **A rate threshold, not a zero test.** **10 of 31 (32.3 %)** long meetings on
   gen 32 at N=2 are still gaining >1 vox/s when the robot currently departs. A
   "counter has stopped" rule over-holds in a third of cases; a rate rule does
   not. (The 52.1 % at N≥3 is cross-generation and is **not** relied on — see
   the withdrawal in §2.10. The N=2 figure is within one binary and carries
   this requirement on its own.)
2. **Per-peer, not fused.** The quantity being thresholded must be *merge
   arrival only*. `total_observed_voxels` sums own sensing and fusion
   (`:16731`), so a rate rule over it is exactly the co-sensing trap the review
   describes. Part 1 is therefore a **hard prerequisite for Part 2**, not a
   parallel workstream — ordering already stated in §5.
3. **The cap stays.** It is the backstop that makes a mis-set `R` a timing loss
   rather than an unbounded hold.
4. **Release is monotonic within a visit.** Once the predicate fires for a
   visit, it **latches released** — the hold does not re-enter on a late burst
   of merge traffic. Raised in review, and the asymmetry is deliberate: the
   counter is bursty by construction (a single fused message can deliver
   thousands of voxels at once), so a re-entrant hold would oscillate around
   `R` and could pin a robot that had already satisfied the exchange. A late
   burst after release is a merge that lands while departing — the §2.5
   truncation defect, which Part 1's counter records and which is a *logging*
   concern, not a reason to stop the robot again. The latch clears on the next
   entry into the meeting state, so a subsequent visit re-arms normally.
5. **The predicate inherits the detector's false-positive rate.** It is
   quantified in §2.2 and must not be silently carried: the robot believes a
   peer is direct while the oracle says DOWN on **2.948 %** of scored grid
   points (13,908 of 471,826). "For every peer believed present" therefore
   means *believed*, and a robot can hold for a peer that is not actually
   there. The design does not claim the hold is conditioned on ground truth,
   because nothing onboard has it.
   The consequence is bounded, and by a specific knob: the cap in (3) is
   **`rendezvous_latched_hold_sec`** (default 300 s; this campaign runs 420 s).
   The failure mode is a wasted wait of at most that cap, never a permanent
   stall.
   > **CORRECTION (implementation, 2026-09-22). An earlier draft of this
   > paragraph argued the cap applied for free, because the node already
   > validates `rendezvous_latched_hold_sec > rendezvous_settle_sec`
   > (`explo_planner_node.cpp:4799-4808`) — "so replacing the settle trigger
   > with the drain trigger cannot produce a hold the cap fails to bound."
   > **That inference does not hold, and the code as written when the design
   > was reviewed did not bound the drain hold at all.** The validated
   > relation is between the cap and the *settle* knob, and the cap's own
   > declaration comment scopes it to the at-the-rendezvous hold of a
   > **coverage-latched** robot. The drain hold is neither: a different gate,
   > on a different path, reached by a robot that need not be latched. A cap
   > that bounds one branch says nothing about a branch that does not consult
   > it.
   >
   > **It is bounded now because it was written in, not because it followed.**
   > Two pieces, both required:
   > - the roll-forward arm of the drain gate tests it explicitly —
   >   `if (settled_sec < rendezvous_latched_hold_sec_) return;` (`:14058`),
   >   and past it the robot leaves on a distinctly-named **UNFINISHED
   >   EXCHANGE** outcome rather than a release, so the cap is visible in the
   >   log rather than silently absorbed;
   > - a configure-time coupling check refuses to start when
   >   `rendezvous_drain_window_sec >= rendezvous_latched_hold_sec`
   >   (`:4867-4875`). Without it the *first* window test (`:13992`) returns
   >   unconditionally until a full window has elapsed, so a window wider than
   >   the cap would carry the hold past it before the cap is ever read. The
   >   cap would be present in the source and inert in the run — the failure
   >   mode `checks-that-stopped-checking` is about.
   >
   > The claim "at most the cap" is therefore true of the shipped code and was
   > not true of the design that asserted it. Both mechanisms are pinned by
   > test; neither may be removed on the grounds that the validation above
   > already covers it.
   > Two neighbouring knobs are **not** the backstop here, and conflating them
   > is a known defect with a pinned regression test
   > (`test_gen20_rendezvous.cpp:969-1004`).
   > `reconnect_midrun_max_wait_sec` bounds a **mid-run reconnect attempt**, not
   > the wait of a robot keeping an appointment. `rendezvous_appointment_wait_sec`
   > is the barrier — waiting for a peer to *arrive* — and it is deliberately
   > unbounded (ruled 2026-09-22); Part 2 does not touch it and proposes no cap
   > on it. Part 2's hold is strictly **post-arrival**, which is why the
   > latched-hold cap is the one that applies.

`R` and `W` are **not chosen here.** Choosing them off `total_observed_voxels`
would be fitting a threshold to the contaminated signal. They get fixed from the
per-peer distribution once Part 1 logs it — the first offline read after Part 1
lands, before Part 2 is enabled.

### Part 3 — R4: peer mode on the wire

`TeamWorld` gains `uint8 mode` (own) and `uint8[] robot_mode` (relayed), with
`EXPLORING < HOMING < DONE`, merged by max, no TTL — the exact semantics
`robot_finished[]` already uses and already justifies: *"A monotonic fact has no
freshness to check."*

**The monotonicity precondition is verified — for `HOMING`.** `doReturnHome`
(`explo_planner_node.cpp:14306-14613`) contains **zero** `transitionTo` calls,
and `startReturnHome` carries two re-entry guards that *"refuse every later
request"*. Homing never reverts to exploring, so a sticky bit cannot become a
permanent lie. **This must be written down at the declaration** — if anyone later
makes homing resumable, there is no TTL to rescue the encoding.

> **`DONE` is a different matter, and the earlier draft was wrong to cover both
> with one argument.** `state_ == DONE` **is not monotone**: `:7452-7458`
> transitions `EXPLOIT_PLAN` with reason `"target-arrived-done-idle"`, so a
> robot that has reached `DONE` can leave it. Encoding the top level of `mode`
> from `state_` would publish a max-merged, TTL-free `DONE` that the robot
> itself has already contradicted — permanently, since max-merge cannot go back
> down.
>
> **The codebase already solved this and the solution must be reused, not
> re-derived.** `finished_announced_` (`:3042-3049`) is set exactly once
> (`:19101`) and **never cleared** — it exists precisely because
> `(coverage_latched_ || state_ == DONE)` is not monotone. `mode`'s `DONE`
> level derives from `finished_announced_`; `HOMING` from the homing latch.
> **Neither may read `state_` directly.** Per §3.3 this is also the predicate
> the `finished`-consumer table above is already written against, so the two
> stay consistent by construction.
>
> **Amended 2026-09-23 (§10, item 5): `DONE` also waits for the meeting.** A
> finished robot keeping its appointment publishes below `HOMING` until the
> appointment manoeuvre ends, and `DONE` from then on, latched in its own
> `done_announced_` (`announcedMode`, `meeting_attendance.hpp`). `DONE` still
> implies `finished`; `finished` no longer implies `DONE`. Every consumer
> above ORs `finished` in, so none of them moves.

Consumers:

- **rendezvous / hybrid barrier** — widen `accountedPeerCount` channel 3 from
  `finished` to `finished || homing`. A homing peer is not coming to the meeting.
  This closes the window between "left for home" and "arrived and published
  finished", which is exactly where a partner burns its wait.
  **NOT SHIPPED in gen 33 — see the correction under the table. The widening
  breaks two consumers this same design protects, and no placement that was
  tried avoids that. The barrier half of R4 is deferred; the allocator half
  ships.**
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
| 1 | `explo_planner_node.cpp:11006` `peerAccounted` | should I stop waiting? | ~~**YES** — the barrier. This is the headline fix~~ → **NO, DEFERRED.** See the correction below: it is not a leaf predicate |
| 2 | `:11050` `reachablePeerCount` | is the appointment releasable? | ~~**YES** — same question, ORs into `teamComplete`~~ → **NO, DEFERRED**, same reason |
| 3 | `:11055` `peerReportsTeamBreak` | should the team *arm* a new appointment? | **YES** — the existing comment's own reasoning applies verbatim: arming for a robot "never coming" is the censoring the exemption exists to stop |
| 4 | `global_allocator.cpp:245` | who gets frontier cells? | **YES** — a homing robot explores no more cells; leaving it in lets makespan balance reserve work for it |
| 5 | `rendezvous_scheduler.cpp:29` | who is the meeting for? | **YES, and it is not optional** — `:20-23` states a contract that this filter *must* match the allocator's, pinned by test, not by compilation. Widening #4 without #5 breaks a documented invariant silently |
| 6 | `:9399` `robots_in_problem` | diagnostic count of the problem | **YES** — it counts #4's set. Not widening it makes the log describe a set the solver never saw |
| 7 | `:8124` `all_in_comms` | is every unfinished peer in radio contact? | **NO** — a *radio* statement, consumed only at `:9152` as a log field. A homing robot out of comms is a real outage; widening hides it from diagnosis |
| 8 | `:8143` separation anchor skip | should this peer still repel candidates? | **NO** — the comment's own justification is that the term "repels from somewhere a robot genuinely is". A homing robot genuinely is there *and is moving*. Skipping it removes a valid force |
| 9 | `:8844` → `reconnect_gate.cpp:21`, `:102` | is reconnecting to this peer profitable? | **NO — re-price instead** (§3.3). Near-zero future-coverage value, full map-merge value, intercept at **home** |
| 10 | `:11263` | reset of the local vehicle list | n/a — not a peer read |
| 11 | `team_model.cpp:227` | gossip OR | n/a — encoding; `mode` gets its own max-merge |

The split is one rule: **widen where the question is "will this peer
participate?" (a mode question); do not widen where it is "can I reach this
peer?" (a radio question).** Sites 7 and 8 are radio/geometry statements that
merely *borrowed* `finished` as a proxy for "parked"; homing is not parked.

Sites 4–6 are a **single atomic change** — they are three views of one set.

> **CORRECTION (implementation, 2026-09-22): sites 1 and 2 are not leaf
> predicates, and widening them breaks two consumers this same table protects.
> The barrier half of R4 is DEFERRED; it is not in gen 33.**
>
> The table classifies each site by the question *that site* asks. For 1 and 2
> the classification is right and the instruction is still wrong, because
> neither is read in isolation: `peerAccounted` feeds `accountedPeerCount`,
> which feeds `teamComplete`, which has ~30 call sites. Two of them are sites
> this table separately rules **must not** change, and widening the leaf
> changes them anyway — through the call graph, silently, with no edit at
> either site to show for it.
>
> 1. **The mid-run reconnect trigger is skipped entirely, not re-priced.**
>    `explo_planner_node.cpp:8704` reads
>    `if (!teamComplete(live, rendezvous_expected_peers_) && cooldown_ok)`
>    over `const int live = accountedPeerCount(trig_now)`. Widen the leaf and
>    a homing, out-of-contact peer counts as accounted, `teamComplete` goes
>    true, and **the gate is never evaluated at all.** That is a blanket skip
>    — the exact verdict row 9 rejects in favour of "re-price, NOT skip". The
>    homing peer's map is worth the same parked as moving, and the widening
>    would stop the fleet from ever pricing it.
> 2. **Pursuit releases on a quarry nobody heard.** `:15873` reads
>    `const bool quarry_heard = !pursue_peer_id_.empty() && peerAccounted(pursue_peer_id_, now);`
>    Widen the leaf and a homing quarry reads as *heard* with no beacon, no
>    closure, no contact of any kind — an active chase abandoned on a false
>    premise, and a log line asserting a reconnection that did not happen.
>    §5.5's whole complaint is that we cannot tell connection from inference;
>    this would manufacture a fresh instance of it inside the fix for it.
>
> Both were confirmed by reading the call graph, not inferred. **Resolution as
> shipped:** site 3 is widened (it is a genuine leaf — `peerReportsTeamBreak`'s
> two callers, the arm at `:11147` and the contagion-hold log, are both
> barrier-side, so the predicate moves identically on both). Sites 4–6 are
> delivered through a **new `AllocRobot::off_frontier` field** rather than by
> widening `AllocRobot::finished`, because `finished` is itself read by rows 7
> and 9 (`reconnect_gate.cpp:21`, `:102`) and `:8844`'s `MissingPeer` — a
> widening there would have hit exactly the same wall one layer down.
> `off_frontier` is folded into `alloc_hash` (it selects the vehicle set, so
> two robots disagreeing about a peer's mode must not agree on the key) and
> forced off in the rendezvous snapshot alongside `finished`, which it needs
> more than `finished` does: a robot's own homing latch flips the instant it
> turns for home while its partner learns a TeamWorld later, or never.
>
> **What this costs.** R4's allocator half ships in full — a homing robot stops
> being reserved frontier work minutes earlier than before. R4's barrier half
> — a partner ceasing to wait on a peer that has turned for home — **does
> not**. That window stays open in gen 33 and remains a live cost in the
> rendezvous and hybrid arms. It is deferred rather than dropped: the fix
> needs a `teamComplete` that distinguishes "this peer will not come" from
> "this peer is reachable", which is a split of that predicate, not a widening
> of its input, and that is a larger change than gen 33's scope. **Do not
> attempt it by widening `peerAccounted`; that is the path this correction
> closes.**
>
> §10 item 5 (2026-09-23) reads `mode` at the appointment barrier and is
> **not** this deferred half. It is a veto: it can only make the barrier wait
> *longer*, for a finished peer still coming, and it stands down when that
> peer says `HOMING` or `DONE`. An **unfinished** homing peer is still waited
> for exactly as before, so the window named here is still open.

**Honest scope limit:** the relay needs a third robot. At N=2 this degrades to
last-contact-only, which is information the robot already has. Part 3's value is
concentrated at N=3/4 and §2.6 could not measure it at all. A flat N=2 result is
not evidence the mechanism failed.

### 4.1 Separation of concerns

The exchange asks a different question from the barrier, and conflating them is
the root of the original A3 defect:

| question | correct signal | why |
|---|---|---|
| "Should I stop waiting?" | `accountedPeerCount` — **keep channel 3** | A finished peer never arrives; without it the barrier hangs to the duration cap. **Stale since gen 32** — a finished robot with an appointment standing *does* arrive (`keepAppointmentOnFinish`). Channel 3 is kept for every consumer; the appointment barrier alone now waits for a finished peer that has not said it is leaving (§10, item 5) |
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

A third question joined these on 2026-09-23 (§10, item 5): **"is somebody
still coming?"** It is answered by the peer's own word: a `finished` peer below
`HOMING` that cannot be heard is still on its way to the meeting, and one at
`HOMING` or above has said it is leaving. It is asked only by the appointment
barrier, as a veto, and never through `accountedPeerCount`.

#### Shipping order — what happens between the parts

Raised in review: §4.1 forward-references Part 1, so shipping Part 0 alone
leaves the hold with **no arrival signal** and it falls back to the old settle
timer — which is the defect this document exists to remove. Correct, and the
answer follows from Part 0 having shrunk to a single harness lever:

| ships | what it is | depends on |
|---|---|---|
| **Part 0** | emulator instrumentation + whatever §5.1 finds | nothing — **no binary change**, can land today |
| **Parts 1 + 2** | per-peer voxel counter, then release on it | **atomic** — Part 2 has nothing to release on without Part 1 |
| **Part 3** | `mode` on the wire | independent of the arrival signal; may ride with 1+2 or alone |

**Parts 1 and 2 are one change and must not be split.** There is no interim
state worth shipping between them: Part 1 alone adds a signal nothing consumes,
Part 2 alone consumes a signal that does not exist. Part 0 escapes the question
entirely because it no longer touches the planner.

---

## 5. Open questions — must close before code

### 5.1 Why does a connected pair deliver nothing? **(blocking)**

**Reframed.** This subsection previously asked whether the emulator's forwarding
gate disagrees with the `connected` column it logs. **Reading the emulator
settles that: it does not.**

- `hmr_comms_sim_node.cpp:668` — `connected` *is* `bandwidth_mbps > 0`.
- `:837` — forwarding gates on that **same field**.

There is a discrete forwarding instant, it is the logged column, and there is no
hysteresis or trailing window to reconstruct. The three candidate causes above
are all dead. *(Note for whoever instruments this:
`hmr_comms_relay_node.cpp` has a different, bandwidth-only gate — it is
**legacy and not launched**. `comms_sim.launch.py:196` starts
`hmr_comms_sim_node`. Instrumenting the wrong file would produce confident
nonsense.)*

The real question is the one §2.7 raised: **a pair can be `connected` and still
deliver nothing**, because three more gates sit between connected and delivered —
`drop_ber` (`:849`), `drop_airtime` (`:854`), `drop_overflow` (`:825`).

**And the instrument already exists but is thrown away.** `PublishStats()`
(`:966-1001`) computes per-link, per-reason counters — exactly what is needed —
and publishes them on `stats_pub_`. Only the **cell-wide aggregate** reaches
`comms.log` (`:998`), and campaigns run `--record 0`, so the topic is never
bagged. **Nothing in the entire bank can attribute a single drop to a link or a
reason.** That is the actual gap.

**Test:** log the per-link per-reason counters that `PublishStats` already
builds, then re-run one convergence-heavy cell. **No binary change, no campaign,
no new computation** — a logging line in the sim node.

**One candidate is already weakened.** The shared `airtime_capacity: 0.6` bucket
was the leading suspect, since TeamWorld is best-effort and hard-dropped while
the dscovox map is reliable and queued on the same bucket. Sampling it during
the blackouts refutes the simple version: tokens sit near the 0.25 s cap
(p50 0.18–0.24) in every window, BLIND and SILENT alike, **0 % at zero**. The
sampler is ~28 s and cannot see sub-second transients — but a 16–103 s blackout
requires near-continuous emptiness, which would have shown.

**Fallback if the counters exonerate the emulator.** Then the messages were
never published, and the defect is upstream in the planner's own publish path
(executor starvation on the TeamWorld timer). That is testable from the same
cell without new instrumentation: a starved publisher logs no
`team_exchange` rows to **any** peer while continuing to log other events —
which is precisely the fleet-wide bidirectional signature §2.7 already observed.
In that case Part 0 becomes a planner change after all, and its lever is timer
priority, not beacon rate.

### 5.2 Does H1 explain the empty meetings? — **ANSWERED (§2.7)**

Refuted at N=2 (1 of 110 BLIND, believed/connected p50 1.00); strongly supported
at N≥3 (72 of 197 BLIND, believed/connected p50 0.14). **Confounded with
generation**, and §5.5 records that the confound is now closed *unmeasured*
rather than resolved — the N≥3 half of this answer stays on the superseded
binary permanently.

### 5.3 Does the relay carry content? — **ANSWERED (§2.8)**

Yes. 1,030 relayed rows at N≥3, **160 of them applied a merge** — a 15.5 % hit
rate against 0.9 % for rows overall. 10 of 82 BLIND visits merged something, so
content arrives with `direct` never true. `via_relay` peers **belong** in the
exchange partner set; §4.1's split (direct gates the start, arrival governs
staying) is confirmed and must not be tightened.

### 5.4 Is the SILENT bucket correct behaviour? — **ANSWERED for the census**

SILENT — connected, believed, nothing applied — is **92 of 110** empty meetings
at N=2 and **89 of 197** at N≥3. §3.2 predicted that `applied` is simply the
wrong instrument: the **100-cell census** converges early and then legitimately
has nothing to say. **Measured, and confirmed.**

| | N=2 | N=3 | N=4 |
|---|---:|---:|---:|
| delivered rows per **believed-connected second** | 1.01 | 0.96 | 0.96 |

Against a configured `team_world_hz = 1.0`, messages arrive at **full rate**.
And they are not being refused: `drop_reason` is empty on **all 14,151 rows**,
`coalesced` is nonzero on **one** row in the entire corpus, and the merge
outcome is `applied` = **0** against `agreed_noop` = **1,363,174** cell
decisions. The census is delivered, on time, and every cell in it already
agrees.

**SILENT is benign.** It is not a defect, not a detector failure, and — this was
raised in review as a third possibility — **not silent message loss in the ROS 2
graph on one side of the exchange.** A dropped-message explanation is
incompatible with a measured 1 Hz arrival rate and an empty `drop_reason` column.
Part 1 does not need to defend against it.

> **Methods note, in the spirit of §2.9.** The first run of this normalised
> delivered rows by **visit duration** and reported 0.44 msg/s — an apparent
> 56 % shortfall that looked exactly like a defect. It was an artifact of the
> denominator: a peer believed for 20 s of a 60 s visit cannot be expected to
> deliver 60 messages, and scoring it against the visit manufactures the gap.
> Re-normalising on believed-connected seconds reversed the conclusion. **The
> denominator, not the count, was carrying the finding.**

**Still open — the voxel half.** This settles that the *census* is converged; it
says nothing about whether the **map** underneath is. That cross-check needs
per-robot `total_observed_voxels` divergence across the same visits, and it is
exactly the quantity Part 1 creates. §5.4 therefore no longer blocks Part 1's
size — it **confirms Part 1's premise** and hands the remaining question to it.

### 5.5 Is the N=2/N≥3 split team size or generation? **(closed — will not be measured)**

> **Decision (Kalhan, 2026-09-22): the gen-32 n3/n4 campaign was stopped at 3 of
> 80 cells and gen 33 proceeds now.** Recorded here because it changes what the
> rest of this section is: not a pending result, but a **permanent limit** on
> the evidence base.
>
> What is forfeited is specific. The ≥60 s blackout band — the phenomenon that
> motivates Parts 0 and 1 — was measured only on `ts1b`/`ts1d`, and is
> overwhelmingly an **N=4** effect (216/1,612 spans, 13 %, against 13/368,
> 3.5 % at N=3). The gen-32 N=4 rung never ran, and it cannot be run later:
> `__TIME__` makes the build non-reproducible, so binary `751d8344bd4dd21f`
> can never be rebuilt, and a gen-33 campaign ships the fix and the test
> together — an absence of blackouts there cannot distinguish *fixed* from
> *never present at N=4 on this generation*.
>
> **What survives, and it is not nothing.** The two completed N=3 cells already
> establish the load-bearing claim: the blackout **reproduces on the current
> binary** (93.0 connected-seconds lost per cell against the control's 228.6),
> so Parts 0 and 1 are not fixing a dead defect. They also establish that it is
> **convergence-linked** within a single binary and seed (`off` 0.0 s, `hybrid`
> 186 s). What is lost is the *magnitude at N=4*, i.e. how much Parts 0 and 1
> are worth — not whether they address something real.
>
> **Consequence for how this design must be read.** Every ≥60 s blackout figure
> below is from a superseded binary and the pre-2026-09 radio regime, and now
> stays that way. Part 0 and Part 1 are justified by a defect confirmed present
> on gen 32 but **sized only on `ts1b`/`ts1d`**. Any gen-33 result must be
> stated in absolute terms rather than as an improvement over an unmeasured
> baseline — which is consistent with the scope decision at the head of this
> document, and is the reason that decision does not cost anything here.

**The stakes rose with §2.7.** This no longer merely sizes Part 0. Every
blackout measurement in this document comes from `ts1b`/`ts1d` — a superseded
binary and the pre-2026-09 radio regime. If the blackout does not reproduce on
the current binary, then **Part 0 and Part 1 are fixing a defect that no longer
exists**, and this design would be a binary change justified by evidence from a
binary already replaced. Nothing else in the bank can settle it, and a
gen-33 campaign cannot either — it would ship the fix and the test together.

**Test (stopped at 3 of 80 cells).** The gate-scope false positive that aborted
`ts4_chain32` was fixed and the n3 rung began on the gen-32 binary pinned at
`751d8344bd4dd21f` — no rebuild, no new generation. Two cells completed
(`off`, `hybrid`, seed 1); a third was killed mid-run and is marked
`ABORTED_PARTIAL_` on disk. **The n4 rung never started.**

> The partial cell's manifest carries `run_gates_verdict=CLEAN` with no `rc=`
> and no `end_reason=` — the gates ran early, passed, and the run was killed at
> 28 min of a 3000 s sim. A truncated cell that advertises CLEAN is exactly the
> shape of thing the harvester would bank as valid, which is why it is renamed
> rather than left in place.

**The one read taken: the blackout survives into gen 32, but milder.** Re-scored on a
general form of the §2.7 probe — per ordered pair, walk the oracle's contiguous
connected spans and count rows delivered in each. It does not depend on
`RETURN_SYNC`, so it runs on any arm, and it is banded by span length because a
2 s span that delivers nothing at 1 Hz is unremarkable while a 60 s one is the
whole finding.

N=3, like for like (`ts1d_n3`, 40 cells, old binary/old radio regime — versus
the first 2 completed gen-32 cells):

| span length | control: zero-rx | gen 32: zero-rx | control rows/s | gen 32 rows/s |
|---|---:|---:|---:|---:|
| 1–5 s | 342/480 (71 %) | 12/24 (50 %) | 0.00 | 0.26 |
| 5–15 s | 339/830 (41 %) | 6/28 (21 %) | 0.37 | 0.68 |
| 15–60 s | 162/1036 (16 %) | 4/12 (33 %) | 0.63 | 0.71 |
| **≥60 s** | **13/368 (3.5 %)** | **0/32** | 0.65 | 0.79 |
| **connected-seconds lost per cell** | **228.6** | **93.0** | | |

Three things follow, and the third is the one that matters:

1. **The defect is real on the current binary.** It is not an artifact of the
   superseded generation, so Parts 0 and 1 are not solving a dead problem.
   Delivery rate improved in every band and per-cell loss fell ~2.5×, but
   **nothing here is fixed.**
2. **It is convergence-linked, as §2.7 hypothesised.** The gen-32 `off` cell —
   which never enters `RETURN_SYNC` and therefore never converges — lost
   **0.0 s** across 28 spans. The `hybrid` cell, which holds `RETURN_SYNC` for
   96–241 s per robot, lost 186 s. Same binary, same seed, same regime; the
   difference is whether the robots come together. Anyone re-running this on an
   `off` cell will conclude the defect is gone, and be wrong.
3. **The long blackouts are not yet tested.** The ≥60 s band drove §2.7 and is
   at 0/32 on gen 32 — but the control's own N=3 rate predicts only ~1.1 such
   events in 32 spans, so **this is underpowered and proves nothing.** Those
   blackouts were overwhelmingly an **N=4** phenomenon: 216 of 1,612 spans
   (13 %) at N=4 against 13 of 368 (3.5 %) at N=3. **The n4 rung was the real
   test of this design's premise, and it will not run** — see the decision
   above. This is the one claim in the document that is now permanently
   un-derisked, and it should be read as a stated limit rather than as a gap
   waiting to be filled.

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
   a hold (the A3 regression). *As decided 2026-09-23 (§10, item 5):* a
   finished peer that has **said it is leaving** is not waited for — at N=2
   the hold ends at its first window as "every peer has said it is leaving";
   a finished peer that has **not** said so is still coming, and the barrier
   waits for it (bounded) before any hold opens.
8. **Blackout is not a drain.** Both cases hold the peer *believed present* and
   both present a `deltas_received` rate of **0** at the moment of decision;
   they differ only in whether the counter ever left its hold-start baseline.
   (a) Counter never moves: the predicate must **not** release, must run to
   the duration cap, and must emit the *unfinished exchange* event — not the
   drained one. (b) Counter rises, then goes flat for `W`: it **must** release
   and emit the drained event. The test fails the instant the baseline
   difference is dropped and the predicate is reduced to a rate test — which
   is exactly how this defect entered the design in the first place.
9. **Counter liveness.** The per-source counters must keep publishing while
   nothing arrives. Assert a fresh sample with an unchanged `deltas_received`
   during a silent interval; a test that only checks values while data flows
   would pass against the `fused_dirty_`-gated publish that caused the bug.

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
  Never `--packages-up-to explo_planner`. Use
  `--packages-select explo_planner_msgs explo_planner --symlink-install`.
  > **This contradicts `explo_planner/README.md:44-55`, which instructs
  > `--packages-up-to explo_planner`.** Flagged in review, and the
  > contradiction is literal. Both are partly right, and the README's *reason*
  > is the part to keep: it warns against `--packages-select` because
  > `explo_planner_msgs` is a sibling that must build first — true, and it
  > assumes the single-package form. Naming **both** packages satisfies that
  > requirement exactly. What `--packages-up-to` additionally does is pull in
  > every upstream dependency, including `scovox`, so an unrelated upstream
  > change can relink `explo_planner_node` and silently invalidate a pinned
  > sha — the failure this doc's second bullet exists to prevent. The
  > two-package select is therefore the strictly safer form, and the README was
  > **stale rather than wrong**. **Fixed 2026-09-22** in the gen-33 window, in
  > both the repository README and the package README (`explo_planner/README.md`
  > `## Build`), which carried the same bare line. The remaining
  > `--packages-up-to` mentions are listed in §10 and deliberately left.
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

- **R1 is not satisfied today, but not for the reason first written.** The
  emulator's forwarding gate does **not** disagree with the `connected` column
  it logs — `hmr_comms_sim_node.cpp:837` gates on the same field `:668`
  defines, so there is a discrete forwarding instant and it is the logged one.
  The defect is **delivery, not detection**: a pair can be `connected` and
  deliver nothing (§2.7), and the per-link per-reason counters that would say
  why are computed by `PublishStats()` and then discarded. §5.1 is blocking and
  costs **a logging line**, not a binary change.
- **R2 is not satisfiable with existing signals.** The quantity that would
  answer it does not exist in the planner; it must be created in dscovox.
  **It is a pair, not a number** — `deltas_received` (arrival) *and*
  `cells_touched` (fused cells written), published on a timer that does **not**
  depend on the fused map being dirty. Adversarial review of this document
  found that a novelty-only counter reads a blackout and a completed drain
  identically (both zero), which would make Part 2 release fastest in precisely
  the failure mode Parts 0 and 1 exist to fix, and record it as a clean
  exchange. Both numbers are already computed side by side at
  `dscovox_node.cpp:437-438` and already logged at `:601-604`; the design had
  simply dropped one of them.
- **The release test is a level, not a rate**, and implementation is what
  established that. Both ends delta-encode, so arrival and novelty fall
  together and no instantaneous pair of rates separates a drained peer from a
  silent one. What separates them is whether `deltas_received` moved off its
  hold-start baseline at all — so Part 2 differences the counter the way
  `MapExchangeBaseline` already differences the census.
- **R3 is small** and rides on whatever R2 produces. Its release predicate is
  now specified as a **rate threshold, not a zero test** (Part 2), because
  §2.10 measures **10 of 31** long meetings still gaining voxels at departure
  within a single binary. It is additionally specified as **monotonic within a
  visit** (no re-entrant hold on a late burst), and it inherits the detector's
  **2.948 %** false-positive rate — bounded by `rendezvous_latched_hold_sec`,
  which is the post-arrival cap and *not* the deliberately-unbounded
  appointment barrier. The threshold constants are deliberately left unset
  until Part 1 exposes an uncontaminated per-peer rate to set them from.
- **R4 is ready to build**, with its monotonicity precondition verified — but
  `HOMING` must be a re-price, not a skip, and its value cannot be demonstrated
  at N=2.
- **The previous drain proposal is withdrawn**, on its own evidence.
- **H1 was tested, split by team size, and then killed outright** (§2.7).
  Refuted at N=2 (1 of 110 empty meetings BLIND; believed/connected is
  **exactly** 1.0 in 56 of 101 visits, with none in [0.995, 1.0)). At N≥3 the
  BLIND rate is high (72 of 197) — but **71 of those 72 visits received zero
  packets** across a median 16–26 s of contact from a live peer, and the
  silence is fleet-wide and bidirectional. That is not slow acquisition; it is
  a channel that is down. **A faster beacon multiplies zero, so Part 0's
  beacon-rate lever is withdrawn.**
- **`direct` is perfectly stable at N=2 and churns at N=4** (§2.8): 0 drops in
  80 pairs, versus 0.34 drops per pair-minute across 70 % of pairs. Part 0 must
  instrument stability, not just onset latency.
- **The blackout reproduces on the current binary**, so this design is not
  fixing a dead problem. At N=3 it is ~2.5× milder than the superseded
  generation (93 vs 229 connected-seconds lost per cell) but **present**, and
  it is **convergence-linked**: the `off` arm, which never meets, lost 0.0 s
  while the `hybrid` arm on the same seed lost 186 s. This rests on **two
  completed cells** — enough to establish presence and the convergence link,
  not enough to size the effect.
- **§5.5 is closed unmeasured, by decision, and this is the document's main
  limit.** The gen-32 campaign was stopped at 3 of 80 cells to start gen 33.
  The ≥60 s blackouts that drove §2.7 are a 13 %-at-N=4 phenomenon against
  3.5 % at N=3, and the N=4 rung never ran; it cannot be run later, because the
  build is not byte-reproducible and a gen-33 campaign ships the fix and the
  test together. **Parts 0 and 1 are therefore justified by a defect confirmed
  present but sized only on a superseded binary.** Gen-33 results must be
  reported in absolute terms, never as a delta to a baseline that does not
  exist.
- **Relay is real and productive** (§2.8): 160 of 1,030 relayed rows carried a
  merge, a 15.5 % hit rate against 0.9 % overall, and content arrives with
  `direct` never true. `via_relay` peers belong in the exchange partner set.
- **SILENT is explained and benign** (§5.4, answered). At the meeting the
  census arrives at **full rate** (0.96–1.01 rows per believed-connected second
  against `team_world_hz = 1.0`), `drop_reason` is empty on **all 14,151 rows**,
  and the outcome is `applied` = **0** against `agreed_noop` = **1,363,174**.
  The robots are connected, believed, and have genuinely nothing left to say
  *about the census*. This rules out silent ROS 2 message loss as an
  explanation, and leaves the voxel map as the only place divergence can hide —
  **which is the strongest argument in this document for Part 1**, now measured
  rather than predicted.
- **The belief model used throughout §2 was challenged and is exact.** If the
  node recomputed `direct` per packet rather than latching it, every number in
  §2 would be wrong. `TeamModel::tick()` recomputes on the **clock**
  (`team_model.cpp:238-244`) from a single unconditional call site
  (`:19435`), which makes the offline reconstruction algebraically identical to
  the node's own computation rather than an approximation of it. The one
  residual bias — rows stamped at drain, `queue_age_sec` p50 **0.08 s** = 1.6 %
  of the TTL — runs in the conservative direction.
- **One claim has been withdrawn on review** (§2.10). The co-sensing
  discriminator argued that flux scaling with team size implicates peer merge,
  but its two columns differed in **binary** as well as in N — the same
  confound §5.5 calls blocking, which cannot be waived in one section and
  enforced in another. Re-run with the binary held fixed it has **3 meetings**
  and no power. The component split is **unmeasured**, and the arm that would
  have given it power went with the stopped campaign — Part 1's per-peer
  counter is now the only route to it. No design decision changes: Part 2's
  rate-threshold requirement rests on the within-generation N=2 tail.

---

## 10. Implementation status — 2026-09-22

Recorded here rather than in a commit message because the campaign reads this
document, not the log. **Shipped** means written, built, covered by a test, and
that test shown to fail against a deliberate mutation. Nothing below is claimed
on a green suite alone (§7, and the eight guards that went inert while printing
PASSes).

### Shipped

| Part | What landed | Where | Tests / mutations |
|---|---|---|---|
| **0** (planner half) | `acquire_sec` and `held_sec` per peer: acquisition latency from the first one-way packet to the packet that completes the handshake, and hold duration for a handshake broken **by a peer that stayed audible**. Both differenced from **packet** stamps, never tick stamps; both one-shot, so a reader counts events instead of diffing a level. Surfaced on `TeamExchangeEvent` and written to the JSONL. | `team_model.hpp/.cpp`, `experiment_log.hpp/.cpp`, node drain second pass | 6 `TeamModelR1.*`; M1–M7, all killed |
| **0** (emulator half, §5.1) | The per-link, per-reason cumulative counters `PublishStats()` already built for `~/stats` (`relayed`, `bytes`, `drop_ber`, `drop_airtime`, `drop_disconnected`, `drop_overflow`, `backlog_bytes`, per direction) now also go to `comms.log` as a `link counters: {json}` line, once per stats period. Campaigns run `--record 0`, so the topic was never bagged and the `relay totals` line was the only trace. **A separate line**, so the totals line and `manoeuvre_events.py`'s `RE_RELAY` are byte-unchanged. Logging only; no forwarding behaviour changes. | `hmr_comms_sim_node.cpp` | No gtest — it is a log line. Smoke run of the built emulator (two robots, reliable and best-effort chatter): both lines appear each period, `RE_RELAY` still matches the totals line, and the new line parses as JSON with every link and reason present |
| **1** | dscovox per-source fusion counters — `deltas_received` **and** `cells_touched`, the pair §9 says the design had dropped one of. New `ScovoxFusionCounters.msg`, published on a timer that does **not** gate on `fused_dirty_` (test-plan 9's defect). | `dscovox_node.cpp`, `scovox_msgs` | consumer-side parse guard in the node; **test-plan 9**: `FusionCountersLiveness.SilenceStillProducesFreshSamplesWithTheCountUnchanged` (`scovox_mapping/test/test_fusion_counters_liveness.cpp`), black-box against the built dscovox binary as a child process. M41 (publish gated on `fused_dirty_`) and M42 (timer never created) killed, three runs each; guard G1 — the test's own fused-map subscription — shown load-bearing by removing it, after which M41 passes |
| **2** | Drain-release trigger, over every peer believed present (`direct` or `via_relay`): a **level** test against the hold-start baseline, then a **rate** below `R` over a tumbling window `W`; a reading that examines no peer is never a drain (F1). Reaching the cap logs UNFINISHED EXCHANGE, a different event from the drained release. Monotone within a visit. **Default OFF, and it refuses to start without a measured `R` and `W`** — the constants are deliberately unset (§9) until Part 1 produces an uncontaminated per-peer rate. The predicate was **extracted** from `doReturnSync` into `stepDrainRelease` so tests 7 and 8 can run it rather than scan it; the node keeps the window state, the log lines and the latch. | `exchange_drain.hpp/.cpp` (in `explo_planner_lib`), `explo_planner_node.cpp` | **Run** — `test_exchange_drain.cpp`, 7 tests: test-plan 7 (`ExchangePresence.*`, four, one a control), 8's behavioural half (`BlackoutIsNotADrain.*`, two), and `DrainUnmeasured`. M27–M30, M35–M37 and M43 killed against the extraction. **M31 is equivalent** while M28's backstop stands; M31+M28 together are killed. **Scan** — `Gen33DrainRelease.TheNodeCallsThePredicateAndObeysIt` pins the call site, argument by argument (M38–M40 killed); `…TheThresholdsAreRefused…` pins the `R`/`W` refusal (M32–M34 killed) |
| **3** | Peer mode on `TeamWorld` (`EXPLORING`/`HOMING`/`DONE`, append-only, open-ended upward), max-merge on relay, first-hand authoritative and able to lower. Consumer shipped: `AllocRobot::off_frontier`, which drops a homing robot from the vehicle set and returns its cells — the window `finished` cannot see — folded into `alloc_hash`, and forced off in the rendezvous snapshot. | `TeamWorld.msg`, `team_model.*`, `global_allocator.*`, node | 4 `TeamModelMode.*`; `OffFrontierRobotIsRemovedAndItsCellsReturn` + `AllocHash.EveryVehicleFieldMovesTheDigest`, both mutated and killed |

### Shipped but **not in this design** — R-3, the latched-hold clock

Found while implementing, fixed, and called out here because it is a
**behavioural change to the campaign binary that no section above reviews**.

`appointmentDue()` has aimed the **arrival** at `t_meet` since 2026-09-19,
departing at `t_meet − appointmentLeadMs` (the travel estimate marked up 1.2×).
Early arrival is therefore the designed case. Both stamps of
`coverage_latch_hold_start_sec_` wrote the bare mission clock, so an early
arrival spent part of the cap before the meeting was due — against a cap
`run_explo_sim_rviz.sh` derives as `interval + max_lateness + 60 s` **measured
from `t_meet`**, with the early margin reaching `interval/6` = 50 s of that
60 s. The failure it buys is the one the hold exists to forbid: the finished
robot walking off the cell as the robot it was waiting for drives onto it.

Both stamps are now floored at `t_meet` when the appointment is valid.

The clamp cannot unbound the hold, and the argument for that is **not** the
lead — an earlier draft of this paragraph bounded `t_meet − arrival` by the
lead the robot departed on, which is only true of the deadline departure.
`keepAppointmentOnFinish` departs the moment the map saturates and never calls
`appointmentDue()` at all, so it carries **no lead**, drives straight there, and
can stand for most of the countdown. That is the case the floor is most worth
having for, and a lead-based bound does not cover it. The bound that covers
both is the lattice: `t_meet` is a rung of spacing `interval` and
`nextAgreedOccurrence` returns the first rung at or after now, so
`t_meet − arrival ≤ interval` whatever brought the robot there, and the total
stand is bounded by `interval + cap`. A schedule that **rolls** does not extend
it, because the roll clears the stamp and sets the sticky teardown.

Two stale comments from the one-day gen-19 departure rule were rewritten with
it: the `doReturnSync` note claiming "arrival is necessarily at or after
`t_meet`", and the scheduler's claim to be `depart_safety_milli`'s only reader.

`Gen33HoldClock.BothStampsFloorTheCapAtTheMeetingInstant`; M23–M26, all killed.
(Numbered from 23 because GROUP E already holds M15–M22 — the collision is
logged under *Review* below.)

### Review of the implementation — 2026-09-22

A second reviewer read the code changes above and returned seven findings. All
seven were checked against source before anything was changed; three did not
survive that check intact, which is the reason the checking happens. Recorded
with verdicts because a finding that was **refuted** is as load-bearing as one
that was fixed — it is the thing a later reader will otherwise re-raise.

| # | Finding | Verdict | What was done |
|---|---|---|---|
| F1 | The drain-release presence loop skipped relayed peers, and a loop that skipped **every** peer fell out with `drained` still `true` — releasing the hold having examined no one | **Confirmed, both halves.** The second is reachable on an ordinary relay-only N≥3 topology, not a corner case | Filter widened to `!p.direct && !p.via_relay`; `if (examined == 0) drained = false;` added as the backstop. Relayed peers are the productive channel — gen-32's census had relayed rows applying a merge 15.5 % of the time against 0.9 % overall |
| F2 | The belief model and the drain test read different clocks | **Refuted.** `TeamModel::tick()` recomputes on the clock; the two blocks in `transitionTo()` are separately guarded and the departure path was traced by brace-matching | Nothing. Left as-is |
| F3 | `kSchemaVersion` still described a gen-32-only behaviour change | **Confirmed — but the suggested fix was wrong.** It asked for a bump to 13; the bump to **12** is itself uncommitted (`git show HEAD` and `git show 7a7e387` both give 11), so 12 **is** the gen-33 stamp | The v12 note rewritten to cover both generations. Version left at 12 |
| F4 | `R` and `W` were read but never refused | **Confirmed, and worse than described.** `R = 0` is not the permissive end of the range: `rate >= R` is true for every peer on every window, so the release can *never* fire, every meeting ends at the cap, and the arm logs UNFINISHED EXCHANGE for exchanges that finished — the treatment silently not running under its own name | Strict `> 0` refusal on both, with the reasoning in the code and in the test docstring |
| F5 | The hold-cap bound comment argued from the lead | **Confirmed** — see the R-3 section above | Rewritten to the lattice argument, which covers the keeper |
| F6 | GROUP F reused mutation IDs M15–M18, already held by GROUP E | **Confirmed.** My own defect | GROUP F renumbered M23–M26; the file-header ledger now states one sequence for the whole file |
| F7 | Two stale claims, one in the harness comment and one about `cells_touched` | **Split.** The harness comment was real. The `cells_touched` half was not — the node never indexes it, so the suggested guard would have been dead code | Harness comment corrected; no guard added |

### Outstanding before launch

1. ~~**§5.1 emulator instrumentation**~~ — **done**, see the Part 0 emulator
   row. It changes the `hmr_sim` binary, which is fingerprinted.
2. ~~**Test-plan 7–9**~~ — **done**, see the Part 1 and Part 2 rows. 4 was
   already done (`TeamModelMode.*`). 5 and 6 follow R4's deferred half and
   stay deferred with it (see the correction under the consumer table in
   Part 3). 8's scan test of the inline predicate
   (`…ThePredicateIsALevelThenARateOverEveryReadablePeer`) is gone: the
   predicate now runs in `test_exchange_drain`, and the scan was retargeted at
   the node's call site.
3. **Set `R` and `W`** from Part 1's per-peer log before `rendezvous_drain_release`
   is enabled. It refuses to start otherwise, by design. **Open:** this waits on
   the `ts4_33` campaign's log, and that campaign runs with the drain off.
4. ~~**Final build → read the sha → restore the NV shim → `ctest` → commit all
   five fingerprinted repos → fresh campaign root.**~~ — **done 2026-09-23** on
   the run box at superproject `6e61300` (`explo_planner` `8efde01`): clean
   fast-forward pull, `ctest` 31/31, NV shim restored and verified by sourcing,
   `explo_planner_node` pinned at `b24df4d8cfba6160` in `ts4_chain33.sh`, fresh
   root `ts4_33_n2`. The first cell, `ts4_33_n2_mtare_rendezvous_r20_ttl0_seed1`,
   ran CLEAN with zero `-dirty`. Both of its meetings reconnected with the
   partner audible, so the silent-partner wait did not run; it is unit-tested
   only and not yet seen end to end. **Repeat this step after pulling `d7493c4`**
   (`explo_planner` `f0fb134`, comment-only). It moves `git_explo_planner` and
   the binary sha, so re-pin and move the probe root aside first.
5. ~~**Decide test-plan 7 at N=2**~~ — **decided 2026-09-23 and implemented**;
   see *Shipped 2026-09-23* below. The decision: a finished robot still comes
   to the meeting, exchanges maps, and then says it is leaving; its partner
   waits for it until it does.

**`--packages-up-to` still appears** in `docs/user_manual.md:178,182`,
`docs/ros_api.md:49,516`, `explo_planner/doc/dscovox_exploration_run.md:41,65`,
`explo_planner/doc/dscovox_exploitation_run.md:52,71` and
`explo_planner/doc/exploitation_plan.md:212`. Left alone on purpose. The
field and dry-run ones build in a scratch workspace with SCovox as an
underlay, where `--packages-up-to` reaches only the two planner packages and
pins nothing. `ros_api.md:49` is the one generic `<ws>` instance, and it is
the same stale line as the READMEs if anyone wants it consistent.

### Shipped 2026-09-23 — item 5: a finished robot still comes to the meeting

**The defect.** Since gen 32 a robot that finishes with an appointment standing
drives to the agreed cell (`keepAppointmentOnFinish`), but it announced
`finished` — and `DONE` — at saturation, before the drive. The partner's
barrier admits a finished peer as accounted for (`peerAccounted`, "never
arrives"), and the inbound veto works only while the peer is heard. A partner
that heard `finished` once and then lost the radio released after the settle
and left; the keeper reached an empty cell and stood out its cap. At N=2 the
same path opened the drain hold on nobody (the old item 5).

**The protocol, as decided.** A finished robot comes to the meeting,
exchanges, and then says it is leaving. Its partner waits for a finished peer
until that peer says so.

| Piece | What landed | Where |
|---|---|---|
| Publisher | `mode` stays below `HOMING` while the robot keeps its appointment (`appointment_manoeuvre_`); `DONE` once the manoeuvre ends, latched in `done_announced_`. `finished` is unchanged | `announcedMode`, `meeting_attendance.hpp/.cpp`; `publishTeamWorld` |
| Barrier | `manoeuvreReleaseEligible`'s appointment branch gains `&& !holdingForFinishedPeer()`: hold while some peer is `finished`, below `HOMING`, and not heard (not direct, not one-way, not in the closure). **A veto, not a change to `peerAccounted`** (Part 3's correction). Appointment manoeuvres only | `finishedPeerStillComing`; node `holdingForFinishedPeer` |
| Bound | `rendezvous_latched_hold_sec`, from `max(first barrier tick, t_meet)` — the R-3 floor. One stamp per appointment manoeuvre, kept across resumed legs (a per-leg restart would let a stop-short/resume cycle wait forever), cleared when the manoeuvre ends. Logged while it holds (throttled) and once when it runs out, above the release gate so the line is reached | node `doReturnSync`, `transitionTo`, `startReturnTo` |
| Drain | New `DrainStep::kAllPeersLeaving`: nobody present (`direct`/`via_relay`, counted independently of measurability) and every peer at `HOMING` or above → release at the first full window, logged as its own outcome, neither drained nor unfinished | `exchange_drain.hpp/.cpp`; node drain branch |

**Tests.** Run: `test_meeting_attendance` (12 tests — the keeper's
lifecycle, the DONE latch, exhaustive monotonicity under latched inputs, and
`finishedPeerStillComing` on a real `TeamModel`: silent, `HOMING`/`DONE`,
direct, one-way, unfinished, closure, relayed level, self/unconfigured);
`test_exchange_drain` gains six `AllPeersLeaving.*`, including a leaving peer
still **here** being read on its counter, measured and unmeasured.
Scan: `Gen33MeetingAttendance.*` (4). Mutations M44–M64, ledgers in each file.

**What it costs.**
- A partner now waits up to one cap longer for a finished peer that turned for
  home out of range without its `DONE` reaching it — at N=2 there is no relay.
  That is the price of not leaving a peer that is still coming.
- **With the drain on,** a no-show pays that bound and then the drain cap on
  top (the hold opens only once the veto runs out, and it examines nobody,
  so it runs to its cap as UNFINISHED — `NobodyReadIsNotEverybodyDrained`).
  Up to 2× the cap. With the drain off, as the campaign runs now, it is the
  bound plus the fixed settle.
- The manoeuvre classifier still labels a release after the bound ran out
  "reconnected" when the absent peer is finished — the accepted residual
  `reachablePeerCount` already names. The WARN line is what tells that meeting
  apart in the log.

**Runtime change.** `explo_planner_node` changes behaviour at the
appointment barrier and on the wire (`mode`), so `git_explo_planner` moves and
the campaign root must be fresh (§8). `TeamWorld.msg` changed in comments
only; `explo_planner_msgs` is rebuilt with it. No new parameter.

### Not shipped, deliberately

- **R-1 (the roll cap)** — refuted by its own falsification test.
- **R4's barrier half** (`peerAccounted` / `reachablePeerCount`) — neither is a
  leaf predicate; the blanket skip at `:8704` rejects them. The window stays
  open in gen 33 and is named in Part 3.
