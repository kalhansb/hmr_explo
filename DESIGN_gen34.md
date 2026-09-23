# Generation 34 design — working draft

**Status:** design agreed (Kalhan, 2026-09-23); adversarial review done (§9), Q51–Q64 open. Next: settle Q51–Q64, then build in the order of §8.10. Decisions are Kalhan's, taken one
question at a time; this document records each one as it lands.
**Baseline:** gen 33 — `explo_planner` `62a069b`, superproject `c3c4443`, both
committed locally and not pushed. The run box is on pin `5b3fe32b9d86c789`
(gen 33 without the cell-12 fix).
**Order of work:** arms revision (§4) → edge cases (§7) → design (§8).

---

## 1. What was asked

> what if we redesign from the top? for all arms? do you think we can get a
> better design, think like a senior software engineer

> I am open for many experiments if the behaviour is better with a less complex
> system. We can run only few seeds for each arm and N 2,3,4 rungs to check if
> it break, I want the exploit part intact as well, consider that in the design
> as well.

> ask about all the edge cases before you develop the design. let's make this
> comprehensive design phase

> We have to revise the arms, I see some things we have to update, let's start
> there

Requirements traced through this document:

- **G1** A less complex system.
- **G2** Better robot behaviour.
- **G3** Validation by a few seeds per arm on the N = 2, 3, 4 rungs, checking
  for breaks.
- **G4** The exploit part stays intact.
- **G5** Every edge case is settled before the design is written.

---

## 2. Settled before this generation

- **No comparison against earlier generations is required** (Kalhan,
  2026-09-22, DESIGN_gen33 header). The goal is a system that is right in
  principle. Gen-33 numbers serve only as a reference for "did it get better".

---

## 3. Decision log

Q9–Q12 (end-of-run meetings, pursuit catch-up exchange, rendezvous scheduled
only, hybrid's rule) are folded into the arms revision, §4. **The arms revision
closed on 2026-09-23** with Q8 and Q18–Q34 agreed.

| ID | Question | Recommendation | Decision |
|---|---|---|---|
| Q1 | What gets rebuilt | The control layer: the mission machine, one drive component and the new team parts (Presence, Exchange, Booking, Chase). Goal selection and exploit move out of the node with their logic unchanged; the scoring, allocation and exploit libraries are reused as they are. Detail: §3.1 | **agreed** (Kalhan, 2026-09-23) |
| Q2 | How it lands | A second node in the same package, old node frozen as the reference, run script picks by flag; built arm by arm with a few-seed check after each | **decided** (Kalhan, 2026-09-23): "build all arms and then do a few seed check in the run box" — all four arms built before any campaign, not arm by arm. Whether the old node stays beside it: Q36 |
| Q3 | Baseline | Re-run all arms on the new node | settled by §2 |
| Q4 | Broken vs better | *Refreshed after the arm revision.* Hard fails, zero allowed: crash or node death; a robot with no progress for > 120 s outside a planned wait (meeting window, exchange hold, dwell); a meeting logged "met" without direct contact and a completed exchange among all attendees; a robot standing short of the cell without contact; a state change reversed within its dwell more than twice in a row; the homing drive running out its 600 s bound (amended by Q48: a run still going at the `--duration` cutoff is recorded unfinished, not failed); any wait outliving its bound; a mechanism that never fires where it should (the mdp intercept, exchange completion). Primary metric: team exploration-done time; also coverage, exchange completion rate, time in meetings and chases, exploit yield in exploit-on cells | **decided** (Kalhan, 2026-09-23): no exploit metrics. Record **per robot** exploration-done time and home-arrival time, and **team** homing (the last robot home) alongside team exploration-done. Hard-fail list as recommended |
| Q5 | Validation matrix | *Refreshed:* 3 seeds × 4 arms × N = 2, 3, 4 = 36 cells, plus Q14's 12 exploit-on cells = 48 per round; same world, range and TTL; cell 12 always included | **decided** (Kalhan, 2026-09-23): the four arms are off, pursuit (mdp intercept first, trail fallback), rendezvous, hybrid; **no exploit** — 3 seeds × 4 arms × 3 rungs = 36 cells per round, cell 12 included |
| Q6 | Interfaces the new node may change | *Refreshed:* jsonl events become the only analysis contract (schema bump, scripts updated in the same change; ~20 text regexes move); log text free. TeamWorld **must** change now (Q18 counters; Q8 drops relayed masks), and scovox needs a send counter — a change outside `explo_planner` | **agreed** (Kalhan, 2026-09-23) |
| Q7 | Tests | *Delegated — "use the best testing methods".* Six layers: (1) characterisation tests that record today's exploit behaviour from the gen-33 code before any of it moves; (2) unit tests per component (Presence, Meeting, Booking, Chase, Leg, Exploit); (3) property tests over seeded random input timelines for the invariants — never "met" without direct contact and a completed exchange, every wait ends by its bound, no state reversed inside its dwell; (4) scenario replays of known failures (cell 12, the gen-26 chain, the gen-27 stand-short); (5) an in-process multi-robot harness — N pure cores, a toy 2-D world with a range-and-trees radio model — running thousands of seeded missions at N = 2–4 in seconds, so N ≥ 3 bugs surface before the run box; (6) mutation testing of the decision core, as in gen 33 | **delegated** (Kalhan, 2026-09-23) |
| Q8 | When a peer is present | *Revised after Q26:* heard **directly** within T s, from a beacon that never goes quiet; relays count for nothing anywhere, so the two-hop closure and relayed masks go. Remembered status never makes a peer present. The value gate keeps a relayed peer from being chased needlessly, since its map already flows (it finds nothing unshared) | **agreed** (Kalhan, 2026-09-23) |
| Q13 | Arm selection | One `arm` parameter (off, pursuit, rendezvous, hybrid — P2 makes mdp the default, so the `_mdp` arms go); everything else derived; stamp = that string | **agreed** (Kalhan, 2026-09-23) |
| Q14 | Exploit in the campaign | Arms campaign exploit-off; separate exploit-on check cells (1 seed × 4 arms × 3 rungs) must match gen 33 on the three trees. Kalhan to confirm whether the ts4 chain runs exploit-on | **decided** (Kalhan, 2026-09-23): **no exploit** in any validation cell. Exploit is verified by tests only (Q7 layers 1–3) — no campaign cell exercises it |
| Q15 | What "intact" covers | Exploit's own decisions identical; three boundary quirks fixed (held vantage not cleared by stand-down; "Target queued" after stand-down; stale exploit claim for ~0.3 s after homing starts). Scenario tests pin today's exploit behaviour before any code moves | **decided** (Kalhan, 2026-09-23): "if there is a target to exploit robots do the exploit action and do whatever it was doing before, keep explore and exploit as separate as we can" — exploit is an interrupt over everything, then the robot resumes its previous activity. See §4.4 |
| Q16 | Exploit vs reconnecting | A due meeting wins: finish a dwell in progress, leave a dwell-barrier wait, stand the target down with credit kept, resume after. A chase never interrupts exploit | **decided** (Kalhan, 2026-09-23): **exploit always wins** — over meetings, chases and homing (reverses the recommendation). Then narrowed by Kalhan's proposal to exploit only when all robots are connected (§4.4, Q39–Q42) |
| Q17 | Exploit when coverage is done | Finish the active target (bounded by its 300 s timeout), then stop | **agreed** (Kalhan, 2026-09-23) — but see Q35, which Q15 puts in tension with it |
| Q18 | What "maps exchanged" means | Acknowledged: each robot announces how much map it has published and how much it has received from each peer; the exchange with a peer is done when each side has received everything the other had published when contact began | **agreed** (Kalhan, 2026-09-23) |
| Q19 | Following a caught peer (P4) | The peer carries on as it was; the follower keeps about 10 m behind (outside the 6 m proximity resume, inside good signal). Following counts toward the chase's hard time limit. Losing it resumes the chase | **agreed** (Kalhan, 2026-09-23) |
| Q20 | The 6-chase cap (P1) | Drop it; keep the cooldown after each chase and the value gate | **agreed** (Kalhan, 2026-09-23) |
| Q21 | The 900 s last-contact cut-off (P3) | Keep it, as the outer sanity bound it was sized for | **agreed** (Kalhan, 2026-09-23) |
| Q22 | Meeting arrival vs the 5 m proximity pause (R5) | Stop on reaching the cell, or on coming within about 10 m of an attendee already there, whichever first; then hold until the maps are exchanged | **agreed** (Kalhan, 2026-09-23) |
| Q23 | End-of-run meetings (R8) | Keep a booking only while another attendee may still be exploring; if every other attendee is known finished, go home instead — everyone meets at home anyway | **agreed** (Kalhan, 2026-09-23); **amended by Q49**: a finished robot goes home only after a meeting where every robot reports finished |
| Q24 | S2's finished-counts-present clause | Drop it (as Q8): under Q18 a meeting cannot end without hearing the peer, so the clause no longer does anything useful and is the cell-12 root | **agreed** (Kalhan, 2026-09-23) |
| Q25 | Chance encounters with a booking (R3) | Keep exploring while the exchange runs. Cancel the booking only when the maps are exchanged with **every** robot on the booking; a partial encounter (2 of 3) keeps it. *(First wording said "when the exchange completes", which at N ≥ 3 cancels on a pair and strands the third robot at the cell — caught by Kalhan)* | **agreed** for N = 2 (Kalhan, 2026-09-23). **N > 2 amended by Kalhan:** "robots should meet together always, because by meeting time they may have more to share" — relayed exchange does not count, and a chance encounter never replaces the meeting; see Q26 |
| Q26 | The N > 2 meeting rule (Kalhan's amendment to Q25) | At N > 2 a booked meeting always happens — no chance encounter cancels it. At every meeting, "together" means every attendee is in **direct** contact with every other and has exchanged maps with each directly; relays count for nothing. Q22 is amended to match: keep closing in on the cell until in direct contact with every attendee present | **agreed** (Kalhan, 2026-09-23) |
| Q27 | An exchange that never completes | In contact but the counters make no progress for 120 s, or 600 s in total: leave, logged "exchange incomplete", and carry on as if the meeting ended. Both numbers re-sized from the first validation round | **agreed** (Kalhan, 2026-09-23) |
| Q28 | Contact lost before the exchange completes | Rendezvous: back to "not met yet" — drive on to the cell, or wait there. Pursuit: the chase resumes (Q19) | **agreed with a change** (Kalhan, 2026-09-23): "if contact lost if already at cell try to move and connect" — how: Q32 |
| Q29 | A teammate that never arrives, and a robot stuck on the way | One shared meeting window: a meeting at time T is over at T + 120 s for anyone who has not made contact. Then book the next occurrence on the timetable and explore on. Escape moves are unlimited inside the window. The same window replaces the finished robot's 420 s hold | **agreed** (Kalhan, 2026-09-23) |
| Q30 | A followed peer that leaves | Follow wherever it goes — to a meeting, home, or after someone else; only the exchange ends the follow, and the chase's hard time limit bounds it | **agreed** (Kalhan, 2026-09-23) |
| Q31 | Hybrid's one chase per booking (H2) | Drop it, as Q20 dropped the per-run cap: the cooldown, the value gate and the due booking already bound chasing | **agreed** (Kalhan, 2026-09-23) |
| Q32 | How a robot at the cell reconnects (Q28) | Drive toward the lost peer's last heard position — seconds old, so near the cell — and stop on contact. Reach it with no contact: go back to the cell and wait. Bounded by Q27's 120 s no-progress rule | **agreed** (Kalhan, 2026-09-23) |
| Q33 | Which peer to chase when several are missing (N > 2) | The value gate prices every missing peer and the chase goes to the best-priced one — one function answers both. Today the gate prices the nearest peer while the chase goes to the freshest, a documented drift | **agreed** (Kalhan, 2026-09-23) |
| Q34 | Two robots chasing each other | No special rule: each heads for the other's area, contact on the way turns both into followers of each other, they hold together until the exchange completes | **agreed** (Kalhan, 2026-09-23) |
| Q35 | A target released while homing | *Revised for the all-connected gate.* A homing robot is not with the whole team, so it does not exploit on the way. Once every robot is home, the team is together: exploit the pooled targets from there, then return home. Q17 still covers an episode that is already running when coverage finishes: finish it, then go home | **agreed** (Kalhan, 2026-09-23) |
| Q36 | Does the gen-33 node stay beside the new one? | Yes: frozen, selectable by flag, deleted once the new node passes its first run-box round. It costs nothing to keep and gives an A/B reference when a cell looks odd | **agreed** (Kalhan, 2026-09-23) |
| Q37 | Clocks while exploit interrupts | Shared deadlines keep running (the meeting window, the timetable); private budgets pause (a follow's time limit, the exchange's no-progress timer). On resume the suspended activity re-checks its own conditions; it never just carries on. Under the gate the only things exploit can interrupt are ones done together: a meeting, an N = 2 follow, exploring after everyone met by chance, and waiting at home | **agreed** (Kalhan, 2026-09-23) |
| Q38 | What exploit and explore still share | Keep one read-only map of places known unreachable (today's failed-goal blacklist, which exploit's vantage filter reads). Split the step counter: exploit steps stop counting against the exploration step budget | **agreed** (Kalhan, 2026-09-23) |
| Q39 | What "all connected" means | Every pair of robots is in **direct** contact at the same moment. This is the same test a meeting uses to count as met (Q26), with no relays (Q8). At N = 2 it just means the pair is in contact. It gates only the **start** of an episode (Q42) | **agreed** (Kalhan, 2026-09-23) |
| Q40 | How a robot learns about a tree | Keep the scheduled list, so every arm and seed sees the same trees at the same times. A robot learns a tree only once the tree is released **and** the robot has come within sensor range of it. It carries the tree and shares it over the gated radio. The ungated `/exploration/targets` bus goes; the shared list rides the TeamWorld change Q6 already requires | **agreed** (Kalhan, 2026-09-23) |
| Q41 | What the team exploits when together | Pool every tree any member knows. Exploit the pool as one episode, nearest tree first, using today's claims and dwell barrier unchanged (G4). Then each robot resumes by re-checking. The next all-together moment may be minutes away, and exploit always wins (Q16) | **agreed** (Kalhan, 2026-09-23) |
| Q42 | Contact lost after an episode starts | The episode carries on. The gate guards only the start, and claims made while together stand. Bound the dwell barrier's wait: today 0 means unbounded. Set it to 120 s, the same limit as the exchange (Q27). This is a parameter value, not an exploit code change | **agreed** (Kalhan, 2026-09-23) |
| Q43 | How long a peer can go unheard before it counts as gone (E1) | One window for every rule, 10 s: a peer is present if heard directly in the last 10 s. Today it is 5 s at a 1 Hz beacon, so a robot at the edge of range flickers in and out. Meetings, follows, exchanges and the exploit gate would all thrash, and Q4 counts reversals as a hard fail. Re-sized from the first validation round | **agreed** (Kalhan, 2026-09-23) |
| Q44 | A robot cannot reach the meeting cell (E12) | Keep trying at every slot. Each attempt is bounded by the window (Q29), and a route often opens as the map grows: the code's own note says an unreachable reading is as often unexplored ground as blocked ground. R1 already avoids cells the proposer's map shows as unreachable | **agreed** (Kalhan, 2026-09-23) |
| Q45 | N > 2: one attendee missing while the rest finish exchanging (E13) | Those present finish their exchange, then wait for the missing robot until T + 120 s, then book the next slot. It may be late but coming; leaving early means it arrives to an empty cell. Cost: up to 120 s for each meeting someone misses | **agreed** (Kalhan, 2026-09-23) |
| Q46 | A finished robot between meetings (E17) | It waits at the meeting cell, since it has nothing left to explore and going home and back wastes travel. It goes home once it knows every other robot has finished (Q23). Same at N = 2 | **agreed** (Kalhan, 2026-09-23) |
| Q47 | When the meeting plan (cell and timetable) may change (E11) | Only at the start and at a meeting where all N robots are together. It must be agreed by all of them before anyone leaves; the exchange takes far longer than the agreement, so it fits. If the agreement does not complete, the standing plan stays. A partial meeting keeps the standing plan, so the missing robot still knows where to go. This replaces today's continuous re-proposal while connected (`maintainRendezvousProposal`, 237 lines) | **agreed** (Kalhan, 2026-09-23) |
| Q48 | The campaign horizon (E36) | The horizon stays a measurement cut-off, not a robot rule, and robots do not know it. Q4's "not home by the mission limit" becomes "the homing drive ran out its 600 s bound". A run still exploring at the horizon is censored, not failed | **agreed** (Kalhan, 2026-09-23). Q4 amended to match |
| Q49 | A finished robot and the last meeting (found while writing §8) | Q23 plus Q46 can deadlock. At N = 2, A finishes first and waits at the cell (Q46). B finishes later, knows A is finished, and goes straight home (Q23). A never learns that B finished, so it waits at the cell for the rest of the run. **Amend Q23:** a finished robot goes home only after a meeting where every robot reports finished. A robot that finishes still attends the next meeting, even if it knows the others are finished. Then everyone knows at once and they all go home together. Off and pursuit are unaffected, since they have no meetings | **agreed** (Kalhan, 2026-09-23) |
| Q50 | Waiting at home with unexploited trees (Q35) | Wait for the whole team with no extra limit. Every teammate's exploring and homing are already bounded, so this wait can only hang after a teammate has already hard-failed. Only exploit-on runs ever wait: with no trees, a robot reports done on arrival. The hard-fail detector counts it as a planned wait | **agreed** (Kalhan, 2026-09-23) |
| Q51 | How a robot knows the maps have crossed (§9, blocker) | Q18 compared running totals: one lost map piece keeps a pair's totals apart for the rest of the run, so every later exchange with that peer ends on the 120 s give-up (dscovox's count never resets and nothing is resent; `map_agreement.py` measured 0.1–3.6 % loss). Done was also one-sided. **Number the map pieces.** scovox stamps a sequence number on each frame; dscovox reports, per source, the newest number received and the gaps. Each beacon carries my newest sent and, per peer, my newest received. The exchange with a peer is done when **both** sides have received up to the number the other had sent when contact began. Gaps are logged as loss and never block done. No `exchanged` field is needed: both halves are read from the two beacons | open |
| Q52 | Beacons lost while a map backlog drains (§9) | The emulator drains queued map data first from one shared airtime budget and drops best-effort messages while it is empty. After a long split every robot's beacons can vanish for tens of seconds, longer than the 10 s window. **The emulator keeps a small airtime reserve for best-effort messages**, so the reliable drain never takes the last slice. Links there are symmetric (`PairKey`), so presence stays "beacon heard". Record per-link `drop_airtime` in the smoke round with a threshold. Alternative: count arriving map data as hearing the peer, which leaves the radio alone but cannot confirm mutual contact (E2) while beacons are starved | open |
| Q53 | How long a robot holds at a meeting (§9) | Three bounds overlap (window 120 s, no progress 120 s, total 600 s), a new contact restarts the no-progress clock so a flickering link holds forever, and Q32's drive is bounded by a record that ended with the contact. **One patience clock per booking:** 120 s from the last progress (a map piece received, or a missing attendee heard); a new contact does not reset it. A 600 s backstop per booking, not per contact. **Q32 kept, one mover:** of a pair that lost each other at the cell, only the higher id drives to the other's last heard position; the lower id holds | open |
| Q54 | The meeting plan can split the team for good (§9, blocker) | At N = 3, robot 0 sees both echoes and leaves with the new plan; the other two lost each other before seeing each other's echo and keep the old one. Different cells for the rest of the run, since only a full meeting changes the plan. The design also lost today's 60 s start hold (`node:2725`, `4814`) and proposes once. **The plan carries a version and robot 0 is the only proposer; a robot adopts any higher version the moment it hears it, from any beacon, at any time.** Keep the 60 s start hold in every arm; robot 0 re-proposes each tick while everyone is in contact and no plan is agreed. A partial meeting keeps the standing plan (Q47) | open |
| Q55 | What counts as "met" at N > 2 (§9) | One trunk kills a link (70 dB). With robots stopped about 10 m apart at the cell, a reviewer estimates some pair is blocked in about 40 % of N = 3 meetings; at a cell with a trunk near the centre every meeting is partial, and the cell never moves. **Met = every pair has completed a direct exchange at some moment inside this meeting**; a pair that finished and then lost each other still counts. Amends Q26 | open |
| Q56 | When to leave and which slot (§9) | "Due" is recomputed each tick, so it turns false as the robot closes in (today it latches). After a split each robot books the first slot **it** can make, so at N = 3 they pick different slots. The lead assumes 0.5 m/s in a straight line; robots measure 0.40 m/s on longer paths and arrive near the window's end. **(a) Departure is stored and clears with the booking. (b) Book the first slot every robot can make, from each robot's last heard position and time. (c) The lead uses 0.40 m/s on the path distance from the cost grid, × 1.2** | open |
| Q57 | Finished robots at the end of the run (§9, blocker) | (a) After the all-finished meeting, contact drops on the walk home, Book makes a new booking and Meet outranks Home: everyone turns back. Nothing stores "everyone finished". (b) One robot that never makes a meeting keeps the finished ones at the cell to the horizon, which counts as censored (the cell-12 shape). **(a) Store `team_finished`, set at a meeting where every beacon says finished; with it set nothing is booked and Home holds. (b) A finished robot goes home after 2 consecutive slots at which some robot never showed; the team can also learn "all finished" at home (homes are 3 m apart). (c) Q50's wait at home gets the 600 s homing bound** | open |
| Q58 | When an exploit episode may start (§9, exploit-on) | The gate opens as soon as all are in contact, before the meeting's exchange and plan agreement, and exploit outranks the meeting; each robot opens it on its own view, so at N = 3 two can start while the third explores; a homing robot passing a teammate starts that teammate alone. **Start is a team event:** a beacon says "ready" when, from that robot's view, everyone is in contact, nobody is homing, and any meeting in progress is met. The episode starts when every beacon says ready; the pool rides the beacons, so all pool the same trees | open |
| Q59 | Exploit rules that changed exploit's decisions (§9) | **Keep today's behaviour in all six:** (1) Q41: one shared tree order on every robot, by tree id, not nearest first (today's queue is arrival order; nearest-first splits a team across trees). (2) Q42: keep the barrier at 0; it already ends on the 5 s claim expiry of a silent peer and on the 300 s target timeout. (3) Q38: a failed vantage drive writes the blacklist, as today. (4) A tree leaves every pool only on a COMPLETE from anyone; PARTIAL stays local. (5) Q17 over §8.4 row 1: finish the active target, go home, the rest is exploited at home (Q35); remove the stand-down call from that path. (6) The pool is topped up each tick during an episode | open |
| Q60 | How exploit is pinned before it moves, and where builds run (§9) | The gen-33 node defines `main()`, so no test can link it (`CMakeLists.txt:309`), and Q36 froze it. **One exception to Q36: move `main()` to its own file, no logic change.** A gtest then constructs the gen-33 node in-process, feeds it scripted map, pose and peer intents on a simulated clock, and records exploit's decisions; the same scripts run against the new code. Builds and unit tests run here in the `hmrexplo:humble` container if the run box is also on Humble; simulations stay on the run box | open |
| Q61 | What the checker treats as a failure (§9, blocker) | (a) Q48 makes a run at the horizon censored, so a hung finished team passes; today `gate_g8.py:846` fails every run not ending all-done. (b) The detector trusts the robots' own events. (c) Run control reads the CSV `state` column and log text (`run_explo_sim_rviz.sh:2040`, `2125-2151`). **(a) At the horizon, a run fails if every robot has finished exploring and some robot is not home; it is censored only while someone still explores. (b) Every met, exchange-done, chase-contact and all-connected event is checked against the simulator's `link_states.csv`. (c) The new node keeps the CSV `state` column (DONE when done) and the "selected goal" and "Exploration complete" lines, pinned by a contract test, so one script runs both nodes; the hang threshold is resized** | open |
| Q62 | Exploit in the run-box round (§9; reverses Q14) | The exploit path is mostly new wiring (sighting, trees in beacons, pooling, the gate, done at home) and no cell runs it; a gate that never opens gives zero episodes and nobody sees it. **Four smoke cells only: 4 arms × N = 3 × 1 seed, exploit on; pass = at least one episode and trees done = trees sighted.** No metrics, not part of the 36 | open |
| Q63 | Simplifications (§9, G1) | 13 states became 8 activities plus about 17 sub-phases inside components; about 170 parameters became about 145. **(1) Booking and Chase each own an explicit phase; Mission reads only `active()`; Follow folds into Chase. (2) Delete row 6: a finished robot's booking is due at once, so Meet covers it. (3) Delete Q37's pause: one clock. (4) One speed, one safety factor, one stop distance (`follow_distance_m` and `meeting_attendee_stop_m` are both 10 m). (5) A booking clears as met or window-closed, with partial and missed as labels** | open |
| Q64 | Chase details (§9) | **(1) Q33: chase the peer the gate priced (`leg_peer_id`); the gate prices only the nearest locatable peer (`reconnect_gate.cpp:110`) and stays unchanged. (2) Hybrid: no chase when the next departure is under 120 s away, so a chase is not always cut off by Meet. (3) The follow point needs clear line of sight to the peer (the vantage planner's test), else closer, down to 6 m; one trunk kills a link. (4) The gate keeps failing open; refusals are logged and counted, bounded per outage by the 900 s cut-off and the 90 s cooldown** | open |


### 3.1 Q1 in detail: what gets rebuilt

**Today's node by concern** (`explo_planner_node.cpp`, 12,719 lines; method
sizes measured 2026-09-23). The package also holds about 11,700 lines of
helper libraries, most of them pure C++ that is already tested.

| Part of the node | Lines | Fate |
|---|---|---|
| Constructor: 175 parameter declarations, subscriptions, timers | 2,569 | **Rebuilt, small.** Declares only what the components use, with `dp()` kept for `equiv_gate`. One `arm` parameter (Q13) |
| Class declaration: members and their notes | 1,780 | Goes with the code it describes |
| `doPlan` | 1,089 | **Split.** About 850 lines of goal selection move **unchanged** into `selectGoal(inputs) → goal`. The ~240-line reconnect prelude is **deleted** (the mission machine replaces it). The exploit check becomes the gate (Q39) |
| Reconnection: booking, meeting hold, chase, proposal (`doReturnSync` 474, `maintainRendezvousProposal` 237, `armAppointment` 211, `startPursuit` 209, …) | 2,103 | **Rebuilt** as Booking, Exchange, Chase |
| Driving: navigate, five budget and watchdog copies, homing, escape and retrace, proximity hold | 1,406 | **Rebuilt** as one Leg |
| State machine: `tick`, `transitionTo`, coverage latch | 532 | **Rebuilt** as Mission |
| Team messages: TeamWorld publish and drain, heartbeat | 703 | **Rebuilt.** TeamWorld changes anyway (Q6, Q18, Q40) |
| Exploit (`doExploitPlan` 331, `doExploitDwell` 161, claims, vantage hold, …) | 813 | **Moved, decisions unchanged.** Only its connections change: the gate and the pooled targets feed it; it hands back "done". Characterisation tests pin it before and after (Q7 layer 1) |
| Goal-selection support: trajectory scoring, allocator call, map load, cell-world update | 512 | **Moved unchanged** with `selectGoal` |
| Logging, metrics, visualisation | 609 | **Kept.** Log lines the analysis reads keep their wording, or the scripts move to jsonl (schema 13) |

**Libraries reused as they are**
- Exploration: candidate_generator, fov_evaluator, cost_grid, map_cache,
  cell_world, global_allocator, coordination, separation,
  failed_goal_blacklist.
- Exploit: target_queue, vantage_planner.
- Logging: experiment_log, metrics_logger.
- Other: fleet_identity, proximity_guard (inside Leg), rendezvous_scheduler
  (R1, R2 unchanged), pursuit_predictor (the intercept, P2), reconnect_gate
  (value gate, Q20, Q33).

**Libraries replaced**
- team_model: presence becomes direct-only (Q8).
- exchange_drain: replaced by Exchange (Q18).
- meeting_attendance: its "finished counts as present" rule goes (Q24).

**New components.** Each is pure C++ with no ROS, tested alone, and runs in the
in-process harness (Q7 layer 5).

| Component | Answers | From |
|---|---|---|
| Presence | Who do I hear directly, right now? | Q8 |
| Exchange | Has each peer's published map fully arrived, stalled, or been given up on? | Q18, Q27 |
| Booking | The next meeting: place, time, attendees, window until T + 120 s | R1–R9, Q22–Q29 |
| Chase | Whom to chase, where (intercept, then trail), following at ~10 m, when to stop | P1–P5, Q19–Q21, Q30–Q34 |
| Leg | Drive to a point: arrival from pose, one budget, one watchdog, stuck → escape, proximity hold | S5, R5 |
| Mission | explore, meet, chase, follow, exploit (interrupt), home, done; suspend and resume by re-checking | §4.4 |

Each arm is the same Mission with different parts switched on: off (neither),
pursuit (Chase), rendezvous (Booking), hybrid (both).

The ROS node becomes a shell. It turns messages into inputs, calls Mission
once per tick, and turns the result into goals, messages and log lines.

**Unchanged outside the planner:** hmr_sim, hmr_localisation, simple_nav_3d,
and scovox/dscovox apart from the send counter (Q6). The target scheduler is
unchanged too; its list becomes a sim stand-in for a camera, filtered by
sighting inside the node (Q40).

---

## 4. Arms revision

### 4.1 Today's behaviour (the baseline being revised)

From the gen-33 code. IDs are stable; revisions in §4.2 refer to them.

**Shared by every arm (S)**
- **S1 Map sharing.** Robots within radio range (up to 30 m, less with trees in
  the path) exchange maps continuously. The map topic queues while the link is
  down and delivers on the next contact. True in every arm, off included.
- **S2 Present.** A peer counts as present if I hold a live claim from it, I
  hear its team message directly, or it has announced it is finished — even if
  I cannot hear it. The last clause is the root of cell 12.
- **S3 Proximity.** A driving robot pauses within 5 m of a peer and resumes at
  6 m.
- **S4 End of run.** In campaigns every robot drives back to its start pose
  at any terminal ending — coverage done, step budget, or a meeting give-up —
  because `run_campaign.sh` sets mission return on by default
  (`MISSION_RETURN_FLAG="1"`). The single-run script defaults to off, and then
  the robot stops where it is. *(Corrected: the first draft said "no homing in
  the sim", reading the single-run default.)*
- **S5 Exploit first.** While a tree target is open, no reconnect can start.
  The campaign driver runs exploit-off, so this matters only in exploit-on runs.

**Off (O)**
- **O1** Never leaves exploring to reconnect. Maps merge only on chance
  encounters.

**Pursuit (P)**
- **P1 Trigger.** The team was together at least once, a peer has been unheard
  for the silence gate, the radio confirms the link is down, and the value gate
  says reconnecting finishes sooner than staying apart and I hold map the peer
  lacks. At most 6 chases per run, cooldown after each.
- **P2 Target.** The missing peer heard most recently: its last announced goal,
  then its last known position.
- **P3 Limits.** 600 s per chase; no chase if the last contact is older than
  900 s.
- **P4 Caught up.** Once the peer, or the whole team, is heard for 6 s: straight
  back to exploring. No wait for the maps to exchange.
- **P5 Chase failed.** Back to exploring (up to 6 times per run); after that,
  stand in place for the team up to 30 s, then explore.
- **P6 End of run.** No chase; the robot drives home (S4). Mission return
  pre-empts the terminal dispatch.

**Rendezvous (R)**
- **R1 Agreement.** While every robot hears every other directly, robot 0
  proposes a meeting cell and a recurring meeting time; agreed when every robot
  echoes it.
- **R2 Booking.** When the team splits, each robot books the next occurrence it
  can reach in time. One booking per split.
- **R3 Before leaving.** Keep exploring. If the team comes back together on its
  own for 6 s, the booking is cancelled.
- **R4 Leaving.** Depart when travel time × safety factor says it's time.
- **R5 The drive.** Arrive within 1.5 m. If the team comes together on the way,
  stop there and treat it as the meeting; drive on if it lapses. Stuck: up to 3
  escape moves, then move the booking to the next occurrence and go back to
  exploring; within 10 m of the cell a stuck robot waits where it is.
- **R6 The wait at the cell.** Until every peer is present or reachable through
  a relay, none is still driving in, and no finished-but-unheard peer is still
  coming; confirmed for 6 s. **Mid-run there is no time limit**
  (`rendezvous_appointment_wait_sec` = 0 in the sim, which the node treats as
  unbounded): a robot whose peer never comes waits until the run ends. A robot
  whose coverage is done waits at most 420 s past the meeting time.
- **R7 Exchange.** Hold 30 s for the maps, then agree the next meeting (waiting
  up to 30 s), then explore on.
- **R8 End of run.** A robot that finishes with a booking keeps it (R5–R7),
  then drives home. The latched wait is at most 420 s past the meeting time.
  Cell 12 was two finished robots doing this.
- **R9** Never chases, never reacts to silence.

**Hybrid (H)**
- **H0** No booking (for example the team never agreed one): behaves exactly as
  pursuit.
- **H1** Everything in R1–R8.
- **H2** While a booking exists but isn't due, P1's trigger may start one chase
  per booking (P2–P3).
- **H3** A due booking interrupts the chase: go to the meeting.
- **H4** Chase failed: explore until the booking is due; never stands and waits.
- **H5 End of run.** As R8; with no booking, it drives home.

**mdp variants (M)**
- **M1** Only P2's first target changes: a predicted intercept on the peer's
  broadcast route.

### 4.2 Revisions

Kalhan, 2026-09-23. Agreed as they stand: S1, S2 (but see Q24), S3, S5, O1,
R4, R6, R9. Hybrid takes every P and R revision below; its structure (H0–H5)
stays — "this is correct behaviour".

| ID | Today | Revised to |
|---|---|---|
| S4 | Campaigns drive home at the end | No change: every arm ends at home (P6) |
| P1 | At most 6 chases per run | **No cap.** The cooldown after each chase and the value gate stay (Q20) |
| P2 | Trail by default; mdp intercept only in the `_mdp` arms | **The mdp intercept is the default target; the trail (last goal, last pose) is the fallback.** The `_mdp` arms stop being separate arms |
| P3 | 600 s per chase; no chase past 900 s since contact | **The chase ends when its goal points run out; time is the outer hard limit.** The 900 s cut-off stays as the sanity bound on starting (Q21) |
| P4 | Heard for 6 s → straight back to exploring | **Stay with the peer, following it, until the maps are exchanged; only then go back to exploring** (Q18, Q19) |
| P5 | Explore on (6 per run), then stand and wait up to 30 s | **Chase failed → back to exploring. Nothing else** |
| P6 | Home | **Home** (unchanged) |
| R1, R2 | — | Agreed after the plain explanation (§4.3) |
| R3 | Booking cancelled after 6 s together | **Booking cancelled only once the maps are exchanged** (Q18, Q25) |
| R5 | Arrive within 1.5 m; team together en route → stop there; 3 escapes, then roll the booking and explore | **Together en route counts only once the maps are exchanged; hold until then. Stuck → escape moves until unstuck, then carry on finding the other robots.** Arrival must not fight the 5 m proximity pause (Q22) |
| R7 | Hold 30 s, then agree the next meeting | **Hold until the maps are exchanged**, then agree the next meeting |
| R8 | Finished with a booking → keep it, then home | **Keep the booking only while another attendee may still be exploring** (heard not finished, or not heard); if every other attendee is known finished, go home (Q23) |
| R3 (N > 2) | Booking cancelled when the team is back together | **Never cancelled early: the team always meets** (Q26) |
| R6 | Wait until every peer is present or reachable through a relay; no mid-run limit | **Done when every attendee is in direct contact with every other and has exchanged maps with each directly** (Q26). **A meeting at T is over at T + 120 s for anyone without contact; book the next slot and explore on** (Q29). An exchange with no progress for 120 s, or 600 s in all, ends as "exchange incomplete" (Q27). Contact lost at the cell: move to reconnect (Q28, Q32) |
| R5 (stuck) | 3 escapes, then roll the booking | **Escape moves unlimited inside the meeting window** (Q29) |
| H2 | One chase per booking | **No per-booking cap** (Q31) |

### 4.3 R1 and R2 in plain terms

- **R1.** While everyone can hear everyone, robot 0 picks a meeting place and a
  timetable — "cell 23, every N minutes from time T". Each robot repeats it
  back; once all have, it is agreed. Like fixing "same place, on the hour"
  before a group splits up.
- **R2.** When the group loses contact, each robot writes down the next time on
  that timetable it can still reach. Robots that can make the earliest one all
  pick the same one, because the timetable is shared. One booking per split;
  the next split books again.


### 4.4 Exploit: where targets come from, and exploiting only when all are connected

**Finding (Kalhan's question, 2026-09-23): today robots learn about trees
from an oracle.** In exploit-on runs a single `target_scheduler` node releases
a preselected list of trees on the clock. It publishes them on
`/exploration/targets`, and the radio emulator neither gates nor relays that
topic (`run_explo_sim_rviz.sh:141`). The result: every robot learns every tree
the moment it is released, wherever it is and whether or not it has ever seen
it. The alternative source, the live `tree_detector`, runs one per robot on the
robot's own map. It publishes on the same ungated topic, so one robot's
detection still reaches every robot at once. Meanwhile the ring claims and the
dwell barrier's "staged" flag ride the gated intents stream. So a robot can
learn about a tree from a peer it cannot hear, then fail to coordinate with
that peer on it. The first impact review assumed "the whole team converges on
each tree", and that assumption is this artefact.

**Decided (Kalhan, 2026-09-23): exploit only when all robots are connected.**
Q39–Q42 settle its edges (all agreed as recommended).

| Rule | Effect under the gate |
|---|---|
| Realism | Trees become first-hand (Q40). They travel over the gated radio and are pooled when everyone is together. The ring claims now run while every claimant can hear the others, which is what they were built for |
| R4, R6, Q26, Q29 — meetings | At N > 2 a meeting brings everyone together in direct contact, so it is the main moment to exploit. Exploit no longer causes missed meetings: it runs **at** the meeting instead of pulling a robot away from it. The exchange carries on in the background while they are together; the meeting counts as met once the exchange is done (Q18), and the next slot is booked there (R2) |
| P1–P4, Q19 — chases | A chase means someone is missing, so exploit cannot interrupt a chase. At N = 2 a caught peer means both robots are together, so exploit can interrupt the follow (Q37) |
| S4, Q17, Q35 — homing | No exploiting on the way home. When everyone is home, exploit the pooled trees from there, then return home. Team homing time includes that work |
| Q16 — exploit always wins | Still true, but only at all-together moments. Most of the conflicts in the first review disappear |
| Step budget, blacklist | Unchanged (Q38) |
| Q4 hard fails | The dwell barrier's wait becomes bounded (Q42) and counts as a planned wait |

**The cost: how much gets exploited now depends on the arm.** Off never
reconnects on purpose, so its trees wait for chance encounters and for the
end, at home. Pursuit at N = 3 or 4 chases in pairs, so all robots are rarely
together. Rendezvous and hybrid bring everyone together at every meeting (Q26).
This does not touch validation, which runs with no exploit (Q14). It will show
up in any later exploit-on study as an effect of the arm, which is arguably the
point of the arms.

Two principles still hold for every interrupt:

1. **Resume by re-checking, never by just carrying on.** Time has passed, so
   the suspended activity picks up through its own rules.
2. **Shared deadlines keep running; private budgets pause** (Q37).

---

## 5. Facts from the code survey that bound the design

- **Node.** 12,719 lines, 13 states, 201 parameters. `doPlan` is 1,094 lines;
  about 240 of them are appointment and mid-run reconnect logic that runs before
  goal selection. Drive budget and watchdog code exists in five copies with
  different formulas, tolerances and failure sinks. Goals are open-loop:
  simple_nav_3d gives no feedback, so arrival is judged from the pose alone.
- **Exploit.** EXPLOIT_PLAN → NAVIGATE → EXPLOIT_DWELL → LOG_STEP. While a
  target is open it blocks every reconnect. The coverage latch can end a run
  mid-dwell. No node-level test covers the exploit states.
- **Comms.** Path loss with a cliff at 30 m and heavy tree attenuation. The
  emulator forwards nothing multi-hop. The map topic is reliable and queued;
  team and intent messages are best-effort and dropped while the link is down.
  The node's relay closure reaches two hops only.
- **Contracts.** A new TeamWorld field needs every robot rebuilt or peers vanish
  silently. About 20 log-text regexes in the analysis scripts depend on exact
  wording. jsonl schema is 12; the gate pins disagree (12 / 11 / 9). Any commit
  to `explo_planner` splits the comparability key. `equiv_gate` reads defaults
  only from `dp("name", literal)`.
- **Sim overrides the yaml.** Arrival tolerance 1.5 m (yaml 4.0); mid-run wait
  30 s (node 240); pursuit budget 600 s (node 240); appointment wait 0 =
  unbounded.
- **Campaign driver defaults.** `run_campaign.sh` runs every cell with mission
  return on and exploit off (`EXPLOIT=0`).
- **Map exchange, from gen 33 (DESIGN_gen33 §2.5, §3.2).** In the gen-32
  campaign 93.2 % of meetings (110 of 118) exchanged nothing measurable, and a
  meeting never contained two merges. Where a merge did arrive, the median was
  55.9 s after reaching the meeting (p90 195.9 s) — against a 30 s hold, so
  meetings released before the exchange they were waiting for. Gen 33 added a
  per-peer arrival counter in dscovox (`ScovoxFusionCounters`), but "done" is
  still undefined: a "gone quiet" rule cannot tell a finished exchange from a
  blackout, because both read zero. A connected link can still deliver nothing
  (bit-error, airtime and queue-overflow drops in the emulator).
- **Why the pursuit limits exist.** The 6-chase cap counts dispatches, not
  failures — six successful chases use it up too — and it has no reset. Its own
  note calls it a dose term that weakens pursuit sooner at N = 4, and says it
  never exceeded 1 of 6 in the banked cells (300 s cells, not 3000 s). The
  900 s cut-off was sized to measured outages of 186–861 s: at the earlier
  180 s, pursuit declined every time it was asked.

---

## 6. Design rules from 33 generations of failures

Each rule answers a recurring root cause; the count is roughly how many
recorded defects it explains.

| Rule | Root cause it answers | ~Defects |
|---|---|---|
| One function per concept; a negation is derived from it, never written out by hand | Predicates meant to match drift apart (cell 12 is one) | 20 |
| Every flag has one set site, one clear site and a declared resting value | Latches cleared in the wrong place, too early, or never | 15 |
| Every mechanism counts its firings; the campaign gate checks they are non-zero | Mechanisms silently inert while looking healthy | 12 |
| Measurement is checked where it matters most | Measurement wrong or missing exactly when it matters, often flattering | 12 |
| A team decision reads only inputs every robot shares | Per-robot inputs to a team decision | 10 |
| Presence comes from a first-hand beacon that never goes quiet | Liveness inferred from a proxy | 9 |
| Every wait has a bound; a declined helper never retries silently | Livelocks and unbounded waits | 9 |
| One clock domain per quantity | Sim vs steady vs epoch vs mission time | 9 |
| Validate at N ≥ 3 | N = 2 hides N ≥ 3 bugs | 8 |
| One knob answers one question | A knob serving two purposes | 5 |

---

## 7. Edge cases

Walked on 2026-09-23 against every decision in §3 and §4. Each case is one of
three kinds:
- **settled** by an earlier decision (cited);
- **derived**: a rule that follows from those decisions, listed so the design
  and the tests carry it;
- **open**: a new question, Q43–Q48.

### 7.1 Presence and radio

| # | Case | Resolution |
|---|---|---|
| E1 | A link at the edge of range flickers | Settled by Q43 |
| E2 | One-way link: A hears B, B does not hear A | **Derived.** "In contact" means mutual. Each beacon lists whom its sender hears directly, and a pair is in contact only when both list each other. Every team test (met, all connected, exchange) uses mutual contact (Q8; rule: shared inputs) |
| E3 | Robots briefly disagree about "all connected" or "met" because of beacon timing | **Derived.** Each robot acts on its own view, and the views converge within a beacon or two. A pair's exchange, once complete, stays complete for that contact. A robot that sees it one beacon late does not treat the leaving peer as lost; a property test checks this |
| E4 | A node crashes or restarts | Hard fail (Q4). No recovery is designed |

### 7.2 Map exchange

| # | Case | Resolution |
|---|---|---|
| E5 | Nothing new to share | Done at once: the snapshot is already held (Q18) |
| E6 | Maps keep growing during the exchange | The target is the snapshot at the start of contact (Q18) |
| E7 | Data lost for good (queue overflow, bit errors) | Bounded by Q27 |
| E8 | Contact flickers mid-exchange | A gap inside Q43's window is not a loss. A real loss falls under Q28 and Q32. A new contact takes a new snapshot |
| E9 | The partner leaves the moment it completes | E3 |

### 7.3 Meetings (rendezvous, hybrid)

| # | Case | Resolution |
|---|---|---|
| E10 | The team never agrees a plan | Rendezvous behaves as off; hybrid as pursuit (H0). Rare, since the robots start together |
| E11 | The plan changes while connected and the team splits mid-agreement. Robots then hold different plans and go to different cells, possibly for the rest of the run | Settled by Q47 |
| E12 | One robot cannot reach the meeting cell | Settled by Q44 |
| E13 | N > 2: one attendee missing while the others finish exchanging | Settled by Q45. **Derived:** attendees are always the whole team. Bookings are made privately after the split, so nobody can know who booked which slot, and "everyone" is the only answer all robots share |
| E14 | A robot arrives after T + 120 s | The window has closed for it too: book the next slot and explore on (Q29) |
| E15 | A robot arrives at T + 100 s | It joins. Its exchange is bounded by Q27, not by the window, which closes only for robots without contact (Q29) |
| E16 | The exchange runs past the next slot's time | **Derived.** The next booking is the first slot after leaving (R2) |
| E17 | A robot finishes its coverage between meetings | It keeps attending while another robot may still be exploring (Q23). What it does between meetings: Q46 |
| E18 | A booking falls due while following a peer (hybrid) | At N = 2 the pair is together, so this is the meeting: hold until exchanged (R5 revised). At N > 2, leave for the cell; the followed peer is an attendee too (H3, Q26) |
| E19 | A booking falls due while exploiting | Exploit wins (Q16). At N > 2 the whole team is exploiting together, so they miss the slot together and book the next (Q37) |
| E20 | Meeting times on different clocks | **Derived fix.** Today the meeting time is measured from each node's own start (`missionElapsed`), and nodes start seconds apart. Shared deadlines (the meeting time and its window) move to the shared sim clock (rule: one clock domain per quantity) |
| E21 | Stuck on the way | Escape moves until unstuck, inside the window (Q29) |
| E22 | At the cell but not hearing an attendee who is there (trees) | Keep closing in (Q22, Q26); otherwise the window |

### 7.4 Chases and following

| # | Case | Resolution |
|---|---|---|
| E23 | The peer moves away faster than the chaser | Goal points or the time limit run out, then explore (P3, P5) |
| E24 | The chased peer is itself chasing someone | Q30, Q33, Q34 |
| E25 | The caught peer is following someone else | Chains form; each follows until its own exchange is done (Q19) |
| E26 | The followed peer stops (at a meeting, at home) | The follower stops ~10 m away and the exchange completes (Q19, Q22) |
| E27 | Two robots chase each other | Q34 |
| E28 | Several peers are missing | Q33 |
| E29 | The chase target is known to have finished | Priced by the value gate, which is reused unchanged |

### 7.5 Exploit

| # | Case | Resolution |
|---|---|---|
| E30 | Contact lost after the episode starts | Q42 |
| E31 | Off is never all together mid-run | It exploits at home at the end (Q35) |
| E32 | A tree sighted before its release | **Derived.** Learned at release if the robot has ever been within sensor range of it; the order does not matter (Q40) |
| E33 | A tree nobody sights | Never exploited. Realistic |
| E34 | A new tree sighted during an episode | **Derived.** At the end of the episode the gate re-checks; if everyone is still connected, pool again (Q41) |
| E35 | An unreachable tree | The per-target timeout (reused) |

### 7.6 End of run

| # | Case | Resolution |
|---|---|---|
| E36 | The campaign horizon arrives before everyone is home | Recorded unfinished, not failed; homing is judged by its own 600 s bound (Q48) |
| E37 | A robot cannot get home | The homing bound (600 s) ends it, and it counts as a hard fail |
| E38 | Everyone home with trees pooled | Exploit, then home again (Q35) |

---

## 8. Design

Agreed by Kalhan on 2026-09-23. It is built from every decision in §3 and
every case in §7.

### 8.1 Shape

```
                 ROS node shell (new executable)
   map, pose, fusion counters, beacons, intents, released trees
                              │  one Inputs struct per tick
                              ▼
   ┌─────────────────────────── Mission ───────────────────────────┐
   │  Presence   Exchange   Booking   Chase   Exploit gate + pool   │
   │  pure C++, no ROS; each owns its own memory                    │
   │  activity = the first line of the priority list that holds     │
   └────────────────────────────────────────────────────────────────┘
                              │  a target, messages, events
                              ▼
        Leg (all driving)   GoalSelector (lifted)   Exploit (moved)
                              │
                              ▼
            goal topic, beacon, intents, jsonl events
```

**Commitments are stored; the activity is not.** Each component holds its own
commitments: a booking, a chase, an exploit episode, an exchange record. Each
commitment has one place that sets it and one that clears it. Every tick,
Mission reads the commitments and picks the activity from a fixed priority
list (§8.4). Nothing stores "what I was doing". So an interrupted activity
resumes by re-checking without extra code (§4.4 principle 1), and there is no
activity flag left to go stale.

Stability comes from the inputs, not from latches. Presence changes only after
10 s of silence (Q43), and each commitment changes only on its own event.

### 8.2 The beacon

A new message, `TeamBeacon`, on a new topic `exploration/team_beacon`. It is
sent once a second from its own timer, in every state (rule 6: a beacon that
never goes quiet). The radio emulator carries it best-effort, as it carries
`TeamWorld` today. `TeamWorld` itself stays untouched for the frozen gen-33
node.

| Field | Used by |
|---|---|
| `robot_id`, `stamp` (shared sim clock), `position` | Presence; Chase (last heard position) |
| `hears`: robots heard directly in the last 10 s | Presence: mutual contact, all connected (E2, Q39) |
| `activity`, `finished` | Booking (Q49), Chase, logs |
| `map_published`: map deltas this robot has sent | Exchange (Q18) |
| `map_received[]`: map deltas received from each robot | Exchange |
| `exchanged`: robots this contact's exchange is done (or given up) with | The N > 2 "met" test (Q26) |
| `plan` (cell, interval, first meeting time, version) and `plan_echo` | Booking agreement (R1, Q47) |
| `trees[]`: id, centre, radius, done | Exploit pool (Q40, Q41) |
| `cells[]`, `tour[]` | Global allocator, cell world, intercept predictor (content unchanged) |

Dropped compared with `TeamWorld`:
- every relayed field: `robot_positions`, `robot_last_heard_sec`,
  `robot_finished`, `robot_mode` (Q8);
- `team_incomplete`, `appointment_inbound*` and `rendezvous_provisional` (Q24,
  Q47).

`RobotIntent` is unchanged. Exploration claims and exploit claims work as
today (G4).

### 8.3 Components

**Presence: who is here** (Q8, Q39, Q43)
- It keeps the arrival time of each robot's latest beacon and that beacon's
  `hears` list.
- `present(j)`: j's beacon arrived in the last 10 s.
- `inContact(j)`: present(j), and j's beacon lists me (E2).
- `allConnected()`: every pair of robots is in contact. It is judged from my
  own view plus the `hears` lists of the robots I hear. At N = 2 it is simply
  `inContact(other)`.
- `lastHeard(j)`: time and position, for chases (P2, Q32).
- Nothing else in the node decides presence (rule 1).

**Exchange: have the maps crossed** (Q18, Q27)
- It keeps one record per peer per contact: from `inContact` turning true
  until it turns false.
- At contact start it records the peer's `map_published` from the peer's first
  beacon, and my own count. The exchange is done when I have received the
  peer's recorded count and the peer reports receiving mine.
- Progress means either counter moving. No progress for 120 s, or 600 s in
  all, means "gave up": logged as "exchange incomplete" and treated as finished
  for leaving.
- Each peer's status is none, running, done or gave up. Done and gave up last
  for the rest of that contact (E3). A new contact records new counts (E8).
- `exchangedWith(j)` and `exchangedAll(set)` are the only completion tests.
- Received counts come from dscovox's `ScovoxFusionCounters`; the sent count
  comes from a new counter on scovox_node (§8.9).

**Booking** (rendezvous and hybrid only)
- **Plan.** Cell, interval (300 s), first meeting time on the shared sim clock
  (E20), and a version. Robot 0 proposes it with the existing
  `RendezvousScheduler::solve` and `RendezvousHandshake` (R1). A plan is
  proposed only at the start and at a meeting where all N robots are present.
  It becomes active once every robot echoes it; if that doesn't happen before
  the meeting ends, the old plan stays (Q47).
- **Book.** Whenever not all robots are in contact and no booking is held,
  book the first slot T whose departure time has not yet passed. Departure
  time is T minus travel time × 1.2, at 0.5 m/s (R2, R4). Attendees are always
  the whole team (E13).
- **Arrive.** At the cell (Leg tolerance), or within 10 m of an attendee
  already there (Q22). At N > 2, keep closing in until in contact with every
  attendee present (Q26).
- **Met.** Every attendee is in contact with every other, and every pair has
  exchanged; at N = 2, the pair (Q26). If all N are present, agree the next
  plan. Then the booking clears.
- **Window.** At T + 120 s the meeting is over for anyone without contact.
  Robots that are present finish their own exchange (bounded by Q27); then the
  booking clears as "partial" or "missed" (Q29, Q45).
- **Contact lost at the cell before the exchange ends.** Drive to the peer's
  last heard position and stop on contact. If there's no contact there, go
  back to the cell. Bounded by the 120 s no-progress rule (Q28, Q32).
- **Chance encounter.** At N = 2 the booking clears once maps are exchanged
  (R3, Q25). At N > 2 an encounter never clears it (Q26).
- **Finished robot.** It waits at the cell between meetings (Q46), and goes
  home after a meeting where every robot reports finished (Q49).
- **Stuck on the way.** Escape moves with no count limit, inside the window
  (Q29). An unreachable cell is tried again at the next slot (Q44).
- One place books; one `clear(reason)` clears, with reason met, partial,
  missed or cancelled.

**Chase** (pursuit and hybrid only)
- **Trigger** (P1, Q20, Q21, Q33). All must hold:
  - the team has been together at least once;
  - the peer has been unheard for at least 90 s;
  - its last contact was no more than 900 s ago;
  - at least 90 s have passed since the last chase ended — now its own knob;
    gen 33 reused the silence gate for this;
  - this robot has not finished;
  - the value gate (`evaluateReconnectGate`) prices every missing peer, and the
    chase goes to the best priced.

  There is no count cap.
- **Target** (P2). The intercept (`PursuitPredictor::predict`) first. Then the
  trail: the peer's last announced goal, then its last heard position. The
  chase fails when these goal points run out (P3), or at 600 s in total, the
  outer limit.
- **Caught** (P4, Q19, Q30). `inContact(peer)` turns the chase into a follow.
  The goal is a point about 10 m behind the peer, on the line from it to me,
  updated with each beacon; the peer carries on as it was. The follow ends
  when `exchangedWith(peer)` is true, wherever the peer goes. If contact is
  lost, the robot goes back to chasing, and the 600 s keeps running (Q19).
- **Failed** → explore (P5). Mutual chases need no special rule (Q34).
- One place starts a chase; one place clears it, with reason done, failed or
  pre-empted.

**Exploit gate and pool** (Q35, Q38–Q42)
- **Sighting** (Q40, E32). The node keeps its own trail, one point every
  metre. The scheduler releases trees on a sim-only topic. A released tree
  becomes known when any trail point is within sensor range of it (20 m,
  `fov_max_range`), whether that was before or after the release.
- **Pool.** Trees I know, plus trees in peers' beacons, minus those done.
- **Start** an episode when `allConnected()` holds, the pool is not empty and
  I am not homing (Q35). The pool goes to `TargetQueue`, nearest first (Q41).
- **Run.** The moved exploit code, with its decisions unchanged. Its driving
  goes through Leg. The dwell barrier's wait is bounded at 120 s. Losing
  contact doesn't stop the episode (Q42).
- **End.** When the queue is empty, the gate re-checks (E34). Exploit steps
  have their own counter. The failed-goal blacklist stays shared, and exploit
  only reads it (Q38).

**Leg: the only code that drives** (S3, R5, §3.1)
- Input: a point, a tolerance and the caller's time bound. Output: driving,
  arrived, escaping or failed.
- It publishes the goal (open loop, re-published) and judges arrival from the
  pose.
- One watchdog: no progress for the watchdog time triggers an escape move
  (today's escape picker), then a retry. Escapes are unlimited; the caller's
  bound ends the leg. The watchdog time is set once, replacing today's five
  values.
- One budget formula: a base, plus distance ÷ speed × `nav_safety_factor`
  (3 in the yaml).
- The proximity pause goes through the existing `ProximityGuard`.

**GoalSelector.** `doPlan`'s ~850 lines of goal selection, lifted unchanged.
Map, pose, claims, cell world and blacklist go in; a goal, or "none left",
comes out. The coverage latch (`maybeLatchCoverageDone`), which decides
per-robot "exploration done", moves with it unchanged.

### 8.4 Mission: the priority list

Every tick, the first line that holds is the activity:

| # | Activity | Holds when | Arms |
|---|---|---|---|
| 1 | Exploit | an episode is running | all (exploit-on runs only) |
| 2 | Meet | a booking is due, or the robot is holding at the cell for an exchange | rendezvous, hybrid |
| 3 | Follow | a chase has caught its peer and the exchange isn't done | pursuit, hybrid |
| 4 | Chase | a chase is running | pursuit, hybrid |
| 5 | Explore | goals remain | all |
| 6 | Wait at cell | finished, and a booking is still needed (Q46, Q49) | rendezvous, hybrid |
| 7 | Home | finished: drive home within the 600 s limit | all |
| 8 | At home | home: wait for the team only if trees remain (Q35, Q50); otherwise report done | all |

- **1 above 2:** exploit always wins (Q16). The gate opens only when everyone
  is together, so it never pulls a robot away from a meeting.
- **2 above 3 and 4:** a due meeting ends a chase (H3). At N = 2, a meeting
  falling due while in contact with the peer is the meeting itself: they hold
  where they are until the maps are exchanged (E18).
- **Exchange runs in the background** in every activity, so a chance
  encounter while exploring still swaps maps.
- **Clocks while exploiting:** the chase and follow clocks pause; team
  deadlines (the meeting time and its window) keep running (Q37).
- **Done** is reported only when home with no unexploited tree. So the run
  script's stop-on-done can't end a run just before the team's exploit at
  home.

### 8.5 Arms

| Arm | Booking | Chase |
|---|---|---|
| off | — | — |
| pursuit | — | on |
| rendezvous | on | — |
| hybrid | on | on |

One `arm` parameter (Q13); these four names replace the six `mtare_*`
tokens.

### 8.6 What goes away

| Gen-33 mechanism | Why |
|---|---|
| Relay closure, relayed masks, "finished counts as present" | Q8, Q24; the root of cell 12 |
| The 6-chase cap; one chase per booking | Q20, Q31 |
| Standing and waiting 30 s after a failed chase | P5 |
| The unbounded appointment wait, the 420 s finished hold, the 30 s meeting hold, the 6 s "together" confirm | Q29, Q27, Q18: replaced by the window and the exchange |
| One appointment per outage (`rendezvous_spent_`) | Q29: book again after every meeting |
| Continuous re-proposal while connected | Q47 |
| Five drive budgets and watchdogs | One Leg |
| The `_mdp` arms | P2 |
| The ungated `/exploration/targets` bus | Q40 |
| A per-node mission clock for team deadlines | E20 |

### 8.7 Parameters

Goal-selection and exploit parameters keep their names and defaults, since
that code moves unchanged. The reconnection parameters are replaced by:

| Parameter | Value | From |
|---|---|---|
| `arm` | off, pursuit, rendezvous, hybrid | Q13 |
| `presence_window_sec` | 10 | Q43 |
| `beacon_hz` | 1 | rule 6 |
| `chase_silence_sec` | 90 | P1 |
| `chase_cooldown_sec` | 90 | P1, now its own knob |
| `chase_max_contact_age_sec` | 900 | Q21 |
| `chase_limit_sec` | 600 | P3 |
| `follow_distance_m` | 10 | Q19 |
| `exchange_stall_sec`, `exchange_total_sec` | 120, 600 | Q27 |
| `meeting_interval_sec` | 300 | R1 (sim value) |
| `meeting_window_sec` | 120 | Q29 |
| `meeting_attendee_stop_m` | 10 | Q22 |
| `depart_safety`, `travel_speed_m_s` | 1.2, 0.5 | R4 (today's defaults) |
| `leg_arrive_m` | 1.5 | the sim value |
| `home_limit_sec` | 600 | S4, Q48 |
| `exploit_dwell_sync_max_wait_sec` | 120 | Q42 |

Each answers one question (rule 10). All are read through `dp()` so
`equiv_gate` sees them.

### 8.8 Logging and measurement

- **Contract.** jsonl schema 13 is the only analysis contract (Q6). The
  analysis scripts move to it in the same change.
- **Events:**
  - activity change: from, to, reason;
  - booking: booked, departed, arrived, met, partial, missed, cancelled, plan
    agreed;
  - exchange, per peer with counts: start, done, gave up;
  - chase: start (with its price and target kind), contact, follow, done,
    failed;
  - exploit: episode start (with the pool), tree done, episode end;
  - exploration done;
  - homing: start, arrival, give-up;
  - run end.
- **Firing counts.** The run-end record holds every mechanism's firing count
  (rule 3):
  - intercept used, trail used;
  - follows;
  - exchanges done and given up;
  - meetings met, partial and missed;
  - escapes;
  - exploit episodes.
- **Metrics (Q4):**
  - per robot: exploration-done time, home-arrival time;
  - team: exploration-done time, homing time (last robot home);
  - coverage, exchange completion rate, time spent in meetings and chases.
- **Hard-fail detector** (Q4, Q48). It reads the events. Planned waits are
  the meeting window, the exchange hold, waiting at the cell, the dwell
  barrier and waiting at home.

### 8.9 Changes outside `explo_planner`

- **scovox:** a sent-delta counter on `scovox_node`, in the same unit dscovox
  counts on receipt ("deltas as presented") (Q6).
- **hmr_sim:** add `exploration/team_beacon` to the emulator's best-effort
  topics.
- **Run scripts:**
  - a flag picks the gen-33 or gen-34 node (Q36);
  - the four arm names;
  - the target scheduler publishes on a sim-only topic.
- **Analysis:** jsonl schema 13, with the gate pins updated to match.

### 8.10 Build order

All four arms are built before the run box sees anything (Q2).

1. Characterisation tests of today's exploit and goal selection (Q7 layer 1).
2. Copy goal selection and exploit into the new library code. The gen-33 node
   is not edited (Q36); the copies are deleted together with it.
3. Presence, Exchange, Booking, Chase, Leg and Mission, each with unit tests
   (layer 2).
4. The in-process harness: N cores, a toy world, a range-and-trees radio. It
   runs the property tests and replays cell 12, the gen-26 chain and the gen-27
   stand-short (layers 3–5).
5. The node shell, the beacon, the scovox counter, sighting, jsonl 13 and the
   analysis scripts.
6. Mutation testing of Mission and its components (layer 6).
7. Commit. Push when told. The run box runs a smoke round first (one seed per
   arm at N = 3), then the 36 cells (Q5). Experiments run only on the run box.

### 8.11 Risks

| Risk | How it is watched |
|---|---|
| Sent and received counts use different units, or dscovox rejects a frame. Exchanges then never complete, and every meeting ends on Q27's bound | A unit test with real binaries; "exchanges done" must be non-zero in every cell |
| The beacon carries `cells[]`, which grows with the map, and at N = 4 it competes with the maps for airtime | The emulator's airtime stats in the smoke run |
| A shared sim clock works in simulation; real robots would need synchronised clocks | Out of scope; noted here |
| Goal selection and exploit exist twice until the gen-33 node is deleted | Deleted after the first run-box round (Q36) |
| Following at 10 m vs the 5 m proximity pause in narrow rows | Harness scenarios |

### 8.12 Open

Q51–Q64, from the adversarial review (§9). §8 is revised in one pass once they are settled, together with the corrections in §9.2.

---

## 9. Adversarial review (2026-09-23)

Asked for by Kalhan once the design closed: "after design is finished
adversarial review using fable 5.1 agents unstreered and some steered".

**Reviewers.** Eight read-only Fable 5.1 agents. Two were unsteered (the doc
and the code, told to break it). Six were steered, one angle each: distributed
agreement, liveness and bounds, code facts, exploit intact (G4), validation,
complexity (G1).

**Method.** About 110 raw findings merged into the issues below. Every code
claim was checked against the source before it was recorded; the ones that did
not hold are in §9.1. Findings that change an agreed decision, or need a choice
between real alternatives, are Q51–Q64 (§9.3), added to §3 as open.

**Outcome.** The shape of §8 survives. The review found two blockers in the
design itself (exchange completion after any lost map piece, Q51; the plan
split, Q54), a cluster of end-of-run holes (Q57), exploit rules that silently
changed exploit's decisions (Q59), and a validation plan that would pass the
cell-12 failure (Q61). No build starts until Q51–Q64 are settled.

### 9.1 Findings rejected or narrowed on checking

| Finding | Why it does not hold |
|---|---|
| dscovox's reader depth defaults to 50, so the reconnect burst is dropped at the reader | The default is 50 (`dscovox_node.cpp:239`), but `simple_nav_3d.launch.py:333` and `:438` set 4000 on both dscovox instances, and the run script launches that file (`run_explo_sim_rviz.sh:1229`) |
| Map frames sent before the emulator subscribes are never relayed, so the counters start apart | scovox sends and counts nothing while it has no subscriber, and sends a full snapshot on the next connect (`scovox_node.cpp:1858-1869`). Q51's finding stands on the other causes |
| A peer never heard directly gets chased every 90 s for the whole run, since the gate fails open and the 6-chase cap is gone | The chase trigger needs a direct last contact at most 900 s old, so a never-heard peer is never chased. Fail-open on an allocator refusal is real but bounded per outage by the 900 s cut-off and the 90 s cooldown (Q64) |

### 9.2 Corrections: facts in §3–§8 that are wrong or missing

None of these needs a decision; each takes the code's behaviour or the obvious
fix. They are applied to §8 in the same pass as Q51–Q64.

| # | Where | The doc says | The code says | Correction |
|---|---|---|---|---|
| K1 | §8.11 | `cells[]` grows with the map | The cell grid is fixed (100 m ROI, 10 m cells: 100 cells) and `toWire()` emits every cell (`cell_world.cpp:315-321`). The beacon is about 1.5 KB and constant; TeamWorld already sends the same at 1 Hz | Drop the risk row. The real airtime risk is Q52 |
| K2 | §3.1, §8.3 GoalSelector | "moved unchanged", `selectGoal(inputs) -> goal` | Three outcomes: goal, none left, and "retry next tick" (`node:5616`). About 90 members, metric and event outputs. Allocation inputs come from TeamModel, including relayed positions (`node:5163-5191`) | Explicit Inputs and Outputs structs; the third outcome kept. The allocator takes each peer's last **directly** heard position and age. State that at N ≥ 3 allocation changes where a peer was known only by relay (a consequence of Q8) |
| K3 | §3.1, §8.3 | The coverage latch moves unchanged | About 20 of the 96 lines of `maybeLatchCoverageDone` are the latch; the rest is RETURN_SYNC, appointment, homing and done-seek teardown (`node:6654-6748`) | Move only the unknown-fraction criterion; the endings are Mission's |
| K4 | §8.4 row 5 | Explore holds while "goals remain" | No candidates is "retry next tick", not an ending (`node:5616`); finishing is the coverage latch or the step budget | Explore holds while not finished. Finished = coverage latch or step budget, latched once as today; it is the only source of the beacon's `finished` |
| K5 | §8.4, E37 | Homing give-up "ends it" | Give-up parks and reports done with result `timeout` or `budget` (`node:9032-9044`) | Same; it is a hard fail by its event, not by the run's length |
| K6 | §8.3 Book | Departure time is recomputed each tick | Today departure latches (`appointment_departed_`, `node:5352`) | Part of Q56 |
| K7 | §3.1 | `RendezvousHandshake` reused | It encodes the provisional and re-agree protocol §8.6 deletes (`node:7397-7411`) | Drop it from the reuse list. Q54's versioned plan is new code |
| K8 | §3.1, §8.3 Leg | Exploit's driving goes through Leg with decisions unchanged | Mid-drive exploit decisions live in doNavigate: target released mid-hop (`node:6363-6377`), vantage yielded to a peer (`6403-6423`), yaw settle before dwell (`6437`), approach-waypoint re-plan (`6443-6454`), vantage reached (`6464`), proximity-hold refund of the target timer (`10359`), 1 Hz intent re-publish (`10195-10206`) | Leg gets a yaw target and a "time held" output; exploit owns a per-tick re-check hook Leg calls. The characterisation tests pin all seven |
| K9 | E29 | The gate prices a finished peer | The gate skips finished peers (`reconnect_gate.hpp:67`, `reconnect_gate.cpp:22, 78`) | Reword: a finished peer is never chased |
| K10 | §5 | Reliable map data is lost on a bad link | Reliable is never dropped except by the 64 MiB queue cap (`hmr_comms_sim_node.cpp:27-31, 772-780`); a bad link only drains slowly | Reword. Q53 treats a slow drain as progress |
| K11 | §8.2 | `team_hash` and `grid_hash` dropped | The grid-hash gate is the only check that a peer's cell ids name our grid (`node:9506-9508`) | Keep both hashes on the beacon |
| K12 | Q6, Q40, §8.2 | TeamWorld changes; also a new TeamBeacon | Both stated | The new node publishes and reads TeamBeacon only; TeamWorld stays as it is for the gen-33 node |
| K13 | §8.9 | The scheduler moves to a sim-only topic | `/exploration/targets` is root-namespaced and the emulator relays only `/{robot}/...` (`hmr_comms_sim_node.cpp:312-315`), so it is already off the radio | No topic change. Only the node-side sighting gate is new. The trail can reuse `home_trail` |
| K14 | §8.9 | scovox gets "a counter" | `emitted` is per call, not kept (`scovox_node.cpp:1882-1886`) | Superseded by Q51's sequence numbers |
| K15 | §8.9 | Only the emulator topic list for the beacon | The run script also wires per-peer rx topics (`run_explo_sim_rviz.sh:1840-1842`) and the bag list (`1292-1295`); the leakage gate's fixed list lacks the beacon (`comms_gates.py:53`, `--gated-extra` at `:1994`) | Add the beacon to all three |
| K16 | §8.3 Leg | Leg is pure C++, no ROS | `ProximityGuard::evaluate` takes `rclcpp::Time` | A thin seconds wrapper |
| K17 | §8.9 | Analysis follows jsonl 13 | `equiv_gate.py:97-98` parses one hard-coded node path | Keep every `dp()` in the new shell; point `EQUIV_NODE_SRC` at it |
| K18 | Q49 | Finished robots attend meetings | `RendezvousScheduler::solve` drops finished and off-frontier robots (`rendezvous_scheduler.cpp:26`) | Finished attendees are included in the plan's reachability and floor |
| K19 | Q15, Q17 | Exploit's code is not edited | Finishing coverage mid-dwell calls `startReturnHome`, which calls `standDownExploitation` and demotes the target (`node:8881`, `7966`) | Part of Q59 |
| K20 | §8.8 | The hard-fail detector reads events | Pose is logged only in `step` events, once per planning step (`experiment_log.hpp:147-149`); a holding, following or stuck robot logs none | Add a `tick` event every 2 s: pose, activity, Leg status, the current wait's start and bound. No-progress is pose displacement over a window |
| K21 | §8.8 | Readers move to schema 13 | `event_log.py` warns on a newer schema instead of refusing (`:80`, `:185`); `gate_g8.py` pins 11 (`:316`) | Readers refuse a schema above theirs; pins move |
| K22 | Step counter (Q38) | Unchanged | Exploit steps no longer spend `max_steps` (`node:10913`) | State it: exploit-on runs explore for more steps |

### 9.3 Questions opened by the review

Full wording in §3 (Q51–Q64). Sources are the reviewer findings merged into
each.

| Q | Issue | Severity | Found by |
|---|---|---|---|
| Q51 | Exchange "done" compares running totals; one lost map piece blocks every later exchange with that peer. Done is also one-sided | blocker | both unsteered, code facts, distributed agreement, complexity |
| Q52 | Map backlog drain starves beacons; presence lapses mid-exchange | major | both unsteered, distributed agreement |
| Q53 | Three bounds on one hold; a flickering link resets the clock; Q32's drive has no live bound and two robots cross | major | complexity, liveness, unsteered B, distributed agreement |
| Q54 | Plan agreement can split the team permanently; the 60 s start hold is gone; one-shot proposal | blocker | distributed agreement, both unsteered, liveness |
| Q55 | "Every pair at one moment" fails often among trunks; a bad cell never moves | major | distributed agreement, unsteered B |
| Q56 | "Due" flickers; private slot choice splits the team; lead uses 0.5 m/s straight-line against 0.40 m/s measured | major | liveness, distributed agreement, both unsteered, complexity |
| Q57 | After the last meeting the team is pulled back; one absent robot holds everyone at the cell to the horizon | blocker | liveness, validation, distributed agreement, both unsteered, complexity |
| Q58 | Exploit gate opens before the meeting's exchange; per-robot and not atomic; homing robot left out | major (exploit-on) | exploit, liveness, both unsteered, distributed agreement |
| Q59 | Q38, Q41, Q42, Q17 and "tree done" change exploit's decisions | major | exploit, code facts |
| Q60 | Characterisation tests cannot link the gen-33 node; builds here | blocker for build step 1 | exploit, validation |
| Q61 | Q48 lets the cell-12 shape pass; detector trusts self-reports; run control reads CSV state and log text | blocker | validation, liveness, exploit, unsteered B |
| Q62 | No cell exercises the new exploit wiring (reverses Q14) | major | validation |
| Q63 | Simplifications: the priority list hides ~17 sub-phases; Q37 pause; duplicate parameters | major (G1) | complexity |
| Q64 | Chase: Q33 needs a gate rewrite; hybrid chase always pre-empted; follow behind a trunk; fail-open | minor–major | complexity, liveness, both unsteered |

### 9.4 Test and validation additions (no decision needed)

- **Power.** Three seeds per cell detect hard fails and gross regressions
  only: TS1B spreads are 200–450 s per rung (`TS1B_TEAM_SIZE_RESULTS.md:369`).
  A break that hits 10 % of runs is missed in one arm's 9 runs with p ≈ 0.39.
  The in-process harness (layer 5) is the break detector; each property test
  states its run count (at least 1,000 missions per arm per N).
- **Firing counts per arm.** A table of which counts must be non-zero over each
  arm's 9 cells (intercept, trail, follow, plan agreed, Q32 move, escape,
  window expiry); an empty population reads UNRESOLVED, as `gate_g8` does.
  Exchanges done after the counters moved are counted apart from the trivial
  start-of-run ones (E5).
- **Give-ups.** Hard fail when give-ups exceed a stated fraction of non-trivial
  exchanges in any cell.
- **Metrics.** Each metric is written as an event pair plus a time base
  (shared sim clock), with a known-answer fixture. Per-robot exploration-done
  is the first latch event.
- **Fixtures.** Hand-written schema-13 logs, one per hard-fail class plus one
  clean, that the detector must flag or pass (the `gate_g8_calib` pattern).
- **Bounds.** The detector holds its own copy of every bound; the run manifest
  is checked against it, not read from.
- **Dwell.** Each activity's minimum dwell is a named value in §8.7 and is
  stamped in `run_start`, so "reversed within its dwell" has a number.
- **Harness radio.** Per-link drop traces and one-way drops; gen-33
  `link_states.csv` traces replayed as input; trunks on the link at meeting
  positions; a long split followed by a backlog drain.
- **Replays.** Cell 12 is replayed as a scenario (one finished robot at the
  cell, one out of range, both finished) with the Q49 and Q57 outcome
  asserted, not as the deleted code path.
- **Smoke round.** Per-link `drop_airtime` recorded, with a threshold.

### 9.5 Build environment

This machine has no ROS on the host, but Docker has the project image
`hmrexplo:humble` (ROS Humble, colcon). Builds and unit tests can run here in
that container; simulations still run only on the run box (Q60).
