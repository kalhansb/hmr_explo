# Generation 34 design — working draft

**Status:** design agreed (Kalhan, 2026-09-23); adversarial review done (§9); Q51–Q66 decided (Q51–Q61 and Q63–Q66 by Claude under Kalhan's delegation, 2026-09-23: "I'm going to let you decide these, I want a robust system that exchange maximum possible info when meet"). §8 revised in one pass with them and with §9.2. Next: build phase 1 (§8.10). Decisions before Q51 are Kalhan's, taken one
question at a time.
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
| Q51 | How a robot knows the maps have crossed (§9, blocker) | Q18 compared running totals: one lost map piece keeps a pair's totals apart for the rest of the run, so every later exchange with that peer ends on the 120 s give-up (dscovox's count never resets and nothing is resent; `map_agreement.py` measured 0.1–3.6 % loss). Done was also one-sided. **Number the map pieces.** scovox stamps a sequence number on each frame; dscovox reports, per source, the newest number received and the gaps. Each beacon carries my newest sent and, per peer, my newest received. The exchange with a peer is done when **both** sides have received up to the number the other had sent when contact began. Gaps are logged as loss and never block done. No `exchanged` field is needed: both halves are read from the two beacons | **decided** (Claude, under Kalhan's delegation, 2026-09-23): as recommended, with one simplification. Each robot's dscovox already subscribes to its own scovox directly, so the newest number in its own entry is the "sent" number; no extra topic. `ScovoxMapBinary` gets `seq`, `ScovoxFusionCounters` gets `newest_seq[]` and `seq_gaps[]` |
| Q52 | Beacons lost while a map backlog drains (§9) | The emulator drains queued map data first from one shared airtime budget and drops best-effort messages while it is empty. After a long split every robot's beacons can vanish for tens of seconds, longer than the 10 s window. **The emulator keeps a small airtime reserve for best-effort messages**, so the reliable drain never takes the last slice. Links there are symmetric (`PairKey`), so presence stays "beacon heard". Record per-link `drop_airtime` in the smoke round with a threshold. Alternative: count arriving map data as hearing the peer, which leaves the radio alone but cannot confirm mutual contact (E2) while beacons are starved | **decided** (Claude, delegated, 2026-09-23): a variant of the reserve. New emulator parameter `best_effort_priority` (default false). When true, a best-effort message on an up link is always admitted and charged, so the balance can go negative and the map drain waits for it; beacons, intents and TeamWorld are small. The run script sets it for the gen-34 node. Per-link `drop_airtime` is still reported |
| Q53 | How long a robot holds at a meeting (§9) | Three bounds overlap (window 120 s, no progress 120 s, total 600 s), a new contact restarts the no-progress clock so a flickering link holds forever, and Q32's drive is bounded by a record that ended with the contact. **One patience clock per booking:** 120 s from the last progress (a map piece received, or a missing attendee heard); a new contact does not reset it. A 600 s backstop per booking, not per contact. **Q32 kept, one mover:** of a pair that lost each other at the cell, only the higher id drives to the other's last heard position; the lower id holds | **decided** (Claude, delegated, 2026-09-23): as recommended. Progress is a sequence number rising from any attendee, or an attendee heard for the first time in this booking. The backstop runs from the later of the meeting time and departure |
| Q54 | The meeting plan can split the team for good (§9, blocker) | At N = 3, robot 0 sees both echoes and leaves with the new plan; the other two lost each other before seeing each other's echo and keep the old one. Different cells for the rest of the run, since only a full meeting changes the plan. The design also lost today's 60 s start hold (`node:2725`, `4814`) and proposes once. **The plan carries a version and robot 0 is the only proposer; a robot adopts any higher version the moment it hears it, from any beacon, at any time.** Keep the 60 s start hold in every arm; robot 0 re-proposes each tick while everyone is in contact and no plan is agreed. A partial meeting keeps the standing plan (Q47) | **decided** (Claude, delegated, 2026-09-23): as recommended, plus: plan data may spread through relays (it is not presence); t0 and the interval are fixed by version 1 and only the cell changes; robot 0 re-solves a provisional plan while everyone is connected; at a met full meeting robot 0 issues version + 1 stamped `renewed_at_slot`, and the others leave only when every beacon shows it (or patience ends). Split recovery: each robot keeps its last two plans and the version each peer last showed directly; while a peer lags on a plan it still holds, slots alternate between the two cells by parity. **Amended (§11.1):** a gossiped table of every robot's known plan replaces the direct view; odd slots use the lowest known version's cell |
| Q55 | What counts as "met" at N > 2 (§9) | One trunk kills a link (70 dB). With robots stopped about 10 m apart at the cell, a reviewer estimates some pair is blocked in about 40 % of N = 3 meetings; at a cell with a trunk near the centre every meeting is partial, and the cell never moves. **Met = every pair has completed a direct exchange at some moment inside this meeting**; a pair that finished and then lost each other still counts. Amends Q26 | **decided** (Claude, delegated, 2026-09-23): as recommended. The beacon carries `meeting_slot` and `met_mask`; a pair is met if either member lists the other for the same slot; a full meeting also needs the Q54 renewal |
| Q56 | When to leave and which slot (§9) | "Due" is recomputed each tick, so it turns false as the robot closes in (today it latches). After a split each robot books the first slot **it** can make, so at N = 3 they pick different slots. The lead assumes 0.5 m/s in a straight line; robots measure 0.40 m/s on longer paths and arrive near the window's end. **(a) Departure is stored and clears with the booking. (b) Book the first slot every robot can make, from each robot's last heard position and time. (c) The lead uses 0.40 m/s on the path distance from the cost grid, × 1.2** | **decided** (Claude, delegated, 2026-09-23): (a), (b), (c) as recommended. Path distance is the allocator's cell-to-cell cost (`GlobalAllocator::costMm`), straight line without a cell world. A robot that books a different slot from the rest misses and books the next, so a mismatch heals itself |
| Q57 | Finished robots at the end of the run (§9, blocker) | (a) After the all-finished meeting, contact drops on the walk home, Book makes a new booking and Meet outranks Home: everyone turns back. Nothing stores "everyone finished". (b) One robot that never makes a meeting keeps the finished ones at the cell to the horizon, which counts as censored (the cell-12 shape). **(a) Store `team_finished`, set at a meeting where every beacon says finished; with it set nothing is booked and Home holds. (b) A finished robot goes home after 2 consecutive slots at which some robot never showed; the team can also learn "all finished" at home (homes are 3 m apart). (c) Q50's wait at home gets the 600 s homing bound** | **decided** (Claude, delegated, 2026-09-23): (a) and (b) as recommended. `team_finished` also spreads by beacon (finished never un-finishes, so it is a fact). (c) is moot in phase 1: with exploit off there is no wait at home (Q66) |
| Q58 | When an exploit episode may start (§9, exploit-on) | The gate opens as soon as all are in contact, before the meeting's exchange and plan agreement, and exploit outranks the meeting; each robot opens it on its own view, so at N = 3 two can start while the third explores; a homing robot passing a teammate starts that teammate alone. **Start is a team event:** a beacon says "ready" when, from that robot's view, everyone is in contact, nobody is homing, and any meeting in progress is met. The episode starts when every beacon says ready; the pool rides the beacons, so all pool the same trees | **deferred** to the exploit port (phase 2, Q66). Recorded as the rule that port starts from |
| Q59 | Exploit rules that changed exploit's decisions (§9) | **Keep today's behaviour in all six:** (1) Q41: one shared tree order on every robot, by tree id, not nearest first (today's queue is arrival order; nearest-first splits a team across trees). (2) Q42: keep the barrier at 0; it already ends on the 5 s claim expiry of a silent peer and on the 300 s target timeout. (3) Q38: a failed vantage drive writes the blacklist, as today. (4) A tree leaves every pool only on a COMPLETE from anyone; PARTIAL stays local. (5) Q17 over §8.4 row 1: finish the active target, go home, the rest is exploited at home (Q35); remove the stand-down call from that path. (6) The pool is topped up each tick during an episode | **deferred** to the exploit port (phase 2, Q66). All six kept as the starting rule |
| Q60 | How exploit is pinned before it moves, and where builds run (§9) | The gen-33 node defines `main()`, so no test can link it (`CMakeLists.txt:309`), and Q36 froze it. **One exception to Q36: move `main()` to its own file, no logic change.** A gtest then constructs the gen-33 node in-process, feeds it scripted map, pose and peer intents on a simulated clock, and records exploit's decisions; the same scripts run against the new code. Builds and unit tests run here in the `hmrexplo:humble` container if the run box is also on Humble; simulations stay on the run box | **decided** (Claude, delegated, 2026-09-23): no characterisation harness. The new node is carved from a **copy** of the gen-33 file: goal selection, map ingest, exploration driving, metrics and the event log stay verbatim, and the reconnection, exploit and homing machinery is deleted from the copy. Q36 stays: the gen-33 node is not edited. Builds and unit tests run here in `hmrexplo:humble`; the run box is on Humble too (`run_explo_sim_rviz.sh:899`) |
| Q61 | What the checker treats as a failure (§9, blocker) | (a) Q48 makes a run at the horizon censored, so a hung finished team passes; today `gate_g8.py:846` fails every run not ending all-done. (b) The detector trusts the robots' own events. (c) Run control reads the CSV `state` column and log text (`run_explo_sim_rviz.sh:2040`, `2125-2151`). **(a) At the horizon, a run fails if every robot has finished exploring and some robot is not home; it is censored only while someone still explores. (b) Every met, exchange-done, chase-contact and all-connected event is checked against the simulator's `link_states.csv`. (c) The new node keeps the CSV `state` column (DONE when done) and the "selected goal" and "Exploration complete" lines, pinned by a contract test, so one script runs both nodes; the hang threshold is resized** | **decided** (Claude, delegated, 2026-09-23): (a), (b), (c) as recommended. The checker is a new script, `gen34_check.py` |
| Q62 | Exploit in the run-box round (§9; reverses Q14) | The exploit path is mostly new wiring (sighting, trees in beacons, pooling, the gate, done at home) and no cell runs it; a gate that never opens gives zero episodes and nobody sees it. **Four smoke cells only: 4 arms × N = 3 × 1 seed, exploit on; pass = at least one episode and trees done = trees sighted.** No metrics, not part of the 36 | **decided** (Kalhan, 2026-09-23: "we are not testing exploitation, it just should not interfere with exploration when it's turned off"): no exploit cells. See Q66 |
| Q63 | Simplifications (§9, G1) | 13 states became 8 activities plus about 17 sub-phases inside components; about 170 parameters became about 145. **(1) Booking and Chase each own an explicit phase; Mission reads only `active()`; Follow folds into Chase. (2) Delete row 6: a finished robot's booking is due at once, so Meet covers it. (3) Delete Q37's pause: one clock. (4) One speed, one safety factor, one stop distance (`follow_distance_m` and `meeting_attendee_stop_m` are both 10 m). (5) A booking clears as met or window-closed, with partial and missed as labels** | **decided** (Claude, delegated, 2026-09-23): (1)–(5) accepted. One travel speed, 0.40 m/s. Two safety factors, one per question: `nav_safety_factor` sizes a Leg's budget, `depart_safety` (1.2) the meeting lead |
| Q64 | Chase details (§9) | **(1) Q33: chase the peer the gate priced (`leg_peer_id`); the gate prices only the nearest locatable peer (`reconnect_gate.cpp:110`) and stays unchanged. (2) Hybrid: no chase when the next departure is under 120 s away, so a chase is not always cut off by Meet. (3) The follow point needs clear line of sight to the peer (the vantage planner's test), else closer, down to 6 m; one trunk kills a link. (4) The gate keeps failing open; refusals are logged and counted, bounded per outage by the 900 s cut-off and the 90 s cooldown** | **decided** (Claude, delegated, 2026-09-23): all four, as recommended |
| Q65 | Proximity pauses at meetings (Kalhan: "can we ignore the closeness stop when meets are active") | Between two robots whose beacons both say Meet, skip the proximity pause. Each attendee drives to its own spot on a 3 m ring around the cell centre, angle by fleet id, stepped outward if the spot is blocked; arrival within 1.0 m. Drop Q22's 10 m attendee stop. The tick event logs the nearest peer's distance; under 1.0 m is a hard fail | **decided** (Claude, delegated, 2026-09-23) |
| Q66 | Exploit in the new node (Kalhan: "we are not testing exploitation, it just should not interfere with exploration when it's turned off") | Phase 1: the new node runs exploit-off only, and `exploitation_enabled=true` is fatal at startup. Exploit-on runs use the gen-33 node until the port (phase 2, which starts from Q58 and Q59). Deleting the gen-33 node waits for that port | **decided** (Claude, delegated, 2026-09-23) |


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
| E3 | Robots briefly disagree about "all connected" or "met" because of beacon timing | **Derived.** Each robot acts on its own view, and the views converge within a beacon or two. A pair's exchange, once complete, stays complete for that contact. A robot that sees it one beacon late does not treat the leaving peer as lost; a property test checks this. **Amended (§11.2):** a gave-up exchange can still finish late in the same contact |
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

Agreed by Kalhan on 2026-09-23 and revised in one pass after the review (§9),
with Q51–Q66 and the corrections K1–K22. This is phase 1: all four arms,
exploit off (Q66).

### 8.1 Shape

```
        gen-34 node: a new executable, carved from a copy of the gen-33 file
   map, pose, fusion counters (with sequence numbers), beacons, intents
                              │  one TeamInputs struct per tick
                              ▼
   ┌───────────────────── TeamCore (pure C++, no ROS) ─────────────────────┐
   │  Presence   Exchange   Plan   Booking   Chase                          │
   │  activity = the first line of the priority list that holds (§8.4)      │
   └────────────────────────────────────────────────────────────────────────┘
                              │  activity, drive target, beacon fields, events
                              ▼
   Explore: gen-33 PLAN → NAVIGATE → INTEGRATE → LOG_STEP, verbatim
   Every other drive: Leg          Both: the proximity guard
                              │
                              ▼
          goal topic, TeamBeacon, RobotIntent, jsonl events, CSV
```

**Carved, not rewritten (Q60).** The new node starts as a copy of
`explo_planner_node.cpp`. These stay verbatim: goal selection, map ingest, the
exploration drive (`doNavigate`, `failGoal`), the coverage criterion, the step
budget, metrics, the event log and the claim stream. These are deleted from
the copy: exploit, rendezvous, pursuit, the reconnect trigger and dispatch,
RETURN_NAV and RETURN_SYNC, the link gate, done-seek, hold escalation,
TeamWorld and TeamModel. The gen-33 node itself is not touched (Q36).

**Team logic is one pure library, TeamCore.** Each component is unit-tested
alone. N cores also run together in an in-process harness (§8.10).

**Commitments are stored; the activity is not.** Each component holds its own
commitments: a booking, a chase, an exchange record, the plan. Each has one
place that sets it and one that clears it. Every tick, TeamCore reads the
commitments and picks the activity from a fixed priority list (§8.4). Nothing
stores "what I was doing", so an interrupted activity resumes by re-checking
and there is no activity flag to go stale. Stability comes from the inputs:
presence changes only after 10 s of silence (Q43), and each commitment changes
only on its own event.

### 8.2 The beacon

`explo_planner_msgs/TeamBeacon` on `exploration/team_beacon`. It is sent once
a second from its own timer, in every state including DONE (rule 6). The new
node never shuts itself down; the run script stops the run. The emulator
carries the beacon best-effort, with priority over map data (Q52).

| Field | Used by |
|---|---|
| `robot_id`, `team_hash`, `grid_hash` | Identity; the grid check before any cell id is used (K11) |
| `header.stamp` (shared sim clock), `position` | Presence, Chase, allocation, separation |
| `goal`, `has_goal` | Chase trail: the peer's last announced goal |
| `hears_mask` | Mutual contact (E2) |
| `activity`, `finished`, `team_finished`, `home` | Booking, Mission, logs |
| `map_sent_seq`, `map_rcvd_seq[]` (by fleet id) | Exchange (Q51) |
| `plan_version`, `plan_cell`, `plan_center`, `plan_t0_sec`, `plan_interval_sec`, `plan_renewed_at_slot`, `plan_provisional` | Plan (Q54) |
| `meeting_slot`, `met_mask` | Met (Q55) |
| `cells[]`, `my_tour[]` | Cell world merge, allocator, intercept predictor; same content as TeamWorld's |

`TeamWorld` stays unchanged for the gen-33 node (K12). `RobotIntent` is
unchanged: exploration claims work as today.

### 8.3 Components

**Presence** (Q8, Q43, E2)
- It keeps, for each robot, the local receipt time of its last beacon and
  that beacon.
- `present(j)`: j's beacon arrived in the last 10 s.
- `inContact(j)`: present(j), and j's `hears_mask` names me.
- `pairInContact(a, b)`: both present, and each one's beacon names the other.
  `allConnected()`: every pair in contact.
- `lastHeard(j)`: time, position, goal and finished, from direct beacons only.
- The allocator and the separation term take each peer's last directly heard
  position and its age (K2). At N ≥ 3 a peer known only through a relay drops
  out of allocation: a consequence of Q8.
- Nothing else in the node decides presence (rule 1).

**Exchange** (Q18, Q27, Q51)
- Map frames carry sequence numbers (§8.9).
  - My "sent" number is the newest number in my own dscovox's entry for me,
    since it subscribes to my scovox directly.
  - "Received from j" is my dscovox's newest number from j.
- It keeps one record per peer per contact, from `inContact` turning true until
  it turns false. At the start it stores two targets: the peer's
  `map_sent_seq` from that beacon, and my own sent number.
- **Done** when my received-from-j has reached its target and j's beacon
  reports received-from-me at or past mine. It is two-sided.
- Gaps (lost frames) are counted and logged but never block done, because any
  later frame carries the newest number past the target.
- **Progress** means either number rising. No progress for 120 s, or 600 s
  since the contact began, is **gave up**.
- Done and gave-up stand for the rest of the contact (E3). A new contact starts
  a new record (E8).
- `exchanged(j)` is the one completion test.

**Plan** (R1, Q47, Q54)
- A plan has: a cell and its centre; a first meeting time `t0` on the shared
  sim clock (E20); an interval (300 s); a version; `renewed_at_slot`; and a
  provisional flag. Slot k is at `t0 + k × interval`.
- **Proposing.** Robot 0 is the only proposer.
  - The first time everyone is connected, it proposes version 1 with
    `RendezvousScheduler::solve`. If there is no tour yet, it uses the
    centroid of the robots' cells, marked provisional.
  - While the plan is provisional and everyone is connected, robot 0 re-solves
    every 5 s. It issues a new version once the solve is no longer
    provisional.
  - `t0` and the interval are fixed by version 1. Later versions change only
    the cell.
- **Adopting.** Any robot adopts a higher version the moment it hears it, from
  any beacon, relayed or not. Plan data is not presence. Each robot keeps its
  current plan and the one before it.
- **Renewal.** At a met full meeting for slot k, robot 0 issues version + 1
  with `renewed_at_slot = k` and a fresh cell from the solver. If the solve
  fails, the new version keeps the same cell.
- **Split recovery** (amended in §11.1). Each robot keeps a table of the
  highest plan it knows every robot to hold, and the beacon carries it. Even
  slots use my plan's cell; odd slots use the cell of the lowest version in
  the table, while that is below mine.

**Booking** (rendezvous and hybrid only)
- **Book** when all of these hold: a plan exists; no booking is held; the team
  is not all connected; `team_finished` is not set.
  - The slot is the first one every robot can make. For each robot, the time
    left before the slot must cover its lead from its last heard position
    (mine: my current position).
  - Lead = path distance to the cell ÷ 0.40 m/s × 1.2 (Q56). Path distance is
    `GlobalAllocator::costMm` between cells, or the straight line without a
    cell world.
- **Depart** when `now + lead ≥ slot time`, or at once if this robot has
  finished (Q63(2)). Departure is latched and clears with the booking.
- **Spot** (Q65). Each attendee drives to its own spot on a 3 m ring around the
  cell centre, at angle 2π·id/N. If the map shows the spot blocked, it steps
  outward 1 m at a time, up to 6 m. Arrived means within 1.0 m.
- **Met** (Q55).
  - While my booking is departed, I set bit j of `met_mask` when
    `exchanged(j)` holds.
  - A pair is met if either member lists the other for the same slot.
  - The meeting is met when every pair of the N robots is met.
  - A full meeting also waits for the renewal: every beacon must show
    `renewed_at_slot = k`, bounded by patience.
- **Patience** (Q53). One clock per booking, started at arrival.
  - It resets on progress: a sequence number from any attendee rising, or an
    attendee heard for the first time in this booking. A new contact alone
    does not reset it.
  - 120 s without progress while holding ends the booking as `patience`.
  - A backstop 600 s after the later of the slot time and departure ends it
    as `backstop`.
- **Window** (Q29, Q45). At slot time + 120 s, if the meeting is not met:
  - A robot with no exchange running ends the booking: `partial` if some
    attendee met it, `missed` if none did.
  - A robot with an exchange still running keeps holding until it is done or
    patience runs out.
- **Reconnect** (Q32, Q53).
  - Trigger: I am holding at the spot, an attendee I was in contact with
    during this booking is no longer in contact, and our pair is not met.
  - Only the higher id of the pair moves. It drives to the other's last heard
    position and stops on contact. If it reaches that position without
    contact, it goes back to its spot. The lower id holds.
  - Once per lost contact.
- **Encounter** (Q25, Q26). A booking that has not departed is cancelled when
  all N robots are connected and every pair has exchanged. A partial encounter
  never cancels one.
- **Missed streak** (Q57b). The count of consecutive bookings that ended with
  some attendee never heard. Any other ending resets it.
- **Clearing.** One `clear(reason)`, with reason met, partial, missed,
  patience, backstop, encounter or team-finished.

**Chase** (pursuit and hybrid only)
- **Trigger.** All of these must hold:
  - the team was all connected at least once;
  - a peer has been silent for at least 90 s, and its last contact was at most
    900 s ago;
  - that peer's last beacon did not say finished (K9);
  - at least 90 s have passed since the last chase ended;
  - this robot has not finished;
  - in hybrid, no booking has departed, and the next departure is more than
    120 s away (Q64(2));
  - the value gate (`evaluateReconnectGate`, unchanged) agrees. It prices the
    nearest locatable missing peer, and the chase goes to that peer (Q64(1)).
    It fails open when the allocator refuses (Q64(4)). Refusals are logged and
    counted.
- **Target** (P2). The intercept (`PursuitPredictor::predict`) comes first,
  then the peer's last announced goal, then its last heard position.
  - Each point is one Leg. Reaching or failing a point moves on to the next.
  - Running out of points is `failed`.
  - The whole chase has a 600 s limit.
- **Follow** (Q19, Q30).
  - `inContact(peer)` turns the chase into a follow. The goal is a point 10 m
    from the peer, on the line from it to me, refreshed on each beacon. If
    that line is blocked on the map, the point is pulled in, down to 6 m
    (Q64(3)).
  - The follow ends when `exchanged(peer)` holds or the exchange gives up.
  - If contact is lost, the robot goes back to chasing from the peer's last
    heard position, and the 600 s limit keeps running.
- **Clearing.** One `clear(reason)`, with reason done, failed, limit or
  pre-empted.

**Leg: every drive that is not an exploration goal** (§3.1)
- Input: a point, a tolerance and a caller-owned bound. It reports driving,
  arrived or holding.
- It publishes the goal, re-published as today, and judges arrival from the
  pose.
- Its watchdog is `progress_window_sec` / `progress_min_distance_m`. When it
  fires, the robot takes one escape move to a free point 1.5–6 m away, using
  gen 33's escape picker, then resumes. Escapes are unlimited; the caller's
  bound ends the leg.
- There is no blacklist. Exploration goals keep gen 33's drive with its
  budget, watchdog, rotation deadline and blacklist, verbatim.

**Proximity** (S3, Q65)
- Gen 33's `ProximityGuard` and PROXIMITY_HOLD cover every drive.
- Exempt: a peer whose latest beacon says Meet, while I am in Meet too.
- The tick event logs the nearest peer's distance. Under 1.0 m is a hard fail.

**Goal selection.** `doPlan`'s goal selection is kept verbatim. "No
candidates" retries next tick (K2, K4). The coverage latch keeps only its
criterion (K3); the endings belong to Mission.

**Finished** (K4). The coverage latch or the step budget, latched once. It is
the only source of the beacon's `finished`.

### 8.4 Mission: the priority list

Every tick, the first line that holds is the activity:

| # | Activity | Holds when | Arms |
|---|---|---|---|
| 1 | Meet | a booking has departed | rendezvous, hybrid |
| 2 | Chase | a chase is held; it follows once in contact | pursuit, hybrid |
| 3 | Explore | not finished | all |
| 4 | Wait | finished; a plan exists; `team_finished` not set; missed streak under 2; no booking, because the team is all connected | rendezvous, hybrid |
| 5 | Home | finished, not yet home | all |
| 6 | Done | home, or homing gave up | all |

- **Meet above Chase:** a due booking ends a chase (H3).
- **Finished robots at meetings.** A finished robot's booking departs at once
  (Q63(2)), so Meet covers "wait at the cell" (Q46). Wait covers the case
  where the team is together and nothing is booked.
- **`team_finished`** (Q57a). It is set when I hear every robot directly and
  every beacon says finished, or when any beacon carries it. It is never
  cleared. Once it is set, nothing is booked and a finished robot goes Home.
- **Missed streak** of 2 or more while finished: Home (Q57b).
- **No plan** (E10). Rendezvous behaves as off; hybrid behaves as pursuit
  (H0).
- **Background.** Exchange runs in every activity, so a chance encounter while
  exploring still swaps maps.
- **Home.** A Leg to the recorded start pose, 1 m tolerance, 600 s bound. A
  give-up parks the robot and ends DONE with result `timeout` (K5).
- **Done.** The CSV state is DONE, the beacon keeps going, and the node stays
  up.

### 8.5 Arms

| Arm | Booking | Chase |
|---|---|---|
| off | — | — |
| pursuit | — | on |
| rendezvous | on | — |
| hybrid | on | on |

One `arm` parameter (Q13). The stamp is the arm string. These four names
replace the six `mtare_*` tokens for the new node.
`exploitation_enabled=true` is fatal at startup (Q66).

### 8.6 What goes away (in the new node)

| Gen-33 mechanism | Why |
|---|---|
| Relay closure, relayed masks, "finished counts as present"; TeamModel | Q8, Q24; the root of cell 12 |
| The 6-chase cap; one chase per booking | Q20, Q31 |
| Standing and waiting 30 s after a failed chase | P5 |
| The unbounded appointment wait, the 420 s finished hold, the 30 s meeting hold, the 6 s "together" confirm | Q29, Q53, Q18: replaced by the window, patience and the exchange |
| One appointment per outage (`rendezvous_spent_`) | Q29: book again after every meeting |
| Continuous re-proposal; `RendezvousHandshake` | Q47, Q54, K7 |
| Five drive budgets and watchdogs for manoeuvres | One Leg |
| The `_mdp` arms | P2 |
| A per-node mission clock for team deadlines | E20 |
| Done-seek coast, link gate, hold escalation, mid-run info trigger | Replaced by Mission and Chase |
| Exploit | Phase 2 (Q66) |
| Shutting the node down at DONE | Rule 6: the beacon never goes quiet |

### 8.7 Parameters

Goal-selection, exploration-drive, proximity and claim parameters keep their
names and defaults, since that code is copied unchanged. So do
`mission_start_hold_sec` (60), `mission_home_tol_m` (1.0) and
`mission_return_max_sec` (600, the homing bound). The reconnection parameters
are replaced by:

| Parameter | Value | From |
|---|---|---|
| `arm` | off, pursuit, rendezvous, hybrid | Q13 |
| `presence_window_sec` | 10 | Q43 |
| `beacon_hz` | 1 | rule 6 |
| `team_beacon_pub_topic`, `team_beacon_sub_topics` | `exploration/team_beacon`; per-peer relays | §8.2 |
| `exchange_stall_sec`, `exchange_total_sec` | 120, 600 | Q27 |
| `chase_silence_sec` | 90 | P1 |
| `chase_cooldown_sec` | 90 | P1, now its own knob |
| `chase_max_contact_age_sec` | 900 | Q21 |
| `chase_limit_sec` | 600 | P3 |
| `chase_departure_margin_sec` | 120 | Q64(2) |
| `follow_distance_m`, `follow_min_distance_m` | 10, 6 | Q19, Q64(3) |
| `meeting_interval_sec` | 300 | R1 (sim value) |
| `meeting_window_sec` | 120 | Q29 |
| `meeting_patience_sec`, `meeting_backstop_sec` | 120, 600 | Q53 |
| `meeting_ring_m`, `meeting_arrive_m` | 3.0, 1.0 | Q65 |
| `travel_speed_mps`, `depart_safety` | 0.40, 1.2 | Q56, Q63 |
| `finished_missed_slots` | 2 | Q57 |
| `leg_arrive_m` | 1.5 | the sim value |
| `tick_event_period_sec` | 2 | K20 |

Each answers one question (rule 10). All are read through `dp()` so
`equiv_gate` sees them (K17).

### 8.8 Logging and measurement

- **Contract.** jsonl schema 13 for the new node; gen 33 stays at 12. Readers
  refuse a schema newer than theirs (K21).
- **Events:**
  - `team_activity`: from, to, reason;
  - `plan`: proposed, adopted, renewed, with version and cell;
  - `booking`: booked, departed, arrived, then one ending (met, partial,
    missed, patience, backstop, encounter, team-finished);
  - `exchange`, per peer: start, done, gave up, with targets, sequence numbers
    and gaps;
  - `chase`: start (with the gate's price and the target kind), contact,
    point, then one ending (done, failed, limit, pre-empted);
  - `homing`: start, arrived, gave up;
  - `tick` every 2 s: pose, activity, Leg status, nearest peer distance, the
    current wait's start and bound (K20);
  - `exploration_complete` and `run_end`, as today. `run_end` carries every
    mechanism's firing count (rule 3).
- **Kept for run control and the scripts** (Q61(c)): the CSV `state` column,
  where DONE means done, and the "selected goal" and "Exploration complete"
  lines. A contract test pins them. New state names: MEET, CHASE, FOLLOW,
  WAIT, RETURN_HOME.
- **Checker** (Q61). `gen34_check.py` reads the events and the simulator's
  `link_states.csv`. Its checks:
  - every met pair had a live link while it exchanged;
  - no wait outlives its bound;
  - no activity is reversed within its dwell more than twice in a row;
  - a robot is never closer than 1.0 m to a peer;
  - at the horizon, a run whose robots have all finished must have every
    robot home.
  - It also checks that firing counts are non-zero per arm (§9.4).

### 8.9 Changes outside `explo_planner`

- **scovox** (Q51):
  - `scovox_msgs`: `ScovoxMapBinary` gets `uint64 seq`;
    `ScovoxFusionCounters` gets `uint64[] newest_seq` and `uint64[] seq_gaps`.
  - `scovox_node` stamps 1, 2, … on each frame as it goes on the wire, so
    deferred chunks keep wire order.
  - `dscovox` keeps each source's newest number and a count of skipped
    numbers.
- **hmr_sim** (Q52): the emulator parameter `best_effort_priority`.
- **Run scripts:**
  - `NODE=gen34|gen33` picks the executable (Q36);
  - `ARM` takes the four names;
  - the beacon is wired into the emulator's best-effort list, the per-peer rx
    topics, the bag list and the leakage gate's list (K15);
  - `best_effort_priority` is on for gen 34.
- **Analysis:** `event_log.py` refuses a newer schema, and `gen34_check.py` is
  added.

### 8.10 Build order (phase 1)

All four arms are built before the run box sees anything (Q2).

1. scovox sequence numbers; the emulator parameter.
2. `TeamBeacon`.
3. TeamCore (Presence, Exchange, Plan, Booking, Chase, Mission) with unit
   tests.
4. The in-process harness: N cores, a 2-D toy world, and a radio with range,
   trunks, drops and backlog. Property tests over at least 1,000 seeded
   missions per arm per N; replays of the cell-12 shape (Q57) and the plan
   split (Q54).
5. The carved node, with the beacon, the Leg and the proximity exemption.
6. Run scripts and the checker.
7. Build and test in the `hmrexplo:humble` container.
8. Code review by Fable 5.1 agents; verified findings fixed, rebuilt and
   retested.
9. Commit locally. Push when told. On the run box: a smoke round (one seed per
   arm at N = 3), then the 36 cells (Q5).

Dropped or deferred from Q7:
- Layer 1, the characterisation tests, is dropped. Exploit does not move in
  phase 1, and goal selection is copied verbatim.
- Layer 6, mutation testing, is deferred to after the smoke round.

### 8.11 Risks

| Risk | How it is watched |
|---|---|
| The map backlog drain starves beacons | `best_effort_priority`; the smoke round reports `drop_airtime` per link |
| A spot ring near trunks is fully blocked | Stepped outward to 6 m, then the Leg's escapes and patience bound it |
| The harness radio is a toy, so tree effects at N ≥ 3 are only approximated | Harness trunks on the link; the smoke round |
| Goal selection exists twice until the gen-33 node is deleted | Deleted after the exploit port (Q66) |
| A shared sim clock works in simulation; real robots would need synchronised clocks | Out of scope; noted here |
| Gen-34 runs use a different radio priority from gen 33 | No comparison is required (§2) |

### 8.12 Open

None for phase 1. Phase 2 ports exploit, starting from Q58 and Q59.

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
| K4 | §8.4 row 5 | Explore holds while "goals remain" | No candidates is "retry next tick", not an ending (`node:5616`); finishing is the coverage latch or the step budget | Explore holds while not finished. Finished = coverage latch or step budget, latched once as today; it is the only source of the beacon's `finished`. **Amended (§11.7):** or PLAN starvation, a mission ending that the metric reader censors |
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

---

## 10. Implementation notes (phase 1 build, 2026-09-23)

This section records three things:
- what the build settled that §8 left open;
- where the build departs from §8 and §9.4;
- what the code review after the build found.

### 10.1 Node and wiring

- **One node.** Gen 34 is the only planner that is built. Its node is
  `explo_planner_node` (`explo_planner/src/explo_planner_node.cpp`). This
  replaces the separate executable of §8.1 and Q36. That plan was dropped once
  gen 33 was no longer needed (user, 2026-09-23).
- **Gen 33 kept as a reference.** Gen 33's node source is kept, unbuilt, in
  `ws/src/explo_planner/backup/gen33/`. It is the reference for the phase-2
  exploit port (Q58, Q59, Q66). The tests that only read that source are kept
  with it: `test_gen20_rendezvous` to `test_gen23_contagion`, and the
  `test_endpoint` version whose group C scanned it.
- **Gen-33-only libraries.** The library components that only gen 33 used stay
  built and tested. Removing them is left for the port.
- **Exploit refused.** `exploitation_enabled=true` is refused at construction
  (Q66).
- **Beacon topic.** The default publish topic is the shared bus
  (`exploration/team_beacon`). The run scripts wire each robot's
  subscriptions through the emulator's `rx/<peer>/` relays
  (`team_beacon_sub_topics`), as gen 33 did for `team_world`.
- **Leg after a proximity hold.** A Leg that resumes after a hold is a new
  leg: it republishes its point and restarts its watchdog window. TeamCore's
  time bound for the activity keeps running through the hold.
- **Fusion counters.** The node reads the dscovox fusion counters from
  `/<robot>/dscovox_node/fusion_counters`, and the scripts pass nothing for it.
  - Until the first sample arrives, the map sequence numbers are unmeasured
    (`TickInputs::seq_valid` is false), and no exchange starts. Without this
    rule every number reads 0. Then `0 >= 0` would make every contact a
    finished exchange, and every meeting would count as met at once.
  - After 20 s without a sample, the node warns and names the topic.

### 10.2 Run scripts

- **Arm.** `run_explo_sim_rviz.sh` always runs `explo_planner_node`.
  `ARM=off|pursuit|rendezvous|hybrid` picks the arm, and the default is hybrid.
  - The `NODE` switch is gone. It also collided with the `NODE` variable that
    npm exports.
  - `run_campaign.sh` has no `--node` and checks the arm name.
  - Its resume guard matches the manifest's `node=` and `arm=` lines.
- **Pinned stack.** The run script pins the stack it runs on:
  `RECONNECT_MODE=mtare_off`, `EXPLOIT=0`, `DONE_SEEK=0`, `MISSION_RETURN=1`,
  `LINK_GATE=0`, the `team_beacon` topic, and the emulator's
  `best_effort_priority:=true`.
  - A conflicting value is refused, not overridden.
  - `run_campaign.sh` refuses `--mission-return 0`. It also refuses the gen-33
    knobs `LINK_GATE`, `MIDRUN_SILENCE`, `DONE_SEEK` and `EXPLOIT` in `--env`.
  - The campaign's mid-run guard, its suffix parsing and its per-cell m-tare
    case are removed.
- **Gen-33 machinery.** The run script still carries gen 33's arm-token and
  knob machinery, pinned to the values above. Removing it is a cleanup for
  phase 2.
- **Manifest.** The manifest keeps `node=gen34` as a label, so that readers can
  refuse a gen-33 cell.
  - It adds `planner_exe=`, `arm=`, `team_exchange_topic=` and
    `best_effort_priority=`.
  - The gen-33 lines stay, at their pinned values.
- **Guard calib.** `campaign_guard_calib.sh` covers the arms, the refusals, the
  pins, the stack readback against the campaign's expectations, and the resume
  guard: 177 known-answer cases.
- **Hang heartbeat.** The heartbeat is 50 s (gen 33 used 40 s). A meeting that
  follows a chase can hold back the "selected goal" lines for longer.

### 10.3 Readers

- **event_log.py** refuses schema 13 (`MAX_SCHEMA = 12`) and points to
  `gen34_check.py`.
- **gate_g8.py** and its calib pin schema 12, which is gen 33's stamp
  (`kSchemaVersion`). Gen 34 stamps `kGen34SchemaVersion = 13`.
- **Gen-33 tools.** `gate_g8.py`, `equiv_gate.py` and their calibs check gen-33
  campaigns only, as does `rendezvous_agreement_calib.py`. They read gen 33's
  node from `backup/gen33/`.
  - equiv_gate_calib's 360° FOV case fails. Gen 33 sets `fov_hfov` with
    `dp_f`, which the gate's parser does not read. It failed the same way
    before gen 34.
  - Both calibs report UNRESOLVED when there is no campaign root
    (`~/hmr_campaign`).

### 10.4 The checker (`sim/gen34_check.py`)

The checker reads schema 13 only, and gen-34 manifests only (`node=gen34`).

**Exit codes:**
- 0: clean.
- 1: hard fail. This outranks a refused cell in the same root.
- 2: usage error, or a cell refused.
- 3: nothing failed, but something could not be checked.

| Check | Hard fail when |
|---|---|
| bounds | a stamped bound differs from the checker's own copy (§8.7 values) |
| contact | an exchange done, booking pair_met, chase contact or all-connected tick has no live link in `link_states.csv` within W + 1.5 s before it (2W + 1.5 s for a pair the robot only hears about) |
| waits | a tick is more than 1 s past its wait bound; homing gave_up or no-home |
| flipflop | more than 6 activity changes in any 60 s; more than two short stints (under 3 s) in a row |
| proximity | a pair under 1.0 m in the link trace |
| progress | under 0.5 m of movement in 120 s while exploring or driving a leg (holds, WAIT, DONE and arrived legs excluded) |
| horizon | censored with every robot finished but not all home; all_done with a robot not home |
| giveups | exchange give-ups above max(1, 25 % of non-trivial exchanges) |
| firing (per arm) | a required mechanism never fired across the arm's cells |

**Trivial exchanges.** An exchange is trivial when both sides already held
each other's maps at contact. Non-trivial exchanges are
`exchange_done - exchange_done_trivial`.

**Required mechanisms:**
- Every arm needs a non-trivial exchange and `homing_arrived`.
- Pursuit and hybrid also need `chase_start`, `chase_first_intercept`,
  `activity_follow` and `chase_done`.
- Rendezvous and hybrid also need `plan_firmed`, `booking_departed` and
  `booking_full_met`.

The other mechanisms fire only in some geometries, so they are reported, not
failed:
- trail and goal chase points;
- the Q32 reconnect move;
- Leg escapes;
- window expiries;
- declined chases;
- plan renewals.

**Calibration.** `sim/gen34_check_calib.py` runs the checker on these inputs:
- a clean hand-written cell;
- one planted defect per check;
- near-miss cases that must stay clean;
- a root with one refused cell and one failing cell (expected exit 1).

The calib also checks the checker's schema, bounds and mechanism names against
the node and TeamCore sources.

**Departure from §9.4.** There is no stamped per-activity dwell parameter. The
flip-flop rule is the harness's P4 (at most 6 changes in 60 s; a short stint
is under 3 s). The checker holds these as its own constants.

### 10.5 Code review after the build

Four Fable 5.1 reviewers read the build. Three were steered: node integration,
the scripts and checker, and TeamCore with Leg. One was unsteered. Every
finding below was checked against the code before it was fixed or deferred.

**Fixed:**

| Finding | Fix | Test |
|---|---|---|
| With no counters sample yet, every contact read as a finished exchange | `seq_valid` (10.1) | `UnmeasuredSeqStartsNoExchange` |
| A fully met booking whose renewal went unseen was held to `full_met + patience`. That is past the backstop that `wait_bound` and the drive's deadline report, so the checker would hard-fail a healthy run | the backstop also ends that wait | `BackstopCapsTheWaitForAnUnseenRenewal` |
| A firming re-solve after a provisional renewal reset `renewed_at_slot` to -1, which lost the proof that the slot was met. Attendees fell back to patience, and a robot whose booking ended by encounter could book the met slot again | a firming re-solve carries the predecessor's value (`TeamBeacon.msg` says so) | `FirmingKeepsTheRenewedSlot` |
| `team_finished` ended only a booking not yet departed, so Meet outranked Home until the window or the backstop (Q57a) | it ends any booking | `TeamFinishedEndsADepartedBookingToo` |
| The chase's 900 s cut-off counted from the last beacon heard, not from the last contact (§8.3) | the cut-off counts from `last_contact` | `OneWayHearingDoesNotRenewTheContactAge` |
| Done was not terminal: a plan adopted after a no-plan homing pulled a robot at home back into Wait or Meet | Done heads the priority list, and nothing is booked once home | `DoneIsTerminalWhenAPlanArrivesAfter` |
| A deferred departure ended on a one-tick presence lapse of any pair | it ends only after a separation of `book_apart_sec`, the same rule as booking | `DeferredDepartureOutlastsAShortLapse` |
| `exchange_done_trivial` counted only exchanges with both targets at 0 | trivial means both sides already held the targets at contact | `AlreadyCurrentAtContactIsTrivial` |
| Leaving NAVIGATE for a Leg with no pose left the navigator on the exploration goal | the goal is abandoned on that transition | node; needs the sim |
| One pending beacon per sender, kept in order of receipt: an older beacon could replace a newer one within a tick | the newer stamp is kept | node |
| The checker's exit 2 (refused) outranked exit 1 (hard fail) | exit 1 first | calib: mixed root |
| The run script read `NODE`, which npm also sets | the switch is removed | `campaign_guard_calib.sh` |

**Deferred.** None of these blocks phase 1. Each is a cost or an observation
item for the smoke round or phase 2. All ten were later fixed or documented in
§11 (2026-09-24); the section each went to is at the end of its entry.

- **Split recovery.** Recovery keys on the last plan version a peer showed
  directly. It does not cover a peer that skipped a version, or a peer whose
  last direct beacon predates the renewal.
  - Cost: a robot can be alone at an odd slot. This heals at the next direct
    contact.
  - The fix needs a plan-ack mask on the beacon, which is a design change.
  - → §11.1: plan gossip.
- **Exchange retry.** A gave-up exchange is not retried while the contact
  lasts (E3 and E8, as designed).
  - At the cell, a peer whose exchange stalled once leaves the robot
    re-booking until `team_finished`.
  - Retry after a stall is a phase-2 candidate, if campaigns show give-ups at
    meetings.
  - → §11.2: a give-up can still finish while the contact lasts.
- **Proximity exemption.** The Q65 exemption covers the whole Meet activity of
  both robots, including the reconnect move toward a braked peer. There,
  separation rests on the navigator's obstacle grid. The smoke round's
  proximity check watches it. → §11.3.
- **Tick-event skew.** The 2 s tick event carries this tick's activity with the
  previous tick's node state. That gives one skewed sample per transition,
  which the checker's 120 s window absorbs. → §11.4.
- **`leg_legs`.** It counts hold restarts and activity changes as legs. Read it
  as leg starts, not as distinct drives. → §11.5.
- **Hang heartbeat.** At 50 s (× 60 = 3000 sim-s), the hang heartbeat is inert
  at `--duration 3000`. There, the checker's progress rule is the only hang
  detector. → §11.6.
- **PLAN starvation.** Starvation is unbounded in the node, as in gen 33 (Q4).
  The progress rule fails an exploring robot whose candidates have all been
  rejected for 120 s. If the smoke round shows this on healthy runs, choose
  between a starvation latch in the node and an exemption in the checker.
  → §11.7: both.
- **equiv_gate keys.** equiv_gate does not register the new manifest keys, so
  a pre-change parent against a new child fails on them. equiv_gate is now a
  gen-33 tool. → §11.8: it refuses gen-34 cells.
- **Emulator relay discovery.** The emulator's `best_effort_topics` always
  lists `team_beacon`, so a gen-33 COMMS=1 run would poll relay discovery for
  the whole run. Gen 33 no longer runs. → §11.9: the list had the opposite
  problem too, `team_world` in a gen-34 run.
- **Map message layout.** `ScovoxMapBinary` gains `uint64 seq`.
  - Old bags of `scovox_bin` and `fusion_counters` do not deserialize with the
    new build.
  - The run box needs a full rebuild: `scovox_msgs`, then everything that uses
    it.
  - → §11.10: documentation only.

### 10.6 Verification on this machine

- **Build.** The `hmrexplo:humble` container builds with no warnings.
- **Tests.**
  - `explo_planner`: 709 tests pass, 62 of them in `test_team_core`.
  - `scovox_mapping`: 117 tests pass.
  - `test_team_harness` runs its full seed sweep, so `colcon test` takes about
    11 minutes.
- **Calibs.**
  - `gen34_check_calib`: all pass.
  - `rendezvous_agreement_calib`: passes.
  - `campaign_guard_calib.sh`: 177 of 177 pass.
  - `gate_g8_calib` and `equiv_gate_calib`: as in 10.3.
- **Not verified here.** Simulations run only on the run box.

## 11. Fixes for the deferred review findings (2026-09-24)

The ten items §10.5 deferred, fixed before the smoke round. Each entry gives
the defect in one line, the fix, and the tests. Nothing here changes an arm,
a bound in §8.7 or the priority list. Three decisions are amended: E3 (11.2),
K4 (11.7) and Q54's split recovery (11.1). Five Fable 5.1 reviewers attacked
the first draft of this section before anything was built; 11.12 records what
they found and what changed. Four more read the built code; 11.13 records
that review and the verification.

### 11.1 Split recovery: plan gossip

**Defect.** Recovery keys on the version a peer last showed directly, and only
alternates to the plan just before mine. It misses a peer that is two versions
behind, and a peer whose last direct beacon predates its adoption. A robot can
then stand alone at the old cell.

**Fix.** Each robot keeps, for every robot, the highest plan it knows that
robot to hold, and the beacon carries that table.
- `TeamBeacon.msg` gains three arrays indexed by fleet id:
  `plan_known_version`, `plan_known_cell` and `plan_known_center`. Entry
  `[robot_id]` is the sender's own plan. Version 0 means nothing is known.
- TeamCore keeps `known_[j]` = (version, cell, centre). Its own entry is its
  plan, set on every adoption and proposal.
- **Merge.** In `updatePlan`, every tick, from each peer's stored beacon with
  a matching grid hash:
  - the sender's own entry comes from its `plan` fields, never from its table;
  - every other entry of its table merges by maximum version, except the
    receiver's own;
  - the three arrays must all have the team's length, or nothing from the
    table merges and `beacon_gossip_bad` counts (once per beacon, on receipt).
- A version only ever rises, per robot, so a merged entry is a lower bound on
  what that robot holds.
- **Invariant.** After `updatePlan`, the highest entry in my table is my own
  version. An entry v for robot k reached me through a chain of beacons that
  starts at k. The robot X whose beacon brought it held v or more itself
  (X adopted v when it merged it), and I adopt X's plan in the same pass. The
  harness checks this after every tick (property P6).
- **`cellForSlot(k)`.** Even slots use my plan's cell. On odd slots, let L be
  the lowest version of 1 or more in the table, over the other robots. If L is
  below my version, the slot uses L's cell and centre, which travel with the
  entry.
  - L only rises, so a booking's cell changes at most once per version the
    laggard adopts, and back to my cell once it catches up.
- **Retarget.** `bookingPass` re-runs `cellForSlot` for the held booking,
  except a fully met one. After a renewal proposed at a full meeting,
  `cellForSlot` already names the new cell, and the proposer must stay while
  it waits for its peers to show the renewal.
  - Before arrival, a changed cell is taken as today. The event gains `late`
    (true when my lead to the new cell no longer fits before the slot), and a
    late retarget counts `booking_retarget_late`.
  - After arrival, a changed cell is taken only if my lead to it still fits
    before the slot. Arrival, the reconnect state and the contact mask are
    cleared, and the drive gets a new leg key. Otherwise the robot stays.
- **Renewal wait.** A fully met booking waits for every peer to show the
  renewal. It now reads `known_[j].version`, so a renewal relayed by a third
  robot counts.
- `PeerRecord::version_seen` is removed; `prev_plan_` stays for tests and the
  harness.

**Why this converges, and the bound.** Assume that every robot eventually
hears some robot that is in contact with the rest, grid hashes match, and
robot 0 does not propose again meanwhile.
- The robots at the newest version meet on even slots.
- On odd slots the lowest-version robot goes to its own cell, or to the cell
  of a still lower entry it holds for a peer that has since caught up. Every
  entry is a lower bound, so such a detour ends at the next contact or even
  slot. Every robot whose table holds the minimum goes to its cell on odd
  slots.
- One contact spreads a table entry one hop. So within one exchange of
  beacons along a connected chain every table holds the minimum, and the next
  odd slot gathers everyone. Being together on a met slot renews the plan, and
  everyone adopts the renewal.

**Residuals.**
- If every robot's entry for C is older than what C really holds, the
  believers go on odd slots to a cell C left. This needs C to adopt a version
  and drop out of range within one beacon, before anyone hears it, and then
  miss the next version too. It heals on any contact.
- A robot heard only one way (it hears the team, nobody hears it) spreads
  nothing. Its peers keep its last entry, as today.
- A robot at version 0 (never heard a plan) is not in L. It joins at the
  first beacon it hears.
- `pairMet` ignores the cell: two robots listing each other for the same slot
  count as met wherever they stood. That is pre-existing and harmless; it only
  records that they exchanged.

**§8.3 and Q54 amended.** Split recovery is the table above; "the version
each peer last showed directly" and the two-plan alternation are gone.

**Tests.** Replace `SplitRecoveryAlternatesOddSlotsToThePreviousCell`.
- A version skipped (v1 to v3) while a third robot is at v2: odd slots use
  v2's cell.
- A stale direct view healed by gossip: no alternation.
- A laggard two versions behind: odd slots use its cell, which is not
  `prev_plan_`.
- Monotone merge: a lower entry does not lower the table; my own entry and
  the sender's table entry for itself are never taken.
- A grid mismatch merges nothing; a wrong-length table is counted and ignored.
- An arrived booking retargets when the lead fits, and stays when it does not.
- A renewal known only through a relay ends the renewal wait.
- Harness: P6 after every tick, in the property sweep and in the replay,
  whose filter rewrites a beacon's table as well as its plan.

### 11.2 Exchange: a give-up can still finish

**Defect.** A gave-up exchange stays gave-up for the rest of the contact (E3),
even when the maps catch up later. A pair in a long contact at the cell then
can never be met.

**Fix.** A gave-up record keeps watching its targets while the contact lasts.
- If both targets are reached later in the same contact (the live received
  sequence numbers, not the values stored at the give-up), the record becomes
  done: action `done_late`, counter `exchange_done_late`.
- It is its own branch, not the running record's end block: `exchange_done`
  and `exchange_done_trivial` are not raised, so starts = done + gave-up +
  lost + open still holds. The give-up stays counted.
- Nothing waits on a gave-up record. `anyExchangeRunning()` is unchanged, so no
  hold, deferral or bound moves.
- `exchanged(j)` turns true, which feeds `met_mask`, the encounter rule and
  `exchanged_mask`. A chase that ended `failed` on the give-up stays ended.
- **Which booking gets it.** A pair is marked met only while a booking is
  held. The booking's window ends it when no exchange is running, and a
  gave-up record is not running. So a late finish counts for the current
  booking only inside its window, or while another exchange keeps it open.
  Otherwise it counts for the next booking, if the contact lasts that long.
- **E3 amended.** Done stands for the rest of the contact. Gave-up stands
  unless the targets are reached later in the contact.

**Checker.** `exchange done_late` joins the contact claims checked against the
link trace. `exchange_done_late` is optional firing. The give-ups rule is
unchanged: a late finish is still a give-up.

**Tests.** A stalled exchange finishes late and turns `exchanged` true without
holding the window. A total give-up finishes late in the same way. A late
finish after the contact ended does not happen: the record ended. Counters:
`exchange_done` does not move.

### 11.3 Proximity exemption narrowed; reconnect point

**Defect.** The Q65 exemption covers all of Meet for both robots: the drive
to the cell and the reconnect move toward a braked peer. On that move the
walker drives to the peer's own last position.

**What actually guards the reconnect walk.** The walker is the higher id, and
the guard yields only to a lower id (the target). The target is braked, so
after `peer_static_sec` it counts as parked and holds no one beyond
`parked_keep_dist_m` (1.5 m). So the guard never protected this walk, with or
without the exemption. What bounds it is the leg's arrival tolerance
(`leg_arrive_m`, 1.5 m) and the navigator's obstacle grid. In the sim the
guard's band is 1.5 m / 2.5 m (the run script's `PROX_HOLD_M` and
`PROX_RESUME_M`), not the yaml's 5 m / 6 m. The checker's floor is 1.0 m.

**Fix, exemption.** `proximityExempt(j)` holds only when all of these hold:
- I am in Meet, with no reconnect move running.
- I have a pose within `meet_exempt_radius_m` (8 m: `ring_max_m` 6 + 2) of my
  booking's centre.
- j is present, its beacon says Meet, and its beacon position is within the
  same radius of my booking's centre.

A peer booked at another cell (a split) is therefore not exempt. Two robots
driving in from far away are guarded as on any drive. A peer that is walking
inside the 8 m disc is exempt; that is the ring's own traffic, which the
exemption is for.

**Fix, reconnect point.** The first draft put the walker on a 6 m ring around
the peer. Robots stand 4.2 to 6 m apart on the meeting ring, so that point is
where the walker already is (review, 11.12). Instead:
- Let P be the peer's last heard position and W the walker's pose.
- If the map shows line of sight from P to W, the map does not explain the
  lost link. The walker drives toward P, as today.
- Otherwise it tries points around P at radius r = |W − P|, clamped to
  [`meeting_ring_m`, `ring_max_m`] (3 to 6 m), at bearings 30°, 60°, 90°,
  120° and 150° either side of W's bearing from P, in that order.
  - A point must be at least 2 × `leg_arrive_m` (3 m) from W, so the move is
    real.
  - It must be standable, with line of sight from P.
  - The first one wins. With none, the walker drives toward P.
- A walk to a sight point that ends with no contact falls back to a walk
  toward P (`reconnect_fallback`). A walk toward P that ends with no contact
  ends the reconnect, as today.
- The walk ends on contact, or on arrival within `leg_arrive_m`.
- The move event says which it was (`kind`: `sight` or `peer`).
- Everything stays inside the booking's patience and backstop. No bound moves.

**Line of sight.** A braked robot is mapped as an obstacle, so a ray that
ends inside it was always blocked. The plan map is also inflated
(`global_planning_map_inflation_m`, 1.5 m in the sim launch, at 0.4 m cells),
so a mapped robot is a disc about 2.3 m in radius, not a body.
- At each end, the oracle skips the run of occupied samples that starts at
  the end, for up to `sight_end_clear_m` (a new node parameter, 3.0 m: the
  inflation, a Husky's half-diagonal, and a cell of rounding at each end of
  the inflation, about 2.8 m). An occupied sample after the first free one
  always blocks, even inside that distance.
- A free end (a candidate point, which must be standable) has no run, so
  nothing is skipped there.
- Two ends under 2 × `sight_end_clear_m` apart whose runs meet read clear:
  the map cannot tell two robots' discs from a wall between them. The walker
  then drives toward P, which is the move when the map cannot explain the loss.
- The first draft skipped a flat 0.5 m, which never left the inflated disc
  (code review, 11.13).

This also fixes `followPoint` and `pickEscape`, which ask the same question.
Unknown cells still do not block: map sight is not radio sight, and the
fallback toward P covers the difference.

**Two id orders.** TeamCore picks the walker by fleet id. The guard yields by
robot name. They agree when the roster lists names in sorted order. The node
warns once at start if it does not. A mismatch cannot deadlock: the target is
braked and parked.

**Tests.**
- Exemption: true near the centre for both; false when either is far from it;
  false when the peer is booked at another cell; false while walking.
- Reconnect: with sight between P and W, the point is P. With sight blocked,
  a point with sight is chosen, at least 3 m from W. With every point
  blocked, the point is P. A sight walk without contact falls back to P.
- `FakeOracle` gains segment blockers for line of sight.
- `LostPairAtTheCellHigherIdMovesLowerHolds` keeps P (its world has sight).
- `segmentClear`: a ray ending in an occupied cell is clear; an inflated
  robot disc at each end is skipped; an obstacle past a free cell blocks
  inside `sight_end_clear_m`; a run longer than it blocks.

### 11.4 Tick event after the dispatch

**Defect.** The 2 s tick event is written inside `teamTick`, before
`applyActivity` and the dispatch. It pairs this tick's activity with last
tick's state.

**Fix.** `tick()` becomes a wrapper around the old body (`tickBody()`).
- `teamTick` marks the event due, and advances the next due time there.
- The wrapper writes it after the body, including after the proximity-hold
  early return. Activity, state, Leg status and the hold flag then describe
  the same instant.
- The event's time is the tick's own `now`, passed through, not a second read
  of the clock.
- Events the dispatch writes (a state change) now come before the tick event
  of the same tick. The checker reads ticks and changes separately, so the
  order does not matter to it.

### 11.5 Leg counters

**Defect.** `leg_legs` counts every leg (re)start. A hold, or an explore stint
between two parts of one drive, counts as a new leg.

**Fix.** `legs()` counts distinct drive keys. TeamCore's keys are unique and
rise, and the tracker keeps the last counted key in its own field, which
`reset()` does not clear. A new `starts()` counts every (re)start. `run_end`
reports `leg_legs` and `leg_starts`. `ResetForgetsTheLeg` expects one leg and
two starts.

A key is a TeamCore drive segment, not a trip: a Meet booking spends one key
to the cell and more on a reconnect, a chase one per point, and a retarget
before arrival keeps its key.

### 11.6 Hang heartbeat

**Defect.** The run script's hang gate counts only `selected goal` lines, so
it has to sit above a meeting's full course: 50 heartbeats, or 3000 sim-s.
That is inert at `--duration 3000`. And any robot's "Exploration complete"
disarms it for everyone.

**Fix.**
- The node prints `Team activity: <activity> (state <from> -> <to>)` when
  `applyActivity` moves the state machine for a new activity. It is printed
  where the state follows, not where TeamCore changes its mind: a robot in
  WAIT_FOR_MAP that TeamCore books and releases prints nothing.
- No other node line may contain `Team activity:`.
- The gate counts `selected goal` and `Team activity:` lines, only over robots
  that have not printed "Exploration complete". It fires when none of those
  advanced for `HANG_HB` heartbeats. With every robot finished, it does not
  run.
- `HANG_HB` defaults to the larger of 25 and ⌈(12 × `ROI_HALF` + 600) / 60⌉:
  25 at the default ROI (1500 sim-s). A smaller explicit value warns. 0 is
  refused (exit 2): it would kill the cell at the first quiet heartbeat. To
  disarm the gate, set it above the run's length in minutes.
- The manifest records `hang_hb` and `hang_window_sim_s`, in place of the
  misnamed `hang_hb_sim_s`.
- The heartbeat and HUNG lines say what they count.

**Sizing.** Per robot still exploring, the longest stretch with no goal and no
state-following activity change:
- One meeting: from the last departure to the backstop. That is at most the
  latest departer's lead plus 600 s. Lead is 3 s per metre of path, and a path
  inside a square ROI of half-width h is taken as at most 4h. At h = 50 that
  is 600 + 600 = 1200 s. The 4h is assumed, not enforced (the lead has no
  cap); a map whose paths run longer needs a larger `HANG_HB`.
- A chase is 600 s at most.
- Wait and Home need a finished robot, which leaves the gate.
- PLAN starvation ends in the 11.7 latch.
- The INERT warning stays, for shorter durations.

### 11.7 PLAN starvation latch

**Defect.** A robot whose candidates are all rejected (or that has none)
stays in PLAN indefinitely, as in gen 33. The progress rule fails such a run
after 120 s.

**Fix.** Starvation becomes a bounded wait that ends in a finish.
- **The clock.** It opens at the first starved PLAN tick (no candidates, or
  all rejected), anchored at the pose.
  - It counts only while the activity is Explore and the robot is not in a
    proximity hold.
  - A team activity pauses it, and the return to Explore re-anchors it at the
    current pose. A proximity hold pauses it without re-anchoring.
  - Moving 1.0 m from the anchor while it counts closes it.
  - Selecting a goal does not close it. A robot that picks a goal, fails
    without moving and starves again stays on one clock. Goals selected while
    it is open are counted.
  - The clock is not reset by a meeting. With 300 s slots, a robot that met
    on every slot would otherwise never finish.
  - The no-map path does not start the clock: a silent dscovox is a fault,
    not the end of exploring. A clock already open keeps counting through
    WAIT_FOR_MAP, where the activity is still Explore, so a map lost with the
    clock open ends in the latch and the robot goes home.
- **The latch.** At `plan_starve_finish_sec` (300 s) of counted time, the node
  latches finished with reason `starved`. The check runs every tick, before
  TeamCore's tick, so the latch holds in NAVIGATE too. 0 disables the clock.
- **One ending per robot.** `exploration_complete` is written once. A coverage
  crossing after another ending (peer maps arriving at a meeting) is recorded
  as the coverage latch, with its own log line that does not say
  "Exploration complete", and no second event.
- **K4 amended.** Finished = the coverage latch, the step budget, or
  starvation.
- **Starved is a mission ending, not an exploration endpoint.** A starved
  robot stopped for want of work, not because it reached coverage. Its done
  time includes up to 300 s of counted waiting, and more in the meeting arms,
  where the clock pauses. The metric reader keys exploration-done on
  `coverage-latched`, as gen 33's did, and treats a starved robot as
  censored. The event gives the reader what it needs.
- **Logging.**
  - The tick event gains `plan_starved_sec` (counted time) and
    `plan_starved_open_sec` (when the clock opened). Both are written only
    while the clock counts; null otherwise, and always null with the latch
    disabled.
  - While it counts, the tick's wait fields carry the starvation bound:
    `now - counted` and `now - counted + plan_starve_finish_sec`.
  - A starved `exploration_complete` carries `starved_open_t_sim`,
    `starved_paused_sec` and `starved_goals`.
  - `run_start` stamps `plan_starve_finish_sec`.
  - `run_end` counts `plan_starved` (clock openings) and
    `plan_starved_finish` (latches).
  - "No valid candidates" is throttled to one line per 5 s, like the
    all-rejected line.
- **A navigation fault under an open clock.** A robot whose goals all fail
  without moving stays on the clock and latches `starved`. The checker is
  blind to it while the clock counts. It is caught after the latch: the Home
  or Meet leg must move (progress), and the robot must reach home (horizon).
  `starved_goals` in the event separates the two for the reader. The checker
  does not fail on it: a frontier the amnesty re-offers and the robot cannot
  reach looks the same.
- **Checker.**
  - `should_move` is false on a tick with `plan_starved_sec` and
    `plan_starved_open_sec` both set.
  - The existing waits rule enforces the bound through `wait_bound_sec`.
  - **Consistency.** On consecutive ticks of one clock (same open time), the
    counter may not run faster than the clock (0.3 s slack), and over any
    stretch of 30 s or more it must advance by at least half of it. Either
    violation fails `waits`: a stuck counter would otherwise hold the
    exemption open forever.
  - **Backstop.** An exploring robot (activity Explore, state not
    WAIT_FOR_MAP or DONE) that moves less than 0.5 m in 540 s fails
    `progress`, whatever its starvation clock claims. 540 s = the starvation
    limit + two progress windows. Proximity-hold ticks pause the window, as
    they pause the node's clock; they do not restart it.
  - **Windows.** Both still-windows (120 s and 540 s) close at the first tick
    at least the window past their start. Tick events are 2 s apart plus
    executor lag, so a window that needs a tick exactly W s back would almost
    never close (code review, 11.13).
  - BOUNDS pins `plan_starve_finish_sec` at 300.
  - Both counters are optional firing.
  - Starved finishes are listed in the cell's notes with their open time,
    paused time and goal count.
- **Calib.**
  - A starved robot still for 250 s passes progress.
  - A starved tick past the bound fails waits.
  - A stamped 200 fails bounds.
  - A stuck counter fails waits; a counter faster than time fails waits.
  - A robot still for 560 s under claims that each look valid (clocks
    reopening) fails progress on the backstop; also with jittered ticks.
  - With 40 s of that held, it passes; still 580 s with 20 s held fails.
  - Still for 130 s on jittered ticks fails progress.
  - A tick with a count and no open time is not exempt.

### 11.8 equiv_gate

**Defects.**
- A gen-34 child carries five manifest keys that equiv_gate does not register
  (`node`, `planner_exe`, `arm`, `team_exchange_topic`,
  `best_effort_priority`). A banked gen-33 parent against it fails five times,
  for the wrong reason.
- The gate's default parser reads `dp(` but not `dp_f(`, so `fov_hfov` looks
  undeclared. This is the calib failure 10.3 carried.
- `dp_f` narrows its default to float, and the run logs the float widened
  back. 6.28318 comes back as 6.283180236816406, outside the gate's 1e-9
  tolerance.
- The gate reads the event vocabulary from the current header, while its
  defaults come from gen 33's backed-up node.

**Fix.**
- A side whose manifest names a node other than gen33 is refused with exit 2.
  An absent `node` means gen33: the key arrived with gen 34.
  - The refusal names `gen34_check.py`.
  - The gate reads gen 33's node source, and a gen-33 parent against a gen-34
    child is not a default-off equivalence claim.
  - Using equiv_gate between gen-34 runs (K17) needs gen 34's vocabulary and
    node source. That is its own change.
- The parser reads `dp_f(` too, and narrows those defaults through float32
  before comparing.
- Gen 33's `experiment_log.hpp` is backed up beside its node, and the gate
  reads the vocabulary from there.
- `equiv_pair.sh` refuses up front when the child's run script pins gen 34,
  before two builds and two runs.

**Calib.** Adds a gen-34 child refused (rc 2) and a gen-34 parent refused
(rc 2). The 360° FOV case passes again; its fixture writes the widened float.
The real-root audit skips cells whose manifest names a node other than gen33,
so a banked gen-34 cell does not break it.

### 11.9 Emulator relay list

**Defect.** The shared `best_effort_topics` lists `exploration/team_world`,
which the gen-34 node never publishes. That topic stays pending for the whole
run:
- the discovery poll runs every second,
- a "waiting" line prints every 10 s,
- "All relay topics discovered" never prints.

The yaml comment claims such a topic "costs nothing".

**Fix.**
- The list becomes `exploration/intents` and `exploration/team_beacon`, and
  the comment says what a never-published entry costs.
- The waiting line names the pending topics.
- The emulator warns once when topics are still pending
  `pending_warn_sec` (120 s) after the last topic it discovered, or after its
  first poll if none. The planners start a minute or more after the emulator, so
  the anchor is the last discovery, not the start. Discovery keeps polling,
  so a late publisher is still relayed.

### 11.10 Map message layout

Documentation only. `ScovoxMapBinary` gained `uint64 seq`, so old bags of
`scovox_bin` do not deserialize with the new build (`comms_gates.py` reads
them). `ScovoxFusionCounters` changed in the same scovox commit. The run box
needs a full rebuild: `scovox_msgs` first, then everything that uses it.
`TeamBeacon.msg` changes again in 11.1, so `explo_planner_msgs` rebuilds too.
The run-box prompt at the next push says both.

### 11.11 Not changed

- **Gen-33 run-script machinery.** The arm-token and `mtare_*` blocks after
  the gen-34 planner block stay.
  - The pinned `mtare_off` token still sets `CELL_WORLD`, `TEAM_WORLD` and
    `GLOBAL_ALLOC` to 1. Those configure the cell world and the allocator
    that the gen-34 oracle reads.
  - Removing the blocks would change the manifest and the comparability key.
    That is a campaign change, not a fix.
- **Schema.** The schema stays at 13. The new fields, the `done_late` action
  and the new counters are additive, and no schema-13 data exist yet: gen 34
  has not reached the run box. The new TeamCore counters are in
  `counterNames()`, so `run_end` reports them at zero.

### 11.12 Adversarial review of this section

Five Fable 5.1 reviewers read the first draft against the code before any of
it was built. Four were steered: 11.1; 11.2 with 11.3; 11.4 to 11.7; and the
scripts (11.6, 11.8 to 11.10). One was unsteered. Each finding was checked
against the code before it was taken. The sections above are the result.

**Taken.**

| Finding | Severity | Change |
|---|---|---|
| The 6 m reconnect ring sits where the walker already stands (spots are 4.2 to 6 m apart), so the move degenerates | high | 11.3: tangential sight points at least 3 m away, fallback toward P |
| An arrived booking never retargets, so a robot that learns the laggard caught up stays at the old cell | medium | 11.1: retarget after arrival when the lead fits |
| The renewal wait reads direct beacons only | medium | 11.1: reads the table |
| Convergence was stated, not bounded; one-way contact and version 0 unmentioned | medium | 11.1: assumptions, bound, residuals, P6 |
| Merging in `onBeacon` lets a beacon's table race its own plan | medium | 11.1: merge in `updatePlan` from stored beacons |
| The sender's table entry for itself could override its plan fields | low | 11.1: taken from `plan` only |
| The starvation clock counts proximity holds (two reviewers) | medium | 11.7: a hold pauses it |
| A starved robot could write `exploration_complete` twice (coverage after the finish) | medium | 11.7: one ending per robot |
| Starved done time is biased by up to 300 s, more in meeting arms, and was counted as an exploration endpoint (two reviewers) | medium | 11.7: starved is a mission ending; the event carries open time, paused time, goals |
| The checker's exemption is a pure self-claim | medium | 11.7: consistency rule and a claim-free backstop |
| A disabled latch still wrote a bound, with 300 as a literal | low | 11.7: null when disabled; the bound uses the parameter |
| The sight ray ends inside the braked peer, which is mapped | medium | 11.3: the oracle ignores 0.5 m at each end |
| The design named the guard's 5 m hold as the reconnect's margin; the sim runs 1.5 m / 2.5 m, and the guard never held this walk | medium | 11.3: states the real bound |
| A mapless robot's activity lines would keep the hang gate quiet | medium | 11.6: printed where the state follows |
| One finished robot disarms the hang gate for all | low | 11.6: per-robot |
| `HANG_HB` sized on an unstated ROI | low | 11.6: derived from `ROI_HALF` |
| `dp_f` values come back widened from float | medium | 11.8: narrowed before comparing |
| The calib's real-root audit breaks on the first banked gen-34 cell | medium | 11.8: skips non-gen-33 cells |
| The emulator's warning anchored on its own start, before the planners exist | low | 11.9: anchored on the last discovery |
| `equiv_pair.sh` builds and runs twice before the refusal | low | 11.8: refuses first |
| The gate's vocabulary comes from the current header | low | 11.8: gen 33's header backed up |
| A late finish may count for the next booking, not the current one | low | 11.2: stated |
| Implementation hazards: `done_late` in its own branch, live sequence numbers, counters listed | — | 11.2, 11.11 |
| Tick event re-read the clock; due time advanced at write | low | 11.4 |
| `leg_legs` counts drive segments; `reset()` clears `key_` | low | 11.5 |
| Two id orders (fleet id, name) | low | 11.3: warn at start |
| §8.3 and Q54 left stale | low | amended |
| `fusion_counters` wording in 11.10 | info | fixed |

**Not taken.**

| Finding | Why not |
|---|---|
| Fail a starved finish that selected goals (`starved-navfail`) | A frontier the amnesty re-offers and the robot cannot reach looks the same. A real navigation fault still fails the run at the Home or Meet leg (progress) or at the horizon. The goal count is in the event for the reader |
| Close the clock on goal reached instead of 1 m of motion | A robot that drives far toward goals it never reaches is working, not starved. 1 m keeps that robot off the clock |
| Shorten the latch to 240 s | With starved finishes censored in the metric (11.7), the latch only decides when the robot heads home. 300 s stays, one meeting interval |
| Treat unknown cells as blocking for the sight query | Most of a cell's surroundings may be unknown, and every candidate would fail. The fallback toward P covers an unmapped blocker |
| Freeze the cell at departure | L only rises, so the cell changes at most once per laggard version, and the lead check stops a late change after arrival |
| Let a rising gave-up record hold the window | It would extend holds, which 11.2 promises not to do |

### 11.13 Code review after the build, and verification

Four Fable 5.1 reviewers read the built change. Three were steered: TeamCore
and Leg; node integration; the scripts, checker and emulator. One was
unsteered. Each finding was checked against the code before it was taken.

**Taken.**

| Finding | Severity | Change |
|---|---|---|
| The 0.5 m end skip never leaves a robot's disc on the 1.5 m-inflated plan map, so every ray from a mapped robot was blocked: sight points, `followPoint` and `pickEscape` were inert in the sim | high | 11.3: skip the occupied run at each end, up to `sight_end_clear_m` (3.0 m) |
| Both checker still-windows closed only on a tick exactly W s back. Tick events are 2 s apart plus executor lag, so the 120 s progress rule and the 540 s backstop almost never fired on a real cell | high | 11.7: a window closes at the first tick at least W past its start; calib cases on jittered ticks |
| `backup/gen33/experiment_log.hpp` was untracked, yet it is now the gate's default header (two reviewers) | medium | committed with this change |
| The backstop counted proximity-hold ticks, which pause the node's clock | low | 11.7: a hold pauses the window |
| A tick with a starvation count and no open time was exempt, yet the consistency rule skips it | low | 11.7: both fields required |
| `HANG_HB=0` was accepted and kills the cell at the first quiet heartbeat | low | 11.6: refused |
| The hang sizing's 4h path bound is not enforced | low | 11.6 and the script say so |
| "The lowest-version robot always goes to its own cell" is false when it holds a still lower, stale entry | low | 11.1: reworded; the detour is bounded |
| The `!B.full_met` retarget guard was not in the text | low | 11.1 |
| The self-skip assertion of the merge test could not fail: adoption rewrote my entry after the merge | low | the claim is sent on a beacon that adopts nothing |
| P6 was not checked in the replay | low | checked there too |
| The emulator's comment on the sim clock was inexact | info | reworded |

Both strengthened tests were checked by mutation: with the merge's self-skip
removed, `TheMergeOnlyRisesAndSkipsMyEntryAndTheSendersOwn` fails; with the
replay filter's table rewrite removed, the replay fails on P6. Each new calib
case fails against the old window logic, the old hold handling, or the old
exemption.

**Not taken.**

| Finding | Why not |
|---|---|
| A robot that loses its pose with the clock open keeps counting and latches `starved` | A fault case. The Home leg then fails `waits` |
| A tick across an activity change is booked by the previous tick's view | At most 0.1 s per transition, inside the 0.3 s slack |
| The coverage-latch time after another ending is in the console log only | No reader needs it. Add a `run_end` field if the metric does |
| A held robot's activity line names the resume state, not PROXIMITY_HOLD | Cosmetic; the gate only counts the line |
| A counter at 1.1× time is not caught | It latches early; it cannot hide a hang |
| A laggard that adopts at radio range retargets before arrival while arrived peers stay | As before §11 |
| `rc_sight` stays set after a sight walk ends by contact | Every reader also requires `rc_active` |

The build's own test run found one more: the new `segmentClear` test placed
its ray on a cell edge, where the float resolution put it in the row below.
The test now uses cell centres.

**Verification on this machine.**
- **Build.** All six packages build in `hmrexplo:humble` with no warnings.
- **Tests.**
  - `explo_planner`: 727 tests pass, 78 of them in `test_team_core` and 10 in
    `test_plan_map_query`.
  - `scovox_mapping`: 117 tests pass.
- **Calibs.**
  - `gen34_check_calib`: all pass (95 cases).
  - `rendezvous_agreement_calib`: all pass (31).
  - `campaign_guard_calib.sh`: 177 of 177 pass.
  - `gate_g8_calib` and `equiv_gate_calib`: every case passes except the
    UNRESOLVED "no campaign root" line, which needs the run box's data.
- **Emulator.** Run in the container with two robots, `pending_warn_sec` 4
  and one topic published late: the waiting line named the six pending
  topics, the late topic was relayed, and the one-time warning named the five
  left, 5 s after that discovery.
- **Not verified here.** Simulations run only on the run box. Its next build
  must be a full one: `scovox_msgs` and `explo_planner_msgs` changed (11.10).

## 12. Full-map sharing at N=2 (2026-09-24)

### 12.1 Question

How much does data selection matter for exploration? Today each robot sends
its peers only what changed, and only when the change is significant. The
new stream sends the whole map instead. The comparison is the `off` arm at
N=2, and it compares two protocols, not selection alone (12.12 R2):
- **control:** selected changes, on a reliable FIFO that delivers every
  delta, however stale.
- **full:** the whole map, on a link that keeps only the newest unsent frame.

A whole-map stream supersedes by nature; a delta stream cannot drop a delta
without losing voxels. Each arm is run as its natural protocol.

| Cell | Map stream | Seed | Root / tag |
|---|---|---|---|
| control | deltas | 1 | `gen34_n2_ctl / gen34n2ctl_off_seed1` |
| control | deltas | 2 | `gen34_n2_ctl / gen34n2ctl_off_seed2` |
| full | whole map | 1 | `gen34_n2_full / gen34n2full_off_seed1` |
| full | whole map | 2 | `gen34_n2_full / gen34n2full_off_seed2` |

All four cells run the new build with `TX_MODEL=progressive` (12.12 R1).
The banked `gen34n2_off_seed1` is not a control: it ran the admission
emulator model and older code, and bestla's planner explored a frozen fused
map from sim time ~871 s (pilot finding 1). The cells run on this machine
after the local pilot queue (N=3 `off` and `pursuit`, then the N=2 `hybrid`
retry), in the order ctl:1, full:1, ctl:2, full:2.

**Endpoints, named before the runs.** Primary: `t_mission`, the project's
pre-registered endpoint (mission end, censored at 3000 s). Secondary: team
coverage over time, per-frame age at the receiver (full arm), delivered map
bytes per direction. Each pair is read against the replication SD
(`replication_sd.py`); two seeds show direction and size, not significance.

### 12.2 What "whole map" means

Every full message carries every voxel of the sender's own map that is not at
the prior. The packing per voxel does not change:
- 8³ block coordinates (a 64 B bitmask, or a u16 index list),
- u8 sqrt-companded evidence (`quant_step` = saturation / 255²),
- LZ4 over the frame.

What goes away is the selection:
- **Touched-only.** Deltas carry only voxels written since the last tick.
  The full frame carries every voxel.
- **Change gate.** Deltas drop changes below `p_eps` 0.02 or `evidence_rel`
  0.10. The full frame has no gate, so it also carries the small changes
  deltas never send. The full arm's receivers therefore hold slightly more
  accurate maps whenever a frame gets through.
- **Coalescing** stays the same: one frame per share tick.

Unchanged, as in the deltas: the optional z-band clip (off here),
`share_tsdf` (false), Dir sections, and the `state_flip` binarize transform
(off here). Dir sections are not empty for lidar: `share_dir` is on, and the
pilot's deltas carried ~6k Dir records per frame, 5.8–9.4 M over a run
(12.12 R9).

The full frame is the sender's own map only. It never carries the peer's
voxels, so a robot relays nothing it heard from someone else.

### 12.3 Rate, and what the link does with a stale frame

**Decision.** A full frame every 0.5 s (sim time), the same cadence as the
delta stream (`share_rate_hz` 2). The link keeps only the newest unsent full
frame from each sender. Everything else is held equal to the control.

**Why the link must supersede.** A full frame is self-contained, and a newer
one makes every older one worthless. Under the current reliable FIFO:
- Demand would be ~2 frames/s × ~9 MB late in a run, about 18 MB/s.
- The link carries at most ~2.7 MB/s per direction: 72 Mbps × 0.6 airtime,
  shared by the two directions.
- The backlog would reach the 1 GiB cap in minutes. Delivered frames would
  then be about 400 s old.

That compares data selection against a broken queue, not against a
full-map protocol. A real full-map system sends its latest map and drops the
old ones, so the emulator gets a new per-topic policy for this stream,
`latest`:
- Reliable, like today's policy.
- When a new message arrives, any message from the same sender on the same
  topic that is still queued for that receiver (not yet started on the air)
  is removed and counted as `drop_superseded`.
- A frame already on the air is never cancelled by a newer one. Under the
  progressive model (12.12 R1) a link drop aborts it (`drop_aborted`), and
  the newest frame goes at the next contact.

**Why 0.5 s and not slower.** Once the link supersedes, a shorter period
costs only sender CPU. A longer period makes delivered frames older and
handicaps the full arm for no reason. Frame sizes, from the pilot:
- Delta traffic measured 3.2–3.6 B per record, 3.5–3.9 B per Beta voxel
  once Dir records are counted (12.12 R9). Full frames pack denser blocks.
- Each robot's own map reaches ~4 M voxels at 0.20 m, so ~10–14 MB late in
  a run.

At 72 Mbps a 10 MB frame is 1.1 s of air. With both directions sending,
each gets half of the 0.6 airtime, so a frame completes every ~3.7 s per
direction; with one direction idle, every ~1.9 s. A 2 s period would add up
to 1.5 s of age for nothing. At the start of a contact the link climbs
7.2 → 28.9 → 72 Mbps over ~3 s, and the progressive model charges each bit
at the tier it is sent at.

**What the link looks like in the pilot.** In `gen34n2_off_seed1` the link
was down 80 % of the run and at 72 Mbps for 18 %. Contact is short, so what
matters is what arrives in the first seconds of a contact:
- **Control.** It must first flush its whole outage backlog, delivered FIFO.
  The pilot saw backlogs of 107 and 124 MB, about 40–46 s of air.
- **Full arm.** Its first frame is the current map, ~2–4 s of air at the
  contact's climbing tier; a contact shorter than that delivers nothing.

This is the effect the experiment measures. The design does not predict
which arm wins.

**CPU.** The sender copies its grid on the executor thread, then serializes
and compresses on a worker thread (12.4). The receiver ingests each frame on
its single executor thread. Neither is insulated from sim time: while a
callback runs the clock keeps going, and the node's other callbacks wait
(12.12 R7). Both costs are logged (sender `copy_ms`, `work_ms`; receiver
`wall_ms` per frame) and benchmarked at 4 M voxels (12.10). If the worker
is still busy at the next tick, that tick is skipped and counted. RTF per
cell is reported as a covariate.

### 12.4 Sender (`scovox_node`)

- **Parameter** `share_full_map_period_s` (double, default 0 = off). When
  > 0, the node publishes a full frame on `~/scovox_full`, i.e.
  `scovox_node/scovox_full`:
  - Type `ScovoxMapBinary`; the message is unchanged, so no rebuild of
    message packages.
  - QoS reliable, KeepLast(2), volatile.
- **Timer** on the node clock (sim time), running alongside the delta
  timer. `scovox_bin` is unchanged: the robot's own `dscovox_node` still
  reads its own deltas at 2 Hz. Only the link payload changes.
- **Tick**, on the executor thread:
  1. If the worker is busy, skip the tick and count `skip_busy`.
  2. Look up `map_from_source` with a zero timeout. On a miss, skip and
     count it.
  3. Take `map_mtx_` shared, and walk the Beta grid (and Dir when
     `share_dir`) into a frame, keeping every voxel not at the prior.
     Apply the z-band and wire transforms exactly as the delta path does.
  4. Stamp `seq = ++share_seq_` and `header.stamp = now`, both at capture,
     still under the lock (see 12.6). Release the lock.
  5. An empty frame publishes nothing and consumes no seq.
- **Worker thread**, one per frame, as the `mem_log` worker does: an atomic
  in-flight flag, the previous thread joined before the next starts, the
  last joined in the destructor. Serialize, LZ4, publish. It never reads
  the grid. An LZ4
  failure drops the frame and counts it. The seq is then a permanent gap
  that a receiver cannot tell from a superseded frame, which is harmless:
  the next frame supersedes it.
- **Isolation from the delta stream.** The full path does not drain touched
  sets, write the gate grids, use the deferral queue or the byte budget, or
  chunk (`share_max_voxels_per_msg` is ignored for full frames). A chunked
  full frame under `latest` would lose chunks, and the receiver would hold a
  partial map with no way to tell.
- **Stats.** A WARN-level line every 60 s, prefixed `full-map share:`, because
  the node runs at `--log-level warn`. It gives frames published, busy
  skips, TF skips, the last frame's voxels, raw bytes and LZ4 bytes, and
  the copy and worker times (mean and max).

### 12.5 Receiver (`dscovox_node`)

- Peers' full frames arrive on `/<self>/rx/<peer>/scovox_node/scovox_full`,
  through `peer_bin_topic_pattern`. The robot's own input stays
  `/<self>/scovox_node/scovox_bin`.
- Ingest is unchanged. It is already snapshot-replace per voxel, so a full
  frame is simply a large delta.
- **New parameter** `skip_unchanged_voxels` (bool, default false; true in
  the full arm). A voxel whose incoming value equals the stored one
  bit-for-bit is not added to the refold set.
  - Refold is a pure function of the sources' values, so the fused map is
    identical.
  - Only `cells_touched`, a diagnostic, counts fewer cells.
  - Without it, each 4 M-voxel frame would refold 4 M cells under the
    exclusive lock, several seconds of wall time, starving the fused-map
    publish. The pilot's frozen-map fault (finding 1) is a publish that
    stopped reaching the planner, so adding lock time there is the wrong
    direction.
  - Default false keeps every other arm bit-identical.
- **Diagnostics** (full arm only): a separate INFO line per peer frame
  (the existing "integrated update" line parses unchanged) giving the seq,
  voxels, unchanged voxels, age (`now − header.stamp`) and the callback's
  wall time.
- **Full-arm only, from review (12.12 R7):** the touched sets are not
  reserved to the frame size, and a frame that changed nothing does not set
  `fused_dirty_`, so it does not force a fused-map republish.
- **Subscription depth.** Topics ending in `scovox_full` subscribe with
  `scovox_full_qos_depth` (default 2), not `scovox_bin_qos_depth` (4000):
  frames supersede, and 4000 × 10 MB of history is memory no run has
  (12.12 R6).
- **Voxels that fall back to the prior.** They are never sent, as in the
  delta stream (at-prior voxels are skipped there too). A receiver keeps
  its last value. The two arms behave the same.

### 12.6 Sequence numbers and exchange accounting (Q51)

The full stream and the delta stream share `share_seq_`, stamped at capture
on the executor thread, the only thread that increments it.
- A full frame with seq k holds the grid as it stood at capture. Every delta
  with seq < k was built from touched sets drained before then, so frame k
  covers all of them.
- The exchange targets are unchanged:
  - `map_sent_seq` is the newest delta seq the robot's own `dscovox` has
    seen.
  - `target_rx` is the peer's `map_sent_seq`.
  - The exchange completes when the receiver's newest seq from that peer is
    ≥ `target_rx`, which happens with the first full frame captured after the
    peer's newest delta. That is the right meaning: the receiver holds
    everything the peer had sent.
- Capture-time stamping matters. If seq were stamped at publish on the
  worker, a delta captured after the frame could get a lower seq, and the
  exchange could complete while the receiver was still missing it.
- `seq_gaps` for a peer grows by the deltas between frames, so it becomes
  meaningless in the full arm. It is a diagnostic only (the `gaps` field in
  exchange events). The analysis must not read it for this arm.
- The robot's own delta stream also shows gaps, one per full frame, for the
  same reason. Also diagnostic only.

### 12.7 Emulator (`hmr_comms_sim`)

- **New parameter** `latest_topics` (string list, default empty), with the
  reliable queue and cap and the rule in 12.3.
  - A topic may appear in only one of the three lists; the node refuses to
    start otherwise.
  - The "nothing configured" warning counts all three lists.
- **Supersede.** On enqueue for a `latest` topic, before the cap check:
  1. Remove queued entries in that directional queue whose `pub` is the same
     publisher (same sender, receiver and topic).
  2. Subtract their bytes, and add each to `drop_superseded`.
  3. Then push, apply the cap, and drain, as today.
- **Stats.** The stats JSON and the link-counters line gain
  `"drop_superseded"`, `"drop_aborted"` and `"latest_relayed"`.
  - None is added to `drop_overflow`, so the overflow gate still fails only
    on a real loss of deltas.
  - None is added to the "relay totals" dropped sum, which
    `manoeuvre_events.py` reads as `dropped_per_s` (12.12 R4).
- **Launch.** A new `comms_sim.launch.py` argument `map_stream:=delta|full`
  (default: the params file). With `full`, the reliable list loses
  `scovox_node/scovox_bin` and the latest list gets
  `scovox_node/scovox_full`. The launcher prints the override like the
  other ones. If that empties the reliable list, the override is `[""]`
  (an override cannot carry an empty list; the node drops empty entries).
- **Relay QoS.** It mirrors the publisher's reliability as for reliable
  topics, but latest topics use `latest_qos_depth` (default 2) instead of
  `rx_qos_depth` for the source subscription and the rx publisher (12.12 R6).
- **Transmission model** `transmission_model:=admission|progressive`
  (default admission, every banked run). See 12.12 R1.

### 12.8 Scripts

**Runner** (`run_explo_sim_rviz.sh`):
- **Knob** `SHARE_FULL_PERIOD_S`, default 0.
  - Anything other than 0 or a positive number is fatal.
  - A value > 0 with `COMMS` other than 1 is fatal. Without the emulator
    there is no link to measure, and `peer_bin_topic_pattern` would read
    peers directly.
- **When > 0:**
  - The comms launch gets `map_stream:=full`.
  - The nav launch gets `share_full_map_period_s:=$SHARE_FULL_PERIOD_S` and
    `skip_unchanged_voxels:=true`.
  - `PEER_BIN_PATTERN` becomes `/{self}/rx/{peer}/scovox_node/scovox_full`.
  - The `comms_gates.py check` call gets `--map-suffix
    scovox_node/scovox_full` (`watch` does not read the suffixes).
  - The comms bag (`RECORD`) takes the `scovox_full` relays instead of
    `scovox_bin`.
- **Knob** `TX_MODEL=admission|progressive`, default admission; anything
  else is fatal, and progressive needs `COMMS=1`. Passed to the comms launch
  as `transmission_model`.
- **Manifest:**
  - `share_full_period_s=<value>`
  - `map_stream=delta|full`
  - `transmission_model=admission|progressive`
  - `sha256_` of `scovox_mapping_node`, `dscovox_mapping_node` and
    `hmr_comms_sim_node` (12.12 R13)
- The `/rx/` wait needs no change; it matches any relay.

**`simple_nav_3d.launch.py`:**
- New arguments `share_full_map_period_s` (default 0.0) and
  `skip_unchanged_voxels` (default false).
- Each is passed to its node only when not at its default, so every other
  launch keeps the exact parameter set it has now.

**`comms_gates.py`:**
- `--map-suffix` (default `scovox_node/scovox_bin`) replaces the map entry
  of `GATED_SUFFIXES`, so the relay-set gate requires the `scovox_full`
  relays.
- When the suffix differs from the default, `scovox_node/scovox_bin` stays
  in the **leakage** check only, since nothing but the robot's own `dscovox`
  may read it. It is not required as a relay.

**Campaign** (`run_campaign.sh`):
- `SHARE_FULL_PERIOD_S` is read via `env_val`, stripped at the per-cell
  launch, and resume-guarded numerically as `share_full_period_s`.
- `TX_MODEL` likewise, as the string key `transmission_model`.
- **One exception to "absent = mismatch".** A banked cell with no
  `share_full_period_s` line counts as 0, and one with no
  `transmission_model` line counts as admission, because every manifest
  older than this change ran deltas on the admission model. Without it, any
  resume of an existing root (the queued N=2 `hybrid` retry, the run box's
  campaigns) run on the new build would abort.
  - The exception applies only when the manifest has no `map_stream` line
    either: the new runner writes all three, so a partial set is damage.
  - A present value compares as usual, so a full or progressive campaign
    still aborts on an old cell of the same name.
- Both knobs are also validated once up front, so a junk value or
  `--comms 0` fails before the first bring-up.
- `campaign_guard_calib.sh` gets cases for:
  - absent vs 0 (skip),
  - absent vs 0.5 (abort),
  - 0.5 vs 0.5 (skip),
  - 0.5 vs 1 (abort),
  - junk (abort),
  - absent beside `map_stream` (abort),
  - the same set for `transmission_model`, and junk up front,
  - the strip-list invariant and default read-back for both knobs.

**`gen34_check.py`:** unchanged. Nothing in it reads the map stream.
`map_stream` is in the manifest for the analysis to read.

**New, from review:**
- `fused_map_liveness.py CELL...`: per robot, the longest stretch over which
  the planner's `total_observed_voxels` did not change; FROZEN when it is
  ≥ 600 s, open at the run's end, and not all DONE. On the pilot it flags
  `gen34n2_off_seed1` bestla (2164 s from t 871) and nothing else. Run on
  every cell of this experiment before it is read (12.12 R5).
- `manifest_pair_check.py CONTROL FULL`: every manifest key equal except
  the map stream and keys that describe the run (timestamps, paths,
  outcomes). Commit and binary hashes must match (12.12 R12).

### 12.9 Edge cases

| Case | Handling |
|---|---|
| Link down for a long time | The queue holds one ~9 MB frame per direction, not a growing backlog. On contact, one frame goes out, then the newest each time the air frees |
| A frame is on the air when the link drops | Progressive: aborted and counted (`drop_aborted`); the newest frame goes next contact. A reliable delta on the air resumes where it stopped |
| Contact shorter than one frame's air time | Progressive: the frame does not arrive. Under admission it would have, whole, and then blocked the channel for longer than a median contact (12.12 R1) |
| Airtime debt starves the beacons | `best_effort_priority=true` (gen 34 sets it) skips the airtime check for beacons and intents |
| Both robots' frames contend | Shared airtime, as for deltas. The measure is bytes on the link |
| Frame before TF is ready | Skipped and counted |
| Empty map at start | Nothing sent, no seq used |
| Worker slower than the period | The tick is skipped (`skip_busy`), and the rate degrades visibly |
| A superseded frame's seq never arrives | Seen as a gap by the receiver, which is diagnostic only (12.6). The exchange completes on the next frame |
| Sender restarts | `share_seq_` restarts at 1. Same behaviour as today (not handled in either arm) |
| Own `dscovox` | Reads deltas as today. Own voxels in the fused map update at 2 Hz in both arms |
| Planner starts before the first frame arrives | As today: the fused map holds own data until peers arrive |
| Mission end (all done) | `STOP_ON_DONE` ends the run. Frames keep flowing until then |
| Frozen fused map (pilot finding 1) | Not fixed here. `fused_map_liveness.py` runs on all four cells; a FROZEN robot is reported and its cell is not pooled silently. The full arm may be more exposed (bigger callbacks on the receiver) |
| Chunking or byte-budget parameters set | Ignored for full frames (12.4), with a WARN at start if set |
| `map_stream:=full` without scovox publishing `scovox_full` | The relay never forms. The bring-up gate fails on the missing `scovox_full` relays, and `GATES_STRICT=1` kills the cell |
| Relay forms but no frame ever crosses (TF never ready, worker wedged) | Not a gate. The per-cell report checks `latest_relayed` > 0 per direction in the link counters and the sender's `published=` count (12.12 R8) |
| `SHARE_FULL_PERIOD_S>0` against a banked delta cell of the same tag | The resume guard aborts. The full cells use their own root and tag |

### 12.10 Tests and verification

- **scovox unit tests:**
  - A full frame of a known grid round-trips through serialize and LZ4 to
    exactly the non-prior voxels.
  - The z-band is honoured.
  - The capture seq is ordered correctly against deltas.
  - The touched sets and gate grids are untouched by a full capture.
- **dscovox test** (black box, two real `dscovox` binaries, flag on and
  off, in `test_fusion_counters_liveness.cpp`): the same five frames from
  two sources (new, overlapping, an exact repeat, a partial change, a
  repeat). The fused maps must be identical, the flag-off node must refold
  every delivered voxel (30 + 20) and the flag-on node only the new and
  changed ones (15 + 10).
- **Emulator** (`hmr_sim/scripts/comms_tx_model_calib.py`, the real node,
  known answers):
  - C1 admission delivers a small message ahead of a large one queued
    before it (the existing behaviour, R3);
  - C2 progressive delivers them in order;
  - C3 progressive takes ~1.25 s for 9 MB at 72 Mbps under 0.6 capacity;
  - C4 progressive latest: a mid-frame drop aborts, frames queued while down
    are superseded, only the newest arrives (1 aborted, 2 superseded);
  - C5 progressive reliable: a mid-message drop resumes, one delivery, no
    loss.
- **Benchmark:** a full-arm smoke cell's own logs: the sender's `copy_ms`
  and `work_ms` and the receiver's `wall_ms` at the largest map reached.
  Recorded here.
- **Calibs:** `campaign_guard_calib.sh`, `comms_gates_calib.py` (new
  map-suffix cases) and `gen34_check_calib.py` all pass.
- **Smoke:** a 400 s N=2 full-arm cell locally. Check:
  - the relays formed and the gates are clean,
  - `drop_superseded` > 0 after an outage,
  - the frame sizes and ages in the logs,
  - the fused map on both robots contains the peer's voxels.

### 12.11 Threats to validity

- **Two seeds.** This is a pilot-scale comparison. It shows the size and
  direction of an effect, not a significant difference.
- **CPU.** Full frames cost sender and receiver CPU, and that is not
  insulated from sim time (12.3). The per-frame times and RTF are reported;
  a host that cannot keep up skips frames, and the skip counter records it.
- **Map accuracy.** The full arm's receivers get ungated values (12.2). Any
  exploration difference therefore bundles two effects:
  - fresher arrival at contact,
  - the small changes the gate suppresses.

  This experiment does not separate them. `map_agreement.py` cannot: it
  measures the spread of observed-voxel counts, not per-voxel accuracy
  (12.12 R10). A later arm could run deltas with `share_change_gate:=false`.
- **Seeds.** SEED fixes the fading trace only; sim sensor noise is unseeded
  and link states follow positions, which diverge. A pair is two runs of
  the same conditions, not a replay.
- **Frozen fused map.** Pilot finding 1 can strike either arm. A cell that
  shows it is reported and flagged, and not silently pooled.

### 12.12 Adversarial review (2026-09-24)

Four reviewers were launched on the closed design: three steered (sender and
seq; emulator and receiver; validity and scripts) and one unsteered. The
sender-and-seq reviewer did not run: Fable 5.1 had run out of usage credits.
Its focus is carried into the code review (12.13). Every finding below was
checked against the code or the pilot data before it was taken or refused.
The revisions are already written into 12.1–12.11.

**Taken**

| # | Finding (reviewers) | Verified | Change |
|---|---|---|---|
| R1 | High (3 of 3). The emulator admits a queued message whole at the tier in force, charges bits/tier, and delivers it at now + cost whatever the link does next. Contacts open at 7.2 Mbps (46 of 47 in the pilot's `link_states.csv`; median contact 9.6 s), so a ~10 MB frame goes out as ~11 s of air, drives the shared airtime ~11 s into debt, locks out the other direction for longer than a median contact, and arrives even if the contact lasted 1 s. Small deltas barely notice; the effect grows with message size, so it would decide the comparison | Yes: `DrainReliable`, `ScheduleDelivery`, `NextBandwidth` | New `transmission_model:=progressive`: one message on the air per directional link; each 5 ms delivery tick sends dt of air at the current tier, the airtime balance split evenly across the links sending; delivered when complete. A link drop aborts a latest message (`drop_aborted`) and pauses a reliable one, which resumes. Default stays `admission`, so every banked and running campaign is unchanged. Both arms of this experiment run progressive, so the seed-1 control is re-run (12.1) |
| R2 | High. The arms differ in queue policy as well as selection (FIFO with stale backlog vs supersede) | Yes | The question is stated as a protocol comparison (12.1). No coalescing-delta arm: a coalesced delta queue is a third protocol nobody runs |
| R3 | High, existing bug. Admission is not FIFO: messages admitted in one drain are delivered at now + their own cost, so a small later delta overtakes a large earlier one; under per-voxel snapshot-replace the older value then overwrites the newer. Pilot: exchange events with `gaps` 6849 and 3739 | Yes: the heap orders by `t_ns`; `gaps` confirmed in `bestla.events.jsonl` | Progressive is FIFO per link (calib C2). Admission is kept as is and its reordering is pinned by calib C1, because fixing it changes the model under the gen-34 campaign mid-way. **Open for the user:** every banked gen-34 cell ran it |
| R4 | Medium. Adding `drop_superseded` to the "relay totals" dropped sum changes `manoeuvre_events.py`'s `dropped_per_s` | Yes (`RE_RELAY`) | Not added; separate JSON fields only (was already so in the code) |
| R5 | High. No frozen-map check exists; "finding 1" is undefined; the full arm may be more exposed. Freeze starts at t≈871 s, not ~1005 s | Yes: bestla flat at 1148734 from t 871 to 3035 | `fused_map_liveness.py`; 12.1 and 12.9 corrected |
| R6 | Medium. A relay and subscriber history of 4000 for ~10 MB messages could hold gigabytes if the RMW keeps samples | Code yes; RMW retention not verified | `latest_qos_depth` 2 in the emulator, `scovox_full_qos_depth` 2 in dscovox |
| R7 | Medium. The receiver's single-threaded executor stalls for the whole ingest of a 4 M-voxel frame; `skip_unchanged_voxels` only removes the refold; the touched sets reserve 4 M buckets; every frame forces a fused republish; the "insulated from CPU" claim is wrong | Yes | Full arm only: no reserve, `fused_dirty_` only when a cell changed, `wall_ms` per frame; claim reworded; RTF a covariate |
| R8 | Medium. Nothing checks that a full frame was ever delivered | Yes (`gate_relay_set` checks topics only) | `latest_relayed` per link in the stats; checked per cell with the sender's `published=` |
| R9 | Medium. Numbers: 3.5–3.9 B per Beta voxel with Dir, Dir not empty, backlog 107/124 MB, 9 MB at 72 Mbps is 1.0 s of air (3.3 s was frame spacing) | Yes against `comms.log` and `nav_*.log` | 12.2 and 12.3 corrected |
| R10 | High. `map_agreement.py` cannot size the gate effect | Yes (it spreads `total_observed_voxels`) | Claim dropped; `share_change_gate:=false` named as the later arm that could (the parameter exists) |
| R11 | Medium. Seed-1 pair mixes code versions; building into the live install would swap binaries under the queue | Yes | Seed-1 control re-run on the new build; the build is in a worktree install until the queue is idle |
| R12 | Medium. Nothing checks that the two roots ran the same settings | Yes (guard is per root) | `manifest_pair_check.py` |
| R13 | Medium. The manifest cannot show which mapper or emulator binary ran | Yes | `sha256_` for the three binaries |
| R14 | Low. The absent-means-0 rationale is only partly right, and it can be tightened | Yes | Applies only when `map_stream` is absent too; rationale rewritten (the `hybrid` retry and the run box's resumes are the real cases) |
| R15 | Low. No up-front validation in `run_campaign.sh`; no default read-back case | Yes | Both added, with calib cases |
| R16 | Low. Only `comms_gates.py check` reads the suffixes; leakage needs its own list | Yes | Text corrected; the code already had `LEAK_ONLY_SUFFIXES` |
| R17 | Medium. Which direction wins is effectively random under contention | Yes (`drain_offset_++` per tick) | Progressive splits each tick's airtime evenly among the links sending |
| R18 | Low. Empty `reliable_topics` override may not launch | The node already aborts on `[]` (`comms-sim-empty-list-abort`) | Override is `[""]`; verified by the container launch |
| R19 | Low. The sender's `scovox_full` is not in any bag | Already in the main bag at RECORD ≥ 1 in the implementation | None |

**Not taken**

| Finding | Why not |
|---|---|
| Chunk full frames into spatial tiles superseded per tile (validity, unsteered) | Progressive transmission fixes the same model error for both arms without changing what the full arm sends. Tiles would make "the whole map" a sequence of partial maps with independent ages |
| Build the full frame from the gate grids, so both arms carry the same values (unsteered) | That removes the selection the question is about. The gate's contribution is named as unseparated in 12.11 |
| A runtime gate on full-frame delivery | A cell with no frames is a finding to report, not a bring-up fault to retry. Checked per cell instead (R8) |
| Strict alternation for latest queues (emulator) | The even split per tick (R17) gives each sending direction the same share without a special case |
| Per-frame seq/size/stamp log on the sender | The receiver's per-frame line has seq, size and age; the sender's 60 s stats line has the counts |

### 12.13 Code review and verification (2026-09-24)

Fable 5.1 was out of usage credits, so the code review ran on Opus 5.5:
three read-only reviewers (sender and seq; emulator; receiver and scripts).
Only the sender review finished before the session's usage limit stopped
the other two; they are re-run when the limit resets and recorded below.

**Sender review.** Threading, the in-flight protocol, destructor order, the
shared lock, the voxel selection against `publishBinaryMap`'s snapshot
branch, and the seq argument of 12.6 were all confirmed. A deferred delta
chunk built before a frame but sent after it gets a higher seq with older
data; that only makes an exchange wait for a later frame.

| Finding | Verified | Action |
|---|---|---|
| An exception in the worker (`serialize` throws on a bad class id; `bad_alloc`) terminates the node; a failed `std::thread` construction leaves the in-flight flag set | Yes | try/catch in the worker (frame dropped, logged, seq becomes a gap); thread construction guarded |
| Frame vectors grow by doubling under the executor wait | Yes | Reserved from the previous frame's sizes + 1/8 |
| `copy_ms` misses the TF lookup and the join | Yes | Timed from tick entry |
| Period set outside rolling mode does nothing, silently; no floor | Yes | WARN outside rolling; 0.05 s floor |
| Design says `full_skipped_busy`, code logs `skip_busy`; the chunking WARN fires even when chunking is off | Yes | Design renamed; WARN names chunking only when set |
| dscovox: a voxel created by `value(mc, true)` at its default could compare equal and be skipped | Unreachable: senders never put at-prior values on the wire | Documented in the code. The suggested existence check with `value(mc, false)` was tried and broke ingest (the accessor caches the missing leaf; the create that follows returns null, so nothing was stored). The black-box test caught it |
| No scovox_node unit tests for the full frame (12.10) | Yes | Not added: the node needs a live lidar pipeline. Covered by the smoke cell (frames published, sizes, receiver ages) and the receiver test. Sharing one predicate between the delta and full paths was declined so the delta path stays byte-identical |

**Verification** (capped container, the worktree build):
- `test_fusion_counters_liveness`: 3 of 3 tests pass, three runs in a row,
  including `SkipUnchangedVoxels.SameFusedMapFewerRefolds`.
- `comms_tx_model_calib.py`: C1–C5 all pass (admission reorders; progressive
  is FIFO, takes ~1.25 s for 9 MB, aborts a cut latest frame and delivers
  only the newest, resumes a cut reliable message).
- `campaign_guard_calib.sh` 201/201, `comms_gates_calib.py` 36/36,
  `gen34_check_calib.py` all pass; `bash -n` on both runners.
- `fused_map_liveness.py` on the pilot flags `gen34n2_off_seed1` bestla only.

**Smoke cell, partial** (`gen34n2smoke_off_seed1`, full arm, progressive,
400 s cell stopped by hand at t_sim ~221 s to hand the campaign to the run
box): gates ok (qos 6/6, no overflow, odom live). Sender at t ~220 s:
bestla published 348 frames, skip_busy 0, lz4_fail 0, 1.06 M voxels ->
3.1 MB raw, 2.2 MB LZ4; copy_ms mean 12.4 / max 51.1, work_ms mean 117.8 /
max 290. Links both ways: latest_relayed 178/179, drop_superseded 249,
drop_aborted 2, drop_overflow 0, backlog ~2.8 MB. Receiver: remote frames of
~575 k voxels with ~460 k unchanged, age 0.2 s, wall_ms 105–130. The two
remaining reviews (emulator; receiver and scripts) were not re-run before
the hand-off.
