# ts1b team-size series — results

Design and endpoints are fixed in `~/hmr_campaign/ts1_analysis/PREREGISTRATION.md`; nothing here may
change them. Comms-model caveats are in `~/hmr_campaign/ts1_analysis/docs/COMMS_MODEL_REALISM.md`. Cross-campaign
synthesis lives in `~/hmr_campaign/CAMPAIGN_FINDINGS.md`.

Completed 2026-09-08. Supersedes the ts1 draft of this file, which reported 8
cells of a 180-cell design on different binaries and a different endpoint.

## Status

    design      3 rungs (N=2,3,4) x 4 arms x 10 seeds = 120 cells, fixed in advance
    complete    120 / 120 all_done      (40 / 40 / 40 per rung)
    censored    5 robot-runs of 360 (1.4%); 0 cells lost
    span        2026-09-05 18:28 -> 2026-09-08 11:24 BST (~64.9 h wall, 35.6 h sim)
    binaries    explo_planner e7c185b
                hmr_explo     7eee1ce-dirty.65cb70d8
                hmr_sim       c9c03de
    provenance  one triple across all 120 cells; no mid-campaign split

**A top-up to 240 cells is in flight** (seeds 11–20 on all three rungs, launched
2026-09-08 14:31 BST, ETA ~64 h from the measured per-rung wall/sim ratios). It
runs on the same `explo_planner e7c185b` and the same radio regime, so it pools
with the 120 below; every number in this document is the 10-seed result and will
be restated at n=240. The top-up extends **all three rungs together** — a
single-rung extension would confound team size with session.

Per-cell data: `~/hmr_campaign/ts1_analysis/ts1b_cells.csv` (one row per cell, all endpoints + link
metrics). Radio regime is the 2026-09 one — 70 dB trunk attenuation, 30 m
horizon. **Do not pool with cr3/cr4/cr5 or anything earlier.**

> **Provenance note — this file changed `hmr_explo`'s stamp.** The manifest
> stamps each repo as `<short-rev>[-dirty.<sha1>]`, where the dirty hash covers
> `git diff HEAD` **plus untracked filenames**
> (`sim/run_explo_sim_rviz.sh:2428`). Placing this document at the `hmr_explo`
> root therefore moved that repo's stamp:
>
>     during the 120 cells   hmr_explo 7eee1ce-dirty.65cb70d8
>     after this move        hmr_explo 7eee1ce-dirty.a6ec2f5e   (top-up seeds 11+ start here)
>     from this commit on    hmr_explo <this commit>, clean
>
> There is no way to add a file that avoids this: untracked changes the hash,
> committing changes the rev, and `.gitignore`-ing it changes the diff. So the
> top-up cells span two `hmr_explo` stamps, split at whichever cell started next
> after this commit. The exact revs are recoverable from any cell's
> `run_manifest.txt`; the third one is not written out here because writing it
> would change it again.
>
> **It does not break comparability, and this was checked rather than assumed.**
> The comparability group key (`build_index.py:15`) is
> `git_explo_planner | radio regime | scenario | done criterion | duration` —
> `git_hmr_explo` is not in it. The planner submodule is untouched and still
> stamps `e7c185b` **clean**, identical to all 120 cells, because this document
> sits in the *umbrella* repo, not in `ws/src/explo_planner`. Gate check 3b is a
> prefix match on the rev (`7eee1ce`, unchanged), so it passes. Cells run after
> this move therefore pool with the 120 reported here.
>
> The stamp difference is still worth knowing about when diffing manifests by
> hand: it is **documentation only** — no source, config, or binary changed.

## Deviation from the preregistration: the primary endpoint

The preregistration names `run_end_t_sim` as the primary team-size endpoint.
**It was replaced mid-campaign with event-log endpoints** and the substitution is
reported here rather than buried:

- `run_end_t_sim` is *run-relative* and inflated by the 30 s done-grace window,
  so it measures the harness's shutdown, not the team's.
- `t_explore` = last per-robot `exploration_complete` (the coverage latch).
- `t_mission` = last per-robot `mission_complete` (latch + return home).

Both endpoint families are reported below so the substitution can be checked. It
does not change the direction or size of any conclusion:

| rung | `run_end_t_sim` (preregistered) | `t_explore` | `t_mission` |
|------|--------------------------------|-------------|-------------|
| N=2  | 1091.9 | 1017.8 | 1110 |
| N=3  | 1067.9 |  984.7 | ≥1078 |
| N=4  | 1043.2 |  952.0 | ≥1054 |

`≥` marks a rung containing a censored robot, imputed at its stamp — a lower
bound. All event reads use the **last** `mission_complete` per robot; nine robots
had two homing episodes and a first-event reader records a false censoring (see
Censoring below).

## Primary — team size

This is the powered comparison (n=40/rung, arms pooled).

| rung | `t_explore` (makespan) | per-robot latch (cell means) |
|------|------------------------|------------------------------|
| N=2  | 1017.8 s | 916.5 s |
| N=3  |  984.7 s | 813.3 s |
| N=4  |  952.0 s | 748.7 s |

**The makespan barely moves (−6.5%) while the per-robot mean drops 18.3%. That
gap is the finding, not a null.** `t_explore` is a *max* over robots, so it gains
a draw at every rung and must rise under a pure null. Reporting it alone would
understate the team-size effect; reporting the per-robot mean alone would ignore
that the team is only done when its last robot is.

Decomposition, N=2 → N=4, with the penalty measured **within** N=4 worlds
(average max over all 2-subsets of each cell's own robots, so no
independent-draw assumption):

    per-robot gain             -167.9 s
    extra-draw penalty (2->4)  +109.6 s
    predicted makespan change   -58.3 s
    observed  makespan change   -65.9 s

Two independent routes agreeing to 7.6 s. The single-drop penalty is +69.8 s at
N=3 and +49.0 s at N=4 — the marginal robot costs less as the team grows.

Within-cell ICC of latch times is 0.295 (N=3) and 0.477 (N=4): robots sharing a
world finish together, and positive correlation shrinks a max. **A pooled
independent-draw null overstates the counterfactual makespan** and must not be
used here.

## Primary — connectivity

Pre-registered as `isolation_mean` and `pair_deep_outage_mean` from
`~/hmr_campaign/ts1_analysis/teamlink.py` (calibration re-run and passing: 12/12 two-robot cells reproduce
`readout.py` exactly, plus the synthetic N=3 known-answer case).

**These are measured in seconds, and cells have different durations, so the raw
means carry an exposure confound.** `r(isolation_mean, run_end_t_sim)` = +0.869 /
+0.390 / +0.189 by rung — at N=2 the raw metric is most of the way to being a
restatement of run length. Both raw and exposure-normalised forms are given; the
**fraction** is the one to quote.

| rung | isolation_mean (s) | as fraction of run | pair_deep_outage_mean (s) | as fraction |
|------|-------------------|--------------------|---------------------------|-------------|
| N=2  | 715.8 | 0.641 | 715.8 | 0.641 |
| N=3  | 344.7 | 0.330 | 590.0 | 0.551 |
| N=4  | 191.8 | 0.192 | 536.1 | 0.518 |

**The clean result: adding robots buys team-level connectivity but almost no
per-link connectivity.** Time with *no* connected peer falls by a factor of 3.3
(0.641 → 0.192), while any given pair's deep-outage time falls only 0.641 →
0.518. That is what a no-relay direct-link model predicts — a robot gains
alternative peers, but no individual link improves — and it is the reason
`isolation_mean` and `pair_deep_outage_mean` must both be reported. Quoting the
pair metric alone would hide the effect; quoting isolation alone would
overstate it as a link improvement.

At N=2 the two are identical by construction (one pair), which is the identity
`teamlink.py --calibrate` asserts.

## Secondary — arm contrast (a screen, not a test)

The preregistration states this is **not powered** at n=10/arm (MDE ≈184 s ≈27%),
that cr3/cr4/cr5 already returned a four-arm null on finish time, and that a null
here is guaranteed by construction and carries no information. It is reported
descriptively and **no arm claim is made from this series.**

`t_explore`, mean / median:

| arm | N=2 | N=3 | N=4 |
|---|---|---|---|
| off | 1058 / 1027 | 855 / 853 | 805 / 805 |
| hybrid | 1178 / 1155 | 1159 / 1183 | 1094 / 1135 |
| pursuit | 959 / 921 | 912 / 930 | 989 / 948 |
| rendezvous | 875 / 900 | 1013 / 970 | 920 / 963 |

`t_mission`, mean / median (`≥` = contains a censored lower bound):

| arm | N=2 | N=3 | N=4 |
|---|---|---|---|
| off | 1136 / 1114 | 919 / 910 | 897 / 878 |
| hybrid | 1238 / 1197 | ≥1230 / 1292 | ≥1191 / 1161 |
| pursuit | 1100 / 1103 | ≥1079 / 1073 | ≥1092 / 1041 |
| rendezvous | 965 / 978 | 1082 / 1011 | ≥1036 / 1099 |

`off` is nominally fastest at every rung. At n=10/arm this is a screen result and
is consistent with the established finding that reconnection buys connectivity,
not speed.

## Secondary — coverage at termination

2D CellWorld `covered_fraction`, team mean over robots. This is a free-varying
outcome: the stopping rule is the 3D scovox `roi_unknown_fraction ≤ 0.64`, not
this quantity.

| rung | mean | median | sd | robot-to-robot spread |
|------|------|--------|-----|----------------------|
| N=2  | 0.534 | 0.520 | 0.072 | 0.005 |
| N=3  | 0.571 | 0.570 | 0.080 | 0.004 |
| N=4  | 0.582 | 0.604 | 0.080 | 0.022 |

Larger teams stop with slightly more of the 2D grid covered, and the robots'
copies of the grid agree closely (spread ≤ 0.022) — the divergence discussed
below is in the 3D voxel maps, not here.

## Secondary — censoring

5 non-arrivals in 360 robot-runs (1.4%); **zero cells lost**. Final-event results: 355 arrived, 3 `budget`, 2 `no-progress`. 369 homing episodes
total, 9 robots with more than one.

| cell | robot | result | t_sim | dist from home |
|---|---|---|---|---|
| n3_hybrid_seed1 | atlas | budget | 1348 | 1.602 m |
| n3_pursuit_seed7 | atlas | budget | 1440 | 1.967 m |
| n4_rendezvous_seed9 | atlas | budget | 1221 | 1.810 m |
| n4_hybrid_seed6 | bestla | no-progress | 2015 | 3.017 m |
| n4_pursuit_seed5 | bestla | no-progress | 950 | 3.017 m |

Two mechanistically distinct modes. **`budget`** (atlas, all three) parks 1.6–2.0
m out, just outside the hard 1.0 m `final_goal_tolerance`, having made the
required progress the whole way — one case logged *zero* watchdog events and
simply ran out of budget on a long detour. **`no-progress`** (bestla, both) is a
home-watchdog escalation: approach → `resend` → `retrace` → 3 × `escape` →
park, fired 8–15 times, with **zero** `nav_goal_failed` from the navigator and
escape legs that succeed outward to 4–6 m. Both park at 3.017 m on different
bearings, different seeds and different arms. Outward is free, inward is not.

Root cause is **not** established and needs an instrument this campaign cannot
provide — the live planning map, which is never bagged. Queued as a
between-campaign capture around bestla's home.

An earlier framing of mine, "no-progress always occurs at 3.0 m", was too narrow:
`n4_hybrid_seed9`'s husky stalled at **20.94 m**, then recovered and arrived.
That case is why the last-event rule is load-bearing.

## Closed questions

Four hypotheses were held open until the rungs filled. All four are now settled;
three of the four went against the hypothesis.

### Airtime bimodality at N=4 — **withdrawn**

At n=25 the N=4 exhaustion fractions looked bimodal (a 0.62–0.68 cluster against
0.20–0.50). At n=40 they are a smooth ladder, 0.204 → 0.710, with no second mode.
It was small-n lumpiness. **Do not read modes off a rung that is not full.**

What *is* real is a modest rung effect: mean bucket-empty fraction 0.390 / 0.384 /
**0.487**, still +0.096 for N=4 after adjusting for run duration (two-sided
permutation p = 0.0092, 200k perms).

The reason a naive read misleads: exhaustion is a **rate**, and
`r(exhaustion, duration)` = +0.199 / −0.632 / **−0.650** by rung. Long cells
dilute the fraction with a tail in which robots are latched, homing or idle and
barely transmitting, so the *highest*-exhaustion N=4 cells are the **shortest**
(0.710 at 578 s; 0.204 at 1525 s). Always adjust for duration before comparing
exhaustion across groups.

### Is homing failure a manoeuvre side-effect? — **null**

All 5 terminal censorings fall on manoeuvring arms, 0/90 robot-runs on `off`.
But the expected count on `off` under a uniform-arm null is 1.25, and
P(0 of 5) = 0.75⁵ = 0.237 (one-sided hypergeometric on robot-runs, 0.235). **Not
evidence.** It would not become evidence at 10/10 either; this design cannot
resolve a rate this low.

### Regroup-spread — **refuted by its own best case**

The hypothesis was that a regrouping manoeuvre injects seed-to-seed variance.
Residual SD of finish time after removing rung means:

    hybrid      433        rendezvous  263
    off         273        pursuit     252

    hybrid alone          SD ratio vs rest = 1.640,  p = 0.0103
    rendezvous alone      SD ratio vs rest = 0.774,  p = 0.8123
    both regrouping arms  SD ratio vs rest = 1.381,  p = 0.0680

`rendezvous` regroups and has the **second-lowest** spread of all four arms. The
pooled "regrouping arms" near-miss exists only because hybrid drags it. The
variance belongs to hybrid specifically, not to regrouping — and hybrid is a
compound arm, so this does not identify which component is responsible.

**Post-hoc, added 2026-09-09:** the residual-SD figures above pool the rungs. A
per-rung Brown-Forsythe of hybrid against `off`
(`analysis/ts1b/ts1b_finish.py contrasts --seeds base`) shows the spread gap is
not flat — it is 1.28× at N=2, 2.19× at N=3, and 2.52× at N=4 (p = 0.006,
exact), against a rung-pooled 1.44×. Pooling charges an arm's between-rung
location trend to its spread, and `off` has a strong rung trend where hybrid has
none, so the pooled number is the *weaker* read rather than the summary. Treat
the N=4 row as a lead, not a result: it is one of nine per-rung spread rows, so
Bonferroni puts it at 0.054, and it is unregistered.

### Endpoint map divergence — **half of it was an artifact of my own statistic**

`map_agreement_n.py` reports the **max over pairs**, and a team has
C(N,2) = 1 / 3 / 6 pairs. That max must grow with N under a pure null — the same
order-statistic trap as `t_explore`.

| rung | max over pairs, at latch | mean over pairs, at latch | max, at end | mean, at end |
|------|------------------------|---------------------------|-------------|--------------|
| N=2  |  3.48% |  3.48% | 1.02% | 1.02% |
| N=3  | 11.09% |  7.49% | 1.79% | 1.20% |
| N=4  | 14.00% |  7.60% | 3.88% | 2.16% |

Calibration — draw k pairs from each N=4 cell's **own** 6-pair pool:

    k=3 (N=3-like): 12.61%  [95% 11.56-13.40]   real N=3 = 11.09%   -> artifact
    k=1 (N=2-like):  7.60%  [95%  5.58- 9.57]   real N=2 =  3.48%   -> real

So **N=3 → N=4 growth is entirely the extra pairs**, while **N=2 → N=3 is a
genuine doubling** of per-pair divergence. The mean-over-pairs column, which
needs no correction, says the same: 3.48 / 7.49 / 7.60%.

This does not threaten `t_explore`. Each robot latches on *its own*
`roi_unknown_fraction ≤ 0.64`, so a robot missing voxels its peers hold latches
**later**, not earlier — the bias is conservative for any "larger teams are
faster" claim.

`n4_hybrid_seed3` has no `at_latch` on any pair and is excluded from that column
(39 cells, not 40).

## Non-replications and corrections

**The separation mediator replicates — but only on the matched estimator, and
the mismatch nearly produced a false non-replication here.**

`teamlink.py`'s `frac_time_near` is a **whole-run** quantity. The mh1 and cr5
analyses deliberately did *not* use a whole-run measure: every run ends homing to
~4 m, so run length contaminates any whole-run separation average, and those
analyses used a **fixed 0–400 s window** whose length cannot depend on the finish
time. On the whole-run measure ts1b appears to overturn the finding; on the
matched estimator it confirms it.

| estimator | N=2 | N=3 | N=4 | pooled |
|---|---|---|---|---|
| whole-run `frac_time_near` (**not** the prior quantity) | −0.457 | −0.018 | −0.017 | −0.169 |
| fixed 0–400 s window, ρ(separation, `t_explore`) | −0.355 (p=0.026) | −0.202 | −0.221 | **−0.277 (p=0.0025)** |
| fixed 0–400 s window, ρ(%within 20 m, `t_explore`) | +0.266 | +0.167 | +0.096 | **+0.231 (p=0.011)** |

Prior values on the same estimator: mh1 ρ(sep) = −0.486, cr5 ρ(sep) = −0.479,
ρ(%near) = +0.345. **ts1b: −0.277 / +0.231 — same direction, attenuated.** More
separation during early exploration, faster finish.

The arm ordering replicates cleanly too. Mean windowed separation, metres:

    off 40.3    rendezvous 34.7    hybrid 32.6    pursuit 31.3

`off` separates most at every rung and is nominally fastest at every rung — the
established "the naive baseline keeps up because the stack was never buying
separation" picture, now shown to hold at N=3 and N=4 as well as N=2.

**Do not substitute `teamlink.frac_time_near` for the windowed mediator.** They
disagree in sign. `teamlink.py`'s version exists for team-level link accounting,
not for this mediator.

**`isolation_mean` does not predict finish time once exposure is removed.** The
raw correlation with `t_explore` is +0.893 at N=2, which is close to
tautological. Normalised to a fraction of run time: +0.340 (p = 0.030) / −0.087 /
−0.185, pooled +0.116 (p = 0.211). No claim is made.

**Multiplicity.** Roughly six hypothesis tests are reported above. The two
nominally significant ones (airtime p = 0.0092, hybrid variance p = 0.0103) sit
either side of a Bonferroni threshold of 0.008. Both are reported as suggestive,
neither as established. The team-size decomposition does not rest on a p-value —
it rests on two independent routes agreeing to 7.6 s.

## Validity bounds

- **The comms model is a manipulation check, not a field prediction.** Shape
  right, magnitude uncalibrated, and wrong in both directions: no spatial reuse
  (one global airtime bucket = one collision domain, too pessimistic exactly in
  the dispersed regime under study) and no collisions/backoff/hidden terminals
  (too optimistic under canopy). Full treatment in `~/hmr_campaign/ts1_analysis/docs/COMMS_MODEL_REALISM.md`.
- **`drop_airtime` as a share of attempted sends was never captured.** Those
  counters publish only on `/hmr_comms_sim/stats` and are never written to a
  file, so they are unrecoverable from a finished cell. The bucket-sign fraction
  used above is a weak proxy. The manipulation check needs a passive rclpy
  subscriber on a *running* cell, one per rung.
- **The nav global planner never planned** in any run of this binary generation.
  No arm confound, but it bounds every result here.
- **`run_gates_verdict` reads SUSPECT on every N≥3 cell** because
  `map_agreement.py` hard-codes 2 planners. Report-only, no FAIL path, no
  validity lost — but it means the one-field health filter is inert above N=2 and
  a real gate failure could hide inside an expected-looking verdict. Every cell
  here was checked on the individual gate lines instead.
- **The three-way interaction is not powered and is not claimed.** Neither is
  redundancy, which needs ~4451 cells and is not an endpoint.
- Single simulator, single world family, single seed set of 10 per arm.

## Open items

1. Live planning-map capture around bestla's home to settle the ~3.0 m
   `no-progress` ring. Needs a running cell; the map is never bagged.
2. Per-rung airtime manipulation check via a passive rclpy subscriber on a
   running cell (read-only, costs no cells).
3. `off`'s N=3 coverage anomaly: covered 0.659 / seen 0.966 / roi_unk 0.555
   against 0.53–0.55 / 0.89–0.92 / 0.60 for the other arms.
4. Fix `map_agreement.py`'s N=2 hard-code; make `event_log` take the **last**
   `mission_complete`; scale the `overflow` gate's 90 s timeout with RTF.
5. Optional top-up to 20 seeds/arm. **Extend all three rungs together** — the box
   drifts ~8% between sessions, and one rung alone would confound arm with
   session. Bump `SEEDS` in `~/hmr_campaign/ts1_analysis/run_series_b.sh` and re-run; the resume guard
   skips completed cells.

## Reproduction

All analysis scripts live with the data they read, in `~/hmr_campaign/ts1_analysis/`.
Run them from there:

    cd ~/hmr_campaign/ts1_analysis

    tools/map_agreement_n.py      all C(N,2) pair gaps; --selftest re-derives 40/40 N=2 gate lines
    tools/ts1b_census.py          censoring census, final-event and all-episode
    tools/ts1b_cells.py           builds ts1b_cells.csv (endpoints + teamlink metrics)
    tools/ts1b_held.py            airtime, censoring-vs-arm, regroup-spread
    tools/ts1b_held_adjusted.py   duration-adjusted airtime; hybrid-alone vs pooled spread
    tools/ts1b_divergence.py      map divergence with the order-statistic control
    tools/ts1b_decomp.py          per-robot / extra-draw decomposition, ICC
    tools/ts1b_confound.py        exposure and mediator confound checks
    tools/ts1b_mediator.py        separation mediator on the MATCHED fixed 0-400 s window
    teamlink.py --calibrate       must PASS before any connectivity number is quoted

Set `PYTHONDONTWRITEBYTECODE=1` when importing from the source tree.
