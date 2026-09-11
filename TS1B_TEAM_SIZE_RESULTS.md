# ts1b team-size series — results

Design and endpoints are fixed in `~/hmr_campaign/ts1_analysis/PREREGISTRATION.md`; nothing here may
change them. Comms-model caveats are in `~/hmr_campaign/ts1_analysis/docs/COMMS_MODEL_REALISM.md`. Cross-campaign
synthesis lives in `~/hmr_campaign/CAMPAIGN_FINDINGS.md`.

Base series completed 2026-09-08; topped up to 20 seeds/arm and completed
2026-09-11. Supersedes the ts1 draft of this file, which reported 8 cells of a
180-cell design on different binaries and a different endpoint.

## Status

    design      3 rungs (N=2,3,4) x 4 arms x 20 seeds = 240 cells
                (preregistered as 10 seeds; topped up to 20, all rungs together)
    complete    240 / 240 all_done      (80 / 80 / 80 per rung)
    censored    9 robot-runs of 720 (1.3%) in 8 cells; 0 cells lost
    span        base   2026-09-05 18:54 -> 2026-09-08 11:24 BST (64.5 h wall, 35.6 h sim)
                topup  2026-09-08 15:00 -> 2026-09-11 05:21 BST (62.3 h wall, 34.1 h sim)
    binaries    explo_planner e7c185b  clean, all 240 cells
                hmr_sim       c9c03de
                hmr_explo     4 stamps (see the provenance note below)
    provenance  comparability key identical across all 240: e7c185b |
                tx30/atten70/range30 | duration 3000 s | done_unknown 0.64

**Updated 2026-09-11 to the full n=240.** Every figure below is the 240-cell
result. The top-up extended **all three rungs together** — a single-rung
extension would have confounded team size with session.

The two halves were checked for poolability rather than assumed to pool
(`analysis/ts1b/ts1b_finish.py session`). No rung shows a significant centre or
spread difference on `t_explore`:

    rung   delta (topup - base)   centre p   spread p   sd base / topup
    n2            -52.1 s           0.477      0.133     372.9 / 255.2
    n3            -60.5 s           0.330      0.776     288.2 / 261.2
    n4            -43.3 s           0.507      0.532     298.3 / 272.5

The deltas share a sign — the top-up half is 4.7–6.3% faster. A stratified
permutation with the session label shuffled **within rung** (so the unequal rung
mixture cannot leak in) pools them at **−52.0 s, p = 0.174**; all three sharing a
sign has probability 0.25 under a pure null. **That is not resolvable and no
adjustment is applied.**

Resist naming a cause for it. It is tempting to write "session drift", but the
only measurement of that on this box is a single re-run that came out 1.082×
different *and also changed binary*, which is an upper bound on drift, not an
estimate of it; an earlier ±25% drift band was retracted as mis-labelled
per-cell noise. Worse, the two halves ran **different seeds** (11–20 against
1–10), so this delta confounds session with which worlds were drawn — a paired
re-run of the same seeds would separate them and nothing here does. The complete
honest statement is: three underpowered contrasts, none significant, direction
shared, cause unidentified.

A pooled ALL row is deliberately **not** reported: the rung mixtures differ
(40/40/40 vs 40/40/38, from the two latch-less cells below), and an ALL row over
unequal mixtures would report a rung-composition difference as a session effect.

Per-cell data: `~/hmr_campaign/ts1_analysis/ts1b_cells.csv` (one row per cell, all endpoints + link
metrics). Radio regime is the 2026-09 one — 70 dB trunk attenuation, 30 m
horizon. **Do not pool with cr3/cr4/cr5 or anything earlier.**

> **Provenance note — this file changed `hmr_explo`'s stamp.** The manifest
> stamps each repo as `<short-rev>[-dirty.<sha1>]`, where the dirty hash covers
> `git diff HEAD` **plus untracked filenames**
> (`sim/run_explo_sim_rviz.sh:2428`). Placing this document at the `hmr_explo`
> root therefore moved that repo's stamp:
>
> There is no way to add a file that avoids this: untracked changes the hash,
> committing changes the rev, and `.gitignore`-ing it changes the diff.
>
> **What actually happened, counted from the 240 manifests:**
>
>     base  seeds 1-10    7eee1ce-dirty.65cb70d8    120 cells   (one stamp)
>     topup seeds 11-20   7eee1ce-dirty.a6ec2f5e      4 cells
>                         f05cdfe                    58 cells
>                         6217bf7                    57 cells
>                         6217bf7-dirty.39d61b78      1 cell
>
> Four `hmr_explo` stamps across the top-up, not the two predicted — ordinary
> documentation and tooling commits landed while it ran.
>
> **It does not break comparability, and this was checked rather than assumed.**
> The comparability group key (`build_index.py:15`) is
> `git_explo_planner | radio regime | scenario | done criterion | duration` —
> `git_hmr_explo` is not in it. The planner submodule is untouched and stamps
> `e7c185b` **clean on all 240 cells** (verified by counting the manifests, not
> by assumption), because this document sits in the *umbrella* repo, not in
> `ws/src/explo_planner`. Every other key field is also constant across the 240:
> `tx30 / atten70 / range30`, `duration_s=3000`, `done_unknown=0.64`. Scenario
> varies by rung, which *is* the manipulation.
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
| N=2  | 1066.8 |  991.8 | ≥1084.8 |
| N=3  | 1030.4 |  954.5 | ≥1040.0 |
| N=4  | 1037.8 |  930.9 | ≥1032.1 |

`≥` marks a rung containing a censored robot, imputed at its stamp — a lower
bound. All event reads use the **last** `mission_complete` per robot; robots with
two homing episodes exist in both halves and a first-event reader records a false
censoring for each (see Censoring below).

Note the preregistered endpoint is now **non-monotonic** across rungs (1066.8 →
1030.4 → 1037.8) while both event-log endpoints fall monotonically. At n=120 all
three fell monotonically. This is the clearest single demonstration of why the
substitution was made: `run_end_t_sim` carries the harness shutdown, and at N=4
that overhead is large enough to reverse the rung ordering.

## Primary — team size

This is the powered comparison (n=80/rung, arms pooled).

| rung | `t_explore` (makespan) | per-robot latch (cell means) |
|------|------------------------|------------------------------|
| N=2  | 991.8 s | 905.2 s |
| N=3  | 954.5 s | 792.7 s |
| N=4  | 930.9 s | 722.7 s |

**The makespan barely moves (−6.1%) while the per-robot mean drops 20.2%. That
gap is the finding, not a null.** `t_explore` is a *max* over robots, so it gains
a draw at every rung and must rise under a pure null. Reporting it alone would
understate the team-size effect; reporting the per-robot mean alone would ignore
that the team is only done when its last robot is.

Decomposition, N=2 → N=4, with the penalty measured **within** N=4 worlds
(average max over all 2-subsets of each cell's own robots, so no
independent-draw assumption):

    per-robot gain             -182.5 s
    extra-draw penalty (2->4)  +110.6 s
    predicted makespan change   -72.0 s
    observed  makespan change   -60.9 s

Two independent routes agreeing to 11.1 s (7.6 s at n=120). The single-drop
penalty is +66.2 s at N=3 and +48.3 s at N=4 — the marginal robot costs less as
the team grows. Both the gain and the penalty grew slightly with n; the residual
grew too, but stays under 12 s against a 900 s makespan.

Within-cell ICC of latch times is 0.345 (N=3) and 0.380 (N=4): robots sharing a
world finish together, and positive correlation shrinks a max. **A pooled
independent-draw null overstates the counterfactual makespan** and must not be
used here. (The N=4 ICC fell from 0.477 at n=120 to 0.380 — the qualitative
conclusion is unchanged, but the two rungs are now much closer together than the
n=120 figures suggested.)

## Primary — connectivity

Pre-registered as `isolation_mean` and `pair_deep_outage_mean` from
`~/hmr_campaign/ts1_analysis/teamlink.py` (calibration re-run and passing: 12/12 two-robot cells reproduce
`readout.py` exactly, plus the synthetic N=3 known-answer case).

**These are measured in seconds, and cells have different durations, so the raw
means carry an exposure confound.** `r(isolation_mean, run_end_t_sim)` = +0.837 /
+0.478 / +0.247 by rung — at N=2 the raw metric is most of the way to being a
restatement of run length. Both raw and exposure-normalised forms are given; the
**fraction** is the one to quote.

| rung | isolation_mean (s) | as fraction of run | pair_deep_outage_mean (s) | as fraction |
|------|-------------------|--------------------|---------------------------|-------------|
| N=2  | 706.6 | 0.648 | 706.6 | 0.648 |
| N=3  | 358.7 | 0.352 | 587.2 | 0.568 |
| N=4  | 193.9 | 0.193 | 524.2 | 0.507 |

**The clean result: adding robots buys team-level connectivity but almost no
per-link connectivity.** Time with *no* connected peer falls by a factor of 3.4
(0.648 → 0.193), while any given pair's deep-outage time falls only 0.648 →
0.507. That is what a no-relay direct-link model predicts — a robot gains
alternative peers, but no individual link improves — and it is the reason
`isolation_mean` and `pair_deep_outage_mean` must both be reported. Quoting the
pair metric alone would hide the effect; quoting isolation alone would
overstate it as a link improvement.

This is the most stable result in the series: every cell of the table moved by
less than 0.02 between n=120 and n=240.

At N=2 the two are identical by construction (one pair), which is the identity
`teamlink.py --calibrate` asserts.

## Secondary — arm contrast (a screen, not a test)

The preregistration states this is **not powered** at n=10/arm, that cr3/cr4/cr5
already returned a four-arm null on finish time, and that no arm claim is made
from this series. At n=20/arm it is still secondary and still unregistered, but
it is no longer uniformly null — see the hybrid row.

`t_explore`, mean / median:

| arm | N=2 | N=3 | N=4 |
|---|---|---|---|
| off | 1038 / 1012 | 860 / 853 | 868 / 823 |
| hybrid | 1106 / 1020 | 1106 / 1141 | 1045 / 1078 |
| pursuit | 918 / 911 | 922 / 930 | 960 / 900 |
| rendezvous | 905 / 873 | 930 / 890 | 862 / 770 |

`t_mission`, mean / median (`≥` = contains a censored lower bound):

| arm | N=2 | N=3 | N=4 |
|---|---|---|---|
| off | 1112 / 1085 | 934 / 910 | 947 / 897 |
| hybrid | 1171 / 1092 | ≥1165 / 1205 | ≥1127 / 1104 |
| pursuit | ≥1058 / 1015 | ≥1050 / 1038 | ≥1067 / 990 |
| rendezvous | 997 / 982 | 1010 / 973 | ≥998 / 1023 |

**`off` is no longer fastest at every rung** — `rendezvous` beats it at N=2 and
N=4 on both endpoints. What did sharpen is the hybrid penalty. Per-rung
contrasts against `off` on `t_explore` (exact-style MC, 200k):

    rung   off vs pursuit      off vs rendezvous     off vs hybrid
    n2     -120.5  (p 0.156)   -133.2  (p 0.151)     +68.1   (p 0.596)
    n3      +61.1  (p 0.356)    +69.1  (p 0.452)    +245.8  (p 0.003)
    n4      +91.1  (p 0.324)     -6.9  (p 0.934)    +176.9  (p 0.065)

**Hybrid is 28.6% slower than `off` at N=3, p = 0.003** — that survives
Bonferroni over the nine contrasts (0.027) and is directionally matched at N=4
(+20.4%, p = 0.065). It is consistent with the earlier mh1 result that hybrid
runs ~12% slower, and with the standing finding that reconnection buys
connectivity, not speed. It remains a secondary, unregistered contrast on a
compound arm: it says the hybrid *stack* costs time, not which half costs it.

No pooled-across-rung contrast is reported. Rung composition differs by arm
(hybrid is 20/20/18 after the two latch-less cells), and `ts1b_finish.py`
refuses to pool unbalanced data rather than quietly returning a rung-weighted
arm contrast.

## Secondary — coverage at termination

2D CellWorld `covered_fraction`, team mean over robots. This is a free-varying
outcome: the stopping rule is the 3D scovox `roi_unknown_fraction ≤ 0.64`, not
this quantity.

| rung | mean | median | sd | robot-to-robot spread |
|------|------|--------|-----|----------------------|
| N=2  | 0.542 | 0.530 | 0.073 | 0.006 |
| N=3  | 0.581 | 0.580 | 0.078 | 0.003 |
| N=4  | 0.581 | 0.580 | 0.086 | 0.014 |

Larger teams stop with slightly more of the 2D grid covered, and the robots'
copies of the grid agree closely (spread ≤ 0.014) — the divergence discussed
below is in the 3D voxel maps, not here. At n=240 the N=3 and N=4 means are
identical to three decimals; the apparent N=3 → N=4 rise at n=120 (0.571 →
0.582) did not survive.

## Secondary — censoring

9 non-arrivals in 720 robot-runs (1.3%) across 8 cells; **zero cells lost**.
Final-event results: 711 arrived, 5 `budget`, 2 `no-progress`, 2 `timeout`. 736
homing episodes total, 16 robots with more than one.

| cell | robot | result | t_sim | dist from home |
|---|---|---|---|---|
| n3_hybrid_seed1 | atlas | budget | 1348 | 1.602 m |
| n3_pursuit_seed7 | atlas | budget | 1440 | 1.967 m |
| n4_rendezvous_seed9 | atlas | budget | 1221 | 1.810 m |
| n4_rendezvous_seed14 | bestla | budget | 1213 | 2.997 m |
| n4_rendezvous_seed19 | bestla | budget | 1197 | 2.777 m |
| n4_hybrid_seed6 | bestla | no-progress | 2015 | 3.017 m |
| n4_pursuit_seed5 | bestla | no-progress | 950 | 3.017 m |
| n2_pursuit_seed14 | atlas | **timeout** | 1695 | **34.725 m** |
| n2_pursuit_seed14 | bestla | **timeout** | 1480 | **30.195 m** |

**Three** mechanistically distinct modes now, not two. **`budget`** (five cases)
parks 1.6–3.0 m out, just outside the hard 1.0 m `final_goal_tolerance`, having
made the required progress the whole way — one case logged *zero* watchdog events
and simply ran out of budget on a long detour. **`no-progress`** (bestla, both)
is a home-watchdog escalation: approach → `resend` → `retrace` → 3 × `escape` →
park, fired 8–15 times, with **zero** `nav_goal_failed` from the navigator and
escape legs that succeed outward to 4–6 m. Both park at 3.017 m on different
bearings, different seeds and different arms. Outward is free, inward is not.

**`timeout` is new in the top-up and is qualitatively unlike the other two.**
`n2_pursuit_seed14` is the only cell in 240 where the *entire team* failed to get
home, and both robots stopped 30–35 m out — an order of magnitude further than
every other non-arrival, which park within 3 m. A metre-scale parking failure and
a 30 m abandonment are not the same phenomenon, and the earlier two-mode account
would have absorbed this one silently. It is a single cell, so it is a lead, not
a result; it is called out because the base series contained nothing like it.

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
0.20–0.50). At n=40 they were a smooth ladder with no second mode, and at n=80
they still are (0.176 → 0.831). It was small-n lumpiness. **Do not read modes off
a rung that is not full.**

What *is* real is a modest rung effect: mean bucket-empty fraction 0.374 / 0.394 /
**0.484**, still +0.099 for N=4 after adjusting for run duration (two-sided
permutation p = 0.0002, 200k perms).

**This is the finding that strengthened most on doubling** — the effect size is
unchanged (+0.0958 → +0.0988) while p fell from 0.0092 to 0.0002, which is what a
real effect does when n doubles.

The reason a naive read misleads: exhaustion is a **rate**, and
`r(exhaustion, duration)` = +0.226 / −0.518 / **−0.606** by rung. Long cells
dilute the fraction with a tail in which robots are latched, homing or idle and
barely transmitting, so the *highest*-exhaustion N=4 cells are the **shortest**
(0.831 at 542 s; 0.176 at 1992 s). Always adjust for duration before comparing
exhaustion across groups.

### Is homing failure a manoeuvre side-effect? — **null at n=120, a trend at n=240**

All 8 censored cells fall on manoeuvring arms; 0 of 180 `off` robot-runs are
censored against 9 of 540 on the manoeuvring arms. Expected count on `off` under
a uniform-arm null is 2.25, and P(0 of 8) = 0.75⁸ = 0.100 (one-sided
hypergeometric on robot-runs, 0.074).

**Still not significant, but no longer unremarkable.** At n=120 this was p =
0.237 and the honest call was "not evidence"; doubling n moved it to 0.074 with
the streak intact — `off` has now gone 180 robot-runs without a single
non-arrival while every other arm has failed at least once. The n=120 text
asserted this "would not become evidence at 10/10 either"; that assertion was
wrong in direction, and the correct reading is that the design is underpowered
for a rate this low rather than that the effect is absent. It is mechanistically
plausible — manoeuvring arms send robots on extra legs, and extra legs are extra
trap exposure — so it is carried forward as a lead for ts1c rather than closed.

Note the arm most implicated changed between halves (pursuit and hybrid at
n=120, rendezvous adding three at n=240), which is what a low-rate process
scattered across arms looks like. No per-arm claim is made.

### Regroup-spread — **refuted by its own best case**

The hypothesis was that a regrouping manoeuvre injects seed-to-seed variance.
Residual SD of finish time after removing rung means:

    hybrid      369        rendezvous  274
    off         270        pursuit     261

    hybrid alone          SD ratio vs rest = 1.376,  p = 0.0167
    rendezvous alone      SD ratio vs rest = 0.880,  p = 0.7552
    both regrouping arms  SD ratio vs rest = 1.256,  p = 0.0527

`rendezvous` regroups and has the **second-lowest** spread of all four arms. The
pooled "regrouping arms" near-miss exists only because hybrid drags it. The
variance belongs to hybrid specifically, not to regrouping — and hybrid is a
compound arm, so this does not identify which component is responsible.

**The hybrid spread effect survived the doubling but shrank materially:** 1.640
(p = 0.0103) at n=120 → **1.376 (p = 0.0167)** at n=240. The direction and
significance hold; the magnitude fell by 16%, so the n=120 figure was inflated by
small-sample noise and must not be quoted.

**Post-hoc claim from 2026-09-09 — now REFUTED.** That note reported a per-rung
Brown-Forsythe of hybrid against `off` rising across rungs — 1.28× at N=2, 2.19×
at N=3, 2.52× at N=4 (p = 0.006) — and flagged it as a lead, not a result, on the
grounds that it was one of nine unregistered rows. At n=240 the same test gives:

    n2   off sd 328.8  vs  hybrid sd 447.4   ratio 1.36x   p = 0.364
    n3   off sd 202.9  vs  hybrid sd 276.1   ratio 1.36x   p = 0.322
    n4   off sd 264.3  vs  hybrid sd 311.3   ratio 1.18x   p = 0.157

The gap is **flat across rungs, not rising**, and no rung is significant. The
striking N=4 row (2.52×, p = 0.006) collapsed to 1.18×, p = 0.157. The lead did
not survive and the rung-trend story is withdrawn. Caveat carried forward: the
pooled hybrid effect above is real, but any *per-rung* structure in it is noise
at this n.

### Endpoint map divergence — **half of it was an artifact of my own statistic**

`map_agreement_n.py` reports the **max over pairs**, and a team has
C(N,2) = 1 / 3 / 6 pairs. That max must grow with N under a pure null — the same
order-statistic trap as `t_explore`.

| rung | n | max over pairs, at latch | mean over pairs, at latch | max, at end | mean, at end |
|------|---|------------------------|---------------------------|-------------|--------------|
| N=2  | 80 |  3.45% |  3.45% | 1.20% | 1.20% |
| N=3  | 80 | 11.70% |  7.89% | 1.57% | 1.06% |
| N=4  | 77 | 15.10% |  8.38% | 3.45% | 1.90% |

Calibration — draw k pairs from each N=4 cell's **own** 6-pair pool, so the
per-pair distribution is held fixed and only the number of draws changes:

    k=1 (N=2-like): resampled max =  8.39%  [95%  7.01- 9.79]   real N=2 =  3.45%
    k=3 (N=3-like): resampled max = 13.58%  [95% 12.86-14.17]   real N=3 = 11.70%
    k=6           =                15.10%   (by construction)   real N=4 = 15.10%

Read as growth factors: the pure order-statistic prediction is
8.39 → 13.58 → 15.10, i.e. **×1.62 then ×1.11**. The observed maxima grow
**×3.39 then ×1.29**. So the N=2 → N=3 step is far larger than more pairs can
explain, and the N=3 → N=4 step is mostly — but not *entirely* — the extra
pairs: the real N=3 value sits just below the k=3 resample interval, so the
per-pair divergence is also a little smaller at N=3 than at N=4.

The **mean-over-pairs column needs no correction at all** and tells the same
story more cleanly: 3.45 → 7.89 → 8.38%, a genuine **×2.29 at N=2 → N=3** and a
flat **×1.06 at N=3 → N=4**. Quote the mean column; the max column only exists
because `map_agreement_n.py` reports it.

*Correction to the n=120 write-up.* That version said "N=3 → N=4 growth is
**entirely** the extra pairs" on the strength of a k=3 resample of 12.61%
against a real N=3 of 11.09%. Those two did not actually agree then either
(11.09 fell below the 11.56 CI floor); "entirely" was an overstatement of a
control that had already flagged a residual. At n=240 the same residual is
larger and unambiguous. The *qualitative* split — N=2 → N=3 real, N=3 → N=4
mostly artifact — is what survives.

This does not threaten `t_explore`. Each robot latches on *its own*
`roi_unknown_fraction ≤ 0.64`, so a robot missing voxels its peers hold latches
**later**, not earlier — the bias is conservative for any "larger teams are
faster" claim.

Three N=4 cells have no `at_latch` reading on any pair and are excluded from the
latch columns (77, not 80): `n4_hybrid_seed3`, `seed12`, `seed15`. The latter
two are the same cells that reach `all_done` with a `DONE` state change but emit
no `exploration_complete` event — an open instrumentation question, not a
divergence result. See *Open items*.

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
| whole-run `frac_time_near` (**not** the prior quantity) | −0.375 (p=0.0005) | −0.045 (p=0.70) | −0.038 (p=0.74) | −0.161 (p=0.0132) |
| fixed 0–400 s window, ρ(separation, `t_explore`) | −0.222 (p=0.048) | −0.169 (p=0.13) | −0.085 (p=0.45) | **−0.172 (p=0.0078)** |
| fixed 0–400 s window, ρ(%within 20 m, `t_explore`) | +0.174 (p=0.12) | +0.175 (p=0.12) | +0.063 (p=0.58) | **+0.143 (p=0.0270)** |

Prior values on the same estimator: mh1 ρ(sep) = −0.486, cr5 ρ(sep) = −0.479,
ρ(%near) = +0.345. **ts1b at n=240: −0.172 / +0.143 — same direction, heavily
attenuated.** More separation during early exploration, faster finish.

*Movement from n=120.* The pooled windowed correlations were −0.277 (p=0.0025)
and +0.231 (p=0.011) on the base series. Doubling the sample cut both roughly in
half while the p-values stayed on the same side of 0.05, which is what a real but
small effect looks like when the n=120 estimate was riding noise upward. Quote
the n=240 figures; the n=120 pair overstates the mediator by about 60%.

**New at n=240: the separation mediator is an N=2 effect.** It is visible at N=2
on both estimators and absent at N=3 and N=4 on both, and the pooled row is
carried by the N=2 rung. This is not something the base series could resolve —
each rung had 40 cells there. Two readings are consistent with it and this data
cannot separate them: either dispersion stops mattering once a third robot
supplies redundant coverage, or the pairwise summary statistics (mean pairwise
distance, %time within 20 m) stop describing team geometry once there are 3 or 6
pairs to average over. The second is a measurement worry, not a finding, and it
should be settled with a per-robot rather than per-pair dispersion measure before
anyone concludes that team size dissolves the mechanism.

The arm ordering replicates cleanly. Mean windowed separation, metres:

    pooled:   off 40.9    rendezvous 34.8    hybrid 32.8    pursuit 32.1
    n2:       off 44.5    hybrid     34.8    pursuit 34.7   rendezvous 31.6
    n3:       off 40.2    rendezvous 37.2    hybrid  32.2   pursuit    30.0
    n4:       off 37.9    rendezvous 35.6    pursuit 31.5   hybrid     31.4

`off` separates most at every rung and is nominally fastest at every rung — the
established "the naive baseline keeps up because the stack was never buying
separation" picture, now shown to hold at N=3 and N=4 as well as N=2. Note that
`off`'s separation advantage *shrinks* with N (44.5 → 40.2 → 37.9 m) while the
manoeuvring arms are near-flat, so the arms converge as the team grows.

**Do not substitute `teamlink.frac_time_near` for the windowed mediator.** They
still disagree in sign at n=240 — whole-run −0.161 against windowed +0.143 for
the same construct — and the disagreement is the run-length confound, not noise.
`teamlink.py`'s version exists for team-level link accounting, not for this
mediator.

**`isolation_mean` does not predict finish time once exposure is removed.** The
metric is in seconds, so it partly restates run length: r(`isolation_mean`,
`t_end`) = **+0.837 / +0.478 / +0.247** by rung, pooled +0.459 — at N=2 it is
close to tautological. Normalised to a fraction of run time it gives +0.301
(p = 0.0060) / −0.062 / −0.095, pooled **+0.126 (p = 0.0522)**. The pooled row is
not significant, the two upper rungs are flat and negative, and no claim is made.
The same N=2-only pattern as the separation mediator, and the same caveat applies.

What the normalised metric *does* show cleanly is the team-size effect itself —
isolated fraction of run time falls **0.648 → 0.352 → 0.193** across the rungs,
and deep-outage fraction **0.648 → 0.568 → 0.507**. Those are the connectivity
numbers to quote, not the raw seconds.

**Multiplicity.** Roughly six hypothesis tests are reported above. At n=240 the
two headline results moved apart rather than together: airtime lift is now
**p = 0.0002**, comfortably past a Bonferroni threshold of 0.008, while hybrid's
finish-time spread is **p = 0.0167**, still short of it. The airtime result is
treated as established; the variance result remains suggestive. The team-size
decomposition does not rest on a p-value — it rests on two independent routes
agreeing to 11.1 s.

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
- **Two sessions, and an unexplained 5% shift between them.** Seeds 1–10 and
  11–20 ran weeks apart; the top-up half finishes 43–60 s faster on every rung,
  pooled −52.0 s at p = 0.174. It is *not* attributed to session drift — see the
  Status section for why that label is not supported. **Why it does not
  contaminate anything here:** each invocation is seed-major across all four
  arms, so every arm contrast is within-session by construction and a common
  shift cancels out of it. What carries the shift is the absolute levels and any
  comparison against another campaign.
- **`hmr_explo` is not stamp-clean across the top-up.** The 240 cells carry four
  `hmr_explo` stamps (f05cdfe ×58, 6217bf7 ×57, 7eee1ce-dirty.a6ec2f5e ×4,
  6217bf7-dirty.39d61b78 ×1 over the top-up half). `explo_planner` — the binary
  under test — is clean `e7c185b` on all 240, and `git_hmr_explo` is not in the
  comparability key, so pooling is defensible. It is recorded because the two
  dirty stamps mean five top-up cells ran against a workspace that was not a
  named commit.
- Single simulator, single world family, single seed set of 20 per arm.

## Open items

1. Live planning-map capture around bestla's home to settle the ~3.0 m
   `no-progress` ring. Needs a running cell; the map is never bagged.
2. Per-rung airtime manipulation check via a passive rclpy subscriber on a
   running cell (read-only, costs no cells).
3. `off`'s N=3 coverage anomaly: covered 0.659 / seen 0.966 / roi_unk 0.555
   against 0.53–0.55 / 0.89–0.92 / 0.60 for the other arms.
4. Fix `map_agreement.py`'s N=2 hard-code; make `event_log` take the **last**
   `mission_complete`; scale the `overflow` gate's 90 s timeout with RTF.
5. ~~Optional top-up to 20 seeds/arm.~~ **DONE 2026-09-10**, all three rungs
   extended together in one seed-major invocation per rung. 240/240 `all_done`.
6. `n4_mtare_hybrid_seed12` and `seed15` reach `all_done` with a `DONE`
   `state_change` but emit **no `exploration_complete` event**, so they have no
   `t_explore` and are dropped from every mediator row. Both are in the top-up
   half; `seed3` additionally has no `at_latch` divergence reading. Three cells
   out of 240, but the failure is silent and the drop is only visible because the
   tools now name what they drop.
7. `n2_pursuit_seed14` is a **third** homing failure mode, seen once in 240: the
   *entire* team fails to get home, both robots `timeout` at 30.2 m and 34.7 m
   from the goal. The two known modes are a single robot parking short (≤3 m) and
   a single robot timing out; this is an order of magnitude beyond either and
   involves every robot in the cell.
8. Re-measure the separation mediator with a **per-robot** dispersion statistic
   rather than a per-pair one, to decide whether the N=3/N=4 null is the mechanism
   dissolving or the summary statistic losing resolution over 3 and 6 pairs.

## Reproduction

All analysis scripts live with the data they read, in `~/hmr_campaign/ts1_analysis/`.
Run them from there:

    cd ~/hmr_campaign/ts1_analysis

    tools/map_agreement_n.py      all C(N,2) pair gaps; --selftest re-derives every N=2 gate line (80/80)
    tools/ts1b_census.py          censoring census, final-event and all-episode
    tools/ts1b_cells.py           builds ts1b_cells.csv (endpoints + teamlink metrics)
    tools/ts1b_held.py            airtime, censoring-vs-arm, regroup-spread
    tools/ts1b_held_adjusted.py   duration-adjusted airtime; hybrid-alone vs pooled spread
    tools/ts1b_divergence.py      map divergence with the order-statistic control
    tools/ts1b_decomp.py          per-robot / extra-draw decomposition, ICC
    tools/ts1b_confound.py        exposure and mediator confound checks
    tools/ts1b_mediator.py        separation mediator on the MATCHED fixed 0-400 s window
    teamlink.py --calibrate       must PASS before any connectivity number is quoted

and in the workspace, `hmr_explo/analysis/ts1b/ts1b_finish.py`:

    ts1b_finish.py --selftest                     11 conformance checks (--selftest is a FLAG)
    ts1b_finish.py table --seeds base|topup|all   endpoint table
    ts1b_finish.py contrasts --endpoint t_explore|t_mission|per_robot
    ts1b_finish.py session                        base vs top-up, per rung

Set `PYTHONDONTWRITEBYTECODE=1` when importing from the source tree.

**Calibration hooks — use them before trusting a re-run.** Two of these tools
were found at the top-up to be reporting verdicts computed from constants frozen
at n=120: `ts1b_held.py` carried a hard-coded 5-name censored set plus
`n_off, n_man = 90, 270`, and `ts1b_finish.py`'s check 6 compared the rebuilt CSV
against `seeds="base"`. Both printed PASS while measuring nothing. The censoring
block is now derived from the events, and `TS1B_MAX_SEED=10 tools/ts1b_held.py`
restricts to the base series so the derived result can be checked against the
published n=120 figures — it reproduces all five names, `off 0/90`,
`manoeuvring 5/270`, p = 0.2373/0.2351 exactly. Any future change to that block
must still pass that calibration.
