# ts1b analysis scripts

Reproducible form of the finish-time numbers in
[`../../TS1B_TEAM_SIZE_RESULTS.md`](../../TS1B_TEAM_SIZE_RESULTS.md).

| script | what |
|---|---|
| `ts1b_finish.py` | arm × rung finish-time tables, arm contrasts, base-vs-top-up session check |

Stdlib only. Reads campaign cell directories directly, so it works while a
campaign is still running — it excludes any cell whose manifest is not
`run_end_reason=all_done` rather than truncating it, and says how many it
skipped.

```bash
./ts1b_finish.py --selftest          # run this first, always
./ts1b_finish.py table               # arm x rung, all three endpoints
./ts1b_finish.py contrasts           # each arm vs off, per rung and pooled
./ts1b_finish.py session             # base half vs top-up half
./ts1b_finish.py table --csv out.csv

# defaults: --root ~/hmr_campaign  --seeds all  --endpoint t_explore
```

Exit code `0` means nothing was suppressed, `2` means the report printed but
something was refused — unbalanced pooling, a mismatched session mixture, a
dropped endpoint. `2` is the normal code mid-campaign, when the rungs are
partly filled. Do not paper over it in a wrapper script; read what it refused.

## Run the selftest first

`--selftest` is not a smoke test. Every case has an answer known independently
of the code: hand arithmetic for the permutation p and the Brown-Forsythe
transform, one case forcing each branch of the permutation test against the
other, and one re-deriving all 120 base cells from their event logs and diffing
against the frozen `ts1b_cells.csv` to 1e-6. A guard whose expected value is
computed by the code it guards is not a guard — six checks in this project once
went inert while still printing PASS.

It has already earned its keep twice. First, the case meant to exercise the
Monte Carlo branch was silently taking the exact branch, because the branch
threshold and the iteration count were the same parameter. Second, a fable
review found that the frozen-CSV comparison used `abs(got - want) > 0.51`,
which is *False* when `got` is NaN — so the one check that re-derives real data
would have passed a resolver that had started returning NaN everywhere. Both
are fixed; both are now cases in their own right.

Know what case 6 cannot do. It proves conformance to
`tools/ts1b_cells.py`'s pipeline on the base series, and the two read event logs
the same way, so a bug shared with the reference is invisible to it. That is why
the hand-arithmetic cases exist beside it.

## What the scripts refuse to do

**They never write `ts1b_cells.csv`.** That file is the frozen 120-cell base
artifact the results document cites; `~/hmr_campaign/ts1_analysis/tools/ts1b_cells.py`
owns it. Everything here is read-only against the campaign tree.

**They refuse to pool across rungs when the arms are unbalanced.** A pooled arm
mean over an unbalanced rung composition is partly a rung mean, so the arm
contrast would be confounded with team size. `check_balance` returns False and
`contrasts` drops the POOLED row rather than printing a caveat nobody reads.
The refusal binds the **spread** test and the **session** check too, which pool
just as hard — the spread section used to ignore it. Its categories come from
the data, not from the `ARMS`/`RUNGS` constants, so a future `n5` rung fails the
check instead of slipping past a loop that never looked at it.

**They refuse an ALL row on the session check when the halves' rung mixtures
differ.** Mid-campaign the base half is 40/40/40 and the top-up half is 40/21/0;
rung means differ by hundreds of seconds, so an ALL row over those would report
a composition difference as session drift.

**They will not carry a missing endpoint into a test.** A cell with no
`mission_complete` gets `t_mission = NaN`; `select` drops it and the report says
how many it dropped. This is not tidiness. `perm_test` on a sample containing
one NaN gives `obs = NaN`, makes every `>= obs` comparison False, and returns
**p = 0.000** — missing data manufacturing a maximally significant result.
`perm_test` now raises on non-finite input as a second line of defence.

**They will not silently pool across binary generations or radio regimes.**
Loading warns if more than one `git_explo_planner` revision appears (different
revisions are different binary generations), and if any of `tx_power_dbm`,
`tree_attenuation_db`, `max_range_m`, `done_criterion`, `duration_s` varies
across the loaded cells. `scenario` is checked *within* each rung only: in a
team-size series the scenario file is the rung.

## Four things the numbers mean

**Censoring bounds `t_mission`, and nothing else.** A censored cell is one where
a robot's last `mission_complete` has `result != "arrived"` — it stopped short of
home and is stamped where it stopped. That makes the affected `t_mission` means
lower bounds. It does **not** touch `t_explore` or `per_robot`, which are latch
events. The note used to print under the `t_explore` table, which is exactly
backwards: it discounted the clean column and left the biased one unmarked.

**Endpoints use the LAST event, not the first.** Both `exploration_complete` and
`mission_complete` are taken as the last such event per robot, then maxed over
robots. Nine robots in the base series had two homing episodes, and a
first-event reader records a false censoring for each. Do not "fix" this to
match another script — a mismatched estimator has already nearly cost this
project a real result once (the separation mediator, `CAMPAIGN_FINDINGS.md` R1).

**`per_robot` is printed beside every makespan on purpose.** `t_explore` and
`t_mission` are maxima over robots, so they gain a draw at every rung and must
rise under a pure team-size null. Reporting the makespan alone understates the
team-size effect; reporting `per_robot` alone ignores that a team is only done
when its last robot is. The finding lives in the gap between them.

**The arm axis is a screen, not a test.** The preregistration fixes it that way
at n=10 per arm-rung. Pooling across rungs to reach a larger n is a post-hoc
analysis and `contrasts` says so in its own output. Report it labelled as such.

## One deliberate divergence from `tools/ts1b_cells.py`

The reference takes `max(mission)` over whatever robots reported
`mission_complete`; this script requires all of them and returns NaN otherwise,
because a makespan over three of four robots is not a makespan. It also drops a
latch-less cell where the reference emits a NaN row. Neither divergence changes
a single number on the 120 base cells — the selftest checks all three endpoints
to 1e-6 and the cell count, and both pass — so the two remain comparable. The
divergence is a guard against a case that has not occurred yet.
