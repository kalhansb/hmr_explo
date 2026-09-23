# ts1b_finish.py — design notes and history

The long comments of `analysis/ts1b/ts1b_finish.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 1
- [selftest](#selftest) — 4

## Module scope

### ts1b-comparability-key-fields

**Comparability key fields for pooling** — attached to `KEY_FIELDS = ("git_explo_planner", "tx_power_dbm", "tree_attenuation_db",` (line 77)

```text
Manifest fields that must not vary across pooled cells. This is the
comparability group key from ~/hmr_campaign/build_index.py minus `scenario`:
in a team-size series the scenario file IS the rung (flatforest_dense_2robot
vs _3robot vs _4robot), so checking it across rungs would fire on every run.
Scenario is checked WITHIN each rung instead.
```

## selftest

### ts1b-selftest-nan-guard-origin

**Origin of the NaN guard test** — attached to `try:` (line 573)

```text
Before the guard, one NaN made obs NaN, every comparison False, and the
exact branch returned p = 0.000 on missing data.
```

### ts1b-selftest-all-seeds

**Resolver check covers all seeds** — attached to `got = {c["cell"]: c for c in load_cells(root, seeds="all", verbose=False)}` (line 619)

```text
Compare on whatever seed set the CSV actually covers, not a hard-coded
"base". This used to pass seeds="base" because 120 cells were all that
existed; after the top-up rebuilt the CSV at 240 it failed on 120
spurious "missing" rows. The point of this check is that two
independent resolvers agree -- that holds at any n, and pinning it to
a sample size turns a conformance check into a sample-size check.
```

### ts1b-latchless-divergence

**Asserted divergence on latch-less cells** — attached to `latchless = {n for n, w in want.items()` (line 626)

```text
The two resolvers diverge BY DESIGN on a latch-less cell: this one
drops it, tools/ts1b_cells.py emits a NaN row (see load_cells'
docstring). Assert that divergence instead of describing it -- a
documented difference that nothing tests is indistinguishable from a
resolver that quietly started dropping good cells.
```

### ts1b-selftest-tolerance

**Why the endpoint tolerance is 1e-6** — attached to `ck("   all endpoints match the frozen values to 1e-6", len(bad), 0)` (line 658)

```text
1e-6, not the 0.51 this used to carry: the frozen CSV stores full
precision (730.3, 1079.25, 44.705914...), so the only legitimate
divergence is fmean's compensated summation, ~1e-12. A 0.51 window
would have passed a resolver that truncated every stamp with int().
```
