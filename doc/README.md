# hmr_explo documentation

Workspace-level documentation: the field-trial campaign and how to run it.
Anything specific to one package lives in that package's own `doc/` or `docs/`
folder — see [Per-package documentation](#per-package-documentation) below.

## Start here

| I want to… | Read |
|------------|------|
| Plan the campaign — area, runs, targets, metrics | [experiment_script_forest_inspection.md](experiment_script_forest_inspection.md) |
| Run a two-robot trial on real hardware | [field_trial_manual.md](field_trial_manual.md) |
| Dry-run the stack off a bag first | [dscovox_exploration_run.md](../ws/src/explo_planner/explo_planner/doc/dscovox_exploration_run.md) |

The two workspace documents are deliberately split: the experiment script is
the **design** (what runs, in what order, and what is measured), the field
trial manual is the **operating procedure** (how to bring it up, what a healthy
run looks like, what to do when it isn't). The manual references the script's
numbers rather than repeating them, so the two cannot drift apart.

## Index

- [experiment_script_forest_inspection.md](experiment_script_forest_inspection.md)
  — multi-robot forest inspection campaign: robots and crew, area of
  operations and sub-areas, exploitation targets, run matrix, expected
  durations, panic-stop triggers, metrics, topics to record, open items.
- [field_trial_manual.md](field_trial_manual.md) — two-robot field trial
  operating manual: what each robot runs, bring-up order, per-robot
  configuration, coordination behaviour, what to watch, troubleshooting,
  data offload.

**Figures** — area of operations and ground-truth map, referenced from the
experiment script:

| File | Shows |
|------|-------|
| `ao_topdown_grid.png` | AO with the lettered/numbered cell grid. |
| `ao_sa_grid_abc.png`, `ao_sa_grid_abc_rowframe.png` | Sub-area split (SA-1/2/3), map frame and row frame. |
| `ao_grid_routes.png`, `ao_grid_routes_rowframe.png` | Planned routes over the grid. |
| `gt_map_topdown.png`, `gt_map_topdown_ao.png` | Ground-truth point cloud, full and clipped to the AO. |

## Per-package documentation

| Package | Docs | Covers |
|---------|------|--------|
| [scovox](../ws/src/scovox/docs/user_manual.md) | `docs/` | Per-robot voxel mapping and multi-robot fusion: bring-up, the three delta-stream gates, configuration, bandwidth tuning. |
| [explo_planner](../ws/src/explo_planner/explo_planner/doc/) | `doc/` | Exploration/exploitation planner: bag-replay runbooks, the exploitation design record, known limitations. |
| [simple_nav_3d](../ws/src/simple_nav_3d/doc/README.md) | `doc/` | Simulation navigation stack: features, interfaces, parameters. |
| [hmr_localisation](../ws/src/hmr_localisation/docs/) | `docs/` | NDT localisation against the shared ground-truth map. |
