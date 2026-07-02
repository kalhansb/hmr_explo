# HMR_Explo — experiments workspace

Orchestration workspace for the HMR autonomous-exploration localisation experiments:
**GLIM** LiDAR-IMU SLAM + **SCovox** occupancy mapping (and **explo_planner**), run over
recorded Ouster/IMU bags in per-repo Docker containers that share one ROS 2 DDS graph.

## Layout

| path | what |
|---|---|
| `ws/src/run_glim_experiment.sh` | host orchestrator — brings up the GLIM + SCovox containers, runs an experiment (`map`/`odom`/`viz`), captures outputs |
| `ws/src/scovox/` | **submodule** → [kalhansb/scovox](https://github.com/kalhansb/scovox) — occupancy mapping node |
| `ws/src/explo_planner/` | **submodule** → [kalhansb/explo_planner](https://github.com/kalhansb/explo_planner) |
| `ws/src/glim_localisation/` | GLIM SLAM + experiment/analysis scripts — **not tracked here yet** (local-only; to be added as a submodule later) |
| `ws/src/glim_upstream/` | vendored upstream GLIM source — not tracked |
| `bags/` | recorded rosbags (~54 GB) — not tracked, keep locally |

## Clone

```bash
git clone --recursive https://github.com/kalhansb/hmr_explo.git
# or, after a plain clone:
git submodule update --init --recursive
```

## Run an experiment

```bash
cd ws/src
./run_glim_experiment.sh viz "" 0.5      # GLIM SLAM + SCovox + RViz, bag at 0.5x
#                          mode  dur rate   modes: map | odom | viz
```

Outputs land in `ws/src/glim_localisation/output/` (trajectory CSV, GLIM map PCD,
SCovox occupancy map, evaluation docs).

## Real-robot trials

**Distributed multi-robot mapping** (per-robot SCovox mapper + DSCovox merger,
each robot ending up with the global fused map) has a start-to-finish runbook —
single robot and two robots — in
[`scovox/docs/distributed_mapping.md`](ws/src/scovox/docs/distributed_mapping.md).

Each package README carries a brief Docker recipe for hardware:

- `ws/src/hmr_localisation` — "Real robot (live sensors, Docker)": the NDT
  `map → odom → base_link` TF tree via `scripts/run_localization_live.sh`.
- `ws/src/scovox` — "Running on a robot" / "Multi-robot mapping": per-robot
  mapper + merger with the low-bandwidth share param files.
- `ws/src/explo_planner` — "Real-robot trials (Docker)": overlay build + run
  inside the scovox container.

Multi-robot mapping assumes every robot runs the hmr_localisation NDT tree
against the same ground-truth map, so the fleet shares one global `map` frame.
