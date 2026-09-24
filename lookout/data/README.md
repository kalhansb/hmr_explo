# Run data

The logs the analysis reads, copied from `runs/lookout/` (not tracked) after
the full runs of 2026-09-24. The per-scan clouds and backgrounds are left out
(`runs/lookout/<layout>/scans`, `bg`).

- `L2_full/`, `L3_full/`: `walks.csv` (one row per walk: first alarm of each
  detector and variant, false alarms, lookout hold), `first_detect.csv`,
  `alarms.csv.gz` (every alarm, hit or false), `steps.csv.gz` (every lidar,
  every walk step: person pose, lidar pose, returns in the person's box),
  `events.jsonl` (sessions, posts, links, timings), `walks.log`.
  A walk re-run after an interruption appears once per session; the analysis
  keeps only the session that finished it (its `walks.csv` row).
- `calib_gpu/`: open-ground calibration (GPU renderer), used by the analysis
  and the go/no-go.
- `worlds/forest_L2.csv`, `forest_L3.csv`: trunk positions (the posts and the
  layout figures come from these).
- `gonogo_full.txt`: stale-scan audit and go/no-go on the full runs (GO).

To re-run the analysis from these files (in the `hmrexplo:humble` container,
with `lookout/` at `/lookout` and `runs/` at `/runs`):

```bash
for L in L2_full L3_full; do
  mkdir -p runs/lookout/$L && cp lookout/data/$L/* runs/lookout/$L/ && gunzip -f runs/lookout/$L/*.gz
done
mkdir -p runs/lookout/calib_gpu runs/lookout/worlds
cp lookout/data/calib_gpu/* runs/lookout/calib_gpu/ && cp lookout/data/worlds/* runs/lookout/worlds/
python3 /lookout/analyse.py --layout L2=/runs/lookout/L2_full --layout L3=/runs/lookout/L3_full \
    --calib /runs/lookout/calib_gpu --out /runs/lookout/results
```
