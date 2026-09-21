# Re-scoring a banked cell onto the 2D coverage measure

A cell run before 2026-09-21 logged `unknown_fraction` from the 3D
column-coverage measure, which plan §6.6 retires: it counts unobservable
z-column volume, saturates near 0.50 and stops responding to exploration
from ~1500 s, while the 2D planning map over the *same* sensor data keeps
falling to ~0.06. The 2D map reconstructs exactly from the cell's bagged
`scovox_bin` streams, so banked cells are re-scored rather than re-run.

    docker run --rm --name rescore --cap-add=NET_ADMIN --shm-size=2g \
      -e CELL=off_rep1 -e RATE=4 -e ROS_DOMAIN_ID=45 \
      -v "$P/ws:/ws" -v "$P/runs:/runs" -v "$S:/sp" -v "$S/gzhome:/gzhome" \
      hmrexplo:humble /sp/gridcheck/rescore.sh

    python3 ingest_2d.py off_rep1 rescore_off_rep1.csv \
      runs/off_rep1/rosbag2 runs/off_rep1/coverage_2d.csv

`rescore.sh` replays the bag through ONE `dscovox_mapping_node` in the
production configuration; `grid_census.py` censuses the ROI once per grid;
`ingest_2d.py` converts the census to the cell's `coverage_2d.csv`, which
`aggregate.py` reads in preference to `planner_<robot>.csv`.

Four things that are easy to get wrong:

* **`/runs` must be mounted read-write.** `ros2 bag play` decompresses `.zstd`
  in place. Mounted `:ro` the container exits instantly.
* **The un-inflated topic is the one that counts.** `global_coverage_map` is
  published before the body-radius dilation; `global_planning_map` after.
  The inflated grid reports inflation as occupied and scores the ROI as more
  known than it is.
* **Census stamps are wall epoch, horizons are sim seconds.** `ingest_2d.py`
  reads the wall→sim mapping from the cell's own bagged `/clock`. It is
  per-cell and measured: the cells do not share an RTF.
* **A missing `coverage_2d.csv` is reported, never filled in.** `aggregate.py`
  leaves the cell blank and names it. The 3D column is a plausible-looking
  number in the same units, so a silent fallback would not be visible.

The replay runs at roughly 2x real time, not the requested `RATE=4` — the
reachability BFS in the census is the limit — so a 3600 s cell takes about an
hour.
