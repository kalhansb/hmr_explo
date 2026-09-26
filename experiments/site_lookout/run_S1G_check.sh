#!/bin/bash
set -u
LAYOUT=S1G; OUT=/runs/site_lookout/S1G_check
NO_LOOKOUTS=1 SIM_TAG=_absent source /lookout/sim_up.sh
timeout 20000 python3 /lookout/walks.py --layout S1G --out $OUT --mode check --check-how absent 2>&1 | tee -a $OUT/walks.log
echo "[S1G] done"
