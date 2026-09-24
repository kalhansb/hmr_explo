#!/bin/bash
# Walk-driver test (in the container): lookouts spawned on their posts, no
# planner; a few short walks. Usage: walk_test.sh <layout> <out> [walks.py args]
set -u
LAYOUT=$1; OUT=$2; shift 2
AT_POST=1
source /lookout/sim_up.sh
sleep 5                             # let the Huskies settle on their wheels
timeout 3000 python3 /lookout/walks.py --layout $LAYOUT --out $OUT "$@" 2>&1 | tee $OUT/walks.log
