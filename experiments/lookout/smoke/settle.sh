#!/bin/bash
set -u
LAYOUT=$1; OUT=$2; AT_POST=1
source /lookout/sim_up.sh
python3 /lookout/smoke/settle.py $(echo $LOOKOUTS | tr ' ' ,) ${3:-240}
