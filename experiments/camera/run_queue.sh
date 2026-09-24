#!/bin/bash
# Run camera replicates in order, one sim at a time (PLAN.md decision rules).
# Usage: run_queue.sh <rep> [<rep> ...]     e.g. run_queue.sh 1 2 3 4
# Each: run_cam_cell.sh rep<k> <k>; if no explore-done cue in the manifest,
# one re-run as rep<k>b. Post-processing (poses, virtual camera, readouts,
# z-band) runs in the background while the next sim starts.
set -u
C=/home/kalhan/Documents/exploitation_experiments/runs/cam_experiment
log() { echo "[$(date -Is)] $*" | tee -a $C/queue.log; }
status() { echo "- $(date '+%Y-%m-%d %H:%M') $*" >> $C/PLAN.status; }
for rep in "$@"; do
  for name in rep$rep rep${rep}b; do
    free=$(df -BG --output=avail / | tail -1 | tr -dc 0-9)
    if [ "$free" -lt 20 ]; then log "STOP: only ${free} GB free"; status "queue stopped: ${free} GB free"; exit 1; fi
    [ -e $C/$name ] && { log "skip $name (exists)"; [ -f $C/$name/run_manifest.txt ] && grep -q explore_done_cue_t_rel $C/$name/run_manifest.txt && break; continue; }
    log "start $name (rep $rep)"
    $C/run_cam_cell.sh $name $rep 2400 > $C/$name.console.log 2>&1
    rc=$?
    cue=$(sed -n 's/^explore_done_cue_t_rel=//p' $C/$name/run_manifest.txt 2>/dev/null)
    end=$(sed -n "s/^run_end_t_sim=//p;s/^run_end_reason=//p" $C/$name/run_manifest.txt 2>/dev/null | tr "\n" " ")
    log "end $name rc=$rc cue=${cue:-none} end=${end:-?}"
    if [ -n "$cue" ]; then
      status "$name finished (rc=$rc): cue t_rel=$cue, end=${end:-?}; post-processing started"
      ( $C/post_run.sh $name zband > $C/$name/post_run.log 2>&1
        status "$name post-processed: $(grep -A5 'conditional gain' $C/$name/cam_score_rendered.txt | sed -n 2,3p | tr -s ' ' | tr '\n' ';')" ) &
      break
    fi
    status "$name had no explore-done cue (rc=$rc)"
  done
done
wait
log "queue done: $*"
status "queue done: reps $*"
