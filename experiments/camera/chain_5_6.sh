#!/bin/bash
# Start reps 5-6 (PLAN.md: "5-6 only if time allows") once rep4's sim has ended.
# Waits for the "end rep4" line in queue.log (rep4's post-processing may still be
# running; that matches run_queue.sh, which starts the next sim during it).
C=/home/kalhan/Documents/exploitation_experiments/runs/cam_experiment
until grep -qE '\] end rep4 rc=[0-9]+ cue=[0-9.]+ |\] end rep4b ' $C/queue.log || grep -q 'queue done' $C/queue.log; do sleep 30; done
if docker ps --format '{{.Names}}' | grep -q '^cam_'; then
  until ! docker ps --format '{{.Names}}' | grep -q '^cam_'; do sleep 30; done
fi
cd $C && exec ./run_queue.sh 5 6 > $C/queue2.console.log 2>&1
