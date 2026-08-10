#!/bin/sh
# Sample nav_health into a file for the length of a test, since nav_health itself only
# prints to the screen -- the node logs it reads are always on disk, but the summary
# view is not, and that is the view worth keeping next to a run.
#   sh nav_health_record.sh [period_s] [window_s]
P=${1:-15}; W=${2:-60}
out="/userdata/x5/logs/nav_health_$(date +%Y%m%d_%H%M%S).log"
echo "recording every ${P}s (window ${W}s) -> $out"
while :; do
  { echo "########## $(date +%H:%M:%S) ##########"; sh /userdata/x5/nav_health.sh "$W"; } >> "$out" 2>&1
  sleep "$P"
done
