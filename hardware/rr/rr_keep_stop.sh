#!/bin/bash
# rr_keep_stop.sh — stop the zero-drive keepalive (run before a Nav2 goal).
P=$(pgrep -f 'topic pub /drive')
echo "stopping keeper: $P"
[ -n "$P" ] && kill -TERM $P 2>/dev/null; sleep 1
kill -KILL $(pgrep -f 'topic pub /drive') 2>/dev/null
echo "remaining: $(pgrep -f 'topic pub /drive' || echo none)"
