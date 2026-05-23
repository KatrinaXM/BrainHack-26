#!/bin/bash
# stop_sim.sh — tear down the PX4 SITL session and any Gazebo processes.

set +e

SESSION="px4"

if screen -ls | grep -q "\.${SESSION}[[:space:]]"; then
    echo "Killing screen session '${SESSION}'"
    screen -S "${SESSION}" -X quit
fi

# Belt-and-braces process cleanup.
pkill -f "px4_sitl" 2>/dev/null
pkill -f "px4 -i"   2>/dev/null
pkill -f "gz sim"   2>/dev/null
pkill -f "gz-sim"   2>/dev/null
pkill -f "ruby.*gz" 2>/dev/null
pkill -f "QGroundControl" 2>/dev/null

sleep 1

remaining=$(pgrep -fl "px4_sitl|gz sim|gz-sim" || true)
if [[ -n "${remaining}" ]]; then
    echo "Warning: leftover processes:"
    echo "${remaining}"
else
    echo "Clean — no sim running"
fi

exit 0
