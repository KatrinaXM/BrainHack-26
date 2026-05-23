#!/bin/bash
# start_sim.sh — non-interactive PX4 SITL launcher for x500_vision in roboverse.
#
# Runs PX4 inside a detached GNU `screen` session named 'px4', tees output to
# /tmp/px4_sim.log, and blocks until the pxh> prompt appears. The EKF origin
# is NOT set automatically — call it via:
#   ./set_ekf_origin.sh
# once the prompt is up, or attach: `screen -r px4` and type the command.
#
# Flags:
#   --no-qgc      do not launch QGroundControl (default behaviour anyway)
#   --qgc         launch QGroundControl alongside
#   --depth       use x500_depth instead of x500_vision (DO NOT USE for the Qualifier)

set -euo pipefail

MODEL="x500_vision"
LAUNCH_QGC=false
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
QGC_PATH="${QGC_PATH:-${HOME}/Desktop/QGroundControl-x86_64.AppImage}"
WORLD="roboverse"
SESSION="px4"
LOG_FILE="/tmp/px4_sim.log"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-qgc) LAUNCH_QGC=false ;;
        --qgc)    LAUNCH_QGC=true ;;
        --depth)  MODEL="x500_depth" ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
    shift
done

if screen -ls | grep -q "\.${SESSION}[[:space:]]"; then
    echo "screen session '${SESSION}' already exists. Stop the previous sim first:"
    echo "  ./stop_sim.sh"
    exit 1
fi

if [[ ! -d "${PX4_DIR}" ]]; then
    echo "PX4 directory not found at ${PX4_DIR}"
    exit 1
fi

export PX4_HOME_LAT=47.397742
export PX4_HOME_LON=8.545594
export PX4_HOME_ALT=488.0

# QGroundControl (optional).
if [[ "${LAUNCH_QGC}" == "true" ]]; then
    if [[ -f "${QGC_PATH}" ]]; then
        "${QGC_PATH}" >/dev/null 2>&1 &
        echo "Started QGroundControl (PID $!)"
        sleep 2
    else
        echo "Warning: QGroundControl not found at ${QGC_PATH}; skipping"
    fi
fi

echo "Launching PX4 SITL in screen session '${SESSION}'"
echo "  Model: ${MODEL}"
echo "  World: ${WORLD}"
echo "  Log:   ${LOG_FILE}"

: > "${LOG_FILE}"

# Start a detached screen with logging enabled so the PX4 commander prompt
# stays interactive while we tee everything to the log file.
screen -dmS "${SESSION}" -L -Logfile "${LOG_FILE}" \
    bash -c "cd ${PX4_DIR} && PX4_GZ_WORLD=${WORLD} make px4_sitl gz_${MODEL}"

echo "Waiting for PX4 startup script (up to 90 s)…"
for i in $(seq 1 45); do
    sleep 2
    if grep -q "Startup script returned successfully" "${LOG_FILE}"; then
        echo "PX4 SITL is ready (pxh> prompt available in screen session '${SESSION}')."
        echo "Next step (in another terminal):"
        echo "  ./set_ekf_origin.sh"
        echo "Or attach: screen -r ${SESSION}    (detach: Ctrl-A then D)"
        exit 0
    fi
done

echo "Timed out waiting for 'Startup script returned successfully' in ${LOG_FILE}"
echo "Tail of log:"
tail -n 40 "${LOG_FILE}"
exit 1
