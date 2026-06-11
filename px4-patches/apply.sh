#!/bin/bash
# apply.sh — install the EKF-tuned x500_vision airframe into PX4-Autopilot.
# Idempotent + reversible. Run --revert to restore the original.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
AIRFRAME_PATH="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/4005_gz_x500_vision"

REVERT=false
[[ "${1:-}" == "--revert" ]] && REVERT=true

if ! ${REVERT}; then
    # Apply: backup once if no backup exists, then copy tuned.
    if [[ ! -f "${AIRFRAME_PATH}.pre-tuning" ]]; then
        cp "${AIRFRAME_PATH}" "${AIRFRAME_PATH}.pre-tuning"
        echo "Backed up original to ${AIRFRAME_PATH}.pre-tuning"
    fi
    cp "${SCRIPT_DIR}/4005_gz_x500_vision.tuned" "${AIRFRAME_PATH}"
    echo "Applied tuned airframe."
    echo "Diff:"
    diff "${AIRFRAME_PATH}.pre-tuning" "${AIRFRAME_PATH}" || true
else
    if [[ -f "${AIRFRAME_PATH}.pre-tuning" ]]; then
        cp "${AIRFRAME_PATH}.pre-tuning" "${AIRFRAME_PATH}"
        echo "Reverted to original airframe."
    else
        echo "No backup found at ${AIRFRAME_PATH}.pre-tuning — nothing to revert."
        exit 1
    fi
fi

echo "Restart sim for changes to take effect:"
echo "  ./stop_sim.sh && ./start_sim.sh"
