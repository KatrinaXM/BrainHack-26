#!/bin/bash
# set_ekf_origin.sh — send the EKF origin via MAVLink (bypasses pxh> screen).
# Discovers the python helper relative to its own location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${SCRIPT_DIR}/set_ekf_origin.py" "$@"
