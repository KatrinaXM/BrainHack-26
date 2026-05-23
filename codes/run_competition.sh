#!/bin/bash
# run_competition.sh — single-script launcher for the BrainHack-26 Qualifier.
# Brings up PX4 SITL + Gazebo (+ optional QGroundControl), sets the EKF
# origin, and runs the mission with the live YOLO annotated-detection
# window. Ctrl-C cleans up everything.
#
# This script is path-agnostic — it discovers its own location and finds
# its sibling files via relative paths. Clone the repo anywhere and run.
#
# Environment overrides:
#   PX4_DIR       (default: ~/PX4-Autopilot)
#   QGC_PATH      (default: ~/Desktop/QGroundControl-x86_64.AppImage)
#   BH26_STRATEGY (default: spin; alt: high_alt)
#
# Usage:
#   ./run_competition.sh            # with QGC
#   ./run_competition.sh --no-qgc   # headless
#   PX4_DIR=/opt/PX4 ./run_competition.sh

set -uo pipefail

# Resolve script + repo locations (self-discovering, no hardcoded paths).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# This script lives either in <repo>/codes/ (when cloned) or in ~/Desktop/codes/
# (the working copy on the dev VM). Either way the codes are siblings.
CODES_DIR="${SCRIPT_DIR}"
# Sim helper scripts are in the sibling 'scripts/' directory when cloned;
# falls back to $HOME for the dev VM convention.
if [[ -d "${SCRIPT_DIR}/../scripts" ]]; then
    SCRIPTS_DIR="$(cd "${SCRIPT_DIR}/../scripts" && pwd)"
elif [[ -f "${HOME}/start_sim.sh" ]]; then
    SCRIPTS_DIR="${HOME}"
else
    echo "Cannot find scripts directory. Expected either:"
    echo "  ${SCRIPT_DIR}/../scripts/ (repo layout) or"
    echo "  ${HOME}/ (dev VM layout)"
    exit 1
fi

# PX4 location override.
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
QGC_PATH="${QGC_PATH:-${HOME}/Desktop/QGroundControl-x86_64.AppImage}"
# Strategy precedence: BH26_STRATEGY env var > interactive prompt > default 'wallfollow'.
# Both strategies use depth-based obstacle avoidance (AvoidancePlanner.compute_clearance);
# they differ only in how they choose the next heading.
STRATEGY="${BH26_STRATEGY:-}"
START_QGC=true
LOG_FILE="/tmp/competition_run.log"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-qgc)     START_QGC=false ;;
        --qgc)        START_QGC=true ;;
        --strategy)   STRATEGY="$2"; shift ;;
        --wallfollow) STRATEGY="wallfollow" ;;
        --spin)       STRATEGY="spin" ;;
        --high-alt)   STRATEGY="high_alt" ;;
        --help|-h)
            grep '^#' "$0" | head -30
            exit 0 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
    shift
done

# If still no strategy, prompt interactively.
if [[ -z "${STRATEGY}" ]]; then
    echo ""
    echo "================ Strategy ================"
    echo "  Both strategies use depth-based obstacle avoidance"
    echo "  (AvoidancePlanner.compute_clearance, per LearningMaterial2)."
    echo ""
    echo "  1) wallfollow  — pure-reactive right-hand wall-follow."
    echo "                   Body-velocity only, EKF-drift-tolerant."
    echo "                   Best result on the example map (5 validated)."
    echo "  2) spin        — rotate-scan + sprint at each spot."
    echo "                   Closer to workshop's recommended pipeline."
    echo "                   More conservative, less coverage (1 validated)."
    echo "  3) high_alt    — climb above walls, sweep from above."
    echo "                   Experimental. EKF unstable past 3m altitude."
    echo "=========================================="
    echo ""
    read -r -p "Select strategy [1=wallfollow / 2=spin / 3=high_alt, default 1]: " choice
    case "${choice}" in
        2)    STRATEGY="spin" ;;
        3)    STRATEGY="high_alt" ;;
        *)    STRATEGY="wallfollow" ;;
    esac
    echo "→ using strategy: ${STRATEGY}"
fi

# Make Ctrl-C clean up everything we started.
cleanup() {
    echo ""
    echo "[run_competition] cleanup..."
    pkill -9 -f "python3.*mission.py" 2>/dev/null || true
    pkill -9 mavsdk_server 2>/dev/null || true
    "${SCRIPTS_DIR}/stop_sim.sh" >/dev/null 2>&1 || true
    pkill -9 -f QGroundControl 2>/dev/null || true
    echo "[run_competition] done."
}
trap cleanup EXIT INT TERM

echo "==============================================================="
echo "    BrainHack-26 RoboVerse Qualifier — competition runner"
echo "==============================================================="
echo "Repo:       ${SCRIPT_DIR}/.."
echo "Codes:      ${CODES_DIR}"
echo "Scripts:    ${SCRIPTS_DIR}"
echo "PX4 dir:    ${PX4_DIR}"
echo "Strategy:   ${STRATEGY}"
echo "QGC:        ${START_QGC}"
echo "Log:        ${LOG_FILE}"
echo "==============================================================="

# Step 1 — verify environment
echo ""
echo "[1/5] Verifying environment..."
for f in "${CODES_DIR}/mission.py" "${CODES_DIR}/mission_config.py" \
         "${CODES_DIR}/barrel_yolo.pt" \
         "${SCRIPTS_DIR}/start_sim.sh" "${SCRIPTS_DIR}/stop_sim.sh" \
         "${SCRIPTS_DIR}/set_ekf_origin.py"; do
    if [[ ! -f "$f" ]]; then
        echo "  MISSING: $f"
        exit 1
    fi
done
if [[ ! -d "${PX4_DIR}" ]]; then
    echo "  MISSING: ${PX4_DIR} (set PX4_DIR=... to override)"
    exit 1
fi
python3 -c "import cv2, mavsdk, ultralytics, torch, pymavlink" 2>/dev/null
if [[ $? -ne 0 ]]; then
    echo "  MISSING Python deps. Run:"
    echo "    pip install --user mavsdk ultralytics torch pymavlink opencv-python"
    exit 1
fi
echo "  OK."

# Step 2 — clean any stale sim/QGC
echo ""
echo "[2/5] Cleaning any stale sim..."
"${SCRIPTS_DIR}/stop_sim.sh" >/dev/null 2>&1 || true
pkill -9 mavsdk_server 2>/dev/null || true
pkill -9 -f QGroundControl 2>/dev/null || true
sleep 2

# Step 3 — start sim
echo ""
echo "[3/5] Starting PX4 SITL + Gazebo..."
QGC_FLAG="--no-qgc"
if $START_QGC; then
    QGC_FLAG="--qgc"
fi
PX4_DIR="${PX4_DIR}" QGC_PATH="${QGC_PATH}" \
    "${SCRIPTS_DIR}/start_sim.sh" ${QGC_FLAG} 2>&1 | tail -3
sleep 3

# Step 4 — set EKF origin so the vision drone becomes armable
echo ""
echo "[4/5] Setting EKF origin via MAVLink..."
python3 "${SCRIPTS_DIR}/set_ekf_origin.py" 2>&1 | tail -4

# Step 5 — run the mission. The live OpenCV window opens automatically.
echo ""
echo "[5/5] Running mission.py — live 'YOLO Detections' window will open."
echo "      Watch the log here. Ctrl-C cleans up everything."
echo ""
cd "${CODES_DIR}"

export BH26_DISPLAY=1
export BH26_STRATEGY="${STRATEGY}"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

python3 -u mission.py 2>&1 | tee "${LOG_FILE}"
MISSION_EXIT=${PIPESTATUS[0]}

echo ""
echo "==============================================================="
echo "    Mission finished (exit ${MISSION_EXIT})"
echo "==============================================================="
echo ""
echo "Outputs:"
echo "  Detections JSON:   /mnt/hgfs/Shared/barrels.json"
echo "                     (or ~/mission_output/barrels.json if no VM share)"
echo "  Annotated frames:  yolo_detections/"
echo "  Full log:          ${LOG_FILE}"
echo ""
echo "Summary:"
BARRELS_JSON="/mnt/hgfs/Shared/barrels.json"
[[ -f "$BARRELS_JSON" ]] || BARRELS_JSON="${HOME}/mission_output/barrels.json"
if [[ -f "${BARRELS_JSON}" ]]; then
    python3 -c "
import json
with open('${BARRELS_JSON}') as f: d = json.load(f)
print(f\"  raw_total       = {d['raw_total']}\")
print(f\"  validated_total = {d['validated_total']}\")
print(f\"  validated_by    = {d['validated_by_class']}\")
"
fi

exit $MISSION_EXIT
