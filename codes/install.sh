#!/bin/bash
# install.sh — one-shot setup for a fresh VMware VM (Ubuntu 22.04, PX4 + Gazebo
# Harmonic pre-installed per the BrainHack-26 VM image).
#
# What this does:
#   1. Verifies / installs Python dependencies (ultralytics + torch + pymavlink + opencv).
#   2. Backs up + patches the stock OakD-Lite SDF to the organiser-recommended
#      lightweight 640x480 @ 10 Hz RGB resolution.
#   3. Verifies the YOLO model file exists.
#   4. Verifies the PX4 build supports x500_vision in roboverse.
#
# Run once after cloning the repo:
#   ./install.sh
#
# Then for each attempt, just run:
#   ./run_competition.sh

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"

echo "==============================================================="
echo "    BrainHack-26 — install / verify"
echo "==============================================================="
echo "Codes dir:  ${SCRIPT_DIR}"
echo "PX4 dir:    ${PX4_DIR}"
echo "==============================================================="

# 1) Python deps
echo ""
echo "[1/4] Verifying Python dependencies..."
MISSING=()
for pkg in cv2 mavsdk ultralytics torch pymavlink; do
    if ! python3 -c "import ${pkg}" 2>/dev/null; then
        MISSING+=("${pkg}")
    fi
done
if [[ -n "${MISSING[*]}" ]]; then
    echo "  Missing: ${MISSING[*]}"
    read -r -p "  Install via pip --user? [y/N] " yn
    if [[ "${yn}" =~ ^[Yy]$ ]]; then
        # cv2 wheel is opencv-python; others use their import name.
        PIP_NAMES=""
        for m in "${MISSING[@]}"; do
            case "${m}" in
                cv2) PIP_NAMES="${PIP_NAMES} opencv-python" ;;
                *)   PIP_NAMES="${PIP_NAMES} ${m}" ;;
            esac
        done
        # PyTorch CPU wheels need the dedicated index URL.
        if echo "${PIP_NAMES}" | grep -q "torch"; then
            pip install --user --index-url https://download.pytorch.org/whl/cpu torch torchvision
            PIP_NAMES="$(echo ${PIP_NAMES} | sed 's/torch//')"
        fi
        if [[ -n "${PIP_NAMES// /}" ]]; then
            pip install --user ${PIP_NAMES}
        fi
    else
        echo "  Skipping install. Mission will fail."
        exit 1
    fi
fi
echo "  OK."

# 2) OakD-Lite SDF patch
echo ""
echo "[2/4] Verifying OakD-Lite SDF is the lightweight 640x480 @ 10 Hz version..."
SDF="${PX4_DIR}/Tools/simulation/gz/models/OakD-Lite/model.sdf"
if [[ ! -f "${SDF}" ]]; then
    echo "  MISSING: ${SDF}"
    exit 1
fi
if grep -q "<width>640</width>" "${SDF}" && grep -q "<update_rate>10</update_rate>" "${SDF}"; then
    echo "  Already patched."
else
    echo "  Patching..."
    BACKUP="${SDF}.original-$(date +%Y%m%d-%H%M%S)"
    cp "${SDF}" "${BACKUP}"
    echo "    backup: ${BACKUP}"
    # IMX214 sensor: change resolution + rate. Use a Python helper for safe XML edit.
    python3 - <<PYEOF
import re, pathlib
p = pathlib.Path("${SDF}")
text = p.read_text()
# Replace inside the IMX214 sensor block only.
def patch_imx(match):
    block = match.group(0)
    block = re.sub(r"<width>\d+</width>", "<width>640</width>", block, count=1)
    block = re.sub(r"<height>\d+</height>", "<height>480</height>", block, count=1)
    block = re.sub(r"<update_rate>\d+</update_rate>", "<update_rate>10</update_rate>", block, count=1)
    return block
text = re.sub(r"<sensor name=\"IMX214\".*?</sensor>", patch_imx, text, flags=re.S)
p.write_text(text)
print("  patched OK.")
PYEOF
fi

# 3) YOLO model
echo ""
echo "[3/4] Verifying YOLO weights..."
if [[ -f "${SCRIPT_DIR}/barrel_yolo.pt" ]]; then
    SIZE=$(stat -c '%s' "${SCRIPT_DIR}/barrel_yolo.pt")
    echo "  Found: ${SCRIPT_DIR}/barrel_yolo.pt (${SIZE} bytes)"
    if [[ "${SIZE}" -lt 1000000 ]]; then
        echo "  WARNING: file is suspiciously small (< 1 MB). Expected ~5.5 MB."
    fi
else
    echo "  MISSING: ${SCRIPT_DIR}/barrel_yolo.pt"
    echo "  Place the trained model file here before running."
    exit 1
fi

# 4) PX4 airframe sanity
echo ""
echo "[4/4] Verifying PX4 airframe..."
AIRFRAME="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/4005_gz_x500_vision"
if [[ ! -f "${AIRFRAME}" ]]; then
    echo "  MISSING: ${AIRFRAME}"
    echo "  This file should ship with PX4-Autopilot."
    exit 1
fi
if grep -q "EKF2_EV_CTRL 3" "${AIRFRAME}"; then
    echo "  x500_vision airframe has EV fusion enabled (EKF2_EV_CTRL=3)."
else
    echo "  WARNING: ${AIRFRAME} does not enable EKF2_EV_CTRL=3."
    echo "  Vision-drone arming will fail without this. See HANDOVER_DEPTH.md."
fi

echo ""
echo "==============================================================="
echo "    Install verification passed. Run:"
echo "        ./run_competition.sh"
echo "==============================================================="
