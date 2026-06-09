#!/usr/bin/env bash
# run_stage2.sh — one-command launcher for BrainHack-26 Finals Pre-U Stage 2.
#
# Mission flow (per Finals brief):
#   1. land 3 HULAs on chosen pads  (Scoring Item 1)
#   2. take off again, hover, detect ArUco markers on ground robots  (Scoring Item 2)
#
# Usage:
#   ./run_stage2.sh                       # mock mode (offline testing)
#   ./run_stage2.sh --pads pads.json      # mock mode with custom pad file
#   ./run_stage2.sh --real --pads pads.json   # real pyhulax (HULA hardware required)
#
# Flags:
#   --mock         use the pyhulax mock (default if neither flag given)
#   --real         use real pyhulax + dola (requires pyhulax installed)
#   --pads PATH    pad-list JSON (default: pads_example.json next to this script)
#   --phase PHASE  'both' (default), 'land' (just land), or 'search' (just search)
#   --ambush S     ambush-watch window in seconds (default 120 for real, 30 for mock)
#   --output DIR   snapshots directory (default ./snapshots)
#   --robos N      mock: number of drones Dola "discovers" (default 3)
#   --no-aerial    do not re-takeoff after landing — watch for ArUco from pad
#   --short        mock: fast-forward timings for quick smoke test (~25 s total)
#   --help         this message

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${BH26_PYTHON:-python3}"

MODE="mock"
PADS="${SCRIPT_DIR}/pads_example.json"
PHASE="both"
AMBUSH=""
OUTPUT="./snapshots"
ROBOS="3"
SHORT_MODE="false"
NO_AERIAL="false"

print_help() {
    sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mock)       MODE="mock";    shift ;;
        --real)       MODE="real";    shift ;;
        --pads)       PADS="$2";      shift 2 ;;
        --phase)      PHASE="$2";     shift 2 ;;
        --ambush)     AMBUSH="$2";    shift 2 ;;
        --output)     OUTPUT="$2";    shift 2 ;;
        --robos)      ROBOS="$2";     shift 2 ;;
        --no-aerial)  NO_AERIAL="true"; shift ;;
        --short)      SHORT_MODE="true"; shift ;;
        --help|-h)    print_help; exit 0 ;;
        *) echo "Unknown arg: $1"; print_help; exit 1 ;;
    esac
done

if [[ ! -f "${PADS}" ]]; then
    echo "Pad file not found: ${PADS}" >&2
    exit 1
fi

# Default ambush windows differ between mock and real.
if [[ -z "${AMBUSH}" ]]; then
    if [[ "${MODE}" == "mock" ]]; then
        AMBUSH="30"
    else
        AMBUSH="120"
    fi
fi

# Common env.
export BH26_OUTPUT_DIR="${OUTPUT}"
export BH26_AMBUSH_S="${AMBUSH}"
if [[ "${NO_AERIAL}" == "true" ]]; then
    export BH26_DO_AERIAL_SEARCH="0"
fi

if [[ "${MODE}" == "mock" ]]; then
    export BH26_MOCK="1"
    export BH26_MOCK_DRONES="${ROBOS}"
    if [[ "${SHORT_MODE}" == "true" ]]; then
        # Snappy timings + frequent marker spawns for ~25 s end-to-end.
        export BH26_MOCK_TAKEOFF_S="0.8"
        export BH26_MOCK_LAND_S="0.5"
        export BH26_MOCK_SPEED_MPS="3.0"
        export BH26_MOCK_ROBO_FIRST="1.0"
        export BH26_MOCK_ROBO_PERIOD="3.0"
        export BH26_PAD_HOLD_S="1"
        export BH26_AMBUSH_S="8"
    fi
else
    if ! "${PYTHON_BIN}" -c "import pyhulax; import dola" 2>/dev/null; then
        echo "ERROR: pyhulax / dola not importable from ${PYTHON_BIN}." >&2
        echo "Install them (hardware-day setup) or use --mock for offline testing." >&2
        exit 1
    fi
fi

echo "[run_stage2] mode=${MODE} phase=${PHASE} pads=${PADS} ambush=${AMBUSH}s output=${OUTPUT}"
if [[ "${MODE}" == "mock" ]]; then
    echo "[run_stage2] mock drones=${ROBOS} short=${SHORT_MODE} no_aerial=${NO_AERIAL}"
fi

exec "${PYTHON_BIN}" "${SCRIPT_DIR}/stage2_mission.py" --pads "${PADS}" --phase "${PHASE}"
