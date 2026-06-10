#!/usr/bin/env python3
"""
stage2_mission.py — BrainHack-26 Finals Pre-University Stage 2 orchestrator
==========================================================================

ONE Python process on the C2 laptop. Spawns one thread per HULA drone,
each running an independent state machine:

    IDLE -> TAKEOFF -> FLY_TO_PAD -> LAND_ON_PAD -> PAD_HOLD
         -> SEARCH_TAKEOFF -> AMBUSH_WATCH -> FINAL_LAND -> COMPLETE

(If BH26_DO_AERIAL_SEARCH=0 the SEARCH_TAKEOFF / FINAL_LAND states are
skipped and AMBUSH_WATCH runs from the landed position.)

Per `materials/Finals brief.pptx` (the load-bearing rules document),
Challenge 2 Pre-University:

  1. Organisers post coordinates of 5 landing pads on Discord with
     valid/invalid flags announced before assessment.
  2. We pick 3 valid pads, launch 3 HULAs from the C2 to land on them
     (SCORING ITEM 1: "Number of landings within hoop").
  3. A convoy of 5 RoboMaster ground robots enters the cage and loiters.
     Each robot carries a printed ArUco marker.
  4. We launch the HULAs again from the pads to search for the robots.
  5. HULAs detect the ArUco markers, log/print the decoded IDs
     (SCORING ITEM 2: "Number of Aruco detections").
  6. Inform judge mission complete.

Hard rules (from the brief — violation = score invalidated):
  - HULA max speed 0.5 m/s
  - Recommended height 1.1 m
  - NO FLYING OVER OBSTACLES (we stay at 1.1 m above ground throughout)
  - NO RE-ATTEMPTS if the drone crashes. Be conservative.

Usage
-----
    python3 stage2_mission.py --pads pads.json
    python3 stage2_mission.py --pads pads.json --phase land    # just land
    python3 stage2_mission.py --pads pads.json --phase search  # just search

Env vars (calibrate at the venue, see START_HERE.md §8)
-------------------------------------------------------
    BH26_PAD_FILE          overrides --pads
    BH26_HULA_IPS          comma-separated drone IPs — skips dola discovery
                           entirely (fly without the dola tool), e.g.
                           "192.168.1.101,192.168.1.102,192.168.1.103"
    BH26_DRONES_FILE       path to a {plane_id: ip} JSON (default drones.json
                           next to this script) — persistent IP config, used
                           if BH26_HULA_IPS is unset (see configure.py)
    BH26_DOLA_LISTEN       Dola discovery window (s), default 5.0
    BH26_OUTPUT_DIR        where snapshots land, default ./snapshots
    BH26_AMBUSH_S          ambush watch window (s), default 120.0
    BH26_PAD_HOLD_S        pause on pad after landing (s), default 3.0
    BH26_TAKEOFF_ALT_M     HULA altitude after takeoff (m), default 1.1
    BH26_SEARCH_ALT_M      HULA altitude during ArUco search (m), default 1.1
    BH26_DO_AERIAL_SEARCH  "1" (default) or "0" — re-takeoff after pad hold
    BH26_ARUCO_DICT        ArUco dictionary name, default DICT_7X7_1000
                           (confirmed by organizers, Discord 2026-06-10)
    BH26_EXPECTED_IDS      comma-separated ArUco IDs expected on the ground
                           robots, default "11,45,51,67,101" (report-only;
                           empty string disables the cross-check)
    BH26_AXIS_FORWARD      "+x" / "-x" / "+y" / "-y", default "+x"
    BH26_AXIS_RIGHT        same options, default "+y"
    BH26_NAV_EPSILON_M     skip moves smaller than this (m), default 0.05
    BH26_SNAPSHOT_COOLDOWN_S  min seconds between snapshots, default 2.0
    BH26_DIST_SCALE        metres -> device-units multiplier, default 100
                           (pyhulax distances are CENTIMETRES; set 1 if a
                           venue build turns out to use metres)
    BH26_NAV_MODE          "move" (default, sequential body-frame moves) or
                           "move_to" (single straight-line flight via pyhulax
                           move_to)
    BH26_QR_LOCALIZATION   "1" to call set_qr_localization(True) after connect
                           (makes move_to use ABSOLUTE mat coordinates),
                           default "0" (move_to relative to takeoff origin)
    BH26_AVOIDANCE         "1" (default) enable firmware obstacle avoidance
                           (barrier mode) at connect; "0" to disable
    BH26_NAV_FEEDBACK      "1" (default) poll get_position() to confirm arrival
                           before landing; "0" trusts the open-loop moves
    BH26_ARRIVE_TOL_M      arrival tolerance (m), default 0.20
    BH26_NAV_MAX_CORRECTIONS  max position-correction nudges, default 3
    BH26_MIN_BATTERY_PCT   refuse takeoff below this %, default 25 (0 disables)
    BH26_SEARCH_YAW_SCAN   "1" (default) rotate in place during the ambush to
                           sweep the camera ~360 deg; "0" = static hover
    BH26_YAW_STEP_DEG      degrees per yaw-scan rotation, default 45
    BH26_YAW_INTERVAL_S    seconds to dwell/watch between rotations, default 5
    BH26_CAMERA_ANGLE      camera tilt at connect, 0(forward)..90(down); empty
                           (default) leaves it. Find the value with dronecheck.py
    BH26_AMBUSH_EARLY_EXIT "1" (default) end the ambush as soon as every expected
                           ArUco ID is detected (faster = more timing points);
                           "0" always runs the full window

Units (CONFIRMED from https://pyhulax.xenops.ae/reference/pyhulax/)
-------------------------------------------------------------------
pyhulax distances are CENTIMETRES, not metres:
  move(direction, distance_cm)   takeoff(height_cm=100)   move_to(x, y, z)  # cm
  get_position() -> Vector3 (cm)  get_altitude() -> float (cm)
Our pad files are in METRES, so every distance is multiplied by BH26_DIST_SCALE
(default 100) at the pyhulax boundary. If the venue's installed build differs,
flip the scale in ONE place instead of touching the navigation maths.

move_to coordinate frame (CONFIRMED from the same reference):
  "QR localization ENABLED: coordinates are ABSOLUTE (on the QR mat).
   DISABLED (default): coordinates are relative to takeoff position (0,0,0)."
The relative mode matches this file's assumption (each HULA starts at its own
takeoff origin), so move_to is a drop-in upgrade that self-corrects drift
versus the sequential-move decomposition.

Reference: `references/finalist_codes/hula_swarm/huladola.py` (workshop
example), `TUTORIAL.md` Ch 24 (pyhulax + open API questions),
`materials/Finals brief.pptx` (rules).
"""

from __future__ import annotations

import argparse
import dataclasses
import enum
import json
import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional

# pyhulax may not be installed on the dev VM — guard the imports so this
# file is at least parseable / importable everywhere.
#
# Set BH26_MOCK=1 to use the offline test mock at codes/finals/mocks/pyhulax_mock.py
# (synthetic drones + synthetic video with periodic "RoboMaster" patches).
PYHULAX_AVAILABLE = False
PYHULAX_BACKEND = "none"
# dola (drone discovery) is a SEPARATE, vendor-provided tool — it is NOT part
# of pyhulax and is NOT the unrelated PyPI package named "dola". It is OPTIONAL:
# pyhulax connects to each drone by IP, so if you know the drone IPs you can
# fly without dola by setting BH26_HULA_IPS (see discover_hulas).
DOLA_AVAILABLE = False

if os.environ.get("BH26_MOCK") == "1":
    # Insert codes/finals/ on sys.path so `from mocks.pyhulax_mock import ...`
    # works regardless of CWD.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from mocks.pyhulax_mock import (    # type: ignore[import-not-found]
        DroneAPI, Direction, VideoStream, Dola,
    )
    PYHULAX_AVAILABLE = True
    DOLA_AVAILABLE = True
    PYHULAX_BACKEND = "mock"
else:
    # pyhulax (flight + video) — the core requirement for --real.
    try:
        from pyhulax import DroneAPI            # type: ignore[import-untyped]
        from pyhulax.core import Direction      # type: ignore[import-untyped]
        from pyhulax.video import VideoStream   # type: ignore[import-untyped]
        PYHULAX_AVAILABLE = True
        PYHULAX_BACKEND = "pyhulax"
    except ImportError:
        DroneAPI = Direction = VideoStream = None  # type: ignore[assignment]
    # dola — optional. Its absence only disables auto-discovery; BH26_HULA_IPS
    # still lets the mission fly.
    try:
        from dola import Dola                   # type: ignore[import-untyped]
        DOLA_AVAILABLE = True
    except ImportError:
        Dola = None  # type: ignore[assignment]

# cv2 is optional at module-import time — only required if we actually
# enter ambush_watch with a video frame. Keeps the module importable on
# stripped-down environments and the unit tests fast.
try:
    import cv2     # type: ignore[import-untyped]
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    cv2 = None  # type: ignore[assignment]
    np = None   # type: ignore[assignment]
    CV2_AVAILABLE = False


# ============================================================================
#  CONFIG (env-var-overridable)
# ============================================================================

DOLA_LISTEN_S      = float(os.environ.get("BH26_DOLA_LISTEN", "5.0"))
OUTPUT_DIR         = Path(os.environ.get("BH26_OUTPUT_DIR",   "./snapshots"))
AMBUSH_WINDOW_S    = float(os.environ.get("BH26_AMBUSH_S",    "120.0"))
PAD_HOLD_S         = float(os.environ.get("BH26_PAD_HOLD_S",  "3.0"))
DO_AERIAL_SEARCH   = os.environ.get("BH26_DO_AERIAL_SEARCH", "1") == "1"
SEARCH_ALT_M       = float(os.environ.get("BH26_SEARCH_ALT_M", "1.1"))

# Search-phase YAW-SCAN: during the ambush, slowly rotate IN PLACE to sweep the
# camera ~360 deg around the pad — a big coverage gain over a static hover, with
# zero translation risk (still "hovering" per the brief). Step-and-stare: dwell
# YAW_INTERVAL_S watching, then rotate YAW_STEP_DEG, repeat. Default ON.
# Timing optimisation: end the ambush EARLY once every expected ArUco ID
# (EXPECTED_MARKER_IDS) has been detected across all drones. Stage-B is scored on
# timing, so finishing sooner = more points. Needs EXPECTED_MARKER_IDS non-empty.
AMBUSH_EARLY_EXIT = os.environ.get("BH26_AMBUSH_EARLY_EXIT", "1") == "1"

SEARCH_YAW_SCAN = os.environ.get("BH26_SEARCH_YAW_SCAN", "1") == "1"
YAW_STEP_DEG    = float(os.environ.get("BH26_YAW_STEP_DEG", "45"))
YAW_INTERVAL_S  = float(os.environ.get("BH26_YAW_INTERVAL_S", "5.0"))

# Camera tilt applied at connect (degrees, 0=forward .. 90=straight down). For
# the yaw-scan to widen coverage the camera must be ANGLED (e.g. ~45), not
# straight down (90 just spins the same patch). Empty = leave the camera default.
# Find the right value on the GROUND with `dronecheck.py --camera-angle N` (the
# camera geometry is the same whether or not the drone flies), then set it here.
CAMERA_ANGLE = os.environ.get("BH26_CAMERA_ANGLE", "").strip()

NUM_DRONES         = 3        # fixed: Pre-U Stage 2 always uses 3 HULAs per brief
# Per Finals brief, mission max time is 8 minutes for Challenge 2.
# We use 9 minutes (540 s) here as a hard safety kill — anything beyond
# that is wasted time the judges will have already cut us off.
MISSION_TIMEOUT_S  = 540.0
TICK_HZ            = 5.0      # per-drone state-machine tick rate


# ============================================================================
#  LANDING ZONES
# ============================================================================

@dataclasses.dataclass
class Pad:
    pad_id: str
    x: float
    y: float
    z: float
    valid: bool


def load_pads(path: Path) -> list[Pad]:
    """Read pad list from JSON. Expected format:

        [
          {"id": "P1", "x": 1.5, "y": 2.0, "z": 0.0, "available": true},
          ...
        ]

    Tolerant parser (edit the file by hand or with `configure.py`):
      - availability flag may be "available" OR "valid" (synonyms); default True
      - "z" is optional (defaults to 0.0 — pads are on the ground)
    Only AVAILABLE pads are ever assigned to a drone (see select_pads).
    """
    with open(path) as f:
        data = json.load(f)
    pads: list[Pad] = []
    for p in data:
        available = bool(p.get("available", p.get("valid", True)))
        pads.append(Pad(
            str(p["id"]), float(p["x"]), float(p["y"]),
            float(p.get("z", 0.0)), available,
        ))
    return pads


def select_pads(pads: list[Pad], n: int = NUM_DRONES) -> list[Pad]:
    """Pick the first n AVAILABLE pads, in file order.

    Only pads flagged available (Pad.valid) are ever assigned to a drone —
    pads marked unavailable are skipped entirely. There are 5 pads at the
    Finals; some may be flagged unavailable, and Pre-U flies n=3, so this
    raises a clear error if fewer than n remain available.

    To control WHICH available pads are used, reorder competition_pads.json
    (or use `configure.py`) — the first n available in file order win.
    """
    available = [p for p in pads if p.valid]
    if len(available) < n:
        raise RuntimeError(
            f"need {n} available pads, only {len(available)} are flagged "
            f"available: {[p.pad_id for p in available]}. "
            f"Edit competition_pads.json (or run configure.py) to make more "
            f"pads available."
        )
    return available[:n]


# ============================================================================
#  PER-DRONE STATE MACHINE
# ============================================================================

class DroneState(enum.Enum):
    IDLE           = "idle"
    TAKEOFF        = "takeoff"
    FLY_TO_PAD     = "fly_to_pad"
    LAND_ON_PAD    = "land_on_pad"
    PAD_HOLD       = "pad_hold"        # landed, waiting briefly for convoy
    SEARCH_TAKEOFF = "search_takeoff"  # second takeoff to SEARCH_ALT_M
    AMBUSH_WATCH   = "ambush_watch"    # watching for ArUco markers (from air or pad)
    FINAL_LAND     = "final_land"      # end-of-mission land
    COMPLETE       = "complete"
    ERROR          = "error"


@dataclasses.dataclass
class DroneMission:
    plane_id: str
    ip: str
    drone: object                 # pyhulax.DroneAPI
    video: Optional[object]       # pyhulax.video.VideoStream
    pad: Pad
    state: DroneState = DroneState.IDLE
    state_entered_at: float = 0.0
    snapshots_saved: int = 0
    last_snapshot_at: float = 0.0
    last_yaw_at: float = 0.0
    last_error: str = ""
    marker_ids_seen: set = dataclasses.field(default_factory=set)

    def transition(self, new: DroneState) -> None:
        print(f"[{self.plane_id}] {self.state.value} -> {new.value}")
        self.state = new
        self.state_entered_at = time.time()

    def time_in_state(self) -> float:
        return time.time() - self.state_entered_at


def _preflight_battery(m: DroneMission) -> None:
    """Refuse to take off a HULA below MIN_BATTERY_PCT — a weak drone will
    failsafe-drop mid-mission. Raises CommandFailed if too low. No-op if the
    check is disabled or the drone can't report battery."""
    if MIN_BATTERY_PCT <= 0:
        return
    getb = getattr(m.drone, "get_battery", None)
    if getb is None:
        return
    try:
        pct = int(getb())
    except Exception as e:
        print(f"[{m.plane_id}] battery read failed ({e!r}) - proceeding")
        return
    print(f"[{m.plane_id}] battery {pct}%")
    if pct < MIN_BATTERY_PCT:
        raise CommandFailed(
            f"battery {pct}% < {MIN_BATTERY_PCT}% - refusing takeoff (set "
            f"BH26_MIN_BATTERY_PCT=0 to override)")


def _set_camera_angle(drone, plane_id: str, angle: int) -> None:
    """Best-effort camera tilt. The real set_camera_angle signature isn't fully
    pinned (it may want a mode + angle), so we try a couple of call shapes and
    just log if none works — never fatal (camera stays at its default)."""
    setter = getattr(drone, "set_camera_angle", None)
    if setter is None:
        print(f"[{plane_id}] set_camera_angle not supported - camera at default")
        return
    last = None
    for call in ((angle,), (1, angle)):     # (angle,) or (mode=1, angle)
        try:
            setter(*call)
            print(f"[{plane_id}] camera angle -> {angle} deg")
            return
        except Exception as e:
            last = e
    print(f"[{plane_id}] set_camera_angle({angle}) failed ({last!r}) - "
          f"leaving camera default; tune with dronecheck.py --camera-angle")


def _yaw_scan_tick(m: DroneMission) -> None:
    """During the ambush, rotate in place every YAW_INTERVAL_S to sweep the
    camera. A rejected/failed rotate is logged but does NOT abort the search —
    the drone just keeps watching from its current heading (fail-safe)."""
    if not SEARCH_YAW_SCAN:
        return
    now = time.time()
    if m.last_yaw_at == 0.0:                 # dwell once before the first rotate
        m.last_yaw_at = m.state_entered_at
    if now - m.last_yaw_at < YAW_INTERVAL_S:
        return
    try:
        result = m.drone.rotate(YAW_STEP_DEG)
        if _result_ok(result):
            print(f"[{m.plane_id}] yaw-scan: rotated {YAW_STEP_DEG:g} deg")
        else:
            print(f"[{m.plane_id}] yaw-scan: rotate rejected ({result!r}) - continuing watch")
    except Exception as e:
        print(f"[{m.plane_id}] yaw-scan: rotate error ({e!r}) - continuing watch")
    m.last_yaw_at = time.time()


def _safe_stop(m: DroneMission) -> None:
    """Bring a drone down safely after an error. A land() issued from a
    confused/MOVING state can trip the firmware emergency-stop (props off), so
    hover() FIRST to settle, then land() gently. Best-effort; never raises."""
    hov = getattr(m.drone, "hover", None)
    if hov is not None:
        try:
            hov(1.0)
            print(f"[{m.plane_id}] safe-stop: hover OK (settling)")
        except Exception as e:
            print(f"[{m.plane_id}] safe-stop: hover failed ({e!r})")
    try:
        m.drone.land()
        print(f"[{m.plane_id}] safe-stop: land OK")
    except Exception as e:
        print(f"[{m.plane_id}] safe-stop: land failed ({e!r})")


def _all_expected_found(missions) -> bool:
    """True once every expected ArUco ID has been seen across all drones.
    False if there is no expected-ID set to check against."""
    if not EXPECTED_MARKER_IDS:
        return False
    seen: set = set()
    for m in missions:
        seen |= m.marker_ids_seen
    return EXPECTED_MARKER_IDS <= seen


def run_drone_mission(
    m: DroneMission, stop_event: threading.Event,
    do_land: bool = True, do_search: bool = True,
    found_all: Optional[threading.Event] = None,
) -> None:
    """Single drone's state machine. One thread per HULA.

    pyhulax is *synchronous* (no asyncio), so each .takeoff()/.move()/.land()
    call blocks this thread until the action completes. Threads are cheap;
    3 threads for 3 HULAs is fine.

    Phase control:
      - do_land=True  do_search=True   → full mission (default)
      - do_land=True  do_search=False  → land only, stop after PAD_HOLD
      - do_land=False do_search=True   → start in air, only do search
    """
    tick_period = 1.0 / TICK_HZ
    m.transition(DroneState.IDLE)

    while not stop_event.is_set():
        try:
            if m.state == DroneState.IDLE:
                if do_land:
                    m.transition(DroneState.TAKEOFF)
                elif do_search:
                    # Skip landing — start from takeoff for search-only mode.
                    m.transition(DroneState.SEARCH_TAKEOFF)
                else:
                    m.transition(DroneState.COMPLETE)

            elif m.state == DroneState.TAKEOFF:
                # Gate on battery, then takeoff. takeoff() defaults to 100 cm
                # (1.0 m); the brief wants 1.1 m, so request it explicitly.
                _preflight_battery(m)
                _cmd(m.drone, m.plane_id, "takeoff",
                     height_cm=int(round(_to_device(TAKEOFF_ALT_M))))
                m.transition(DroneState.FLY_TO_PAD)

            elif m.state == DroneState.FLY_TO_PAD:
                arrived = navigate_to_pad(m.plane_id, m.drone, m.pad)
                if not arrived:
                    # Fail-safe: a controlled land in place beats hovering until
                    # the battery failsafes. Lose the pad's points, not the drone.
                    m.last_error = "pad arrival not confirmed (landed in place)"
                    print(f"[{m.plane_id}] WARN: arrival NOT confirmed - "
                          f"landing in place")
                m.transition(DroneState.LAND_ON_PAD)

            elif m.state == DroneState.LAND_ON_PAD:
                _cmd(m.drone, m.plane_id, "land")
                m.transition(DroneState.PAD_HOLD)

            elif m.state == DroneState.PAD_HOLD:
                # Brief pause: lets scoring item 1 ("landings within hoop")
                # complete cleanly and gives time for the convoy to enter
                # the cage before we start the search phase.
                if m.time_in_state() >= PAD_HOLD_S:
                    if not do_search:
                        m.transition(DroneState.COMPLETE)
                    elif DO_AERIAL_SEARCH:
                        m.transition(DroneState.SEARCH_TAKEOFF)
                    else:
                        # Land-based watch only (drone stays on pad).
                        m.transition(DroneState.AMBUSH_WATCH)

            elif m.state == DroneState.SEARCH_TAKEOFF:
                # Second takeoff for the ArUco search; hover at SEARCH_ALT_M.
                _cmd(m.drone, m.plane_id, "takeoff",
                     height_cm=int(round(_to_device(SEARCH_ALT_M))))
                m.transition(DroneState.AMBUSH_WATCH)

            elif m.state == DroneState.AMBUSH_WATCH:
                ambush_tick(m)            # detect ArUco on the current frame
                _yaw_scan_tick(m)         # rotate in place to widen coverage
                early = found_all is not None and found_all.is_set()
                if early:
                    print(f"[{m.plane_id}] all expected ArUco IDs found - "
                          f"ending search early (timing)")
                if early or m.time_in_state() >= AMBUSH_WINDOW_S:
                    if DO_AERIAL_SEARCH and do_search:
                        m.transition(DroneState.FINAL_LAND)
                    else:
                        m.transition(DroneState.COMPLETE)

            elif m.state == DroneState.FINAL_LAND:
                _cmd(m.drone, m.plane_id, "land")
                m.transition(DroneState.COMPLETE)

            elif m.state in (DroneState.COMPLETE, DroneState.ERROR):
                return

        except Exception as e:
            m.last_error = repr(e)
            print(f"[{m.plane_id}] EXCEPTION in {m.state.value}: {e!r} -> SAFE STOP")
            # Stabilise THEN land (hover first) — a land() from a confused state
            # can trip the firmware emergency-stop (props off / crash).
            _safe_stop(m)
            m.transition(DroneState.ERROR)
            return

        time.sleep(tick_period)


# ============================================================================
#  NAVIGATION (stub — hardware-day work)
# ============================================================================

# Takeoff is assumed to leave the drone at this altitude (arena +Z, metres).
# Used as the starting Z for navigate_to_pad's vertical correction.
# Per Finals brief: "recommended height is 1.1m". Anything higher risks
# the "no flying over obstacles" violation.
TAKEOFF_ALT_M = float(os.environ.get("BH26_TAKEOFF_ALT_M", "1.1"))

# Axis convention at takeoff. Default assumption: HULAs face arena +X at
# launch (so Direction.FORWARD moves arena +X, Direction.RIGHT moves +Y).
# Each value below can be flipped on calibration day if the convention is
# different at the venue. Recognized values: "+x", "-x", "+y", "-y".
BH26_AXIS_FORWARD = os.environ.get("BH26_AXIS_FORWARD", "+x").lower()
BH26_AXIS_RIGHT   = os.environ.get("BH26_AXIS_RIGHT",   "+y").lower()

# How close to the waypoint (in metres) we consider "arrived" — moves
# smaller than this are skipped to save time. Checked in METRES, before
# DIST_SCALE is applied.
NAV_EPSILON_M = float(os.environ.get("BH26_NAV_EPSILON_M", "0.05"))

# pyhulax distances are CENTIMETRES (confirmed from the official reference,
# https://pyhulax.xenops.ae/reference/pyhulax/). Pad files are METRES, so we
# multiply every distance by this scale at the .move()/.takeoff()/.move_to()
# boundary. Default 100 (m -> cm). If a venue build uses metres, set
# BH26_DIST_SCALE=1 — one knob, no maths changes.
DIST_SCALE = float(os.environ.get("BH26_DIST_SCALE", "100.0"))

# Real pyhulax firmware validates EVERY single move: the distance must be an
# INTEGER number of centimetres within [5, 500] (pyhulax taskprocessor SF*TP:
# `if not isinstance(distance, int) or distance > 500 or distance < 5`). So we
# round to whole cm and split any leg longer than 500 cm into <=500 cm hops.
MOVE_MAX_CM = int(os.environ.get("BH26_MOVE_MAX_CM", "500"))
MOVE_MIN_CM = int(os.environ.get("BH26_MOVE_MIN_CM", "5"))

# pyhulax move() `speed` is a position-controller P-gain LEVEL, not cm/s
# (VelocityLevel: SLOW=300 gentle ... ZOOM=100 default ... TURBO=50 aggressive).
# The Finals brief caps HULA speed at 0.5 m/s and says "be conservative", so we
# default to SLOW (300) — the gentlest motion profile. Override if needed.
MOVE_SPEED_LEVEL = int(os.environ.get("BH26_MOVE_SPEED_LEVEL", "300"))

# Navigation strategy:
#   "move"    sequential body-frame moves (UP/DOWN, FWD/BACK, LEFT/RIGHT),
#             decomposed from the pad delta. Default; matches what we tested.
#   "move_to" single straight-line flight via pyhulax move_to(x, y, z). With
#             QR localization off this is relative to the takeoff origin —
#             same assumption as the decomposition, but self-corrects drift.
NAV_MODE = os.environ.get("BH26_NAV_MODE", "move").lower()

# If set, call set_qr_localization(True) after connecting so move_to uses
# ABSOLUTE coordinates on the arena QR mat (solves the multi-drone launch-
# origin problem — every drone shares one coordinate frame). Only useful in
# NAV_MODE=move_to and only if the arena actually has a QR localization mat.
QR_LOCALIZATION = os.environ.get("BH26_QR_LOCALIZATION", "0") == "1"

# Obstacle avoidance: enable the HULA's firmware barrier mode at connect time
# so it avoids obstacles in the flight path (brief: do NOT fly over them — the
# firmware side-steps/halts rather than climbing). Default ON. Disable with
# BH26_AVOIDANCE=0 if it makes a drone halt short of its pad at the venue.
AVOIDANCE = os.environ.get("BH26_AVOIDANCE", "1") == "1"

# Closed-loop position feedback: after the open-loop moves, poll get_position()
# to confirm the drone actually reached the pad (within ARRIVE_TOL) before we
# land — instead of blindly assuming the moves worked. Default ON; auto-skips
# if the drone has no get_position(). Disable with BH26_NAV_FEEDBACK=0.
NAV_FEEDBACK    = os.environ.get("BH26_NAV_FEEDBACK", "1") == "1"
ARRIVE_TOL_M    = float(os.environ.get("BH26_ARRIVE_TOL_M", "0.20"))   # 20 cm
NAV_MAX_CORRECTIONS = int(os.environ.get("BH26_NAV_MAX_CORRECTIONS", "3"))

# Pre-flight battery gate: refuse to take off a drone below this %, so we never
# fly a weak HULA that will failsafe-drop mid-mission. 0 disables the check.
MIN_BATTERY_PCT = int(os.environ.get("BH26_MIN_BATTERY_PCT", "25"))


class CommandFailed(Exception):
    """Raised when a pyhulax command returns a non-SUCCESS CommandResult."""


def _result_ok(result) -> bool:
    """Did a pyhulax command succeed? Tolerant of the mock (returns None) and
    of the real CommandResult IntEnum (SUCCESS == 255)."""
    if result is None:
        return True                      # mock / void return
    name = getattr(result, "name", None)
    if name is not None:
        return name == "SUCCESS"
    try:
        return int(result) == 255        # CommandResult.SUCCESS
    except (TypeError, ValueError):
        return bool(result)


def _cmd(drone, plane_id: str, name: str, *args, **kwargs):
    """Call drone.<name>(*args), LOG it, and CHECK the CommandResult.

    This is the fail-safe heart of the rewrite: every flight command is logged
    (easy to debug) and its result verified. A rejected command raises
    CommandFailed instead of being silently ignored — so the supervisor can
    stabilise and land safely rather than the drone flying blind into a crash.
    """
    fn = getattr(drone, name)
    result = fn(*args, **kwargs)
    if not _result_ok(result):
        print(f"[{plane_id}] CMD FAILED: {name}{args} -> {result!r}")
        raise CommandFailed(f"{name} returned {result!r}")
    return result


def _to_device(distance_m: float) -> float:
    """Convert a distance in metres to pyhulax device units (cm by default)."""
    return distance_m * DIST_SCALE


def _read_position_cm(drone):
    """Return the drone's (forward_cm, right_cm, up_cm) from get_position(), or
    None if unavailable. ASSUMPTION: get_position().x == forward, .y == right
    (drone body frame, relative to takeoff). If that's wrong at the venue, set
    BH26_NAV_FEEDBACK=0 to fall back to open-loop — that's the safe escape."""
    getpos = getattr(drone, "get_position", None)
    if getpos is None:
        return None
    try:
        p = getpos()
    except Exception as e:
        print(f"  [nav] get_position() unavailable ({e!r}) — skipping feedback")
        return None
    try:
        return float(p.x), float(p.y), float(p.z)
    except AttributeError:
        try:
            return float(p[0]), float(p[1]), float(p[2])
        except Exception:
            return None


def _split_move_cm(total_cm: float) -> list[int]:
    """Split a distance (cm) into INTEGER hops each within [MOVE_MIN_CM,
    MOVE_MAX_CM]. Real pyhulax rejects a single move that is non-integer, or
    >500 cm, or <5 cm. Hops are distributed as evenly as possible so no hop is
    a tiny leftover remainder (e.g. 750 -> [375, 375], not [500, 250])."""
    total = int(round(total_cm))
    if total < MOVE_MIN_CM:
        return []
    n = -(-total // MOVE_MAX_CM)            # ceil(total / MOVE_MAX_CM)
    base, rem = divmod(total, n)
    return [base + (1 if i < rem else 0) for i in range(n)]


def _emit_move(plane_id: str, drone, direction, distance_cm: float) -> None:
    """Issue a body-frame move of distance_cm centimetres as one or more
    pyhulax .move() calls, each an INTEGER-cm distance within the firmware's
    [5, 500] limit, at the conservative MOVE_SPEED_LEVEL, with the result of
    every hop checked (via _cmd) so a rejected move can't be silently ignored."""
    for hop in _split_move_cm(distance_cm):
        print(f"  [{plane_id}] move {direction.name} {hop} cm")
        _cmd(drone, plane_id, "move", direction, hop, speed=MOVE_SPEED_LEVEL)


def _axis_split(axis: str) -> tuple[str, int]:
    """Parse axis spec like '+x', '-y' into (axis_letter, sign)."""
    if len(axis) != 2 or axis[0] not in "+-" or axis[1] not in "xy":
        raise ValueError(f"invalid axis {axis!r}; expected one of +x/-x/+y/-y")
    return axis[1], (1 if axis[0] == "+" else -1)


def _direction_for_axis(axis_letter: str, sign: int, positive: bool) -> "Direction":
    """Pick the pyhulax Direction enum value for the requested axis + polarity.

    `positive` here is the *desired* polarity in arena coordinates after
    accounting for axis sign convention. e.g. if FORWARD maps to +x and we
    need to move arena +x, we return Direction.FORWARD; if FORWARD maps to
    -x we return Direction.BACK.
    """
    # Look up which pair of Directions to consider for this axis letter.
    if axis_letter == "x":
        forward_dir, back_dir = Direction.FORWARD, Direction.BACK
        right_dir,   left_dir = Direction.RIGHT,   Direction.LEFT
    else:  # 'y'
        forward_dir, back_dir = Direction.RIGHT,   Direction.LEFT
        right_dir,   left_dir = Direction.FORWARD, Direction.BACK
    # Resolve sign + polarity.
    want_pos = positive if sign == 1 else not positive
    return forward_dir if want_pos else back_dir


def _verify_arrival(plane_id: str, drone, target_fwd_cm: float,
                    target_rt_cm: float) -> bool:
    """Poll get_position() and nudge the drone until it's within ARRIVE_TOL of
    the (forward, right) target — or give up safely. Returns True only if
    arrival is confirmed. Fail-safe by design:
      - if feedback is off / unavailable, trust the open-loop moves (True);
      - if the error stops improving, STOP correcting and report False
        (so the caller holds/lands rather than oscillating or flying away).
    Every check is logged so progress is easy to follow.
    """
    if not NAV_FEEDBACK:
        return True
    tol_cm = ARRIVE_TOL_M * DIST_SCALE
    prev = None
    dist = float("inf")
    for i in range(NAV_MAX_CORRECTIONS + 1):
        pos = _read_position_cm(drone)
        if pos is None:
            print(f"  [{plane_id}] nav: no position feedback - trusting open-loop")
            return True
        cur_fwd, cur_rt, _ = pos
        err_fwd = target_fwd_cm - cur_fwd
        err_rt = target_rt_cm - cur_rt
        dist = (err_fwd ** 2 + err_rt ** 2) ** 0.5
        print(f"  [{plane_id}] nav-check {i}: pos=({cur_fwd:.0f},{cur_rt:.0f})cm "
              f"target=({target_fwd_cm:.0f},{target_rt_cm:.0f})cm err={dist:.0f}cm")
        if dist <= tol_cm:
            return True
        if prev is not None and dist >= prev - 1.0:   # not improving
            print(f"  [{plane_id}] nav: not converging (err {dist:.0f}cm) - holding")
            return False
        prev = dist
        if i == NAV_MAX_CORRECTIONS:
            break
        if abs(err_fwd) >= MOVE_MIN_CM:
            _emit_move(plane_id, drone,
                       Direction.FORWARD if err_fwd > 0 else Direction.BACK,
                       abs(err_fwd))
        if abs(err_rt) >= MOVE_MIN_CM:
            _emit_move(plane_id, drone,
                       Direction.RIGHT if err_rt > 0 else Direction.LEFT,
                       abs(err_rt))
    print(f"  [{plane_id}] nav: gave up after {NAV_MAX_CORRECTIONS} corrections "
          f"(err {dist:.0f}cm)")
    return dist <= tol_cm


def navigate_to_pad(plane_id: str, drone, pad: Pad) -> bool:
    """Fly the drone to (pad.x, pad.y) at cruise altitude. Returns True if
    arrival is confirmed (or feedback is disabled/unavailable and the open-loop
    moves completed); False if we could not confirm reaching the pad.

    Sequence (mode "move"): climb-only if needed, then forward/back, then
    left/right — each leg split into <=500 cm integer hops — then a
    position-feedback loop confirms/corrects arrival before we report success.
    land() (in LAND_ON_PAD) performs the final descent onto the pad.

    Assumptions (venue-calibratable via env vars):
      - Drone launches from the C2 origin at (0, 0, TAKEOFF_ALT_M).
      - Direction.FORWARD maps to BH26_AXIS_FORWARD, RIGHT to BH26_AXIS_RIGHT.
      - get_position().x == forward, .y == right (relative to takeoff).
        If wrong at the venue, set BH26_NAV_FEEDBACK=0 to fall back to open-loop.
    """
    cur_z = TAKEOFF_ALT_M
    dx, dy, dz = pad.x, pad.y, pad.z - cur_z
    print(f"  [navigate_to_pad] {pad.pad_id} mode={NAV_MODE} scale={DIST_SCALE:g}: "
          f"(0,0,{cur_z:.2f}) -> ({pad.x:.2f},{pad.y:.2f},{pad.z:.2f}); "
          f"delta=({dx:+.2f},{dy:+.2f},{dz:+.2f})")

    # ----- Absolute / straight-line mode: one pyhulax move_to call. -----
    # move_to is itself a closed-loop straight-line flight, so we trust it.
    if NAV_MODE == "move_to":
        tx, ty, tz = _to_device(pad.x), _to_device(pad.y), _to_device(pad.z)
        print(f"  [{plane_id}] move_to({tx:.1f},{ty:.1f},{tz:.1f}) [cm]")
        _cmd(drone, plane_id, "move_to", tx, ty, tz)
        print(f"  [navigate_to_pad] {pad.pad_id} arrival (move_to)")
        return True

    # ----- Default mode: sequential body-frame moves + feedback. -----
    fwd_axis, fwd_sign = _axis_split(BH26_AXIS_FORWARD)
    rt_axis, rt_sign = _axis_split(BH26_AXIS_RIGHT)
    fwd_delta = dx if fwd_axis == "x" else dy
    rt_delta = dx if rt_axis == "x" else dy

    # 1) Climb first only if the pad hover is ABOVE the current altitude
    #    (never descend mid-flight — land() does the descent).
    if dz >= NAV_EPSILON_M:
        _emit_move(plane_id, drone, Direction.UP, _to_device(dz))
    # 2) Forward / back along BH26_AXIS_FORWARD.
    if abs(fwd_delta) >= NAV_EPSILON_M:
        _emit_move(plane_id, drone,
                   _direction_for_axis(fwd_axis, fwd_sign, fwd_delta > 0),
                   _to_device(abs(fwd_delta)))
    # 3) Left / right along BH26_AXIS_RIGHT.
    if abs(rt_delta) >= NAV_EPSILON_M:
        _emit_move(plane_id, drone,
                   _direction_for_axis(rt_axis, rt_sign, rt_delta > 0),
                   _to_device(abs(rt_delta)))

    # 4) Confirm we actually got there (the drone-frame forward/right targets
    #    are the signed displacements we intended to travel).
    arrived = _verify_arrival(plane_id, drone,
                              fwd_sign * _to_device(fwd_delta),
                              rt_sign * _to_device(rt_delta))
    print(f"  [navigate_to_pad] {pad.pad_id} "
          f"{'ARRIVED' if arrived else 'NOT CONFIRMED'}")
    return arrived


# ============================================================================
#  ARUCO DETECTION (the actual scored thing — per Finals brief)
# ============================================================================

# Per Finals brief: ground robots carry ArUco markers; "Number of Aruco
# detections" is one of the two scoring items.
#
# CONFIRMED by organizers (Discord, 2026-06-10):
#   "Use DICT: cv2.aruco.DICT_7X7_1000 — ids are 11, 45, 51, 67, 101"
#   (for Challenge 2 & 3). The markers are ON THE GROUND ROBOTS; the Stage-2
#   landing points themselves carry no markers.
ARUCO_DICT_NAME = os.environ.get("BH26_ARUCO_DICT", "DICT_7X7_1000")

# The set of ArUco IDs the organizers said will actually appear (Discord,
# 2026-06-10). We do NOT hard-filter detections to these (a real marker we
# fail to allowlist would cost a point) — but we report how many of the
# expected IDs we saw, and flag any unexpected ID as a possible misread /
# false positive. Override/clear with BH26_EXPECTED_IDS (comma-separated;
# empty string disables the check).
_expected_ids_raw = os.environ.get("BH26_EXPECTED_IDS", "11,45,51,67,101")
EXPECTED_MARKER_IDS = {
    int(s) for s in _expected_ids_raw.split(",") if s.strip()
}

# Lazy-initialised so cv2-less environments don't pay the cost.
_ARUCO_DETECTOR = None


def _get_aruco_detector():
    """Lazy-init the ArUco detector. Returns None if cv2 isn't available."""
    global _ARUCO_DETECTOR
    if _ARUCO_DETECTOR is not None:
        return _ARUCO_DETECTOR
    if not CV2_AVAILABLE:
        return None
    dict_id = getattr(cv2.aruco, ARUCO_DICT_NAME, None)
    if dict_id is None:
        raise RuntimeError(
            f"unknown ArUco dictionary {ARUCO_DICT_NAME!r}; "
            f"try DICT_4X4_50, DICT_5X5_100, DICT_6X6_250, DICT_APRILTAG_36h11"
        )
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    params = cv2.aruco.DetectorParameters()
    _ARUCO_DETECTOR = cv2.aruco.ArucoDetector(aruco_dict, params)
    return _ARUCO_DETECTOR


def ambush_tick(m: DroneMission) -> None:
    """Pull the latest video frame, run ArUco detection, snapshot if hit.

    Every detected marker's ID is the scored artefact (brief: "Number of
    Aruco detections"), so we log each NEWLY-seen ID the instant it appears
    — even if the snapshot itself is throttled by the cooldown — so no
    detection is ever missing from the run log.
    """
    if m.video is None:
        return
    f = m.video.latest_frame
    if f is None:
        return
    frame = f.to_rgb()
    detected, markers = detect_aruco_markers(frame)
    if detected:
        ids = sorted({int(mk["id"]) for mk in markers})
        new_ids = [i for i in ids if i not in m.marker_ids_seen]
        if new_ids:
            print(f"[{m.plane_id}] >>> ArUco detected IDs {ids}  (NEW: {new_ids})")
            # Record at DETECTION time (not just on a saved snapshot) so each ID
            # is logged once and the summary counts every detection even when the
            # JPEG is throttled by the cooldown.
            m.marker_ids_seen.update(new_ids)
        save_snapshot(frame, m, markers)


def detect_aruco_markers(frame) -> tuple[bool, list]:
    """Detect ArUco markers in an RGB frame.

    Returns (detected, markers). Each marker is a dict:
        {"id": int, "corners": [[x,y], [x,y], [x,y], [x,y]],
         "bbox": {"x": int, "y": int, "w": int, "h": int}}

    Per Finals brief, this is the load-bearing scoring path: "Number of
    Aruco detections" is scored at 30% (Pre-U). The decoded ID is what
    we need to "print" per the rules — we save it in the JSON sidecar
    and overlay it on the snapshot JPEG.
    """
    if not CV2_AVAILABLE or frame is None:
        return (False, [])

    det = _get_aruco_detector()
    if det is None:
        return (False, [])

    # ArUco detection works on grayscale; convert if needed.
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = det.detectMarkers(gray)

    if ids is None or len(ids) == 0:
        return (False, [])

    markers: list[dict] = []
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        # marker_corners shape: (1, 4, 2) — 4 corner xy points
        pts = marker_corners.reshape(4, 2)
        xs, ys = pts[:, 0], pts[:, 1]
        x_min, y_min = int(xs.min()), int(ys.min())
        x_max, y_max = int(xs.max()), int(ys.max())
        markers.append({
            "id": int(marker_id),
            "corners": [[float(x), float(y)] for x, y in pts],
            "bbox": {
                "x": x_min, "y": y_min,
                "w": x_max - x_min, "h": y_max - y_min,
            },
        })
    return (True, markers)


def draw_markers(bgr, markers: list):
    """Annotate a BGR image in place: green outline + yellow 'ID=N' per marker.
    Shared by save_snapshot (saved JPEGs) and the live viewer (view_camera)."""
    if not CV2_AVAILABLE:
        return bgr
    for mk in markers:
        pts = [(int(c[0]), int(c[1])) for c in mk["corners"]]
        for i in range(4):
            cv2.line(bgr, pts[i], pts[(i + 1) % 4], (0, 255, 0), 2)
        label = f"ID={mk['id']}"
        x, y = pts[0]
        cv2.putText(bgr, label, (x, max(20, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(bgr, label, (x, max(20, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return bgr


# How long after the last snapshot before we'll save another one.
# Prevents "100 snapshots of the same robot in 2 seconds" floods while
# still letting different robot appearances through.
SNAPSHOT_COOLDOWN_S = float(os.environ.get("BH26_SNAPSHOT_COOLDOWN_S", "2.0"))


def save_snapshot(frame, m: DroneMission, markers: list) -> None:
    """Persist the detected frame + ArUco marker metadata to OUTPUT_DIR.

    Writes both an annotated JPEG (with marker outlines + decoded IDs)
    and a JSON sidecar listing each marker ID. The IDs are the load-
    bearing artefact for scoring — judges will look for them.

    Cooldown-throttled to avoid flooding when a marker lingers in frame.
    """
    now = time.time()
    if now - m.last_snapshot_at < SNAPSHOT_COOLDOWN_S:
        return

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    m.snapshots_saved += 1
    m.last_snapshot_at = now
    ts = int(now * 1000)
    stem = OUTPUT_DIR / f"{m.plane_id}_{m.snapshots_saved:03d}_{ts}"

    marker_ids = [int(mk["id"]) for mk in markers]
    # Track unique marker IDs seen across the whole mission for reporting.
    for mid in marker_ids:
        m.marker_ids_seen.add(mid)

    if CV2_AVAILABLE:
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        draw_markers(bgr, markers)   # green outline + 'ID=N' label per marker
        cv2.imwrite(str(stem) + ".jpg", bgr)

    with open(str(stem) + ".json", "w") as f:
        json.dump({
            "plane_id":      m.plane_id,
            "pad_id":        m.pad.pad_id,
            "snapshot_num":  m.snapshots_saved,
            "ts_ms":         ts,
            "marker_ids":    marker_ids,
            "markers":       markers,
        }, f, indent=2)
    print(f"[{m.plane_id}] snapshot {m.snapshots_saved} -> {stem}.jpg "
          f"(ids={marker_ids})")


# ============================================================================
#  HULA DISCOVERY + CONNECTION
# ============================================================================

def discover_hulas(need: int = NUM_DRONES) -> dict[str, str]:
    """Return {plane_id: ip} for at least `need` HULAs (default NUM_DRONES=3).
    Pass need=1 for a single-drone bring-up/smoke test.

    Three ways to get IPs, in priority order:
      1. BH26_HULA_IPS — a comma-separated list of drone IPs (env override).
      2. drones.json — a {plane_id: ip} map next to this script (edit by hand
         or with configure.py). Easy persistent config; no dola needed.
      3. dola auto-discovery — the vendor tool listens for the HULA broadcast
         (UDP 8668) and returns plane_id -> IP. NOTE: the broadcast carries
         only IP/serial/identity, NOT arena position.

    All three skip nothing on pyhulax's side: pyhulax always connects by IP,
    so dola is purely a convenience. Raises if none yields >= NUM_DRONES.
    """
    if not PYHULAX_AVAILABLE:
        raise RuntimeError(
            "pyhulax not installed on this machine. Install it with "
            "`pip install \"pyhulax[video,vision]\"` (hardware day) or run with "
            "--mock / BH26_MOCK=1 for offline orchestrator testing."
        )

    # 1) Manual IP list (env) — highest priority, no dola needed.
    manual = os.environ.get("BH26_HULA_IPS", "").strip()
    if manual:
        ip_list = [s.strip() for s in manual.split(",") if s.strip()]
        ips = {f"plane{i + 1}": ip for i, ip in enumerate(ip_list)}
        print(f"[discover] BH26_HULA_IPS set - skipping dola, using {ips}")
        if len(ips) < need:
            raise RuntimeError(
                f"need {need} HULA(s), BH26_HULA_IPS lists {len(ips)}: {ips}"
            )
        return ips

    # 2) drones.json file — persistent {plane_id: ip} config, no dola needed.
    drones_path = Path(os.environ.get(
        "BH26_DRONES_FILE", str(Path(__file__).resolve().parent / "drones.json")))
    if drones_path.is_file():
        with open(drones_path) as f:
            raw = json.load(f)
        ips = {str(k): str(v) for k, v in raw.items() if v}
        print(f"[discover] {drones_path.name} found - skipping dola, using {ips}")
        if len(ips) < need:
            raise RuntimeError(
                f"need {need} HULA(s), {drones_path.name} lists "
                f"{len(ips)} with IPs: {ips}. Add more, or use dola/BH26_HULA_IPS."
            )
        return ips

    # 3) dola auto-discovery.
    if not DOLA_AVAILABLE:
        raise RuntimeError(
            "no drone discovery available: the dola tool isn't installed, "
            "BH26_HULA_IPS isn't set, and drones.json doesn't exist. Either get "
            "dola from the organizers, create drones.json (configure.py drone ...), "
            "or set BH26_HULA_IPS=192.168.1.101,192.168.1.102,192.168.1.103"
        )
    dola = Dola()
    dola.start()
    try:
        ips = dola.get_all_ips(listen_seconds=DOLA_LISTEN_S)
    finally:
        dola.stop()
    if len(ips) < need:
        raise RuntimeError(
            f"need {need} HULA(s), only found {len(ips)}: {ips}"
        )
    return ips


def connect_hulas(ips: dict[str, str]) -> list[tuple[str, str, object, object]]:
    """For each (plane_id, ip): connect, start video. Returns list of
    (plane_id, ip, drone, video) tuples sorted by plane_id for determinism."""
    out = []
    for plane_id in sorted(ips.keys()):
        ip = ips[plane_id]
        d = DroneAPI()
        d.connect(ip)
        # Obstacle avoidance: enable firmware barrier mode so the HULA avoids
        # obstacles in its path (brief: no flying OVER them). On by default.
        if AVOIDANCE and hasattr(d, "set_barrier_mode"):
            d.set_barrier_mode(True)
            print(f"[connect] {plane_id}: obstacle avoidance (barrier mode) ON")
        # Tilt the camera down for the ground-rover search (if configured).
        if CAMERA_ANGLE:
            _set_camera_angle(d, plane_id, int(float(CAMERA_ANGLE)))
        # Optional: enable QR-mat localization so move_to uses absolute
        # arena coordinates shared across all drones (see NAV_MODE notes).
        if QR_LOCALIZATION and hasattr(d, "set_qr_localization"):
            d.set_qr_localization(True)
        v = d.create_video_stream()
        d.set_video_stream(True)
        if v is not None:
            v.start()
        out.append((plane_id, str(ip), d, v))
    return out


# ============================================================================
#  MAIN
# ============================================================================

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument(
        "--pads",
        default=os.environ.get("BH26_PAD_FILE"),
        help="JSON file with pad list (or BH26_PAD_FILE env var)",
    )
    parser.add_argument(
        "--phase", choices=["both", "land", "search"], default="both",
        help=("'both' (default): land HULAs on pads then take off again to "
              "search for ArUco markers. 'land': land only (Scoring Item 1). "
              "'search': start from current state, run only the search "
              "phase (Scoring Item 2)."),
    )
    args = parser.parse_args()

    if not args.pads:
        sys.exit("--pads (or BH26_PAD_FILE env var) is required")

    do_land = args.phase in ("both", "land")
    do_search = args.phase in ("both", "search")

    print(f"[main] backend={PYHULAX_BACKEND} cv2={'yes' if CV2_AVAILABLE else 'no'} "
          f"phase={args.phase} aerial_search={DO_AERIAL_SEARCH}")
    print(f"[main] output_dir={OUTPUT_DIR} ambush_window_s={AMBUSH_WINDOW_S} "
          f"takeoff_alt_m={TAKEOFF_ALT_M} search_alt_m={SEARCH_ALT_M}")

    # 1. Load + select landing zones
    pads = load_pads(Path(args.pads))
    chosen = select_pads(pads, NUM_DRONES)
    print(f"[main] loaded {len(pads)} pads, chose {[p.pad_id for p in chosen]}")

    # 2. Discover + connect HULAs
    ips = discover_hulas()
    print(f"[main] discovered {len(ips)} HULAs: {sorted(ips.keys())}")
    connections = connect_hulas(ips)[:NUM_DRONES]

    # 3. Build mission objects (pair each drone with its assigned pad)
    missions = []
    for (plane_id, ip, drone, video), pad in zip(connections, chosen):
        missions.append(DroneMission(
            plane_id=plane_id, ip=ip, drone=drone, video=video, pad=pad,
        ))
        print(f"[main] {plane_id} ({ip}) -> pad {pad.pad_id}")

    # 4. One thread per drone. `found_all` lets the supervisor end the ambush
    #    early once every expected ArUco ID has been detected (timing score).
    stop = threading.Event()
    found_all = threading.Event()
    threads = []
    for m in missions:
        t = threading.Thread(
            target=run_drone_mission, args=(m, stop),
            kwargs={"do_land": do_land, "do_search": do_search,
                    "found_all": found_all},
            name=f"drone-{m.plane_id}", daemon=True,
        )
        t.start()
        threads.append(t)

    # 5. Supervise until all done, timeout, or Ctrl-C
    start_t = time.time()
    try:
        while time.time() - start_t < MISSION_TIMEOUT_S:
            if all(m.state in (DroneState.COMPLETE, DroneState.ERROR) for m in missions):
                break
            # Timing optimisation: signal early-exit once all expected IDs seen.
            if (AMBUSH_EARLY_EXIT and not found_all.is_set()
                    and _all_expected_found(missions)):
                print(f"[main] all expected ArUco IDs {sorted(EXPECTED_MARKER_IDS)} "
                      f"detected - ending ambush early")
                found_all.set()
            time.sleep(0.5)
        else:
            print("[main] MISSION TIMEOUT - signalling stop")
            stop.set()
    except KeyboardInterrupt:
        print("\n[main] Ctrl-C - signalling stop")
        stop.set()

    for t in threads:
        t.join(timeout=5.0)

    # 6. Summary — show per-drone state + total unique marker IDs detected,
    # since that's the score-relevant artefact.
    all_marker_ids: set = set()
    print("\n=== MISSION SUMMARY ===")
    for m in missions:
        all_marker_ids.update(m.marker_ids_seen)
        print(f"  {m.plane_id}: state={m.state.value} pad={m.pad.pad_id} "
              f"snapshots={m.snapshots_saved} "
              f"marker_ids={sorted(m.marker_ids_seen)} "
              f"err={m.last_error or '-'}")
    print(f"  TOTAL unique ArUco IDs detected: {sorted(all_marker_ids)} "
          f"(count={len(all_marker_ids)})")

    # Cross-check against the IDs the organizers said will be present.
    if EXPECTED_MARKER_IDS:
        hit = sorted(all_marker_ids & EXPECTED_MARKER_IDS)
        missing = sorted(EXPECTED_MARKER_IDS - all_marker_ids)
        unexpected = sorted(all_marker_ids - EXPECTED_MARKER_IDS)
        print(f"  EXPECTED IDs {sorted(EXPECTED_MARKER_IDS)}: "
              f"found {hit} ({len(hit)}/{len(EXPECTED_MARKER_IDS)}), "
              f"missing {missing}")
        if unexpected:
            print(f"  [!] UNEXPECTED IDs (possible misread/false positive): {unexpected}")

    return 0 if all(m.state == DroneState.COMPLETE for m in missions) else 1


if __name__ == "__main__":
    sys.exit(main())
