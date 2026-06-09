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
    BH26_DOLA_LISTEN       Dola discovery window (s), default 5.0
    BH26_OUTPUT_DIR        where snapshots land, default ./snapshots
    BH26_AMBUSH_S          ambush watch window (s), default 120.0
    BH26_PAD_HOLD_S        pause on pad after landing (s), default 3.0
    BH26_TAKEOFF_ALT_M     HULA altitude after takeoff (m), default 1.1
    BH26_SEARCH_ALT_M      HULA altitude during ArUco search (m), default 1.1
    BH26_DO_AERIAL_SEARCH  "1" (default) or "0" — re-takeoff after pad hold
    BH26_ARUCO_DICT        ArUco dictionary name, default DICT_6X6_250
    BH26_AXIS_FORWARD      "+x" / "-x" / "+y" / "-y", default "+x"
    BH26_AXIS_RIGHT        same options, default "+y"
    BH26_NAV_EPSILON_M     skip moves smaller than this (m), default 0.05
    BH26_SNAPSHOT_COOLDOWN_S  min seconds between snapshots, default 2.0

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

if os.environ.get("BH26_MOCK") == "1":
    # Insert codes/finals/ on sys.path so `from mocks.pyhulax_mock import ...`
    # works regardless of CWD.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from mocks.pyhulax_mock import (    # type: ignore[import-not-found]
        DroneAPI, Direction, VideoStream, Dola,
    )
    PYHULAX_AVAILABLE = True
    PYHULAX_BACKEND = "mock"
else:
    try:
        from pyhulax import DroneAPI            # type: ignore[import-untyped]
        from pyhulax.core import Direction      # type: ignore[import-untyped]
        from pyhulax.video import VideoStream   # type: ignore[import-untyped]
        from dola import Dola                   # type: ignore[import-untyped]
        PYHULAX_AVAILABLE = True
        PYHULAX_BACKEND = "pyhulax"
    except ImportError:
        DroneAPI = Direction = VideoStream = Dola = None  # type: ignore[assignment]

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
          {"id": "P1", "x": 1.5, "y": 2.0, "z": 0.0, "valid": true},
          ...
        ]

    Organisers haven't published the exact format yet — when they do,
    extend this parser to be tolerant of their field names.
    """
    with open(path) as f:
        data = json.load(f)
    return [
        Pad(p["id"], float(p["x"]), float(p["y"]),
            float(p["z"]), bool(p["valid"]))
        for p in data
    ]


def select_pads(pads: list[Pad], n: int = NUM_DRONES) -> list[Pad]:
    """Pick the n best VALID pads. Skeleton picks the first n valid ones.

    Real impl on hardware day could optimise for total flight time
    (greedy nearest-neighbour from C2 origin), or for separation
    (so HULAs don't fly into each other), or both.
    """
    valid = [p for p in pads if p.valid]
    if len(valid) < n:
        raise RuntimeError(f"need {n} valid pads, got {len(valid)}")
    return valid[:n]


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
    last_error: str = ""
    marker_ids_seen: set = dataclasses.field(default_factory=set)

    def transition(self, new: DroneState) -> None:
        print(f"[{self.plane_id}] {self.state.value} -> {new.value}")
        self.state = new
        self.state_entered_at = time.time()

    def time_in_state(self) -> float:
        return time.time() - self.state_entered_at


def run_drone_mission(
    m: DroneMission, stop_event: threading.Event,
    do_land: bool = True, do_search: bool = True,
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
                m.drone.takeoff()
                m.transition(DroneState.FLY_TO_PAD)

            elif m.state == DroneState.FLY_TO_PAD:
                navigate_to_pad(m.drone, m.pad)
                m.transition(DroneState.LAND_ON_PAD)

            elif m.state == DroneState.LAND_ON_PAD:
                m.drone.land()
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
                m.drone.takeoff()
                m.transition(DroneState.AMBUSH_WATCH)

            elif m.state == DroneState.AMBUSH_WATCH:
                ambush_tick(m)
                if m.time_in_state() >= AMBUSH_WINDOW_S:
                    if DO_AERIAL_SEARCH and do_search:
                        m.transition(DroneState.FINAL_LAND)
                    else:
                        m.transition(DroneState.COMPLETE)

            elif m.state == DroneState.FINAL_LAND:
                m.drone.land()
                m.transition(DroneState.COMPLETE)

            elif m.state in (DroneState.COMPLETE, DroneState.ERROR):
                return

        except Exception as e:
            m.last_error = repr(e)
            print(f"[{m.plane_id}] EXCEPTION in {m.state.value}: {e}")
            # Best-effort safe land before exiting.
            try:
                m.drone.land()
            except Exception:
                pass
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
# smaller than this are skipped to save time.
NAV_EPSILON_M = float(os.environ.get("BH26_NAV_EPSILON_M", "0.05"))


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


def navigate_to_pad(drone, pad: Pad) -> None:
    """Fly the drone to (pad.x, pad.y, pad.z) using sequential body-frame moves.

    Decomposes the target into three orthogonal moves issued in order:
    vertical first (so the drone isn't dragging across pads at low alt),
    then forward/back, then left/right.

    Assumptions (all venue-calibratable via env vars at top of file):
      - Drone launches from the C2 origin at (0, 0, TAKEOFF_ALT_M).
      - At takeoff, Direction.FORWARD maps to BH26_AXIS_FORWARD in arena frame.
      - At takeoff, Direction.RIGHT   maps to BH26_AXIS_RIGHT   in arena frame.
      - pyhulax `.move(direction, distance)` takes a positive distance in metres.

    Open question (hardware day):
      - If pyhulax exposes `.goto(x, y, z)` or similar absolute-position
        API, prefer that — it self-corrects for accumulated drift across
        moves. The decomposition here is just the fallback for the
        documented `.move()` interface.
    """
    # Where the drone currently believes it is (in arena coordinates).
    cur_x, cur_y, cur_z = 0.0, 0.0, TAKEOFF_ALT_M
    # Compute target deltas.
    dx = pad.x - cur_x
    dy = pad.y - cur_y
    dz = pad.z - cur_z
    print(f"  [navigate_to_pad] {pad.pad_id}: from ({cur_x:.2f},{cur_y:.2f},"
          f"{cur_z:.2f}) -> ({pad.x:.2f},{pad.y:.2f},{pad.z:.2f}); "
          f"delta=({dx:+.2f},{dy:+.2f},{dz:+.2f})")

    # 1) Vertical first (climb or descend to pad altitude).
    if abs(dz) >= NAV_EPSILON_M:
        v_dir = Direction.UP if dz > 0 else Direction.DOWN
        print(f"  [navigate_to_pad] {v_dir.name} {abs(dz):.2f} m")
        drone.move(v_dir, abs(dz))

    # 2) Forward / back along BH26_AXIS_FORWARD.
    fwd_axis, fwd_sign = _axis_split(BH26_AXIS_FORWARD)
    fwd_delta = dx if fwd_axis == "x" else dy
    if abs(fwd_delta) >= NAV_EPSILON_M:
        fwd_dir = _direction_for_axis(fwd_axis, fwd_sign, fwd_delta > 0)
        print(f"  [navigate_to_pad] {fwd_dir.name} {abs(fwd_delta):.2f} m")
        drone.move(fwd_dir, abs(fwd_delta))

    # 3) Left / right along BH26_AXIS_RIGHT.
    rt_axis, rt_sign = _axis_split(BH26_AXIS_RIGHT)
    rt_delta = dx if rt_axis == "x" else dy
    if abs(rt_delta) >= NAV_EPSILON_M:
        rt_dir = _direction_for_axis(rt_axis, rt_sign, rt_delta > 0)
        print(f"  [navigate_to_pad] {rt_dir.name} {abs(rt_delta):.2f} m")
        drone.move(rt_dir, abs(rt_delta))

    print(f"  [navigate_to_pad] {pad.pad_id} arrival")


# ============================================================================
#  ARUCO DETECTION (the actual scored thing — per Finals brief)
# ============================================================================

# Per Finals brief: ground robots carry ArUco markers; "Number of Aruco
# detections" is one of the two scoring items. DICT_6X6_250 matches what
# was documented for the Mapping Drone in TUTORIAL Ch 22 — most likely
# the same dictionary is used on the ground robots, but CONFIRM AT VENUE.
ARUCO_DICT_NAME = os.environ.get("BH26_ARUCO_DICT", "DICT_6X6_250")

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
    """Pull the latest video frame, run ArUco detection, snapshot if hit."""
    if m.video is None:
        return
    f = m.video.latest_frame
    if f is None:
        return
    frame = f.to_rgb()
    detected, markers = detect_aruco_markers(frame)
    if detected:
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
        # Draw each marker's outline + its decoded ID as a label.
        for mk in markers:
            corners = mk["corners"]
            pts = [(int(c[0]), int(c[1])) for c in corners]
            # Outline in green.
            for i in range(4):
                cv2.line(bgr, pts[i], pts[(i + 1) % 4], (0, 255, 0), 2)
            # ID label above the marker, in yellow on black background.
            label = f"ID={mk['id']}"
            x, y = pts[0]
            cv2.putText(bgr, label, (x, max(20, y - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(bgr, label, (x, max(20, y - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
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

def discover_hulas() -> dict[str, str]:
    """Return {plane_id: ip}. Raises if fewer than NUM_DRONES found."""
    if not PYHULAX_AVAILABLE:
        raise RuntimeError(
            "pyhulax / dola not installed on this machine. "
            "Either install pyhulax (hardware day) or build a mock under "
            "codes/finals/mocks/ for offline orchestrator testing."
        )
    dola = Dola()
    dola.start()
    try:
        ips = dola.get_all_ips(listen_seconds=DOLA_LISTEN_S)
    finally:
        dola.stop()
    if len(ips) < NUM_DRONES:
        raise RuntimeError(
            f"need {NUM_DRONES} HULAs, only found {len(ips)}: {ips}"
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

    # 4. One thread per drone
    stop = threading.Event()
    threads = []
    for m in missions:
        t = threading.Thread(
            target=run_drone_mission, args=(m, stop),
            kwargs={"do_land": do_land, "do_search": do_search},
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
            time.sleep(0.5)
        else:
            print("[main] MISSION TIMEOUT — signalling stop")
            stop.set()
    except KeyboardInterrupt:
        print("\n[main] Ctrl-C — signalling stop")
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

    return 0 if all(m.state == DroneState.COMPLETE for m in missions) else 1


if __name__ == "__main__":
    sys.exit(main())
