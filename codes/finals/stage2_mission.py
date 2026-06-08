#!/usr/bin/env python3
"""
stage2_mission.py — BrainHack-26 Finals Pre-University orchestrator
===================================================================

ONE Python process on the C2 laptop. Spawns one thread per HULA drone,
each running an independent state machine:

    IDLE -> TAKEOFF -> FLY_TO_PAD -> LANDING -> AMBUSH_WATCH -> COMPLETE

Per Pre-U rules (`materials/RoboVerse 2026 Finals.pdf`, Challenge Two):

  1. Organisers provide a landing-zone list with valid/invalid flags.
  2. We pick 3 valid zones, launch 3 HULAs from the C2 to land on them.
  3. 5 RoboMaster ground robots enter the cage; HULAs detect + snapshot.

Scored on landing accuracy + time, then snapshot count + accuracy + time.

Usage
-----
    python3 stage2_mission.py --pads pads.json

Env vars
--------
    BH26_PAD_FILE     overrides --pads
    BH26_DOLA_LISTEN  Dola discovery window (s), default 5.0
    BH26_OUTPUT_DIR   where snapshots land, default ./snapshots
    BH26_AMBUSH_S     ambush watch window (s), default 120.0

Status: SKELETON. Real pyhulax navigation and RoboMaster detection are
stubbed. See the TODO blocks for what hardware-day work unblocks.

Reference: `references/finalist_codes/hula_swarm/huladola.py`,
`TUTORIAL.md` Ch 24 (pyhulax glossary + the open API questions).
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

DOLA_LISTEN_S    = float(os.environ.get("BH26_DOLA_LISTEN", "5.0"))
OUTPUT_DIR       = Path(os.environ.get("BH26_OUTPUT_DIR", "./snapshots"))
AMBUSH_WINDOW_S  = float(os.environ.get("BH26_AMBUSH_S",   "120.0"))
NUM_DRONES       = 3        # fixed: Pre-U Stage 2 always uses 3 HULAs
MISSION_TIMEOUT_S = 600.0   # safety: hard kill after 10 min
TICK_HZ          = 5.0      # per-drone state-machine tick rate


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
    IDLE          = "idle"
    TAKEOFF       = "takeoff"
    FLY_TO_PAD    = "fly_to_pad"
    LANDING       = "landing"
    AMBUSH_WATCH  = "ambush_watch"
    COMPLETE      = "complete"
    ERROR         = "error"


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

    def transition(self, new: DroneState) -> None:
        print(f"[{self.plane_id}] {self.state.value} -> {new.value}")
        self.state = new
        self.state_entered_at = time.time()

    def time_in_state(self) -> float:
        return time.time() - self.state_entered_at


def run_drone_mission(m: DroneMission, stop_event: threading.Event) -> None:
    """Single drone's state machine. One thread per HULA.

    pyhulax is *synchronous* (no asyncio), so each .takeoff()/.move()/.land()
    call blocks this thread until the action completes. Threads are cheap;
    3 threads for 3 HULAs is fine.
    """
    tick_period = 1.0 / TICK_HZ
    m.transition(DroneState.IDLE)

    while not stop_event.is_set():
        try:
            if m.state == DroneState.IDLE:
                m.transition(DroneState.TAKEOFF)

            elif m.state == DroneState.TAKEOFF:
                m.drone.takeoff()
                m.transition(DroneState.FLY_TO_PAD)

            elif m.state == DroneState.FLY_TO_PAD:
                navigate_to_pad(m.drone, m.pad)
                m.transition(DroneState.LANDING)

            elif m.state == DroneState.LANDING:
                m.drone.land()
                m.transition(DroneState.AMBUSH_WATCH)

            elif m.state == DroneState.AMBUSH_WATCH:
                ambush_tick(m)
                if m.time_in_state() >= AMBUSH_WINDOW_S:
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
TAKEOFF_ALT_M = float(os.environ.get("BH26_TAKEOFF_ALT_M", "1.0"))

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
#  ROBOMASTER DETECTION (stub — software-day work)
# ============================================================================

def ambush_tick(m: DroneMission) -> None:
    """Pull the latest video frame, run RoboMaster detection, snapshot if hit."""
    if m.video is None:
        return
    f = m.video.latest_frame
    if f is None:
        return
    frame = f.to_rgb()
    detected, bboxes = detect_robomaster(frame)
    if detected:
        save_snapshot(frame, m, bboxes)


# Detector tunables (env-overridable for calibration day at the venue).
DETECT_MIN_AREA_PX  = int(os.environ.get("BH26_DETECT_MIN_AREA",  "400"))
DETECT_MAX_AREA_PX  = int(os.environ.get("BH26_DETECT_MAX_AREA",  "60000"))
DETECT_MIN_ASPECT   = float(os.environ.get("BH26_DETECT_MIN_ASPECT", "0.3"))
DETECT_MAX_ASPECT   = float(os.environ.get("BH26_DETECT_MAX_ASPECT", "3.0"))


def detect_robomaster(frame) -> tuple[bool, list]:
    """Return (detected, bboxes). Colour-based RoboMaster armour-plate detector.

    HSV thresholding for red (RoboMaster armour plates are red/blue
    backlit panels). Two hue ranges because red wraps the hue cylinder.
    Filters contours by area + aspect ratio to reject noise.

    Tunable via BH26_DETECT_* env vars — calibrate on real footage at
    the venue. For competition day, also consider replacing this with
    a YOLOv8 model (Qualifier-trained pipeline can be reused), but the
    colour approach has no model file to manage.
    """
    if not CV2_AVAILABLE or frame is None:
        return (False, [])

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    # Red wraps around H=0; two windows to capture both sides.
    mask1 = cv2.inRange(hsv, (0,   120, 60), (10,  255, 255))
    mask2 = cv2.inRange(hsv, (170, 120, 60), (180, 255, 255))
    mask = cv2.bitwise_or(mask1, mask2)
    # Light cleanup so single-pixel speckles don't pass the area filter.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    bboxes: list[dict] = []
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < DETECT_MIN_AREA_PX or area > DETECT_MAX_AREA_PX:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if h == 0:
            continue
        aspect = w / float(h)
        if aspect < DETECT_MIN_ASPECT or aspect > DETECT_MAX_ASPECT:
            continue
        bboxes.append({
            "x": int(x), "y": int(y), "w": int(w), "h": int(h),
            "area": area, "aspect": round(aspect, 3),
        })
    return (len(bboxes) > 0, bboxes)


# How long after the last snapshot before we'll save another one.
# Prevents "100 snapshots of the same robot in 2 seconds" floods while
# still letting different robot appearances through.
SNAPSHOT_COOLDOWN_S = float(os.environ.get("BH26_SNAPSHOT_COOLDOWN_S", "2.0"))


def save_snapshot(frame, m: DroneMission, bboxes: list) -> None:
    """Persist the detected frame + bbox metadata to OUTPUT_DIR.

    Writes both an annotated JPEG and a JSON sidecar (one per snapshot).
    Cooldown-throttled to avoid flooding when a robot lingers in frame.
    """
    now = time.time()
    if now - m.last_snapshot_at < SNAPSHOT_COOLDOWN_S:
        return

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    m.snapshots_saved += 1
    m.last_snapshot_at = now
    ts = int(now * 1000)
    stem = OUTPUT_DIR / f"{m.plane_id}_{m.snapshots_saved:03d}_{ts}"

    if CV2_AVAILABLE:
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # Draw bboxes in green so reviewers can see what triggered the save.
        for b in bboxes:
            cv2.rectangle(bgr, (b["x"], b["y"]),
                          (b["x"] + b["w"], b["y"] + b["h"]),
                          (0, 255, 0), 2)
        cv2.imwrite(str(stem) + ".jpg", bgr)

    with open(str(stem) + ".json", "w") as f:
        json.dump({
            "plane_id": m.plane_id,
            "pad_id": m.pad.pad_id,
            "snapshot_num": m.snapshots_saved,
            "ts_ms": ts,
            "bboxes": bboxes,
        }, f, indent=2)
    print(f"[{m.plane_id}] snapshot {m.snapshots_saved} -> {stem}.jpg "
          f"({len(bboxes)} bbox)")


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
    args = parser.parse_args()

    if not args.pads:
        sys.exit("--pads (or BH26_PAD_FILE env var) is required")

    print(f"[main] backend={PYHULAX_BACKEND} cv2={'yes' if CV2_AVAILABLE else 'no'} "
          f"output_dir={OUTPUT_DIR} ambush_window_s={AMBUSH_WINDOW_S}")

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

    # 6. Summary
    print("\n=== MISSION SUMMARY ===")
    for m in missions:
        print(f"  {m.plane_id}: state={m.state.value} pad={m.pad.pad_id} "
              f"snapshots={m.snapshots_saved} err={m.last_error or '-'}")

    return 0 if all(m.state == DroneState.COMPLETE for m in missions) else 1


if __name__ == "__main__":
    sys.exit(main())
