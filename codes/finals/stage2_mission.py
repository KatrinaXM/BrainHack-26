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
PYHULAX_AVAILABLE = False
try:
    from pyhulax import DroneAPI            # type: ignore[import-untyped]
    from pyhulax.core import Direction      # type: ignore[import-untyped]
    from pyhulax.video import VideoStream   # type: ignore[import-untyped]
    from dola import Dola                   # type: ignore[import-untyped]
    PYHULAX_AVAILABLE = True
except ImportError:
    DroneAPI = Direction = VideoStream = Dola = None  # type: ignore[assignment]


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

def navigate_to_pad(drone, pad: Pad) -> None:
    """Fly the drone to the pad's (x, y, z).

    TODO (hardware day): pick one of three approaches and remove the stub.

    Option A — `drone.goto(x, y, z)` if pyhulax exposes it.
        Cleanest. Closed-source SDK — check pyhulax.xenops.ae before
        depending on it.

    Option B — decompose into body-frame `.move(Direction.X, distance)`
        steps. Requires knowing the drone's yaw at takeoff (all 3 HULAs
        launch from the same C2 pad facing the same direction, so a
        single calibration at startup is enough). Bench-tested distance
        unit (huladola.py comment claims metres but library is closed-
        source — verify with a tape measure).

    Option C — visual servo: climb to known altitude, search the camera
        feed for the assigned pad's ArUco / colour patch, descend onto it.
        Most robust to positioning errors but most code.

    For now this stub just sleeps to simulate flight time.
    """
    print(f"  [navigate_to_pad] stub — pretending to fly to "
          f"pad {pad.pad_id} ({pad.x:.2f}, {pad.y:.2f}, {pad.z:.2f})")
    time.sleep(2.0)


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


def detect_robomaster(frame) -> tuple[bool, list]:
    """Return (detected, bboxes). Skeleton returns False.

    Real impl options (pick one and unit-test against stock RoboMaster
    images before competition):

      1. YOLOv8 fine-tuned on RoboMaster images. ~5 MB model. Repurpose
         the Qualifier `barrel_yolo.pt` training pipeline.
      2. Colour-segment red/blue armour plates, contour-check shape.
         Faster, no model file, but fails under poor lighting.
      3. ArUco markers attached to RoboMasters (if rules allow it — check
         with organisers; PDF doesn't specify).
    """
    return (False, [])


def save_snapshot(frame, m: DroneMission, bboxes: list) -> None:
    """Persist the detected frame + bbox metadata to OUTPUT_DIR.

    TODO: add `cv2.imwrite()` call. Skipped here to keep the skeleton
    cv2-dependency-free until the detector is wired up.
    """
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    m.snapshots_saved += 1
    ts = int(time.time() * 1000)
    stem = OUTPUT_DIR / f"{m.plane_id}_{m.snapshots_saved:03d}_{ts}"
    # cv2.imwrite(str(stem) + ".jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    with open(str(stem) + ".json", "w") as f:
        json.dump({
            "plane_id": m.plane_id,
            "pad_id": m.pad.pad_id,
            "snapshot_num": m.snapshots_saved,
            "ts_ms": ts,
            "bboxes": bboxes,
        }, f, indent=2)
    print(f"[{m.plane_id}] snapshot {m.snapshots_saved} -> {stem}.jpg")


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
