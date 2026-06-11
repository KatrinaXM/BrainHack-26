#!/usr/bin/env python3
"""
pyhulax_mock.py — drop-in mock for pyhulax + dola, for offline test of
the BrainHack-26 Stage 2 orchestrator.

Mimics the API surface documented in
`references/finalist_codes/hula_swarm/huladola.py` and TUTORIAL.md Ch 24:

    from pyhulax import DroneAPI
    from pyhulax.core import Direction
    from pyhulax.video import VideoStream
    from dola import Dola

Activated by setting `BH26_MOCK=1` before running `stage2_mission.py`.
The orchestrator detects the env var and swaps these names in.

Faithful behaviours
-------------------
- `DroneAPI` enforces a state machine (DISCONNECTED -> ON_GROUND ->
  TAKING_OFF -> HOVERING -> MOVING -> HOVERING -> LANDING -> LANDED).
  Calling `move()` before `takeoff()` or `takeoff()` twice raises, so
  orchestrator bugs surface immediately.
- All blocking calls (`takeoff`, `move`, `land`) sleep proportionally
  to a fake speed model, so the supervisor loop and threading model are
  exercised realistically.
- `VideoStream` runs a background thread that produces 15 Hz synthetic
  frames; periodically a real ArUco marker (rendered with cv2.aruco) is
  painted into the frame for ~3 s so the ArUco detector can fire and
  validate the snapshot pipeline end-to-end.
- `Dola.get_all_ips()` honours its `listen_seconds` arg (capped at 1 s
  for test speed) and returns 3 deterministic drone IDs by default.

Tunables (env vars)
-------------------
    BH26_MOCK_DRONES       number of drones Dola discovers (default 3)
    BH26_MOCK_TAKEOFF_S    seconds takeoff blocks for (default 2.0)
    BH26_MOCK_LAND_S       seconds land blocks for     (default 1.5)
    BH26_MOCK_SPEED_MPS    fake horizontal speed       (default 0.5, matches brief HULA cap)
    BH26_MOCK_ROBO_FIRST   first ArUco-marker appearance delay (s, default 4)
    BH26_MOCK_ROBO_PERIOD  mean seconds between appearances (default 12)
    BH26_MOCK_MARKER_IDS   comma-separated ArUco IDs to cycle (default 1,2,3,4,5)

Intentional limitations
-----------------------
- No simulated drift, no failures, no comm dropout. If those matter for
  resilience testing, layer them on by wrapping the mock.
- `move()` does not track absolute position. There is no "goto" because
  the real pyhulax navigation API isn't documented here yet. The real
  navigate_to_pad implementation goes into stage2_mission.py; this mock
  just makes its calls return on a realistic timeline.
"""

from __future__ import annotations

import enum
import os
import random
import threading
import time

import numpy as np


# ============================================================================
#  Direction enum  (mirrors pyhulax.core.Direction)
# ============================================================================

class Direction(enum.Enum):
    FORWARD = "forward"
    BACK = "back"
    LEFT = "left"
    RIGHT = "right"
    UP = "up"
    DOWN = "down"


# ============================================================================
#  Frame  (mirrors the object returned by VideoStream.latest_frame)
# ============================================================================

class _Frame:
    """Wraps a numpy RGB array; mimics pyhulax's frame object with .to_rgb()."""

    def __init__(self, rgb: np.ndarray):
        self._rgb = rgb

    def to_rgb(self) -> np.ndarray:
        return self._rgb


# ============================================================================
#  VideoStream  (mirrors pyhulax.video.VideoStream)
# ============================================================================

class VideoStream:
    """Generates synthetic frames at 15 Hz with periodic ArUco markers
    drawn into them so the orchestrator's ArUco detector can fire.

    Real ArUco markers (cv2.aruco.generateImageMarker) are used so the
    detection path exercises actual cv2.aruco code, not a stub.

    Mock ArUco IDs are deterministic per drone (so test runs replay),
    cycling through the drone's marker pool (env BH26_MOCK_MARKER_IDS).
    """

    FRAME_HZ = 15.0
    WIDTH = 640
    HEIGHT = 480

    def __init__(self, drone_id: str):
        self._drone_id = drone_id
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest: _Frame | None = None
        # Deterministic per-drone RNG so test runs are repeatable.
        self._rng = random.Random(hash(drone_id) & 0xFFFFFFFF)

        # Marker pool to cycle through. Comma-separated env var, e.g. "1,2,3".
        # Default = the real IDs the organizers said appear on the ground
        # robots (Discord, 2026-06-10) so the mock exercises the real values.
        pool_str = os.environ.get("BH26_MOCK_MARKER_IDS", "11,45,51,67,101")
        self._marker_pool = [int(s) for s in pool_str.split(",") if s.strip()]
        self._marker_idx = 0

        # Appearance scheduling — same names kept for back-compat with launcher.
        first_delay = float(os.environ.get("BH26_MOCK_ROBO_FIRST", "4.0"))
        self._marker_period_mean = float(
            os.environ.get("BH26_MOCK_ROBO_PERIOD", "12.0"))
        self._marker_visible_until: float = 0.0
        self._next_marker_at: float = time.time() + first_delay
        # Cache for the current visible marker's pre-rendered patch + position.
        self._current_marker_id: int | None = None
        self._current_patch: np.ndarray | None = None
        self._current_xy: tuple[int, int] = (0, 0)

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, name=f"mock-video-{self._drone_id}", daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    @property
    def latest_frame(self) -> _Frame | None:
        with self._lock:
            return self._latest

    # ----- internal -----

    def _loop(self) -> None:
        period = 1.0 / self.FRAME_HZ
        while self._running:
            frame = self._generate_frame()
            with self._lock:
                self._latest = _Frame(frame)
            time.sleep(period)

    def _make_marker_patch(self, marker_id: int, size: int = 160) -> np.ndarray:
        """Render an ArUco marker as an RGB patch using cv2.aruco.

        Uses the same dictionary the detector expects (BH26_ARUCO_DICT,
        default DICT_7X7_1000) so the mock and the real detection path stay
        in sync even if the dictionary is overridden. Import cv2 lazily so
        the mock stays importable on cv2-less envs; a missing cv2 makes
        _generate_frame silently skip the marker paint.
        """
        import cv2  # local import — keeps module-level import minimal
        dict_name = os.environ.get("BH26_ARUCO_DICT", "DICT_7X7_1000")
        dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_7X7_1000)
        d = cv2.aruco.getPredefinedDictionary(dict_id)
        gray = cv2.aruco.generateImageMarker(d, marker_id, size)
        # Convert to 3-channel RGB so we can paste into the frame buffer.
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

    def _generate_frame(self) -> np.ndarray:
        # Mid-grey background, slightly textured to look camera-ish.
        rgb = np.full((self.HEIGHT, self.WIDTH, 3), 96, dtype=np.uint8)
        now = time.time()

        # Schedule next marker appearance.
        if (now >= self._next_marker_at and
                now >= self._marker_visible_until and
                self._marker_pool):
            marker_id = self._marker_pool[self._marker_idx % len(self._marker_pool)]
            self._marker_idx += 1
            try:
                self._current_patch = self._make_marker_patch(marker_id, size=120)
                self._current_marker_id = marker_id
                cx = self._rng.randint(150, self.WIDTH - 150)
                cy = self._rng.randint(120, self.HEIGHT - 120)
                self._current_xy = (cx, cy)
            except Exception:
                # cv2 not available — silently skip
                self._current_patch = None
            self._marker_visible_until = now + 3.0
            self._next_marker_at = now + self._rng.uniform(
                0.5 * self._marker_period_mean,
                1.5 * self._marker_period_mean,
            )

        # If a marker should be visible, paint its rendered patch in.
        if now < self._marker_visible_until and self._current_patch is not None:
            patch = self._current_patch
            ph, pw = patch.shape[:2]
            cx, cy = self._current_xy
            x0, y0 = max(0, cx - pw // 2), max(0, cy - ph // 2)
            x1, y1 = min(self.WIDTH, x0 + pw), min(self.HEIGHT, y0 + ph)
            rgb[y0:y1, x0:x1] = patch[:y1 - y0, :x1 - x0]
        return rgb


# ============================================================================
#  DroneAPI  (mirrors pyhulax.DroneAPI)
# ============================================================================

class _Vector3:
    """Minimal stand-in for pyhulax's Vector3 (cm). Has .x/.y/.z."""

    def __init__(self, x: float, y: float, z: float):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def __repr__(self) -> str:
        return f"Vector3(x={self.x:.1f}, y={self.y:.1f}, z={self.z:.1f})"


class DroneAPI:
    """Mock pyhulax DroneAPI with a strict per-drone state machine.

    Blocking methods (`takeoff`, `move`, `land`) sleep on a fake-speed
    timeline; concurrent calls on the same drone are rejected, so the
    orchestrator's threading model is exercised realistically."""

    class _State(enum.Enum):
        DISCONNECTED = 0
        ON_GROUND = 1
        TAKING_OFF = 2
        HOVERING = 3
        MOVING = 4
        LANDING = 5
        LANDED = 6

    # Cache env-var lookups at class load.
    _TAKEOFF_S = float(os.environ.get("BH26_MOCK_TAKEOFF_S", "2.0"))
    _LAND_S = float(os.environ.get("BH26_MOCK_LAND_S", "1.5"))
    _SPEED_MPS = float(os.environ.get("BH26_MOCK_SPEED_MPS", "0.5"))

    _CM_PER_M = 100.0   # real pyhulax distances are centimetres

    def __init__(self) -> None:
        self._state = self._State.DISCONNECTED
        self._ip: str | None = None
        self._video: VideoStream | None = None
        self._video_enabled = False
        self._qr_localization = False
        self._barrier_mode = False
        # Tracked position in DRONE FRAME, centimetres: [forward, right, up]
        # relative to the takeoff origin. Updated by takeoff/move/move_to so
        # get_position() is consistent with the commands issued.
        self._pos = [0.0, 0.0, 0.0]
        self._battery = int(os.environ.get("BH26_MOCK_BATTERY", "100"))
        self._lock = threading.Lock()

    # ----- connection -----

    def connect(self, ip: str | None = None, timeout: float = 5.0) -> None:
        time.sleep(0.05)
        with self._lock:
            self._ip = str(ip)
            self._state = self._State.ON_GROUND

    def is_connected(self) -> bool:
        with self._lock:
            return self._state != self._State.DISCONNECTED

    def set_qr_localization(self, enabled: bool) -> None:
        # Real pyhulax: switches move_to between absolute (mat) and
        # takeoff-relative coordinates. The mock just records the flag.
        self._qr_localization = bool(enabled)

    def set_barrier_mode(self, enabled: bool):
        # Real pyhulax: enable/disable firmware obstacle avoidance.
        self._barrier_mode = bool(enabled)

    def set_camera_angle(self, mode, angle: int = 0):
        # Real pyhulax: tilt the camera. Mock records the request.
        self._camera_angle = (mode, angle)

    def rotate(self, angle_degrees, led=None, blocking: bool = True):
        # Real pyhulax: yaw rotate in place. Mock sleeps proportionally and
        # records cumulative yaw so the search yaw-scan is exercised.
        time.sleep(min(abs(float(angle_degrees)) / 180.0, 0.2))
        with self._lock:
            self._yaw_deg = getattr(self, "_yaw_deg", 0.0) + float(angle_degrees)

    # ----- telemetry -----

    def get_battery(self) -> int:
        return self._battery

    def get_position(self) -> "_Vector3":
        with self._lock:
            return _Vector3(self._pos[0], self._pos[1], self._pos[2])

    # ----- video -----

    def create_video_stream(self) -> VideoStream:
        if self._video is None:
            self._video = VideoStream(self._ip or "unknown")
        return self._video

    def set_video_stream(self, enabled: bool) -> None:
        self._video_enabled = bool(enabled)

    # ----- flight -----

    def takeoff(self, height_cm: int = 100, led=None,
                blocking: bool = True, flags=0) -> None:
        # Signature mirrors the real DroneAPI.takeoff(height_cm=100, ...).
        # Re-takeoff after landing is allowed — the Stage 2 mission flow
        # requires it (land on pad for scoring, then take off again to
        # search). The real pyhulax behaves the same way.
        with self._lock:
            if self._state not in (self._State.ON_GROUND, self._State.LANDED):
                raise RuntimeError(
                    f"takeoff: invalid state {self._state.name} "
                    f"(must be ON_GROUND or LANDED)"
                )
            self._state = self._State.TAKING_OFF
        time.sleep(self._TAKEOFF_S)
        with self._lock:
            self._pos[2] = float(height_cm)   # now at hover altitude
            self._state = self._State.HOVERING

    _DELTA = {
        "FORWARD": (0, +1), "BACK": (0, -1),
        "RIGHT": (1, +1), "LEFT": (1, -1),
        "UP": (2, +1), "DOWN": (2, -1),
    }

    def move(self, direction: Direction, distance_cm: float, led=None,
             blocking: bool = True, speed=None) -> None:
        # Signature mirrors real DroneAPI.move(direction, distance_cm, ...).
        # distance_cm is CENTIMETRES (per the official reference); convert to
        # metres for the fake-speed timing model. Type kept loose — real
        # pyhulax takes a Direction enum, but tests sometimes pass strings.
        with self._lock:
            if self._state != self._State.HOVERING:
                raise RuntimeError(
                    f"move: invalid state {self._state.name} (must be HOVERING)"
                )
            self._state = self._State.MOVING
        dt = (abs(float(distance_cm)) / self._CM_PER_M) / max(self._SPEED_MPS, 0.01)
        time.sleep(dt)
        with self._lock:
            # Track position so get_position() is consistent with the command.
            name = getattr(direction, "name", str(direction)).upper()
            if name in self._DELTA:
                axis, sign = self._DELTA[name]
                self._pos[axis] += sign * abs(float(distance_cm))
            self._state = self._State.HOVERING

    def move_to(self, x: float, y: float, z: float, led=None,
                blocking: bool = True, speed=None) -> None:
        # Mirrors real DroneAPI.move_to(x, y, z, ...) — absolute (QR on) or
        # takeoff-relative (QR off) straight-line flight. x/y/z are cm.
        with self._lock:
            if self._state != self._State.HOVERING:
                raise RuntimeError(
                    f"move_to: invalid state {self._state.name} (must be HOVERING)"
                )
            self._state = self._State.MOVING
        dist_m = ((float(x) ** 2 + float(y) ** 2 + float(z) ** 2) ** 0.5) / self._CM_PER_M
        time.sleep(dist_m / max(self._SPEED_MPS, 0.01))
        with self._lock:
            self._pos = [float(x), float(y), float(z)]
            self._state = self._State.HOVERING

    def hover(self, duration_seconds: float, led=None, blocking: bool = True) -> None:
        # Real pyhulax: hold position for the given time. Mock just sleeps
        # briefly and keeps HOVERING (used by the fail-safe stop path).
        time.sleep(min(float(duration_seconds), 0.2))
        with self._lock:
            if self._state == self._State.MOVING:
                self._state = self._State.HOVERING

    def land(self) -> None:
        with self._lock:
            if self._state in (self._State.LANDING, self._State.LANDED):
                return    # idempotent
            if self._state == self._State.DISCONNECTED:
                raise RuntimeError("land: drone not connected")
            self._state = self._State.LANDING
        time.sleep(self._LAND_S)
        with self._lock:
            self._state = self._State.LANDED

    # ----- introspection (mock-only; real pyhulax doesn't have this) -----

    @property
    def _mock_state(self) -> str:
        with self._lock:
            return self._state.name


# ============================================================================
#  Dola  (mirrors dola.Dola)
# ============================================================================

class Dola:
    """Mock dola. Generates deterministic plane_id/ip pairs."""

    def __init__(self) -> None:
        self._started = False

    def start(self) -> None:
        self._started = True

    def stop(self) -> None:
        self._started = False

    def get_all_ips(self, listen_seconds: float = 5.0) -> dict[str, str]:
        # Honour listen_seconds but cap at 1.0 s for test speed.
        time.sleep(min(float(listen_seconds), 1.0))
        n = int(os.environ.get("BH26_MOCK_DRONES", "3"))
        return {f"plane{i+1}": f"192.168.1.{100 + i}" for i in range(n)}
