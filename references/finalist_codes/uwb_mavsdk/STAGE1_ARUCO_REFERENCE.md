# Reference — University Stage-1 UWB/MAVSDK/RealSense ArUco mission

Notes on the teammate's **Stage-1** offboard mission (MAVSDK + PX4 + UWB +
RealSense + ArUco + top-down BEV occupancy mapping). That stack is **not** what
Pre-U Stage 2 uses (we use pyhulax, not MAVSDK/PX4/UWB), so we do **not** port it
wholesale. But several parts confirm or inform our `codes/finals/` code:

## Directly useful / confirming

- **ArUco dictionary matches ours.** It builds the detector with
  `cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)` +
  `cv2.aruco.ArucoDetector(...)` — identical to `stage2_mission.py`'s
  `DICT_7X7_1000` default. Independent confirmation the dictionary is right.
- **Per-marker ID output is the goal.** Its `detect_aruco_once(...)` decodes and
  reports marker IDs each frame — same contract our `detect_aruco_markers()`
  fulfils (returns `{"id": int, ...}` per marker; we log/save every ID).

## Ideas worth borrowing (patterns, not code)

- **Waypoint array + per-waypoint acceptance radius.** It loops a list of
  `(north, east)` targets and accepts arrival within `HORIZONTAL_TOLERANCE`
  (0.20 m). Our pad list is the analogue; `NAV_EPSILON_M` is our tolerance.
- **Proportional deceleration near the target** (`SLOW_DOWN_RADIUS`, MIN/MAX
  speed interpolation) for smooth stops. pyhulax handles its own motion profile,
  so we express "be gentle" via `MOVE_SPEED_LEVEL=300` (SLOW) instead — but the
  intent is the same as the brief's "max 0.5 m/s, be conservative".

## Deliberately NOT adopted (different stack)

- MAVSDK `System`, `offboard.set_position_velocity_ned`, PX4 serial link.
- UWB absolute positioning (`UWBPositionQuerier`) — Pre-U HULAs self-position by
  optical flow relative to their own takeoff point; there is **no UWB and no
  absolute arena coordinate** reported to us (see `STATUS.md` §3 / discovery note).
- RealSense depth + BEV occupancy mapping (`generateTopDownTD`) — University
  Stage-1 mapping task, not Pre-U Stage 2.

**Takeaway:** the ArUco config and "report every ID" contract are validated; the
positioning approach is fundamentally different (UWB vs optical-flow), which is
exactly why our drones' arena coordinates can't be auto-discovered.
