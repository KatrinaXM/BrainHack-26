# BrainHack-26 RoboVerse Qualifier — One-Page Summary

## What we built

A fully autonomous barrel-search mission for the GNSS-denied `x500_vision`
drone in the Roboverse Gazebo SITL. The code:

- Connects to PX4 via MAVSDK, takes off, and enters offboard mode.
- Subscribes to the depth camera (`/depth_camera`) and the RGB camera
  (`/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image`).
- Runs a custom-trained YOLOv10n model (`barrel_yolo.pt`, mAP50 = 0.995)
  on every RGB frame in a background worker pool.
- Visits up to 12 sequential "spots" within the world. At each spot the
  drone rotates 360° in place (~15 s) so YOLO and depth see every
  direction, then sprints up to 12 m forward to the next spot.
- Projects every YOLO bounding-box centre into NED world coordinates via
  the depth pixel and the drone's pose. Detections within 4 m of an
  already-known barrel (same class) are merged.
- Validates detections: a barrel only counts if it has been seen ≥ 2
  times OR its peak confidence is ≥ 0.75.
- Bails at 280 s and returns to land, well within the 5-min time bonus
  cliff. Multiple attempts are expected within the 10-minute window.

## Live evidence

A live OpenCV window ("YOLO Detections") opens on launch and shows every
annotated detection in real-time with the bounding box, class name and
confidence. Annotated frames are also saved to disk as a persistent
backup trail in `/mnt/hgfs/Shared/yolo_detections/`.

## Output

`/mnt/hgfs/Shared/barrels.json` contains two lists:

- `barrels` — raw deduplicated detections (may include low-confidence noise).
- `validated` — entries meeting the validation filter. **This is the list
  judges should count.**

Schema fields per entry: `class_name`, `north`, `east`, `down`,
`confidence`, `hits`, `first_seen_t`, `last_seen_t`, `bboxes`,
`image_paths` (links back to the saved annotated frames).

## Compliance

- **Drone model:** `x500_vision` (per organiser Discord ruling).
- **GPS check:** code gates arming on `is_armable` and `is_home_position_ok`,
  never on `is_global_position_ok` (which is permanently False with EV
  fusion).
- **EKF origin:** set explicitly via `commander set_ekf_origin
  47.397742 8.545594 488.0` (sent over MAVLink at startup; see
  `set_ekf_origin.py`).
- **No manual control:** the run is fully autonomous from `mission.py`
  start to land. Keyboard/joystick are never used.
- **All camera processing in background:** depth and RGB receivers run on
  dedicated gz-transport threads; YOLO inference runs on a 2-worker
  thread pool; the main asyncio loop never blocks on image I/O.

## Architecture (modules in `~/Desktop/codes/`)

| File | Role |
|---|---|
| `mission.py` | Main orchestrator + spin-at-spots strategy |
| `mission_config.py` | All tunable parameters (one place to retune) |
| `barrel_tracker.py` | NED projection + dedup + validation filter |
| `Detector.py` | YOLO inference worker pool + live display + image save |
| `depth_receiver.py` | Depth camera gz-transport subscriber |
| `AvoidancePlanner.py` | Depth → clearance / blocked detection |
| `drone_control.py` | MAVSDK wrapper (arm, takeoff, offboard, send_*) |
| `barrel_yolo.pt` | Fine-tuned YOLOv10n weights (5.5 MB) |

## Honest limitations

- The "VIO" in `x500_vision` SITL is a noisy ground-truth pose feed from
  the Gazebo odometry plugin; PX4's EKF can drift up to ~10 m during a
  280-s flight. This caps how far the drone can effectively navigate
  away from spawn.
- We compensate with the multi-attempt rule — running the mission 2-3
  times within the 10-minute window increases the chance of detecting
  barrels in different physical regions.
- Reported NED positions are *consistent for deduplication* but offset
  from absolute world coordinates by the accumulated EKF drift.
