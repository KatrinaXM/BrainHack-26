# BrainHack-26 RoboVerse Qualifier

Autonomous barrel-detection mission for the `x500_vision` drone in PX4 SITL +
Gazebo Harmonic. Built for the BrainHack-26 RoboVerse Qualifier 2026.

## Repo layout

```
.
├── codes/                # mission Python + launcher scripts
│   ├── mission.py            # main orchestrator (spin-at-spots strategy)
│   ├── mission_config.py     # all tunable parameters
│   ├── barrel_tracker.py     # NED projection + dedup + validation
│   ├── barrel_yolo.pt        # fine-tuned YOLOv10n (5.5 MB)  *** REQUIRED ***
│   ├── run_competition.sh    # ONE-COMMAND launcher: sim + QGC + EKF + mission
│   ├── install.sh            # one-shot setup / verify
│   ├── show_camera.py        # optional raw RGB viewer
│   ├── RUN.txt               # human-readable competition runbook
│   ├── JUDGE_SUMMARY.md      # one-page pitch for the judges
│   └── (Detector.py, AvoidancePlanner.py, drone_control.py, ... — reference modules)
├── scripts/              # simulator helpers
│   ├── start_sim.sh          # launches PX4 SITL + Gazebo (+ optional QGC) in screen
│   ├── stop_sim.sh           # tears everything down
│   ├── set_ekf_origin.py     # sends SET_GPS_GLOBAL_ORIGIN over MAVLink
│   └── set_ekf_origin.sh     # thin shell wrapper around the .py
├── sdf-patches/          # SDF replacements
│   └── OakD-Lite.model.sdf   # lightweight 640x480 @ 10 Hz RGB sensor
├── references/           # workshop reference code (untouched)
└── materials/            # competition PDFs
```

## Quick start on a fresh VM

Prerequisite: VMware image from the organiser with Ubuntu 22.04 + PX4 +
Gazebo Harmonic + MAVSDK pre-installed. Plus the trained YOLO weights
copied into `codes/barrel_yolo.pt`.

```bash
git clone <this-repo> ~/BrainHack-26
cd ~/BrainHack-26/codes

# One-time setup (verifies deps + patches the OakD-Lite SDF):
./install.sh

# Each attempt — single command, brings up sim + QGC + flies the mission:
./run_competition.sh
```

`run_competition.sh` opens an OpenCV "YOLO Detections" window showing live
annotated bounding boxes as the drone flies. **Ctrl-C cleans up everything.**

Outputs land in `/mnt/hgfs/Shared/` (the VM shared folder) or
`~/mission_output/` if no shared folder is mounted:

- `barrels.json` — detection list (raw + validated subsets)
- `yolo_detections/` — annotated JPEGs (one per detection event)

## Environment overrides

```bash
PX4_DIR=/path/to/PX4-Autopilot      ./run_competition.sh
QGC_PATH=/path/to/QGroundControl    ./run_competition.sh
BH26_STRATEGY=high_alt               ./run_competition.sh  # experimental
BH26_DISPLAY=0                       ./run_competition.sh  # headless mode
```

## What the mission does

1. Take off to ~1.2 m altitude.
2. Spin 360° in place (~15 s) so the depth camera + YOLO see all directions.
3. Pick the heading with best clearance + bias away from previous transits.
4. Sprint forward up to 12 m via bounded position setpoints.
5. Repeat up to 12 times or until 280 s elapse.
6. Return-to-launch + land.

YOLO detections are projected to NED via pixel + depth + drone pose, then
deduplicated (4 m radius) and validated (hits ≥ 2 OR conf ≥ 0.75).

Multi-attempt is encouraged — the rules let teams re-run within 10 min and
score the best attempt. Just run `./run_competition.sh` again after Ctrl-C.

## Compliance notes

- **Drone model:** `x500_vision` (organiser Discord ruling — "has to be vision drone").
- **No GPS check:** code gates arming on `is_armable` and `is_home_position_ok`,
  never on `is_global_position_ok` (always False with EV fusion).
- **EKF origin:** set explicitly via MAVLink `SET_GPS_GLOBAL_ORIGIN` (see
  `scripts/set_ekf_origin.py`). Slide 14 of LearningMaterial2 confirms this
  is required for the vision drone to become armable.
- **All camera processing in background:** depth + RGB run on dedicated
  gz-transport threads; YOLO on a worker pool. The main asyncio loop never
  blocks on image I/O (organiser's explicit guidance).
- **No manual control:** entire run is autonomous via `mission.py`.

## Known limitations

The `gz-sim-odometry-publisher` plugin on `x500_vision` feeds PX4's EKF a
noisy ground-truth pose stream that the estimator cannot fully reject.
Horizontal drift up to ~10 m can accumulate in a 280-s flight. We
compensate via the multi-attempt rule and conservative spin-then-sprint
behaviour. See `codes/JUDGE_SUMMARY.md` for background.
