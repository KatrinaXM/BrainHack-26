# RoboVerse Qualifier — Step-by-Step Runbook

> Execute this top-to-bottom. Every section ends with a **GATE** — a check that must pass before moving on. Don't skip gates; broken subsystems compound.
>
> Estimated time: ~10 hours spread across Days 1–2. Day 3 is integration. Day 4 is competition.

---

## Phase 0 — Environment setup (before Day 1)

### 0.1 Get the VMware image

1. Join the **RoboVerse Discord** server (link in `LearningMaterial1.pdf` p. 6).
2. Download the VMware virtual machine image they've prepared. It contains:
   - Ubuntu 22.04
   - PX4-Autopilot pre-built
   - MAVSDK-Python installed
   - Gazebo Harmonic
   - QGroundControl AppImage on the Desktop
   - OpenVINS + `vio-node` bridge (only matters for Final)
3. Install VMware Workstation Player (free) on your host machine.
4. Open the `.vmx` file. Allocate **at least 8 GB RAM and 4 CPU cores**; Gazebo is heavy.

> **Alternative**: native Ubuntu 22.04 install. Doable but ~6 hours of setup. **Not recommended given the 4-day timeline.**

### 0.2 First-boot checklist inside the VM

Open a terminal and run each line. Each must succeed.

```bash
# Verify Python and key libraries
python3 --version          # expect 3.10.x
python3 -c "import mavsdk; print(mavsdk.__version__)"
python3 -c "import cv2; print(cv2.__version__)"
python3 -c "from gz.transport13 import Node; print('gz-transport OK')"
python3 -c "from ultralytics import YOLO; print('YOLO OK')"

# Verify PX4 source
ls ~/PX4-Autopilot/Tools/simulation/gz/worlds/  # should list .sdf files including roboverse.sdf

# Verify reference code path
ls ~/Desktop/codes 2>/dev/null || echo "Copy reference codes to ~/Desktop/codes"
```

### 0.3 Copy the reference code into the VM

From your host: copy `references/codes/` and `references/start_px4.sh` into the VM at `~/Desktop/codes/` and `~/start_px4.sh` respectively. Use VMware Shared Folders, USB stick, or `scp` — whichever your VM has set up.

```bash
chmod +x ~/start_px4.sh
chmod +x ~/Desktop/codes/*.py
```

### 0.4 GATE 0

- [ ] All imports above succeed without `ModuleNotFoundError`.
- [ ] `~/start_px4.sh` exists and is executable.
- [ ] `~/Desktop/codes/avoid_with_detect.py` exists.

Do not proceed until all three boxes are ticked.

---

## Phase 1 — Subsystem verification (Day 1, ~4 hours)

Run each test in order. **One terminal per command** (PX4 needs to keep running while you test other things — open new terminal tabs).

### 1.1 Boot the simulator

**Terminal 1:**
```bash
cd ~
./start_px4.sh
# Choose: 2 (x500_depth)
# Choose: 1 (roboverse)
# Choose: 1 (Yes, start QGC)
```

Wait ~30 seconds. You should see:
- Gazebo window opens showing the space-port world.
- QGroundControl opens and connects (top bar shows the vehicle).
- PX4 console prints heartbeats and ends in a `pxh>` prompt.

**GATE 1.1**: Drone visible in Gazebo on the launchpad? PX4 console at `pxh>` prompt? QGC shows the vehicle? ✅

### 1.2 Set EKF origin (required for `x500_depth` / `x500_vision`)

In the PX4 console (the terminal that's at the `pxh>` prompt):

```
commander set_ekf_origin 47.397742 8.545594 488.0
```

You'll see `[commander] EKF set origin successful`. This tells PX4 it has a valid "home" and is OK to arm.

Alternative via QGroundControl: click on the map anywhere → "Set Estimator Origin".

**GATE 1.2**: PX4 console shows `EKF set origin successful`. ✅

### 1.3 Verify MAVSDK connection — simplest possible test

**Terminal 2:**
```bash
cd ~/Desktop/codes
python3 takeoff_and_land.py
```

Expected behaviour:
- Console prints "Connected".
- Drone arms, takes off ~2 m, hovers, lands.
- Total ~30 seconds.

**Failure modes:**
- `Connection refused` → PX4 isn't running. Re-run §1.1.
- Arms but doesn't take off → EKF origin not set. Re-run §1.2.
- Drone flips or crashes → reset world (`reset` in PX4 console, then re-run takeoff).

**GATE 1.3**: Drone takes off, hovers, lands cleanly. ✅

### 1.4 Verify telemetry stream

**Terminal 2 (after gate 1.3):**
```bash
python3 get_position_with_task.py
```

You should see continuous prints of `(north, east, down, yaw)`. Move the drone manually via QGC (click somewhere on the map → "Go to location") and watch the values change.

**GATE 1.4**: Position values update when drone moves. North/East change when drone moves; down is negative when in air. ✅

### 1.5 Verify depth camera stream

**Terminal 3:**
```bash
cd ~/Desktop/codes
python3 depthtest.py
```

A grayscale OpenCV window should open showing the depth view. White = far, black = close. Closer objects darker.

If the window opens but is all-black or all-white:
- Check the topic name in `depthtest.py`. Discover the real one:
  ```bash
  gz topic -l | grep -i depth
  ```
- Replace the `/depth_camera` string in `depthtest.py` if the real topic differs.

**GATE 1.5**: Live depth image visible, changes as you fly the drone around. ✅

### 1.6 Verify RGB camera stream

**Terminal 3:**
```bash
python3 get_video.py
```

A colour OpenCV window should show the drone's forward view.

**Likely fix needed**: the topic in `get_video.py` is hardcoded to:
```
/world/roboverse/model/x500_mono_cam_0/link/camera_link/sensor/camera/image
```
But you're flying `x500_depth`, not `x500_mono_cam`. Discover the right topic:
```bash
gz topic -l | grep image
```
Edit `get_video.py` and replace the topic string with the correct one. Save the working topic string — you'll need it everywhere.

**GATE 1.6**: Colour image visible, updates live. **Note down the working RGB and depth topic names.** ✅

### 1.7 Verify YOLO detection (stock model)

**Terminal 3:**
```bash
python3 UseDetectorExample.py
```

You should see the camera feed annotated with YOLO bounding boxes for whatever generic objects it recognises (likely **none** for fuel barrels — the stock model doesn't know them). Annotated images save to `./detections/`.

This test is just to confirm YOLO + the Detector class + the camera pipeline work end-to-end. **Detecting real barrels comes later**, with a custom-trained model.

**GATE 1.7**: YOLO inference runs without errors. Bounding boxes appear (even if for wrong objects). ✅

### 1.8 Verify reactive avoidance

In QGC, "Reset" the world to reset the drone's position. Then in PX4 console: `commander set_ekf_origin 47.397742 8.545594 488.0` again.

**Terminal 2:**
```bash
python3 avoid.py
```

The drone should arm, take off, and start moving forward while avoiding obstacles. Don't expect it to be elegant — it'll oscillate. Just watch for:
- No crashes.
- It reacts to walls (slows, turns, sidesteps).
- Console prints `Blocked: True/False` and clearance values.

**GATE 1.8**: Drone moves forward, reacts to walls, doesn't crash within 60 seconds. ✅

---

## Phase 2 — Subsystem confidence checks (Day 1 end / Day 2 start)

### 2.1 Capture training images (do this even if you don't retrain)

```bash
python3 save_photo.py
```

Manually fly the drone via QGC near each barrel in the world. Capture ~100 images covering:
- Yellow barrels from multiple angles, distances 1–8 m.
- Red barrels (they're elevated — fly up to ~4 m to see them).
- Various lighting / backgrounds.
- Some images **without** barrels (negative examples).

Save these — even if you skip retraining for now, you'll want them on Day 2.

### 2.2 (Optional, ~3 hours) Train a custom YOLO model

If you want to score reliably you need this. Steps:

1. Annotate the images. Use **Roboflow** (free, browser-based) — sign up, create a project, upload your captured images, label each barrel with `fuel_barrel_yellow` or `fuel_barrel_red`. Export as **YOLOv8 format**.
2. Open `Train_YOLO_Models_new.ipynb` in Google Colab. Upload your dataset. Run all cells.
3. Download `runs/detect/train/weights/best.pt`. Rename to `barrel_yolo.pt`.
4. Copy into `~/Desktop/codes/`.

### 2.3 GATE 2

- [ ] You have a list of working topic names (depth + RGB).
- [ ] `avoid.py` runs for 60 s without crashing.
- [ ] You have ≥ 100 training images saved (whether or not you've trained yet).

---

## Phase 3 — Integration (Day 2 afternoon / Day 3)

**Tell me when you've cleared all of Phase 1 and 2 gates.** I'll then write `mission.py` with your specific topic names, hooked into:
- Background telemetry task
- Background depth subscription
- Background RGB → YOLO submission
- `BarrelTracker` (NED-space deduplication, 2 m matching radius)
- `Lawnmower` exploration policy
- 8-minute bail-out timer
- Graceful RTL + land

You'll then iterate by tuning parameters in `mission_config.py`:

```python
# Defaults to start from
CRITICAL_DISTANCE = 1.0   # m — below this = "danger" cost
SAFE_DISTANCE = 3.0       # m — above this = "free path"
LOOK_AHEAD = 1.5          # m — how far ahead to project the next setpoint
SWEEP_SPACING = 4.0       # m — distance between lawnmower lanes (matches grid)
SWEEP_ALTITUDE = 1.5      # m
ALTITUDE_PASS_HEIGHT = 4.0  # m — for the red-barrel altitude pass
LOOP_HZ = 20.0            # how often the mission loop runs
YAW_TOLERANCE_DEG = 5.0   # when rotating, consider "aligned" within this
DETECTION_CONF = 0.5      # YOLO confidence threshold
DEDUP_RADIUS = 2.0        # m — barrels closer than this in NED considered same
BAILOUT_SECONDS = 480     # 8 minutes
```

### Tuning ladder (in order)

1. **Avoidance not reactive enough → drone clips walls**: lower `SAFE_DISTANCE` to 2.0, or raise `CRITICAL_DISTANCE` to 1.5.
2. **Drone oscillates / wiggles**: raise `alpha` (velocity smoothing) in `VelocityPlanner.py` from 0.7 → 0.85.
3. **Drone covers ground too slowly**: raise loop rate to 30 Hz, lower `LOOP_DT`.
4. **Misses barrels visible in camera**: lower `DETECTION_CONF` to 0.35.
5. **Counts same barrel twice**: raise `DEDUP_RADIUS` to 3.0.
6. **Runs out of time before reaching far corners**: raise base velocity in `VelocityPlanner.py`; reduce `SWEEP_SPACING` only if you have time to spare.

---

## Phase 4 — Map-reveal day (21 May, evening)

The actual map drops ~24 h before your run. When you get it:

1. Load the `.sdf` file into PX4's worlds folder.
2. Run a 3-minute exploration with `mission.py` and watch where it gets stuck.
3. Add **hardcoded waypoints** in `exploration.py` to push the drone through narrow passages it can't reason its way out of (e.g. "first go to (10, 5, -1.5) before starting the sweep" to bypass a known dead-end).

Don't redesign the whole pipeline. Patch around the specific layout.

---

## Phase 5 — Competition day (22 or 23 May)

### 5.1 USB stick contents (prepare night before)

```
/competition_drive
├── codes/
│   ├── mission.py
│   ├── mission_config.py
│   ├── barrel_tracker.py
│   ├── exploration.py
│   ├── (all reference modules: drone_control_new.py, depth_receiver.py, etc.)
│   └── barrel_yolo.pt
├── start_px4.sh
├── requirements.txt
└── RUN.txt              # one-page instructions for the judge's laptop
```

### 5.2 `RUN.txt` template

```
ROBOVERSE QUALIFIER — RUN INSTRUCTIONS

Terminal 1: cd ~ && ./start_px4.sh
            choose: 2 (x500_depth), 1 (roboverse), 2 (no QGC)
            wait for pxh> prompt
            type: commander set_ekf_origin 47.397742 8.545594 488.0
            press enter

Terminal 2: cd /path/to/usb/codes
            python3 mission.py

That's it. Mission runs autonomously for ~8 minutes,
then returns and lands.
```

### 5.3 Pre-run checklist (15-min setup window)

- [ ] USB plugged in, codes copied to laptop.
- [ ] `pip list | grep -i mavsdk` shows MAVSDK installed.
- [ ] Test run in a fresh terminal — kill it after takeoff confirmed.
- [ ] Reset Gazebo world.
- [ ] Re-set EKF origin.
- [ ] Hand-flying the drone in QGC works (sanity).
- [ ] Wait for judge's start signal.

### 5.4 During the run

- Do not touch keyboard/mouse on the drone — that's a DQ.
- You may watch logs. If `mission.py` crashes, the rules say the clock keeps running, so the rerun has to happen within the same 10-min window. Have a one-line restart command ready: `python3 mission.py`.

---

## Phase 6 — Troubleshooting reference

### Common failures and fixes

| Symptom | Likely cause | Fix |
|---|---|---|
| `Connection refused` to UDP 14540 | PX4 not running | Re-run `start_px4.sh` |
| Drone arms but won't take off | EKF not initialised | `commander set_ekf_origin ...` |
| Drone takes off, then immediately lands | Offboard mode lost (heartbeat dropped) | Check for `time.sleep()` in your code |
| Offboard refused with "no setpoint" | Did `offboard.start()` before any setpoint | Send a velocity/position setpoint first |
| Depth image all-zero | Wrong topic | `gz topic -l | grep depth` |
| Depth image NaN-heavy | Pointing at sky / too close to surface | Normal; AvoidancePlanner already handles |
| YOLO crashes on first frame | Model path wrong | Check `model_path=` argument |
| Drone drifts north slowly when commanded to hover | VIO drift | Acceptable; just don't trust position absolutely at minute 8+ |
| Drone yaws constantly | Yaw setpoint not held | Always include yaw in `set_position_ned()` |
| Position values look fine but drone flies wrong way | Coordinate frame mix-up | Body vs NED — re-read TUTORIAL.md §3 |

### When to ask the Discord coordinators

Open a `#support-ticket` for:
- VM image fails to boot or crashes
- PX4 SITL won't build
- Specific gz-transport topic names not appearing
- MAVSDK version mismatch errors

Don't burn tickets on parameter tuning — that's your job.

---

## What to send me to unblock Phase 3

When you've cleared all Phase 1 and Phase 2 gates, paste back:

1. The actual **depth topic** name that worked (e.g. `/depth_camera` or `/world/.../image`).
2. The actual **RGB topic** name that worked.
3. The actual **drone model** you settled on (`x500_depth` vs `x500_vision` vs `x500_mono_cam`).
4. The output of `python3 get_position_with_task.py` — first 5 lines — so I can confirm pose format.
5. Whether you've trained a custom YOLO model (and its file name if yes).

With those, I can write `mission.py` that **drops in correctly** on your machine, not generic boilerplate.

---

*End of runbook. Phase 1 alone (~4 hours) gets you from zero to "every subsystem confirmed working". That's the highest-leverage time investment in this whole project — every bug found here saves you 10× later.*
