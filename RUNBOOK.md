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

*End of qualifier runbook. Phase 1 alone (~4 hours) gets you from zero to "every subsystem confirmed working". That's the highest-leverage time investment in this whole project — every bug found here saves you 10× later.*

---

# PART 2 — Finals Runbook

> Same structure as Part 1: phases top-to-bottom, each ending with a **GATE**. Skip nothing.
>
> **Estimated time:** ~30 hours total spread across the prep window. Plan ahead — every flight test needs battery charging and arena access.

## Phase 7 — Pre-arena prep (offline; do before you have hardware access)

You can do most of this on any Linux laptop while waiting for the venue.

### 7.1 Read Part 2 of TUTORIAL.md

Mandatory before touching code. The Finals vocabulary (UWB, ArUco, RKNN, pyhulax) doesn't appear in Part 1.

### 7.2 Read kolomee.py end-to-end

It's 406 lines and is the reference architecture for the Mapping Drone. Read it with `docs/kolomee_dissection.md` open next to you. By the end you should be able to explain:

- Why `rclpy.spin` runs in a daemon thread.
- Why the yaw is locked at takeoff and never changed.
- Why the P-controller uses different gains in `hover()` vs `fly_to_position_velocity()`.
- What the `for _ in range(20): send_velocity(0,0,0)` block does before `offboard.start()`.

### 7.3 Set up the model-conversion toolchain on your laptop

This step can be done with no drone present.

```bash
# Create a clean conda env (rknn-toolkit2 doesn't play nicely with system Python)
conda create -n rknn python=3.10 -y
conda activate rknn
pip install ultralytics onnx onnxruntime
pip install rknn-toolkit2 -i https://pypi.org/simple/
```

Then export a sample model:

```bash
yolo export model=yolov11n.pt format=onnx opset=12
python3 references/finalist_codes/model_convert/convertrknn.py     # produces yolov11n.rknn
```

If this builds without error you're ready to convert custom-trained models later.

### 7.4 GATE 7

- [ ] You can recite Part 2 §20 (UWB) and §22 (ArUco) without re-reading.
- [ ] `yolo export … format=onnx` succeeds on your laptop.
- [ ] `rknn-toolkit2` builds a `.rknn` from the ONNX without errors.
- [ ] You can answer "why does kolomee.py lock the yaw at takeoff?" without looking.

---

## Phase 8 — Mapping-drone first contact

This is the first hardware step. The drone arrives in a charged state from the venue.

### 8.1 Get into the mapping drone via NoMachine

1. Power the mapping drone (it brings up its own WiFi hotspot OR joins the venue's network — confirm with organisers).
2. Join that network from the C2 Terminal.
3. Open NoMachine, point it at the drone's IP (visible in the venue documentation).
4. Log in with the credentials provided.

You should see a full Ubuntu 22.04 desktop. **This is your dev environment for Stage 1 — write code here, not on the C2 laptop.**

### 8.2 Smoke-test ROS2 and UWB

In the mapping drone's terminal:

```bash
ros2 topic list                          # expect to see /uwb_tag and others
ros2 topic info /uwb_tag -v              # confirm message type is PoseStamped, note QoS
ros2 topic hz /uwb_tag                   # expect ~10 Hz
ros2 topic echo /uwb_tag --once          # expect a PoseStamped message
```

If `hz` shows 0.0 or `echo` blocks → UWB infrastructure isn't live; ping the organiser. If the topic exists but `echo` is silent → QoS mismatch; try the env variable trick from Tutorial §25.5.

### 8.3 Smoke-test RealSense

```bash
# Quick command-line frame grab
realsense-viewer       # graphical, may not work over NoMachine
# OR pure Python
python3 references/finalist_codes/realsense_cam/getRGB.py
```

Confirm a colour window opens with the drone's view. If `RuntimeError: No device connected`:

- Check `lsusb | grep -i intel` — RealSense should appear as Intel Corp. RealSense Camera.
- Check `lsusb -t` — confirm USB3 (not USB2) link. USB2 throttles to ~6 fps and breaks `align()`.
- Power-cycle the camera (unplug + replug). The first init after boot is sometimes flaky.

### 8.4 GATE 8

- [ ] NoMachine remote desktop works at acceptable lag.
- [ ] `ros2 topic hz /uwb_tag` shows ≥ 5 Hz.
- [ ] You see a live `getRGB.py` window with the drone's view.
- [ ] `pip show pyrealsense2` confirms version ≥ 2.50.

---

## Phase 9 — Mapping-drone navigation (kolomee-based)

Goal: take off, hover for 5 s, land — autonomously.

### 9.1 Copy kolomee.py into your working directory

```bash
mkdir -p ~/finals/codes
cp /path/to/repo/references/finalist_codes/uwb_mavsdk/kolomee.py ~/finals/codes/
```

Edit one line — replace the serial path with whatever the mapping drone uses:

```python
await drone.connect(system_address="serial:///dev/ttyS6:921600")  # ← check this matches the drone's wiring
```

(MAVSDK accepts `serial://`, `udp://`, or `tcp://`. The mapping drone is typically wired serial; verify with the organiser.)

### 9.2 Stand back, run it

```bash
cd ~/finals/codes
python3 kolomee.py
```

The script prompts `Do you want to proceed? (y/n):` before arming — that's intentional safety. Confirm visually clear, type `y`, watch.

Expected sequence:
1. UWB position log → confirms position broadcast working.
2. Arming → drone armed.
3. 20 zero-velocity setpoints sent (the "offboard warmup").
4. Offboard mode entered.
5. Drone takes off to 0.8 m.
6. Flies to waypoint 1 (1 m north).
7. Flies to waypoint 2 (1 m east-of-start).
8. Lands and disarms.

### 9.3 Common failure modes

- **`PreflightCheck` rejection** → flight controller refuses to arm. Causes: low battery (replace), poor GPS health flag (irrelevant indoors but PX4 may still expect it — check the airframe params), missing accelerometer calibration.
- **`Offboard not allowed`** → you forgot the 20 zero-velocity setpoints. They must be sent **before** `offboard.start()`.
- **Drone takes off but drifts** → UWB swap (Tutorial §20.2). Walk the drone manually north; verify `current_n` increases, not `current_e`. If swapped, fix the `n = pose.position.y` / `e = pose.position.x` lines.
- **Drone overshoots and oscillates** → `KP_XY` too high or UWB noisy. Lower to 0.07, raise `MAX_VEL_XY` ceiling, slow loop to 5 Hz.

### 9.4 GATE 9

- [ ] One full takeoff-→-2-waypoint-→-land cycle without manual override.
- [ ] Walked-test confirms UWB north/east axes are correctly oriented.
- [ ] You can answer: "What happens if UWB drops for 2 s mid-flight?" (answer: kolomee logs "UWB data not ready, cannot navigate" and sends `(0,0,0)`).

---

## Phase 10 — ArUco detection bench

Goal: detect a printed ArUco marker from the drone's camera, on the bench (not in flight).

### 10.1 Print test markers

On a colour printer, generate 3-4 markers from the `DICT_6X6_250` dictionary:

```python
import cv2
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
for marker_id in [7, 12, 42, 99]:
    img = cv2.aruco.generateImageMarker(arucoDict, marker_id, 600)
    cv2.imwrite(f"marker_{marker_id}.png", img)
```

Print each at **~10 cm side length** on white paper. Stick them to a wall or stand.

### 10.2 Run the detection script on the drone

Adapt the snippet from `references/finalist_codes/aruco_detection/aruco_detection.py` (clean up the indentation first — the published version has a bug on lines 11-12). Use the RealSense pipeline:

```python
# Full skeleton in TUTORIAL.md §22.2
```

Hold the drone (powered off) and point the camera at each marker. Confirm:
- ID is decoded correctly.
- Detection works at 0.5 m, 2 m, 4 m distance.
- (X, Y, Z) values from depth back-projection look sensible.

### 10.3 GATE 10

- [ ] At least 4 different marker IDs detect cleanly at ≥2 m.
- [ ] Detection survives gentle motion (hand-held panning).
- [ ] You know your "valid vs invalid" ID scheme (asking the organiser if needed).

---

## Phase 11 — RKNN smoke test on the drone

Goal: run a YOLO model on the NPU at the documented ~50 fps.

### 11.1 Get a `.rknn` file onto the drone

```bash
# On your laptop (with rknn-toolkit2 environment activated):
python3 references/finalist_codes/model_convert/convertyolotoonnx.py
python3 references/finalist_codes/model_convert/convertrknn.py
# Produces yolo11n.rknn (or similar)

# Transfer to drone
scp yolo11n.rknn user@mappingdrone:~/finals/models/
```

### 11.2 Confirm rknnlite runtime works

```bash
ssh user@mappingdrone
cd ~/finals
python3 references/finalist_codes/rknn_detect/testrknn_with_display.py
```

Replace the RKNN_MODEL path. Expected: opens an OpenCV window showing the test image with bounding boxes. Confidence threshold 0.25 may be too tight for a stock model — drop to 0.1 to see *any* output, then verify behaviour.

### 11.3 Benchmark fps

```python
import time
# After model loaded:
N = 100
t0 = time.time()
for _ in range(N):
    out = rknn.inference(inputs=[img])
print(f"NPU inference: {N/(time.time()-t0):.1f} fps")
```

Expect ~50 fps for yolov11n on RK3588 with `NPU_CORE_AUTO`. Anything under 20 fps → check core mask, check input size matches the model's expected size, check NHWC vs NCHW.

### 11.4 GATE 11

- [ ] `.rknn` loads on the drone without error.
- [ ] Inference returns non-empty output arrays.
- [ ] Throughput ≥ 25 fps for yolov11n-class model.

---

## Phase 12 — Stage 1 integration: lawnmower + ArUco logging

Goal: drone flies a predetermined path, logs every ArUco it sees with arena coordinates, produces a top-down summary.

### 12.1 Architecture

Single-file mission script `stage1_mission.py`:

```
┌── UwbNode (ROS2 thread)
├── RealSense pipeline (worker thread)
├── ArUco detector (per-frame, in worker thread)
├── ArucoLogger (writes detections to ./aruco_log.json)
└── MissionLoop (asyncio)
    1. takeoff via kolomee primitives
    2. for each waypoint in lawnmower_grid:
         await fly_to_position_velocity(wp_n, wp_e, -1.5)
         await hover(2.0)        # let ArUco detector capture
    3. land
```

### 12.2 Lawnmower waypoints

Hardcode for the venue. Example (4×4 m arena, 1 m spacing, 1 row at altitude 1.5 m):

```python
WAYPOINTS = [(0, 0), (0, 1), (0, 2), (0, 3),
             (1, 3), (1, 2), (1, 1), (1, 0),
             (2, 0), (2, 1), (2, 2), (2, 3)]
```

Replace with the actual arena dimensions once you have them.

### 12.3 Top-down deliverable

The Finals expect a **top-down depth map**. Generate it from accumulated RealSense point clouds — see `references/finalist_codes/realsense_cam/generateTopDown.py`. Output: a single PNG file showing occupancy (free/occupied/unknown).

### 12.4 GATE 12

- [ ] Drone completes the lawnmower without crashing.
- [ ] `aruco_log.json` contains all visible markers with positions.
- [ ] Top-down PNG saved and visually correct (you can see arena floor + obstacles).

---

## Phase 13 — Stage 2 integration: HULA swarm

### 13.1 Verify HULA discovery from the C2 Terminal

This step runs on the **C2 Terminal**, *not* the mapping drone. From the Ubuntu VM on the C2:

```bash
pip install pyhulax       # may need vendor-provided wheel
python3 - <<'EOF'
from dola import Dola
d = Dola(); d.start()
try:
    ips = d.get_all_ips(listen_seconds=5)
    print(f"Discovered: {ips}")
finally:
    d.stop()
EOF
```

Expect a dict of `{plane_id: ip}` for each powered HULA.

### 13.2 Single-drone control test

Pick one HULA, connect, takeoff, hover 5 s, land. Verify video stream renders.

### 13.3 Three-drone parallel test

Skeleton: one Python thread per drone, each running its own state machine. Shared resources (chosen landing zones, completion flags) behind `threading.Lock`.

### 13.4 Mission integration

Each HULA's mission (Pre-U Stage 2 per Finals brief):
1. Discovery (Dola).
2. Receive target landing zone (x, y, z) from organizer Discord post.
3. Takeoff.
4. Move via body-relative commands toward the target. (HULA doesn't accept world coordinates — you must convert via locked-yaw assumption like the mapping drone.)
5. Land on pad (Scoring Item 1: number of landings within hoop).
6. Brief pad-hold (convoy enters cage).
7. **Take off again** to ~1.1 m hover altitude (brief: "recommended height is 1.1m").
8. Run ArUco detection on the live video stream — log every decoded ID.
9. End-of-window: final land.
10. Scoring Item 2: number of ArUco detections logged.

### 13.5 GATE 13

- [ ] All 3 HULAs discovered + connected within ≤ 10 s.
- [ ] Each can takeoff/land independently from the C2 terminal.
- [ ] Three parallel takeoffs work without deadlock.
- [ ] cv2.aruco runs on snapshot frames from a HULA's video stream and decodes test markers correctly.
- [ ] Re-takeoff after landing works (Stage 2 mission requires landing then taking off again).

---

## Phase 14 — Full mission dress rehearsal

Run Stage 1 + Stage 2 end-to-end on the actual arena, in the time budget the Finals will use. Log everything.

### 14.1 Run sheet

T+0:00  Stage 1 starts. Operator presses run button on mapping drone.
T+0:05  Mapping drone airborne; logging ArUco.
T+0:08  Mapping drone landed; ArUco log finalised.
T+0:09  Stage 1 results loaded into Stage 2 strategy code on C2.
T+0:10  HULAs discovered + connected.
T+0:11  HULAs airborne.
T+0:14  HULAs landed on chosen pads. Begin convoy detection.
T+0:17  Convoy snapshot capture phase.
T+0:20  Mission end.

Adjust to actual venue timing.

### 14.2 GATE 14

- [ ] One full end-to-end run completed without manual intervention.
- [ ] All artefacts produced: ArUco log, top-down PNG, HULA landing positions, RoboMaster snapshots.

---

## Phase 15 — Competition day

Same idea as Qualifier Phase 5: USB stick contents, pre-run checklist, behaviour during run. Adapt the deliverables list to the Stage 1 / Stage 2 outputs.

`★ Insight ─────────────────────────────────────`
- The Finals are **much more sequenced** than the Qualifier was. Stage 1's output feeds Stage 2 directly. If your Stage-1 ArUco log is bad, your Stage-2 landings target the wrong pads. Test the **data interchange** between stages early (a JSON schema is enough).
- The single biggest time risk in a real-hardware comp is **battery management**. Flight time per HULA is ~3-5 minutes. Have ≥2 charged batteries per drone, and a charge plan that accounts for inter-rehearsal cooldown.
`─────────────────────────────────────────────────`

*End of finals runbook. Phase 7-8 alone (Tutorial + first hardware contact) get you out of "what is this hardware" mode and into "I can iterate" mode. That's the hardest psychological transition; everything after is mechanical.*
