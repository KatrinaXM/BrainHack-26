# RoboVerse 2026 Qualifier — A Plain-English Tutorial

> Read this before the slides. The slides assume you already know the vocabulary; this document gives you the vocabulary.
>
> Reading time: ~45 minutes. Re-readable as a reference.

---

## 0. The big picture in one paragraph

You are flying a **simulated quadcopter** in a **virtual space port** using a piece of software that pretends to be a real flight controller. The drone has cameras. Your job is to write a Python program that reads from those cameras, decides where to fly, and tells the flight controller to go there — all by itself, in under 10 minutes, while finding coloured barrels along the way. Everything mathematical or "robotics-y" in the slides is in service of one of four jobs: **(1) know where you are, (2) know what's around you, (3) decide where to go next, (4) tell the drone to actually move there.**

If something in the slides confuses you, ask which of the four jobs it's serving. That alone unlocks most of the theory.

---

## 1. The cast of characters

You'll keep meeting these names. Get them straight first.

| Name | What it is | Plain-English analogy |
|---|---|---|
| **Drone / quadcopter** | The flying robot with 4 rotors | A car |
| **PX4** | The autopilot software running on the drone | The car's electronic control unit (ECU) — handles throttle, brakes, steering, but doesn't know where you want to go |
| **Mission computer** | A separate computer (in our case, just your laptop) that tells PX4 what to do | The driver sitting in the car deciding the route |
| **MAVLink** | The wire protocol used between PX4 and the mission computer | The cable plugged into the car's OBD-II port |
| **MAVSDK** | The Python library that speaks MAVLink for you | A friendly steering-wheel-and-pedals interface so you don't have to flip raw electrical switches |
| **Gazebo** | The physics simulator pretending to be the world | A driving video game with realistic physics |
| **SITL (Software In The Loop)** | Running PX4 as software on your laptop instead of on a real drone | Practicing in Gran Turismo instead of in a real Lamborghini |
| **QGroundControl (QGC)** | A GUI app that monitors and configures PX4 | The OBD-II diagnostic display |
| **gz-transport** | The messaging system Gazebo uses to publish sensor data | A radio station: Gazebo broadcasts on channel "depth_camera", you tune in |

`★ Insight ─────────────────────────────────────`
- The single most important separation: **PX4 handles "fly stably and not crash"; YOUR CODE handles "where to fly and why".** PX4 will *never* explore, plan, or detect objects. You give it setpoints (positions or velocities), and it executes them.
- The same MAVSDK code that flies a simulated drone in Gazebo flies a real one — that's the whole point of SITL. The competition Final uses real hardware; the qualifier uses simulation. Your code largely doesn't care which.
`─────────────────────────────────────────────────`

---

## 2. Why the qualifier is hard: no GNSS

GNSS = Global Navigation Satellite System (GPS is one example). A normal outdoor drone:

1. Listens to satellites.
2. Knows its (latitude, longitude, altitude) within ~1 m.
3. Reports this to PX4, which can then fly "to GPS coordinate (X, Y)".

Inside a building, walls block satellite signals. So PX4 has **no idea where it is** unless you give it position information from *somewhere else*. This is the **GNSS-denied** problem.

Three common GNSS replacements:

- **Visual Odometry (VO)**: cameras watch the world go by and estimate motion from how features shift between frames.
- **Visual-Inertial Odometry (VIO)**: VO + the drone's IMU (accelerometer + gyroscope) fused together. More accurate.
- **LIDAR/SLAM**: spinning laser scanners. Not used in this competition.

For the qualifier, you don't implement VIO yourself. PX4 has a special simulated drone model called **`x500_vision`** that pretends to have working VIO baked in. You just read its position via MAVSDK. The other model, **`x500_depth`**, adds a depth camera (we'll use this — it makes obstacle avoidance easy).

**Analogy:** Imagine being blindfolded in a room. GPS would be a voice saying "you're 3 m from the north wall". VIO is feeling the air moving past your face and counting your steps. The drone, like you, will gradually get a wrong sense of distance over many minutes — this is called **drift**. Don't trust the drone's position after 8 minutes as much as you trust it at 30 seconds.

---

## 3. Coordinate frames — the #1 source of bugs in robotics

Everything in 3D space needs an origin and three axes. Different systems pick different origins and axes, and mixing them up makes the drone fly the wrong way.

### 3.1 NED (North-East-Down) — the world frame PX4 uses

- **X axis = North** (positive X means "go north")
- **Y axis = East** (positive Y means "go east")
- **Z axis = Down** (positive Z means "go DOWN, into the ground")

Yes, **down is positive**. That means **negative Z is up**. If you want to fly up at 2 m altitude, your target Z is **−2.0**. This will trip you up if you don't internalise it. Write it on a sticky note.

The origin (0, 0, 0) is wherever the drone took off from.

### 3.2 FRD (Forward-Right-Down) — the body frame

- **X axis = where the drone's nose is pointing** (forward)
- **Y axis = right side of the drone**
- **Z axis = down (out of the drone's belly)**

If the drone yaws (rotates), the body frame rotates with it. The world frame doesn't.

**Analogy:** When you say "the kitchen is north of me", that's the world frame. When you say "the kitchen is to my left", that's your body frame. If you turn 90°, "north" doesn't change but "my left" does.

### 3.3 Camera frame

Cameras typically use:
- **X axis = right** (in the image)
- **Y axis = down** (in the image — yes, also down)
- **Z axis = forward** (out of the lens, the direction the camera is pointing)

### 3.4 Conversions are matrix multiplications

Going from camera → body → world frame requires rotating the coordinates. The code in `AvoidancePlanner.py` lines ~220 does this. **You don't have to derive the math** — the reference code already implements it. You just have to know:

1. Depth camera says "there's a wall 2 m in front of me" (camera frame).
2. Your code converts that to "there's a wall at body-X=2, body-Y=0, body-Z=0" (body frame).
3. Then converts to "there's a wall at world (north=23, east=5)" (NED) using the drone's current pose.
4. Only now can you draw it on a map.

`★ Insight ─────────────────────────────────────`
- A common bug: you compute a target in body frame (e.g. "5 m forward of me") and accidentally send it as if it were NED. The drone interprets "5 m forward" as "5 m north" and flies the wrong way if it's facing east.
- The reference code's `compute_position_ned()` function does the conversion for you — its output is already in NED. The name is the giveaway.
- **Yaw is measured clockwise from north in PX4.** This is opposite of standard math (which is counter-clockwise from east). Another sticky-note item.
`─────────────────────────────────────────────────`

---

## 4. Python's async/await — why it matters here

MAVSDK-Python is **asynchronous**. You'll see code like:

```python
async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    await drone.action.arm()
    await asyncio.sleep(2)

asyncio.run(main())
```

The `async` and `await` keywords don't make your code run in parallel. They let your program **pause one task to run another while waiting**. Without it, your program would freeze waiting for the drone to reply, missing other sensor data.

**Analogy:** A waiter taking five tables' orders. A synchronous waiter stands at table 1 until they finish ordering, ignoring table 2. An async waiter takes table 1's order, walks away while table 1 chats, takes table 2's order, comes back. Same staff, more throughput.

### Three rules that *will* bite you

1. **Never use `time.sleep()` inside an async function.** It freezes the entire program, including the MAVLink heartbeat. PX4 will see no heartbeat for 0.5 s and trigger a failsafe (= the drone tries to land itself). Always use `await asyncio.sleep(seconds)`.
2. **Use `await` on anything that returns immediately a "Task" or "coroutine".** Forgetting `await` silently does nothing.
3. **To read a stream, use `async for`.** Telemetry like position comes as an infinite stream:
   ```python
   async for pos in drone.telemetry.position_velocity_ned():
       print(pos.position.north_m)
       break  # if you only want one reading
   ```

---

## 5. Offboard mode — how your code drives the drone

PX4 has many flight modes (Manual, Position Hold, Mission, Land, etc.). The one we use is **Offboard**. It means:

> "PX4, stop deciding things. Take orders from the mission computer over MAVLink instead."

Lifecycle:

1. **Send at least one setpoint first.** PX4 refuses to enter offboard mode unless it already has a target. So before starting offboard, you do `await drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))`.
2. **Start offboard mode.** `await drone.offboard.start()`.
3. **Send setpoints continuously, at minimum once every 0.5 s.** If you stop sending, PX4 assumes you crashed and triggers a failsafe. In practice you send every 50–200 ms.
4. **Stop offboard before landing.** `await drone.offboard.stop()` then `await drone.action.land()`.

This is why the reference code has loops with `await asyncio.sleep(0.1)` — that's the heartbeat keeping offboard mode alive.

**Analogy:** A learner driver. You're in the passenger seat. You have to keep giving instructions ("left… straight… slow down…") at least every half-second, or the learner panics and stops the car. The car never drives itself — your silence does *not* mean "carry on".

---

## 6. Cameras — RGB vs Depth vs Stereo

### 6.1 RGB camera

A normal camera. Output: a 2D grid of pixels, each pixel a `(red, green, blue)` triplet. Resolution typically 640×480 or 1280×720. Used for: **object detection** (YOLO).

### 6.2 Depth camera

A camera that, for every pixel, tells you **how far the thing in that pixel is from the camera**. Output: a 2D grid of pixels, each pixel a single number — the distance in metres.

Three technologies under the hood (don't worry which is which, but be aware):
- **Stereo**: two lenses spaced apart compare their views (like your two eyes).
- **Structured light**: project a known infrared dot pattern, see how it deforms.
- **Time-of-Flight**: emit light pulse, measure return time.

In Gazebo, the `x500_depth` model gives you a depth camera that publishes on the topic `/depth_camera`.

### 6.3 The depth image's secret pain points

The depth camera output you get from Gazebo is **not** a nice 2D NumPy array. It's a flat stream of bytes you have to reshape:

```python
# msg.data is raw bytes, msg.width, msg.height are dimensions
depth = np.frombuffer(msg.data, dtype=np.float32)
depth = depth.reshape((msg.height, msg.width))
# now depth[row, col] is the distance in metres for pixel (col, row)
```

Each pixel is a **`float32`** = 4 bytes = a number in metres. Important edge cases:

- `0.0` or `NaN` means "no valid reading" (the ray missed everything, or hit a transparent surface, or was too close).
- `inf` means "further than the camera can see" (clipped).
- Real value e.g. `2.34` means "2.34 m from the lens to whatever is at that pixel".

`★ Insight ─────────────────────────────────────`
- A depth camera doesn't tell you *what* is at each pixel — just *how far*. Combine it with the RGB camera to know what (an RGB-D pair).
- Depth is given relative to the **camera optical centre**, not the drone's centre of gravity. There's a small offset to be aware of for precision work; you can ignore it for this competition.
- Depth cameras have a useful range (~0.3 m to ~10 m typically). Closer than 0.3 m or further than 10 m and you'll get `0` or `inf`. Don't expect them to see across the whole 40 m space port.
`─────────────────────────────────────────────────`

---

## 7. Camera intrinsics — from pixel to 3D point

A pixel at column `u`, row `v` in the depth image, with depth `Z` metres, corresponds to a 3D point in the camera frame:

```
X_cam = (u - cx) * Z / fx
Y_cam = (v - cy) * Z / fy
Z_cam = Z
```

The four numbers `fx, fy, cx, cy` are the **camera intrinsics**:
- `fx, fy` = focal length in pixels (how zoomed-in the lens is, separately for horizontal and vertical).
- `cx, cy` = optical centre (usually the middle of the image, e.g. (320, 240) for a 640×480 sensor).

You'll see these as a 3×3 matrix `K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]`. The reference code uses `K = np.array([[433, 0, 320], [0, 433, 240], [0, 0, 1]])` — focal length 433, optical centre (320, 240).

**Analogy:** Imagine looking through a porthole at a tree. The tree appears small near the edges (because it's at an angle). The intrinsics describe the porthole — its size and shape — so you can geometrically back out what direction the tree actually is.

**Why you care:** when YOLO finds a red barrel at pixel (450, 300), and the depth at that pixel is 4.2 m, you can compute the 3D position of that barrel relative to the camera. Then via the coordinate-frame conversions in §3, you get its (north, east, down) in world coordinates. **That's how you deduplicate detections** — same NED → same barrel.

---

## 8. Obstacle avoidance using a depth histogram

The reference `AvoidancePlanner.py` doesn't use the full depth map directly (too much data, too messy). Instead:

### 8.1 Bin the image into vertical strips

Divide the 640-column-wide depth image into, say, 12 vertical strips (`num_of_bins = 12`). Each strip is one "direction the drone might fly".

### 8.2 Summarise each strip with one number

For each strip, take the **20th percentile of valid depth pixels**. Why the 20th percentile and not the average?
- Average is dragged up by background pixels (sky, far walls).
- Minimum is dragged down by noisy single-pixel glitches.
- 20th percentile picks "the close stuff", ignoring noise — gives a robust "what's the nearest obstacle in this direction".

### 8.3 Cost each strip

- If strip distance < `critical_distance` (e.g. 1.0 m): cost = very high (don't go there!).
- If strip distance > `safe_distance` (e.g. 3.0 m): cost = 0 (free path).
- Between: cost interpolated.

### 8.4 Pick the lowest-cost strip

Find which strip is safest, compute the angle to it using the camera intrinsics (since you know strip-to-angle from `fx` and `cx`), and that becomes your "avoidance direction".

**Analogy:** Standing in a doorway with your hands out, sweeping them through the air. Your hands tell you where the walls are. You walk toward whatever direction your hands are *least* obstructed.

The code also flags the *whole scene* as `Blocked: true/false` and gives a high-level label: `"open"`, `"forward_clear"`, `"left_open"`, `"right_open"`. Your mission logic uses these to decide things like "the front is completely blocked, time to rotate 90°".

---

## 9. Goal vector + avoidance vector — combining "where to go" with "what to dodge"

Avoidance alone is reactive — the drone just bumps around aimlessly like a Roomba. To get purposeful movement, you add a **goal vector**.

- **Avoidance vector**: points away from obstacles. Computed from §8.
- **Goal vector**: points from where you are *now* toward where you want to *go* (e.g. the centre of the next unexplored grid cell).
- **Resultant**: add them up, normalise. Project a short distance (`look_ahead`, e.g. 1 m) along that direction. That's your next setpoint.

If `look_ahead` is too big, the drone tries to cut corners through walls. Too small and it crawls. 1–2 m is typical.

**Analogy:** You want to walk to the kitchen (goal vector). There's a chair in the way (avoidance vector). You don't go *around* the chair to the kitchen by stopping and re-planning — you naturally lean a bit sideways while still pointing kitchen-ward. The resultant of "kitchen-vector + chair-vector" makes you veer.

---

## 10. Occupancy grids — drawing a map as you fly

An **occupancy grid** is a 2D array representing the floor (top-down view). Each cell is one of:
- **Unknown**: never seen.
- **Free**: depth camera saw past this cell — empty space.
- **Occupied**: depth camera saw something solid here.

As the drone flies, for each depth-image frame:
1. Compute 3D points (§7).
2. Convert to NED world coordinates (§3.4).
3. Mark cells along each depth ray as "free" up to the obstacle, and the obstacle cell as "occupied".

After a minute of flight you have a map of where walls are, where you've been, and where's still unknown. **Unknown cells = where to go next.**

Resolution trade-off: 0.5 m cells are crisp but big; 0.1 m cells are precise but slow. The space port is 40×40 m. At 0.5 m resolution that's an 80×80 grid = manageable.

The reference code `GlobalMapper_new.py` implements this.

---

## 11. Exploration strategies — "where do I go next?"

Three approaches, simplest to fanciest:

### 11.1 Lawnmower (the one I recommend)

Pre-plan a fixed sweep pattern: fly north until blocked or out of bounds, sidestep east one cell, fly south, sidestep east, fly north, etc.

**Pros**: simple, predictable, covers everything systematically.
**Cons**: doesn't adapt to dead ends — if a wall blocks the row, you might miss what's behind it.

This is what `avoid_with_detect.py` is structured for (notice `self.grid_headings = [0, 90, 180, -90]`).

### 11.2 Frontier-based exploration

Look at your occupancy grid. The **frontier** is the boundary between known-free cells and unknown cells. Always fly to the nearest frontier. Repeat until no frontiers left.

**Pros**: provably covers everything reachable, adapts to maze layout.
**Cons**: requires the occupancy grid working well, and a path-finder (A* / RRT*) to route you to the chosen frontier without bumping into walls. More moving parts.

### 11.3 Wall-following

Hug one wall and follow it (like solving a maze with the right-hand rule).

**Pros**: robust, guaranteed to traverse all rooms connected to the start.
**Cons**: slow; you might walk a long way before discovering a barrel.

`★ Insight ─────────────────────────────────────`
- Don't over-engineer. A lawnmower that handles dead-ends with a simple "rotate to next free direction" works for 90% of layouts. Frontier-based looks more impressive but adds two new failure modes (grid bug, planner bug).
- Always **time-cap exploration**. After 8 minutes, regardless of state, return home. Crashing in the last minute loses you all your detections.
`─────────────────────────────────────────────────`

---

## 12. YOLO — what it actually does

**YOLO (You Only Look Once)** is a family of neural networks for object detection. You feed it an RGB image, it returns a list of **detections**, each one being:

```
{
  "class_name": "fuel_barrel_red",
  "confidence": 0.87,
  "box": (x1=412, y1=233, x2=485, y2=341)   # pixel rectangle in the image
}
```

`confidence` is the model's self-estimated probability (0–1). You set a **confidence threshold** (e.g. 0.6) to discard noisy guesses.

YOLO is called "you only look once" because older detectors ran sliding-window classifiers many times per image; YOLO predicts every box in a single neural-network forward pass. **Why you care**: it's fast enough for real-time use on a CPU — ~10–30 frames per second.

### 12.1 Training a custom model

The model file shipped (`yolov10n.pt`) is the "nano" version of YOLOv10 trained on generic objects (cars, people, etc.) — it won't reliably detect a fuel barrel. You'd train a custom one by:

1. Capturing ~100–300 images of barrels from the drone's camera (`save_photo.py`).
2. Labelling each barrel with a bounding box and class (`fuel_barrel_yellow`, `fuel_barrel_red`) using a tool like LabelImg or Roboflow.
3. Running `Train_YOLO_Models.ipynb` on Google Colab (free GPU) to fine-tune the model. Takes ~30 minutes once labelled.
4. Replacing `yolov10n.pt` with your new `best.pt`.

### 12.2 The Detector class

`Detector.py` wraps YOLO in a background thread so it doesn't block your flight loop. You call `detector.submit_image(frame)` from the camera callback; YOLO runs on another thread; when done, your callback `on_detection(detections, image, context)` fires. This keeps the drone flying smoothly during inference.

**Analogy:** YOLO is a security camera operator. You hand them a photo; they look at it; they call you back later to say "yes, I see a red barrel in the top-right". You don't wait at the desk while they look.

---

## 13. Putting it all together — the mission loop

Here's the entire mission as one mental flowchart:

```
┌─────────────────────────────────────────────────────────────┐
│                       BACKGROUND TASKS                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │ Position     │  │ Depth frame  │  │ RGB frame        │  │
│  │ telemetry    │  │ subscriber   │  │ subscriber       │  │
│  │ (MAVSDK)     │  │ (gz)         │  │ (gz)             │  │
│  │ updates pose │  │ updates depth│  │ → submits to YOLO│  │
│  └──────┬───────┘  └──────┬───────┘  └────────┬─────────┘  │
└─────────┼─────────────────┼───────────────────┼────────────┘
          ▼                 ▼                   ▼
    ┌──────────────────────────────────────────────────┐
    │              MAIN MISSION LOOP (20 Hz)            │
    │                                                   │
    │  1. read latest pose                              │
    │  2. read latest depth frame                       │
    │  3. AvoidancePlanner → (safe_direction, blocked)  │
    │  4. ExplorationPolicy → goal_position            │
    │  5. combine into target setpoint                  │
    │  6. send via drone.offboard.set_position_ned()   │
    │  7. sleep 50 ms                                   │
    │                                                   │
    │  also, separately:                                │
    │  - on YOLO callback: convert pixel + depth        │
    │    to NED position, deduplicate, log              │
    └───────────────────────────────────────────────────┘
```

This is exactly the structure of `avoid_with_detect.py`. Your job over the four days is to:
- Verify the background tasks work.
- Tune avoidance parameters (`critical_distance`, `safe_distance`, `alpha`) so the drone doesn't oscillate.
- Replace the basic exploration with a deliberate lawnmower.
- Train a better YOLO model.
- Add detection deduplication.
- Add a timer to bail at 8 minutes.

---

## 14. Concept checklist — what to drill before coding

Before writing any meaningful code, you should be comfortable answering each of these out loud. If you can't, re-read the relevant section.

- [ ] What does **PX4** do that my code doesn't, and vice versa?
- [ ] What's the difference between **NED** and **FRD**? Why is up *negative* Z?
- [ ] What is **offboard mode** and what's the minimum I have to do to keep it alive?
- [ ] Why is `time.sleep()` forbidden inside an async drone script?
- [ ] How is a Gazebo depth frame stored, and what do `0.0` and `NaN` mean?
- [ ] What are **camera intrinsics** and what do they convert?
- [ ] How does the **AvoidancePlanner** turn a depth image into a flight direction?
- [ ] What's the difference between a **goal vector** and an **avoidance vector**, and how are they combined?
- [ ] What is an **occupancy grid** and where do "unknown" cells come from?
- [ ] What does **YOLO** output and what is the **confidence threshold** for?
- [ ] What is **drift** in visual odometry and why does it matter at minute 8?

---

## 15. The "minimum viable competition entry"

If I could only get one thing working, here's the priority order. **Stop at any level and you have *something* that scores.**

1. **Takeoff + hover + land autonomously.** (1 hour of work using `takeoff_and_land.py`.)
2. **Move forward 5 m then back, no avoidance.** (1 hour, `basic_offboard.py`.)
3. **Reactive avoidance loop** (`avoid.py` as-is, no goal). The drone wanders but doesn't crash. (2 hours of tuning.)
4. **Add YOLO detection on top** (`avoid_with_detect.py`). Now wandering + spotting yellow barrels. Probably scores 50 points if you wander long enough.
5. **Add a lawnmower goal pattern.** Now it covers ground systematically. Probably 100–250 points.
6. **Add altitude variation to see red barrels.** Now eligible for university tier scoring. 250–500 points.
7. **Custom-trained YOLO model + detection deduplication.** Faster, more reliable detections, time bonus eligible. 500+ points.
8. **Frontier-based exploration with occupancy grid.** Optimal coverage. Top-tier score.

Levels 1–5 are realistic in 4 days. Level 6 if you push. Levels 7–8 are the differentiators.

---

## 16. Glossary (alphabetical)

- **Async / await**: Python keywords that let one task pause while waiting, so other tasks can run.
- **Avoidance vector**: a direction pointing away from obstacles.
- **Body frame (FRD)**: Forward-Right-Down coordinate axes attached to the drone; rotates with it.
- **Camera intrinsics**: `fx, fy, cx, cy` that describe how a camera turns 3D points into pixels.
- **Confidence threshold**: minimum probability YOLO must report for you to accept a detection.
- **Depth camera**: a camera that outputs distance-per-pixel instead of (or in addition to) colour.
- **Drift**: gradual accumulation of error in visual odometry's estimate of position.
- **EKF (Extended Kalman Filter)**: PX4's sensor-fusion algorithm that combines IMU, GPS, vision, etc. into one position estimate. You don't tune it.
- **Failsafe**: PX4's defensive behaviour when something goes wrong (e.g. lost heartbeat → auto-land).
- **Frontier**: in a map, the boundary between known-free and unknown space.
- **Gazebo**: the physics simulator.
- **GNSS**: Global Navigation Satellite System; GPS is one.
- **Goal vector**: a direction pointing from now to your target.
- **gz-transport**: Gazebo's publish/subscribe messaging system.
- **IMU**: Inertial Measurement Unit; accelerometer + gyroscope + (sometimes) magnetometer.
- **Lawnmower pattern**: a zig-zag systematic search path.
- **MAVLink**: the wire protocol PX4 speaks.
- **MAVSDK**: the Python (or C++/Swift/Java) library that speaks MAVLink to PX4.
- **Mission computer**: the laptop or onboard computer running your high-level code.
- **NED (North-East-Down)**: the world coordinate frame PX4 uses; Z points DOWN.
- **Occupancy grid**: a 2D map of free / occupied / unknown cells.
- **Offboard mode**: PX4 flight mode where it follows commands from the mission computer.
- **PX4**: the open-source autopilot software running on the drone.
- **QGroundControl (QGC)**: GUI for monitoring/configuring PX4.
- **RGB-D**: a sensor or image pair giving both colour and depth.
- **Setpoint**: a target (position, velocity, attitude) you send to PX4 in offboard mode.
- **SITL**: Software In The Loop; running PX4 as software on a PC instead of on real hardware.
- **SLAM**: Simultaneous Localisation and Mapping — building a map while tracking your position in it; more advanced than VIO. Not needed for the qualifier.
- **VIO**: Visual-Inertial Odometry; estimating motion from camera + IMU.
- **VO**: Visual Odometry; estimating motion from camera alone.
- **Yaw**: rotation around the vertical axis (turning left/right). In PX4: measured clockwise from north.
- **YOLO**: "You Only Look Once" — a fast neural-network object detector.

---

## 17. What to read next, in order

1. This document (you just did).
2. **`LearningMaterial1.pdf`** — the slides will now make sense.
3. **`Supplmentary_LearningMaterial1.pdf`** — deep dive on coordinate frames.
4. **`LearningMaterial2.pdf`** — VIO concept + depth-camera avoidance pipeline.
5. **`LearningMaterial3.pdf`** — exploration + YOLO + occupancy grid.
6. Open `references/codes/avoid_with_detect.py` and read it top to bottom with this document next to you.
7. Then `AvoidancePlanner.py`, `drone_control_new.py`, `Detector.py`, `GlobalMapper_new.py`.

After that, you'll be ready to start writing.

---

*End of qualifier tutorial. If a concept still feels fuzzy after reading the relevant section twice, send the exact sentence that confuses you and I'll re-explain it with a different analogy.*

---

# PART 2 — Finals tutorial

> Read this only after you've internalised Part 1 (NED, MAVSDK, async, intrinsics, offboard mode). Everything in Part 1 still applies — the Finals just replace the simulator with real hardware and add new sensors/libraries on top.
>
> Reading time: ~60 minutes. Re-readable as a reference.

---

## 18. The big picture, mark 2

You no longer fly a simulated drone in Gazebo. Instead:

- **Stage 1 (University only — Reconnaissance):** you fly one **Mapping Drone** over an arena. It carries an **Intel RealSense stereo depth camera**, a **Rockchip NPU** for fast YOLO, a **UWB tag** that broadcasts its x,y position into the arena's positioning system, and an onboard Linux box you reach via **NoMachine** remote desktop. Your code produces a top-down depth map and identifies which landing pads are valid by decoding **ArUco markers** placed next to each.
- **Stage 2 (Deployment & Ambush):** Using your Stage-1 map, you pick 3 landing zones. You launch 3× **Highgreat HULA** drones from the **C2 Terminal** (a Windows laptop running an Ubuntu VM) via the **pyhulax** library. The HULAs land on the chosen pads. Then 5× **RoboMaster ground robots** enter the cage as a "convoy"; the HULAs search for them and capture snapshot images.

The mental model: Stage 1 is "build the map"; Stage 2 is "execute on the map". The good news is your Qualifier code (MAVSDK offboard, YOLO pipeline, depth→world projection) **transfers wholesale** — only the libraries supplying camera frames, position, and inference change.

`★ Insight ─────────────────────────────────────`
- The Qualifier taught you to bypass a broken pose estimator with body-velocity wall-follow. The Finals give you a *working* pose estimator (UWB) — so the Workshop's prescribed pipeline (poll position → compute error → send velocity command) finally works. **kolomee.py is that pipeline.**
- This is a "real hardware" event: you cannot iterate as quickly as in sim. Every test run involves charging batteries, walking onto the arena, recording video, etc. Bias toward *bench tests* and *dry runs* over flight tests.
`─────────────────────────────────────────────────`

---

## 19. New cast of characters

| Name | What it is | When you'll meet it |
|---|---|---|
| **Mapping Drone** | A custom build with an Intel RealSense camera, Rockchip RK35xx onboard computer, UWB tag, and PX4-based flight controller. You SSH/NoMachine into it. | All of Stage 1. |
| **C2 Terminal** | The team's laptop. Windows host + Ubuntu 22.04 VM. Runs your strategy code. | Both stages, but actively only for Stage 2 (controlling the HULAs). |
| **HULA drones** | 3× Highgreat HULA mini-quadcopters. Programmed via the **pyhulax** Python library over WiFi. | All of Stage 2. |
| **Intel RealSense** | A consumer stereo depth camera. Streams synced RGB + depth + IR via the **pyrealsense2** Python library. | Mapping drone sensor. |
| **ArUco marker** | A printed black-and-white square pattern OpenCV can detect and decode to an integer ID. | Placed beside each landing pad; you decide valid/invalid based on ID. |
| **NPU / RKNN** | Neural Processing Unit on the Rockchip SoC. You convert YOLO weights to RKNN format and run inference at ~50fps via the **rknnlite** Python library. | Real-time detection on the mapping drone. |
| **UWB** | Ultra-Wideband indoor positioning. Anchor stations around the arena + a tag on each drone. Gives ~10–30 cm accuracy x,y at ~10 Hz. | All positioning everywhere. |
| **ROS2 (Humble or Jazzy)** | A robotics middleware. You'll mostly only use it to subscribe to one topic (`uwb_tag`). | Optional — you can also access UWB without ROS2. |
| **RoboMaster ground robots** | 5× DJI RoboMaster S1-class robots driving programmatically around the arena as your "convoy targets". | Stage 2 detection targets. |
| **NoMachine** | A remote-desktop protocol like VNC but faster. Gives you a real Linux desktop on the mapping drone over the network. | Your daily editor for mapping-drone code. |

---

## 20. UWB (Ultra-Wideband) — the new "GPS-indoors"

In the Qualifier we had a broken simulated "VIO". In the Finals we have **real UWB positioning** with anchor stations bolted to the arena's truss.

### 20.1 How UWB works in two sentences

Several **anchor stations** mounted around the arena broadcast precisely-timed UWB pulses. A **tag** on each drone measures the time-of-flight to each anchor, multilaterates its x,y in the arena frame, and broadcasts the result over WiFi at ~10 Hz.

**Accuracy:** ~10–30 cm in good conditions. **Refresh rate:** ~10 Hz. **Latency:** typically <100 ms.

### 20.2 Coordinate frame — the swap

The Finals materials and `kolomee.py` reveal a quirk: UWB outputs a `PoseStamped` ROS message where:

```
msg.pose.position.x  ←  east of arena origin
msg.pose.position.y  ←  north of arena origin
msg.pose.position.z  ←  up (sometimes unused)
```

But `kolomee.py` immediately stores them swapped:

```python
self.n = msg.pose.position.y    # arena-north
self.e = msg.pose.position.x    # arena-east
```

This makes the variable names align with PX4's NED convention. **If your drone flies the wrong way, this swap is the first place to check** — different installations of the UWB system may have different axis conventions. Sanity-test it by manually walking the drone north and watching which value increases.

### 20.3 Failure modes

- **NLOS (Non-Line-Of-Sight):** if the tag can't see ≥3 anchors (metal box, person standing in the way), its solution becomes garbage. Don't fly through wireframe scaffolding columns.
- **Multipath:** in a metallic indoor venue, UWB pulses bounce off walls and arrive at the tag multiple times. The receiver picks the wrong one and reports an incorrect distance. Effect: position jitters or jumps by ~1 m. Mitigation: gentle velocity gains so the drone doesn't chase the jitter.
- **Anchor calibration:** if an anchor was bumped, all measurements through it are biased. You can't fix this; it's the venue's job. But if your drone consistently drifts a fixed direction, suspect this.

`★ Insight ─────────────────────────────────────`
- Treat UWB like a noisy GPS, not a perfect oracle. Always low-pass filter or use proportional control with modest gain (`KP=0.1` in kolomee), not bang-bang. Otherwise the drone bounces around chasing every UWB twitch.
- UWB gives x,y, **not yaw**. The PX4 IMU still provides yaw via `attitude_euler()`. This is why kolomee.py locks `takeoff_yaw` at startup and sends it with every `VelocityNedYaw` setpoint — the drone never turns; it strafes.
`─────────────────────────────────────────────────`

---

## 21. Intel RealSense via pyrealsense2

The Mapping Drone's depth camera is an **Intel RealSense** (likely D435i or D455). It plugs in via USB3 and exposes synchronised colour + depth + IR streams through Intel's **pyrealsense2** Python SDK.

### 21.1 The canonical pipeline pattern

Every pyrealsense2 program follows the same shape:

```python
import pyrealsense2 as rs
import numpy as np

# 1. Create pipeline + config
pipeline = rs.pipeline()
config = rs.config()

# 2. Enable the streams you want
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)   # 30 Hz, raw uint16
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 3. Start
profile = pipeline.start(config)

# 4. (Optional but recommended) get the depth scale to convert z16 → metres
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
# usually 0.001 — meaning each z16 unit = 1 mm

# 5. (Optional) align depth to color so the same (u,v) refers to the same pixel
align_to_color = rs.align(rs.stream.color)

try:
    while True:
        frames = pipeline.wait_for_frames()    # blocks until a fresh frame is ready
        frames = align_to_color.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())  # H×W×3 uint8 BGR
        depth_image = np.asanyarray(depth_frame.get_data())  # H×W uint16 (z16)

        # depth in metres for a pixel (cx, cy):
        distance_m = depth_frame.get_distance(cx, cy)
        # OR vectorised:
        depth_m_array = depth_image.astype(np.float32) * depth_scale

        # ... your logic ...

finally:
    pipeline.stop()
```

### 21.2 The 6 things that will bite you

1. **USB3 is required.** A USB2 cable or hub silently downgrades you to 6 fps maximum. If your camera "feels slow", check `lsusb -t`.
2. **`get_distance(u, v)` returns 0.0 for invalid depth** (too close, too far, transparent surface, specular reflection). Filter zeros.
3. **`align()` is expensive** (~5 ms on a Rockchip). If you don't need pixel-perfect RGB↔depth correspondence, skip it.
4. **Resolution mismatches:** depth stream max is usually 1280×720; color max varies by model. If a config call silently fails, the pipeline starts but you get no frames — check `pipeline.try_wait_for_frames(2000)`.
5. **Holes in depth** at object boundaries are normal. The post-processing filters (`rs.spatial_filter()`, `rs.temporal_filter()`) help if you have CPU budget.
6. **First few frames are garbage** (auto-exposure / autofocus). Discard the first 5 frames.

### 21.3 Getting camera intrinsics

You'll need `fx, fy, cx, cy` to back-project pixels to 3D (same as Qualifier §7). With pyrealsense2:

```python
depth_intrinsics = depth_frame.profile.as_video_stream_profile().get_intrinsics()
fx, fy, cx, cy = depth_intrinsics.fx, depth_intrinsics.fy, depth_intrinsics.ppx, depth_intrinsics.ppy
# Distortion coefficients are usually all zero for RealSense (factory-rectified).
```

Use the *depth* intrinsics when projecting depth pixels, and the *color* intrinsics when projecting RGB pixels (after `align()` they're the same).

### 21.4 What the workshop's example files cover

The PDF lists these reference scripts in `references/finalist_codes/realsense_cam/`:

- `getRGB.py` — RGB only.
- `getDepth.py` — depth, pixel distance demo.
- `getInfra.py` — IR streams (useful if the venue is too dark for RGB YOLO).
- `getSyncDepthColor.py` — what we showed above. **Read this first.**
- `getDepthPointCloud.py` — depth → 3D point cloud (`rs.pointcloud()`).
- `generateTopDown.py` — point cloud → top-down occupancy grid (Stage 1 deliverable).
- `getDepthAndDetect.py` — same as `getSyncDepthColor.py` plus YOLO inference.

Official docs: [github.com/IntelRealSense/librealsense](https://github.com/IntelRealSense/librealsense).

---

## 22. ArUco markers — encoding "valid landing pad" in a printed square

An **ArUco marker** is a printed black-and-white square inside a thick black border. OpenCV detects it, reads the bit pattern in the centre, and returns an integer **ID**. The competition uses these to encode "valid" vs "invalid" landing zones; you decide the encoding scheme.

### 22.1 The dictionary

A "dictionary" is the set of all possible markers and IDs. Each comes from a tradeoff between marker complexity and resilience to misreads. The workshop's reference uses:

```python
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
```

This means **6×6 = 36 internal bits, up to 250 possible IDs (0–249)**. Wider grids hold more IDs but need higher resolution to detect.

### 22.2 The modern OpenCV detector pattern (OpenCV ≥ 4.7)

```python
import cv2

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector  = cv2.aruco.ArucoDetector(arucoDict, parameters)

# every frame:
gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
corners, ids, rejected = detector.detectMarkers(gray)

if ids is not None:
    cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
    for i, marker_id in enumerate(ids.flatten()):
        c = corners[i].reshape((4, 2))   # 4 corners in pixel coords
        cx_pix = int(c[:, 0].mean())
        cy_pix = int(c[:, 1].mean())
        # Get the marker's 3D position from depth at its centre:
        depth_m = depth_frame.get_distance(cx_pix, cy_pix)
        if depth_m == 0:
            continue   # depth miss; skip this detection
        # Back-project pixel → 3D using camera intrinsics
        X = (cx_pix - cx) * depth_m / fx
        Y = (cy_pix - cy) * depth_m / fy
        Z = depth_m
```

`★ Insight ─────────────────────────────────────`
- The deprecated `cv2.aruco.estimatePoseSingleMarkers` is replaced in modern OpenCV by `cv2.solvePnP`. For our use you don't need full 6-DOF pose — just the 3D **position** of the marker centre, which you get from the depth pixel at the marker's centroid (as above). If you do need full pose, pass `flags=cv2.SOLVEPNP_IPPE_SQUARE` — the planar-square specialised solver is faster and more numerically stable than the iterative default.
- The dictionary you read must match the dictionary printed. If you ever see `ids = None` when a marker is clearly visible, you've got the wrong dictionary.
`─────────────────────────────────────────────────`

### 22.3 Encoding "valid" vs "invalid"

The Finals briefing will tell you the scheme. Common patterns teams pick:

- **ID parity:** even IDs valid, odd invalid. Easy but coarse.
- **ID range lookup:** valid IDs explicitly listed (e.g. `VALID = {7, 12, 31}`). Most flexible.
- **Bit pattern in the ID:** e.g. lowest bit = valid flag.

In your code, after detecting an ID:

```python
VALID_IDS = {7, 12, 31, 44}  # to be filled from briefing
is_valid = int(marker_id) in VALID_IDS
```

### 22.4 Detection gotchas

- **Motion blur** ruins ArUco. If the drone yaws while reading a marker, the detector silently fails. Hover steady for >0.5 s when scanning.
- **Marker size matters.** A 10 cm marker at 3 m distance covers ~30 px at 640×480 — close to the limit. If the venue's markers are small, fly low.
- **Lighting glare:** bright LED reflections on glossy paper hide bits. The detector returns no ID rather than a wrong one (good news), but coverage drops.
- The `rejected` return from `detectMarkers` lists "candidate quadrilaterals that failed bit decoding" — useful to log if you're debugging "why didn't it detect this one".

Official docs: [docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).

---

## 23. RKNN — running YOLO on the Rockchip NPU

The Mapping Drone's onboard computer is a Rockchip SoC (likely RK3588 or RK3576). These chips include a dedicated **Neural Processing Unit** that runs quantised models at ~10× the speed of CPU inference for the same wattage. The Finals PDF cites **~50 fps for YOLOv11n** via NPU vs ~5 fps for the same model on the CPU.

### 23.1 The conversion pipeline

You can't run a `.pt` PyTorch file directly on the NPU. You go through three formats:

```
yolov10n.pt   →   yolov10n.onnx   →   yolov10n.rknn
   (PyTorch)        (open format)       (NPU-native)
```

Tools:
- **PyTorch → ONNX**: `ultralytics`' built-in `model.export(format='onnx')` (the workshop's `convertyolotoonnx.py`).
- **ONNX → RKNN**: `rknn-toolkit2` package on x86 (with quantisation dataset). Workshop's `convertrknn.py`.

The conversion is **done on your desktop, not on the drone.** The drone only runs the pre-converted `.rknn` file.

### 23.2 Inference at runtime via rknnlite

The lightweight inference SDK is `rknnlite.api.RKNNLite`. On the drone:

```python
from rknnlite.api import RKNNLite

rknn = RKNNLite()
rknn.load_rknn("yolo11n.rknn")    # load the converted model
rknn.init_runtime()                # allocates NPU resources
# (optional) rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)  # pin cores

# per frame:
outputs = rknn.inference(inputs=[img])   # img is a NumPy array, shape (1, H, W, 3) typically

# Then YOU run NMS + decode yourself — rknnlite does not bundle YOLO post-processing:
boxes, classes, scores = post_process_yolov8(outputs, ori_w, ori_h)
```

### 23.3 The thing that catches everyone: NHWC vs NCHW, normalisation, NMS

- **Input layout** is **NHWC** (height-width-channels-last) for most RKNN models, not the **NCHW** PyTorch uses. The conversion script normally bakes this in — but if your inference returns garbage, this is the first thing to check.
- **Input normalisation** is baked into the model graph at conversion time (mean/std). Don't re-divide pixels by 255 unless the model expects raw 0-255.
- **NMS is not bundled.** The model returns raw box predictions — typically `(8400, 84)` for YOLOv8 with 80 classes. You apply confidence threshold + Non-Maximum Suppression in NumPy. The workshop's `rknndecoder.py` and `testrknn_with_display.py` show this. (See lines 18-50 of `testrknn_with_display.py`.)

### 23.4 What you'll train

For Stage 1 we want to detect *landing pads and ArUco markers*. Two practical strategies:

- **Use ArUco detection (Section 22) directly** — no YOLO needed.
- **Train a custom YOLO** that classifies "landing_pad" so you have redundancy when ArUco fails (poor lighting, far range). Then sanity-check the pad with ArUco.

For Stage 2 we want to detect *RoboMaster ground robots*. Definitely YOLO — `roboflow.com` already has a "RoboMaster S1" dataset you can fine-tune from.

Official docs: [github.com/airockchip/rknn-toolkit2](https://github.com/airockchip/rknn-toolkit2) and [github.com/airockchip/rknn_model_zoo](https://github.com/airockchip/rknn_model_zoo).

---

## 24. HULA swarm via pyhulax

The 3 swarm drones are **Highgreat HULA** quadcopters. They're controlled from the C2 Terminal via the **pyhulax** Python library over WiFi.

### 24.1 Discovery: Dola

`pyhulax.dola.Dola` is a broadcast discovery service. You start it, it listens for HULA drones announcing themselves on the local WiFi, and returns a `{plane_id: ip}` dict:

```python
from pyhulax import DroneAPI
from pyhulax.core import Direction
from dola import Dola

dola = Dola()
dola.start()
try:
    drones_ips = dola.get_all_ips(listen_seconds=5)   # blocking, budget ≥5 s
finally:
    dola.stop()
```

### 24.2 Connecting + controlling

Each drone is independently connectable. The library is *synchronous*, unlike MAVSDK:

```python
drones = {}     # ip -> DroneAPI
streams = {}    # ip -> VideoStream
for plane_id, ip in drones_ips.items():
    d = DroneAPI()
    d.connect(ip)
    drones[ip] = d
    v = d.create_video_stream()
    d.set_video_stream(True)
    v.start()
    streams[ip] = v

# Movement commands (units of distance/speed unclear without docs — likely metres):
drones[ip].takeoff()
drones[ip].move(Direction.FORWARD, 0.5)   # 0.5 m forward
drones[ip].land()
```

### 24.3 Per-drone state machine

The skeleton in `huladola.py` strongly hints at a per-drone state-machine pattern. Each drone needs its own state because they fly different missions in parallel:

```python
states = {ip: 0 for ip in drones}    # per-drone state

while True:
    for ip in drones:
        d, s = drones[ip], states[ip]
        if s == 0:
            d.takeoff(); states[ip] = 1
        elif s == 1:
            d.move(Direction.FORWARD, target_distance[ip]); states[ip] = 2
        elif s == 2:
            d.land(); states[ip] = 3
        # else done

        # Snapshot detection on each drone's video stream
        f = streams[ip].latest_frame
        if f is not None:
            img = f.to_rgb()    # numpy array
            detect_and_log(img, drone_id=ip)
    time.sleep(0.05)
```

### 24.4 What we don't know yet (need to verify on hardware)

- **Distance/speed units** — `Direction.FORWARD, 0.5` is documented as "0.5 m"-ish but library is closed-source. Bench-test before committing to mission distances.
- **Coordinate frame** — is `Direction.FORWARD` body-frame (relative to current yaw) or world-frame? Almost certainly body-frame, but verify.
- **Failure semantics** — what happens if WiFi drops mid-flight? Does the drone hover or RTL?
- **Concurrency** — can you `move()` multiple drones simultaneously without lockstep, or does the library serialise calls?

Plan to spend a half-day bench-testing on a single HULA before scaling to 3.

Reference for library: comment in `huladola.py` cites [pyhulax.xenops.ae](https://pyhulax.xenops.ae).

`★ Insight ─────────────────────────────────────`
- pyhulax is *synchronous* (no asyncio). This is a big difference from MAVSDK and complicates parallel control of 3 drones. The textbook fix: one Python thread per drone, each running its own state machine. Shared state (target zones, completion flags) goes through a `threading.Lock()`.
- The Dola discovery's `listen_seconds=5` is a hard floor — budget this into your mission start time, not "0.5 s" or you'll miss drones.
`─────────────────────────────────────────────────`

---

## 25. ROS2 basics (just enough)

The mapping drone runs ROS2 (likely Humble or Jazzy). You only **need** ROS2 to subscribe to UWB if you use the ROS2 topic pathway. The non-ROS2 path uses a provided Python UWB class — simpler if available.

### 25.1 The 5-line subscriber

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

class UwbNode(Node):
    def __init__(self):
        super().__init__("uwb_listener")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(PoseStamped, "uwb_tag", self.cb, qos)
        self.n, self.e, self.ready = 0.0, 0.0, False
    def cb(self, msg):
        self.n = msg.pose.position.y
        self.e = msg.pose.position.x
        self.ready = True

rclpy.init()
node = UwbNode()
# run rclpy.spin in a background thread so it doesn't block asyncio:
import threading
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

# now `node.n`, `node.e`, `node.ready` are live, thread-safe to read.
```

This is exactly what `kolomee.py` does in lines 50-91.

### 25.2 QoS — BEST_EFFORT vs RELIABLE

- **RELIABLE** is TCP-like: the publisher retransmits dropped messages. Default. Use for command/control messages.
- **BEST_EFFORT** is UDP-like: fire-and-forget. Use for high-rate sensor streams. **UWB position should be BEST_EFFORT** — you don't want to receive a stale position because the system was retrying.

If publisher and subscriber QoS don't match, **you'll silently receive nothing**. This catches everyone once.

### 25.3 Debugging tools

In a separate terminal on the drone:

```bash
ros2 topic list                          # what topics exist?
ros2 topic echo /uwb_tag                 # print every message
ros2 topic hz /uwb_tag                   # publish rate
ros2 topic info /uwb_tag --verbose       # type + QoS
```

If `ros2 topic list` shows the topic but `echo` blocks forever → QoS mismatch.

### 25.4 What you DON'T need

You will **not** be writing your own ROS2 nodes that other people consume. You're only consuming one or two topics that the venue's infrastructure publishes. No need to learn launch files, packages, colcon, or RViz. Keep your code Python-script-flat, not a ROS2 package.

### 25.5 Environment variables that bite

- **`ROS_DOMAIN_ID`** isolates ROS2 networks. Two teams on the same WiFi with `ROS_DOMAIN_ID=0` (default) will see each other's topics. The venue should set a per-team domain ID; export it in every shell you open.
- **`RMW_IMPLEMENTATION`** picks the DDS implementation. ROS2 Humble defaults to FastDDS; CycloneDDS is a drop-in alternative with better introspection (`ros2 topic info -v` shows accurate QoS under Cyclone, often `UNKNOWN` under FastDDS). Set consistently across **every** terminal — mixed implementations on the same `ROS_DOMAIN_ID` don't talk:

  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```

Official docs: [docs.ros.org/en/jazzy/](https://docs.ros.org/en/jazzy/).

---

## 26. The Finals control loop — putting it all together

Same structure as Qualifier (§13), but with different boxes:

```
┌─────────────────────────────────────────────────────────────┐
│                       BACKGROUND TASKS                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │ MAVSDK pos   │  │ RealSense    │  │ ROS2 UWB sub     │  │
│  │ telemetry    │  │ pipeline.    │  │ (rclpy.spin in   │  │
│  │ (yaw, alt)   │  │ wait_for_    │  │  daemon thread)  │  │
│  │              │  │ frames in    │  │                  │  │
│  │              │  │ worker       │  │                  │  │
│  └──────┬───────┘  └──────┬───────┘  └────────┬─────────┘  │
└─────────┼─────────────────┼───────────────────┼────────────┘
          ▼                 ▼                   ▼
    ┌────────────────────────────────────────────────────┐
    │              MAIN MISSION LOOP (10 Hz)              │
    │                                                     │
    │  1. uwb_n, uwb_e = get_uwb_position()              │
    │  2. yaw, alt     = get_attitude(), get_height()    │
    │  3. depth, color = pipeline.wait_for_frames()      │
    │  4. ids, corners = aruco.detectMarkers(color)      │
    │     → record marker_id + (x,y) in arena            │
    │     → top_down_grid.mark(occupied, distance)       │
    │  5. err = target_xy - (uwb_n, uwb_e)               │
    │  6. (vn, ve) = clamp(KP_XY * err, MAX_VEL_XY)      │
    │  7. set_velocity_ned(vn, ve, vd, locked_yaw)       │
    │  8. sleep 100 ms                                   │
    │                                                     │
    │  every N frames: rknn.inference(color) for         │
    │  RoboMaster detection (Stage 2)                    │
    └─────────────────────────────────────────────────────┘
```

Differences vs Qualifier:
- **Position source is UWB, not EKF.** Reliable and absolute.
- **Camera frames come from pyrealsense2, not gz-transport.** You actively pull frames, no callback.
- **Inference target hardware is the NPU, not CPU.** You batch differently and post-process manually.
- **Yaw is locked at takeoff.** Body frame and world frame stay aligned for the mission. The drone strafes; it does not turn.

---

## 27. Updated minimum viable competition entry

Same triage logic as Qualifier §15. Order of attack:

1. **NoMachine into mapping drone successfully.** (Phase 0; nothing else works without this.)
2. **Read UWB once and print it.** Confirms the venue's positioning system is live.
3. **Take off, hover for 5 s, land — using `kolomee.py` as-is.** Trust the reference.
4. **One-waypoint flight using UWB feedback.** Replace kolomee's hardcoded waypoints with `current + 1 m north`.
5. **RealSense frame capture + display.** Prove the camera works on the drone.
6. **Detect one ArUco marker on a printed sheet from the bench.** Confirm dictionary + camera.
7. **Stage 1 minimal path: fly a 3-waypoint lawnmower over the arena, log every ArUco ID + position.**
8. **Top-down depth map writer.** Save the deliverable Stage 1 wants.
9. **HULA discovery + single takeoff/land from C2.** Switch context to Stage 2.
10. **All three HULAs to predetermined landing pads.**
11. **RoboMaster detection** with the RKNN-converted YOLO.

Levels 1-5 are realistic in 2 days. Levels 6-8 are a third day. Levels 9-11 are full integration; reserve a half-week.

---

## 28. Updated glossary additions

- **ArUco marker**: a printed black-and-white square encoding an integer ID, detectable by `cv2.aruco`.
- **C2 Terminal**: the team's laptop (Windows + Ubuntu VM) running strategy code for Stage 2.
- **Dola**: broadcast discovery protocol for finding HULA drones on the WiFi.
- **HULA**: Highgreat HULA mini-quadcopter; the Stage-2 swarm drones.
- **kolomee.py**: the workshop's reference UWB-based navigation skeleton.
- **Mapping Drone**: the single Stage-1 drone with RealSense + NPU + UWB tag.
- **NoMachine**: a remote-desktop tool used to access the mapping drone's Linux box.
- **NPU**: Neural Processing Unit; the Rockchip SoC's hardware accelerator for YOLO.
- **pyhulax**: closed-source Python library to control HULA drones from C2.
- **pyrealsense2**: Intel's Python SDK for RealSense cameras.
- **QoS profile**: ROS2's reliability/durability/depth settings; UWB topics typically BEST_EFFORT.
- **RealSense**: Intel's consumer RGB-D camera family (D435i / D455 etc.).
- **RKNN**: Rockchip Neural Network — file format + runtime for NPU inference.
- **rknnlite**: the lightweight inference Python library used on the drone.
- **rknn-toolkit2**: the desktop conversion toolkit (ONNX → RKNN).
- **RoboMaster ground robots**: 5× ground robots Stage 2 detects.
- **Stage 1**: Reconnaissance (University-only). Mapping + ArUco identification.
- **Stage 2**: Deployment & Ambush. HULA swarm landing + convoy detection.
- **UWB**: Ultra-Wideband; the indoor positioning system replacing GPS.
- **UWB tag**: the small UWB transceiver mounted on the drone.

---

## 29. What to read next, in order

1. This document, Part 2 (you just did).
2. **`materials/RoboVerse 2026 Finals.pdf`** — re-read with new vocabulary.
3. **`references/finalist_codes/uwb_mavsdk/kolomee.py`** — line-by-line with `docs/kolomee_dissection.md` next to you.
4. **`references/finalist_codes/realsense_cam/getSyncDepthColor.py`** — RealSense pipeline pattern.
5. **`references/finalist_codes/aruco_detection/aruco_detection.py`** — ArUco snippet.
6. **`references/finalist_codes/rknn_detect/testrknn_with_display.py`** — end-to-end YOLO via NPU.
7. **`references/finalist_codes/hula_swarm/huladola.py`** — swarm discovery + per-drone control.

Then start working through `RUNBOOK.md` Phase 7 (Finals onwards).

*End of Part 2.*

