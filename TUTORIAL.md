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

*End of tutorial. If a concept still feels fuzzy after reading the relevant section twice, send the exact sentence that confuses you and I'll re-explain it with a different analogy.*
