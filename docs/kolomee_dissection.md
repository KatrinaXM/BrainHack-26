# `kolomee.py` Dissection

> A line-by-line architectural read of the workshop's reference Mapping-Drone navigation script.
>
> File: `references/finalist_codes/uwb_mavsdk/kolomee.py` — 406 lines.
>
> Read with the script open in another pane. Section references look like `kolomee.py:115` = line 115.

---

## 0. What is kolomee.py?

It is the workshop's **smallest complete reference** for autonomous UWB-based navigation of the Mapping Drone. Specifically:

- Connects to PX4 over **serial** (real hardware, not UDP/SITL).
- Reads **UWB position** via a ROS2 subscriber to `/uwb_tag`.
- Reads **yaw + altitude** via MAVSDK telemetry.
- Sends **velocity setpoints** to PX4 offboard mode via MAVSDK.
- Flies a hardcoded mini-mission: takeoff → waypoint 1 → waypoint 2 → land.

It does *not* do: obstacle avoidance, path planning, vision, ArUco, RKNN, top-down mapping, multi-waypoint sequencing, dynamic targets, or anything HULA-related. Those are your code to add.

Think of it as the workshop's `Hello, World!` for the Finals — the smallest non-trivial program that proves the UWB+MAVSDK loop closes.

## 1. The big architecture in one picture

```
┌──────────────────────────────────────────────────────────────────────┐
│                              kolomee.py                              │
│                                                                      │
│   ┌─────────────────────┐         ┌──────────────────────────────┐   │
│   │  ROS2 daemon thread │         │   asyncio event loop (main)  │   │
│   │  (rclpy.spin)       │         │                              │   │
│   │                     │         │  ┌────────────────────────┐  │   │
│   │  UwbNode subscriber │         │  │ attitude_task          │  │   │
│   │  topic: /uwb_tag    │  ──┐    │  │ pos_task               │  │   │
│   │  type: PoseStamped  │    │    │  │ battery_task           │  │   │
│   │  cb writes n,e,     │    │    │  │   (MAVSDK telemetry)   │  │   │
│   │     ready globally  │    │    │  └────────────────────────┘  │   │
│   └─────────────────────┘    │    │                              │   │
│                              │    │  ┌────────────────────────┐  │   │
│   read by:                   ├──▶ │  │ run() — sequenced      │  │   │
│   get_uwb_position()  ◀──────┘    │  │  mission body:         │  │   │
│   (line 73)                       │  │   wait UWB → arm →     │  │   │
│                                   │  │   offboard.start() →   │  │   │
│   reads:                          │  │   fly_to_position×N    │  │   │
│   get_current_yaw_deg() ──────────┤  │   → land               │  │   │
│   get_current_height() ───────────┤  └────────────────────────┘  │   │
│                                   │                              │   │
│                                   │  control primitives:         │   │
│                                   │   fly_to_position_velocity() │   │
│                                   │   hover()                    │   │
│                                   │                              │   │
│                                   └──────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────┘
```

Two threads of execution. One (ROS2) sucks UWB messages from the venue's infrastructure and pokes them into module globals. The other (asyncio) reads those globals plus MAVSDK telemetry, computes velocity errors, and pushes setpoints back through MAVSDK to PX4. Glue is **shared module-level state**, not message queues.

`★ Insight ─────────────────────────────────────`
- The asyncio↔ROS2 boundary is one of the most painful integrations in modern robotics. Neither runtime wants to yield to the other, and `rclpy.spin()` blocks forever. The escape hatch used here — **run rclpy.spin in a daemon thread and read its state via plain Python globals** — is a recognised pattern. It works because individual float assignments are atomic under the CPython GIL, so no `Lock` is needed for `(n, e, ready)`.
- The alternative (`spin_once` in a callback inside asyncio) is more "correct" but trickier to implement and harder to debug. The daemon-thread approach trades a tiny amount of cleanliness for major simplicity.
`─────────────────────────────────────────────────`

---

## 2. Module configuration (`kolomee.py:1-46`)

```python
TAKEOFF_HEIGHT = 0.8         # metres above ground
KP_XY = 0.1                  # P-controller gain for north + east
KP_Z = 0.1                   # P-controller gain for down (altitude)

MAX_VEL_XY = 0.5             # m/s ceiling during navigation
MAX_VEL_Z = 0.3              # m/s ceiling for descent/climb

MAX_HOVER_XY = 0.15          # m/s ceiling while holding station
MAX_HOVER_Z = 0.10           # m/s ceiling for vertical hold

WAYPOINT_THRESHOLD = 0.20    # legacy / unused
N_THRESHOLD = 0.1            # m — "arrived" tolerance per axis
E_THRESHOLD = 0.1
D_THRESHOLD = 0.1
KP_SCALE = 0.2               # (DEAD — see §5.4)

HOVER_DEADBAND = 0.03        # m — ignore error below this when hovering
```

Two distinct gain/limit *profiles*:

| Mode | Speed cap | Deadband |
|---|---|---|
| Navigation (`fly_to_position_velocity`) | 0.5 m/s XY, 0.3 m/s Z | 0.1 m per axis |
| Hover (`hover`) | 0.15 m/s XY, 0.10 m/s Z | 0.03 m per axis |

**Why the split?** When you're transiting between waypoints you want speed. When you're holding station for a sensor read (ArUco capture, top-down depth frame), you want jitter rejection — the deadband filters UWB noise from becoming real motion. Different problems, different gains.

`★ Insight ─────────────────────────────────────`
- `KP_XY = 0.1` looks low until you realise UWB error is in *metres*. A 2 m error → 0.2 m/s commanded velocity, which is gentle. With a higher gain (e.g. 0.5), a 2 m error would saturate `MAX_VEL_XY` and the drone would race at full speed toward target, overshoot under UWB jitter, then race back — classic P-controller oscillation. **For noisy sensors, lower gains beat higher gains for stability.** The cost is slower convergence; that's usually acceptable.
- A safer / standard alternative is a tuned PID, but PID under UWB noise needs Kalman pre-filtering or you get derivative-kick spikes. Sticking to pure P keeps the code (and the mental model) simple.
`─────────────────────────────────────────────────`

---

## 3. UWB ingestion (`kolomee.py:51-91`)

```python
class UwbNode(Node):
    def __init__(self):
        super().__init__('uwb_listener_node')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscription = self.create_subscription(
            PoseStamped, 'uwb_tag', self.uwb_callback, qos)
        self.n = 0.0
        self.e = 0.0
        self.ready = False

    def uwb_callback(self, msg):
        self.n = msg.pose.position.y    # ◀── arena-north
        self.e = msg.pose.position.x    # ◀── arena-east
        self.ready = True
```

### Three things to notice

**1. `BEST_EFFORT` QoS.** UWB messages stream at ~10 Hz. If the network drops one, we'd rather get the *next* one than have the publisher retransmit a stale one. This pairs with the publisher's likely QoS — and as Tutorial §25.2 warns, **mismatched QoS means silent zero-message delivery**.

**2. The coordinate swap.** UWB hardware publishes:
```
msg.pose.position.x  =  east  (in many anchor calibrations)
msg.pose.position.y  =  north
```
…or sometimes the opposite. `kolomee.py:67-68` assumes the swap-to-NED is right. **Verify on hardware by walking the drone north and watching `self.n` increase, not `self.e`.** If wrong, swap the two lines.

**3. No `Lock`.** Two threads (ROS2 callback + asyncio loop) touch `self.n / self.e / self.ready`. Under CPython, single-float assignments are atomic, so this is safe *for individual reads*. It is **not** safe if you wanted "consistent snapshot of all three at once" — you could read `self.n` updated, then `self.e` not-yet-updated. For our gentle control loop this is invisible (sub-millimetre); for tighter loops, switch to a `threading.Lock` or a `Queue`.

```python
uwb_node = None    # module global, set by start_ros2_thread()

def get_uwb_position():
    if uwb_node is not None:
        return (uwb_node.n, uwb_node.e, uwb_node.ready)
    return (0.0, 0.0, False)

def start_ros2_thread():
    global uwb_node
    if not rclpy.ok():
        rclpy.init(args=None)
    uwb_node = UwbNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(uwb_node,), daemon=True)
    ros_thread.start()
    print("ROS2 UWB subscriber thread started.")
    return uwb_node
```

`daemon=True` is critical. A non-daemon thread would prevent Python from exiting because `rclpy.spin` runs forever; the daemon flag tells Python "kill this thread when main exits".

---

## 4. MAVSDK telemetry (`kolomee.py:101-117`)

Three small coroutines, each consuming one MAVSDK telemetry stream forever and writing to a module global:

```python
async def attitude_task(drone):
    global current_yaw
    async for attitude in drone.telemetry.attitude_euler():
        current_yaw = attitude.yaw_deg

async def battery_task(drone):
    global battery_remain
    async for battery in drone.telemetry.battery():
        battery_remain = battery.remaining_percent

async def pos_task(drone):
    global current_d
    global height_telemetry_ready
    async for pos in drone.telemetry.position_velocity_ned():
        current_d = pos.position.down_m
        height_telemetry_ready = True
```

Pattern parallels the UWB side: stream into globals, exposed via getters (`get_current_yaw_deg`, `get_current_height`). All three are kicked off as background asyncio tasks (`kolomee.py:141-143`):

```python
asyncio.create_task(attitude_task(drone))
asyncio.create_task(pos_task(drone))
asyncio.create_task(battery_task(drone))
```

`asyncio.create_task` schedules the coroutine on the loop and returns immediately. They run "concurrently" with the mission body. This is the Qualifier pattern from TUTORIAL.md §4 — async/await letting one task pause for I/O while others progress.

`★ Insight ─────────────────────────────────────`
- Note **`current_d`** uses PX4's `position.down_m` (NED frame, positive = below origin), *not* UWB. UWB gives x,y only; PX4 still handles altitude via barometer / optical-flow / VIO. So the mapping drone has two pose sources: **UWB for x,y, PX4 for z**. This split is why the controller treats horizontal and vertical separately throughout.
- Yaw also comes from PX4 (`attitude_euler`), not UWB. PX4's IMU integrates yaw rate and is accurate to within ~1°/min drift indoors. Good enough for our needs since we **never rotate** during the mission.
`─────────────────────────────────────────────────`

---

## 5. The control primitive: `fly_to_position_velocity` (`kolomee.py:154-241`)

This is the workhorse — the "fly from here to (target_n, target_e, target_d)" function. Every waypoint goes through it.

### 5.1 Signature

```python
async def fly_to_position_velocity(
    target_n, target_e, target_d,
    ignore_height=True,
    n_threshold=N_THRESHOLD, e_threshold=E_THRESHOLD, d_threshold=D_THRESHOLD,
    scale=KP_SCALE, test=False
):
```

`ignore_height=True` means "control x,y only, leave altitude to PX4's automatic hold". This is the default — Finals navigation usually doesn't change altitude per waypoint. When you do want to change altitude, pass `ignore_height=False` and set `target_d` (NED-down, negative for up).

`test=False` is a dry-run flag — if `True`, the function computes everything but doesn't actually send velocity commands. Useful for tuning gains offline.

### 5.2 The loop body (the heart of it)

```python
while True:
    current_n, current_e, state = get_uwb_position()
    current_d = get_current_height()

    if state == False:
        print("UWB data not ready, cannot navigate.")
        if test == False:
            send_velocity(0.0, 0.0, 0.0)        # ◀── BUG: missing await
        await asyncio.sleep(0.5)
        continue

    if height_telemetry_ready == False:
        if test == False:
            send_velocity(0.0, 0.0, 0.0)        # ◀── BUG: missing await
        print("height data not ready, cannot navigate.")
        continue                                 # ◀── BUG: no sleep, will CPU-spin

    err_n = target_n - current_n
    err_e = target_e - current_e
    err_d = target_d - current_d

    if ignore_height and abs(err_n) < n_threshold and abs(err_e) < e_threshold:
        await send_velocity(0.0, 0.0, 0.0)
        print("Waypoint reached")
        return

    vn = KP_XY * err_n if abs(err_n) >= n_threshold else 0.0
    ve = KP_XY * err_e if abs(err_e) >= e_threshold else 0.0
    vd = KP_Z  * err_d if abs(err_d) >= d_threshold else 0.0

    # Speed clamp (horizontal)
    horizontal_speed = math.sqrt(vn**2 + ve**2)
    if horizontal_speed > MAX_VEL_XY:
        scale = MAX_VEL_XY / horizontal_speed
        vn *= scale
        ve *= scale

    # Speed clamp (vertical) — clearer if written this way:
    vd = max(-MAX_VEL_Z, min(MAX_VEL_Z, vd))

    if ignore_height:
        vd = 0.0

    if test == False:
        await send_velocity(vn, ve, vd)
    await asyncio.sleep(0.1)
```

Loop runs at 10 Hz (`asyncio.sleep(0.1)`). The flow per tick:

1. Sample UWB → `current_n, current_e`.
2. Sample PX4 height → `current_d`.
3. Compute errors.
4. If within tolerance on all controlled axes → stop and return.
5. Otherwise P-control: velocity = gain × error, then saturate at max speed.
6. Send velocity to PX4 via `send_velocity()`.

### 5.3 `send_velocity` (`kolomee.py:150-152`)

```python
async def send_velocity(vn, ve, vd):
    global takeoff_yaw
    await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, takeoff_yaw))
```

Every velocity setpoint includes the **locked takeoff yaw**. PX4 holds that heading throughout the flight. This is the "drone strafes, doesn't turn" architecture.

### 5.4 Bugs and code smells worth knowing

These are **real bugs** in the workshop's reference. If you're using kolomee as your skeleton, fix them:

| Line | Issue | Severity | Fix |
|---|---|---|---|
| `:168` | `send_velocity(0.0, 0.0, 0.0)` missing `await` | Medium — silently does nothing in the UWB-not-ready branch | Add `await` |
| `:174` | Same — UWB OK but height not ready branch | Medium | Add `await` |
| `:176` | `continue` without `await asyncio.sleep(...)` before it | Low (would spin only if height stayed broken forever) | Add `await asyncio.sleep(0.5)` before `continue` |
| `:36, :154, :218` | `KP_SCALE` declared and passed as parameter, then immediately overwritten on `:218` | Cosmetic — confusing dead code | Delete the parameter and the constant |
| Everywhere | Pure P-control, no derivative term | Design choice — see §2 insight | Accept it; PID needs Kalman pre-filtering for UWB |
| `:228` | Vertical saturation written as nested `if/else` with three branches when one `min/max` would do | Cosmetic | Replace with `vd = max(-MAX_VEL_Z, min(MAX_VEL_Z, vd))` |

`★ Insight ─────────────────────────────────────`
- The missing-`await` bug is **the** classic asyncio mistake. Without `await`, the coroutine is created but never run; Python doesn't even warn (sometimes). The function silently no-ops. In kolomee's failure-path code, this means **the drone keeps moving with its last commanded velocity when UWB drops**, rather than stopping like the comment claims. This is a safety issue — fix before flying.
- A defensive pattern many teams use: enable Python's `asyncio` debug warnings via `asyncio.run(run(), debug=True)`. It logs unawaited coroutines.
`─────────────────────────────────────────────────`

---

## 6. The other control primitive: `hover` (`kolomee.py:243-296`)

Structurally identical to `fly_to_position_velocity` but:

| Aspect | fly_to_position_velocity | hover |
|---|---|---|
| Target | a user-given waypoint | the position at function entry |
| Exit condition | within `*_THRESHOLD` of target | `seconds` elapsed |
| Speed cap | `MAX_VEL_XY = 0.5` | `MAX_HOVER_XY = 0.15` |
| Deadband | per-axis `*_THRESHOLD = 0.1` | `HOVER_DEADBAND = 0.03` |

```python
async def hover(seconds, ignore_height=False):
    hover_n, hover_e, state = get_uwb_position()
    hover_d = get_current_height()
    print(F"Hover lock: at N={hover_n:.2f} E={hover_e:.2f} D={hover_d:.2f}")
    end_time = asyncio.get_running_loop().time() + seconds

    while asyncio.get_running_loop().time() < end_time:
        # ... same P-control loop as above, but using MAX_HOVER_XY and HOVER_DEADBAND
```

You use `hover()` whenever you need a *steady* drone — capturing a top-down depth frame, scanning ArUco markers, taking RoboMaster snapshots. The tighter deadband suppresses UWB jitter from becoming visible motion. The slower speed cap means any correction is gentle.

`★ Insight ─────────────────────────────────────`
- Note the use of `asyncio.get_running_loop().time()` instead of `time.time()`. This is **monotonic** — it doesn't get rewound by NTP clock corrections mid-flight. For a 2-second hover this matters little; for a 5-minute mission, it can save you from spurious "negative elapsed time" bugs if the system clock is corrected during flight.
- The hover function ignores the `state` return from `get_uwb_position()` (line 257). If UWB drops mid-hover, the drone will P-control toward `(hover_n, hover_e)` using the **last-known UWB value** (the daemon thread keeps the globals current). That's actually fine — the drone holds its last known position rather than zeroing velocity and drifting. But it does mean a stale UWB reading could keep the drone fighting an imaginary error.
`─────────────────────────────────────────────────`

---

## 7. The mission sequence (`kolomee.py:121-380`)

The actual flight choreography lives in `async def run()`. Linear top-to-bottom:

### 7.1 Wait for sensors (`:125-148`)

```python
start_ros2_thread()                          # bring up UWB subscriber
await asyncio.sleep(1.0)                     # let it settle

n, e, state = get_uwb_position()
while state == False:                        # block until first UWB message
    print("Waiting for UWB data...")
    await asyncio.sleep(0.5)
    uwb_pos = get_uwb_position()             # (assigned but unused; another minor bug)

drone = System()
await drone.connect(system_address="serial:///dev/ttyS6:921600")
asyncio.create_task(attitude_task(drone))
asyncio.create_task(pos_task(drone))
asyncio.create_task(battery_task(drone))

async for health in drone.telemetry.health():
    if health.is_local_position_ok:
        break
```

Pattern: spin up sensors, then *await* their first valid reading before progressing.

### 7.2 Pre-arm interaction (`:300-328`)

```python
home_n, home_e, state = get_uwb_position()
takeoff_yaw = get_current_yaw_deg()           # ◀── lock yaw for the whole mission
print(f"Takeoff yaw locked at {takeoff_yaw:.1f} deg")

target_altitude_m = TAKEOFF_HEIGHT
await drone.action.set_takeoff_altitude(target_altitude_m)
await asyncio.sleep(1.0)

user_input = await loop.run_in_executor(None, input, "Do you want to proceed? (y/n): ")
choice = user_input.strip().lower()
if choice in ['y', 'yes']:  print("Proceeding...")
elif choice in ['n', 'no']: sys.exit(0)
```

`loop.run_in_executor(None, input, "…")` is the asyncio-safe way to read stdin without blocking the event loop. **The manual `y/n` is intentional safety** — kolomee assumes a human is watching before any real-world flight. If you adapt this for autonomous Finals scoring, remove this prompt and replace with a 5-second LED countdown or similar.

### 7.3 Arming + offboard warmup (`:329-341`)

```python
print("Arming...")
await drone.action.arm()

print("PX4 - Sending initial velocity setpoints...")
for _ in range(20):
    await send_velocity(0.0, 0.0, 0.0)
    await asyncio.sleep(0.1)

print("Starting Offboard...")
await drone.offboard.start()
```

The 20 × 100 ms = 2 seconds of zero-velocity setpoints **before** `offboard.start()` is **mandatory**. As TUTORIAL.md §5 explains: PX4 refuses to enter offboard mode without a fresh setpoint in its buffer. Send a few before flipping the switch; otherwise `offboard.start()` raises with "no setpoint".

### 7.4 Waypoint execution (`:343-355`)

```python
print("WAYPOINT 1...")
current_n, current_e, state = get_uwb_position()
await fly_to_position_velocity(target_n=current_n+1.0, target_e=current_e, target_d=-1.5,
                                ignore_height=True, test=False)

print("WAYPOINT 2...")
current_n, current_e, state = get_uwb_position()
await fly_to_position_velocity(target_n=current_n, target_e=current_e-1.0, target_d=-1.5,
                                ignore_height=True, test=False)
```

Both waypoints are **relative to wherever the drone currently is**, not absolute. That's an artefact of the reference being a "fly a square" demo. For your Stage 1 mission you'll pass **absolute arena coordinates** instead:

```python
LAWNMOWER_WAYPOINTS = [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (1.0, 2.0), (1.0, 1.0), (1.0, 0.0), ...]
for n, e in LAWNMOWER_WAYPOINTS:
    await fly_to_position_velocity(target_n=n, target_e=e, target_d=-1.5, ignore_height=True)
    await hover(2.0)    # let your ArUco / depth thread capture
```

### 7.5 Landing + cleanup (`:357-404`)

```python
try:
    await drone.offboard.stop()
except Exception as e:
    print(f"Offboard stop error: {e}")

await drone.action.land()
async for in_air in drone.telemetry.in_air():
    if not in_air:
        break
    await asyncio.sleep(0.5)

try:
    await drone.action.disarm()
except Exception:
    pass
```

Two patterns to copy:

1. **Exception swallowing around `offboard.stop()` and `disarm()`**: these can race against PX4's own internal state machine. If PX4 has already exited offboard (e.g. because it triggered a land due to lost heartbeat), `offboard.stop()` raises a benign error. Logging it but continuing is the right move.
2. **`async for in_air in drone.telemetry.in_air()`**: waits until PX4 reports "wheels on ground". Land is asynchronous — `await drone.action.land()` returns immediately after PX4 *accepts* the command, not after touchdown. You must poll for actual landing.

### 7.6 Outer error handling (`:382-404`)

```python
except Exception as e:
    print(f"Exception: {e}")
    try: await send_velocity(0.0, 0.0, 0.0)
    except Exception: pass
    try: await drone.offboard.stop()
    except Exception: pass
    try: await drone.action.land()
    except Exception: pass
finally:
    try:
        if uwb_node is not None:
            uwb_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as e:
        print(f"ROS2 shutdown error: {e}")
```

The defensive pattern: on any failure, **stop motion → exit offboard → land → shut down ROS2 cleanly**. Each step wrapped in its own try/except so a failure in one doesn't prevent the others. This is mandatory for safety — a `KeyboardInterrupt` at the wrong time without these guards leaves the drone hovering with no controller, eventually triggering PX4's failsafe.

`★ Insight ─────────────────────────────────────`
- The `try/except/pass` pattern repeated four times in the cleanup is sometimes called "**defensive cleanup**" or "**bulletproof exit**". It's intentionally permissive — we'd rather log a benign error than abort the rest of cleanup. A purist might extract a helper `safe_call(coro)` that wraps each step; for a 406-line reference script, inlining is fine.
- `rclpy.shutdown()` is idempotent but `destroy_node()` is *not*. Calling `destroy_node()` twice raises. The guard `if uwb_node is not None` ensures we only destroy once.
`─────────────────────────────────────────────────`

---

## 8. What kolomee.py does NOT do

For Finals Stage 1 you will need to add:

| Capability | Where to add |
|---|---|
| **ArUco detection on RealSense frames** | New worker thread, runs concurrent with the asyncio loop, logs to a shared dict guarded by a lock. Triggered by `hover()` so the drone is steady during capture. |
| **Top-down depth map writer** | Same worker thread; on each `hover()` it grabs a depth frame, projects to world coords, and writes occupancy. |
| **Lawnmower waypoint generator** | Pre-flight: compute a `(n,e)` list covering the arena. Replace the hardcoded waypoints in §7.4. |
| **Per-marker pose logging** | When ArUco detector fires, store `(marker_id, current_n, current_e, depth)` in a JSON file. |
| **Mission-end summary** | After landing, post-process the ArUco log to deduplicate (same physical marker seen multiple times) and emit the deliverable JSON. |

For Finals Stage 2 the architecture splits — you're not using kolomee at all; you're using `pyhulax` on the C2 Terminal.

## 9. The tunable-parameters table (when something goes wrong)

| Symptom | Likely parameter to tune | Direction |
|---|---|---|
| Drone overshoots waypoint and oscillates | `KP_XY` | Lower (0.05) |
| Drone reaches waypoint too slowly | `KP_XY` | Higher (0.15) — but verify UWB isn't already saturating `MAX_VEL_XY` first |
| Drone "vibrates" while hovering | `HOVER_DEADBAND` | Higher (0.06) |
| Drone drifts away during hover | `HOVER_DEADBAND` | Lower (0.02) |
| Drone refuses to declare "waypoint reached" | `N_THRESHOLD` / `E_THRESHOLD` | Higher (0.2) |
| Drone overshoots, declaring victory mid-overshoot | `N_THRESHOLD` / `E_THRESHOLD` | Lower (0.05) |
| Drone too slow during transit | `MAX_VEL_XY` | Higher (0.8) — within drone's actual capability |
| Altitude wobble | `MAX_VEL_Z` | Lower (0.15), or pass `ignore_height=True` always |

`★ Insight ─────────────────────────────────────`
- A common subtle failure: gains tuned beautifully on the bench, drone wobbles violently at the venue. The culprit is usually **UWB noise level changing** (more anchors / cleaner LOS at home, more multipath at the venue). Re-tune at the venue, not on the laptop.
- Always tune **bottom-up**: get hover steady first (tight deadbands suppress noise), then nav (looser tolerances + speed caps). Tuning nav before hover means you can't tell which behaviour is at fault when something misbehaves.
`─────────────────────────────────────────────────`

---

## 10. How we'll extend kolomee.py for Stage 1

The shape of `stage1_mission.py` will be something like:

```python
import asyncio
# Reuse kolomee's primitives:
from kolomee import (
    start_ros2_thread, get_uwb_position, get_current_yaw_deg, get_current_height,
    # plus we'll factor out fly_to_position_velocity and hover into a kolomee_lib module
)
from realsense_pipeline import RealSensePipeline        # to be written
from aruco_logger        import ArucoLogger              # to be written
from topdown_writer      import TopDownWriter            # to be written

LAWNMOWER_WAYPOINTS = [...]    # generated from arena dimensions

async def run_stage1():
    start_ros2_thread()
    rs    = RealSensePipeline()
    aruco = ArucoLogger(rs, output_path="aruco_log.json")
    topdn = TopDownWriter(rs, output_path="topdown.png")
    aruco.start()
    topdn.start()

    # ... kolomee-style telemetry init + arming + offboard warmup ...

    for n, e in LAWNMOWER_WAYPOINTS:
        await fly_to_position_velocity(n, e, -1.5, ignore_height=True)
        await hover(2.0)                # ArucoLogger captures while steady

    # ... kolomee-style landing + cleanup ...

    aruco.stop(); topdn.stop()
    aruco.write(); topdn.write()
```

Three new modules. Each gets its own follow-up dissection when we build it.

---

*End of dissection. Read kolomee.py once more end-to-end with this beside you — by the end you should be able to mentally execute the script's first 30 seconds (sensor init → arm → offboard warmup) without looking at the code.*
