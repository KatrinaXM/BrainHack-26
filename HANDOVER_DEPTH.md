# Handover — Depth Camera Diagnostic + Custom YOLO Wired In

**Date:** 2026-05-23
**Context:** BrainHack-26 RoboVerse Qualifier. The drone takes off cleanly in SITL but cannot explore the world because the avoidance planner sees `L=nan C=nan R=nan` (NaN clearances). After several diagnostic flights, root cause is **depth camera returning all-`+inf`** at every pose tested. This handover captures the full investigation, the suspected root cause, and a step-by-step plan for the next person (or Claude session) to close the loop.

---

## Section 1 — TL;DR

1. **Custom YOLO model is wired and verified.** `barrel_yolo.pt` (5.5 MB, mAP50=0.995) loads with correct class indices `{0: fuel_barrel_yellow, 1: fuel_barrel_red}`. Smoke test passed. Path is set in `mission_config.py`.

2. **The blocker is depth, not YOLO and not exploration strategy.** Depth returns `307200 / 307200 px = +inf` at every (N, E, yaw) tested, on a fresh sim, both pre-flight and airborne. The visited-grid exploration logic is fine — it's getting starved of valid depth.

3. **Hypothesis (strongly supported by web + local evidence, not yet proven):** Gazebo's `depth_camera` sensor cannot raycast against `base6.glb` (the GLB-format spaceport mesh), while the RGB camera renders it fine. This is a documented class of Gazebo rendering-pipeline divergence.

4. **The one test that would prove it** — drop a primitive `<box>` model into `roboverse.sdf`, fly the drone to face it, run `depth_diag.py`. Detailed instructions in Section 7. **This test was about to run but was interrupted.**

5. **If hypothesis is confirmed**, the fix is either: convert `base6.glb` → `.stl`/`.dae`, or rebuild the world from primitive `<box>` geometry. Either is a 1-2 hour task.

6. **All other "issues" we tracked may be downstream symptoms of this one** — SITL VIO blowups, low coverage, 0 barrel detections. The drone flying into walls is what happens when the avoidance planner thinks everywhere is clear.

---

## Section 2 — Files I touched this session

### Files added (kept on disk)

| Path | Purpose | Status |
|---|---|---|
| `/home/drone/Desktop/codes/barrel_yolo.pt` | Custom-trained YOLO weights, 5.5 MB | Ready to use |
| `/home/drone/Desktop/codes/depth_diag.py` | 5-station flight + depth probe diagnostic | Ready to run |
| `/home/drone/BrainHack-26/HANDOVER_DEPTH.md` | This document | Persistent |
| `/home/drone/.claude/projects/-home-drone-BrainHack-26/memory/project_depth_world_issues.md` | Memory: depth all-inf finding | Saved |
| `/home/drone/.claude/projects/-home-drone-BrainHack-26/memory/project_protobuf_workaround.md` | Memory: protobuf ABI workaround | Saved |
| `/home/drone/.claude/projects/-home-drone-BrainHack-26/memory/project_lawnmower_allowed.md` | Memory: lawnmower research outcome | Saved |

### Files modified (kept)

| Path | Change | Why |
|---|---|---|
| `/home/drone/Desktop/codes/mission_config.py` | `YOLO_MODEL_PATH` → `barrel_yolo.pt`; `BAILOUT_SECONDS` 480 → 90 | Wire in custom model; cap verification flights short |
| `/home/drone/.claude/projects/-home-drone-BrainHack-26/memory/MEMORY.md` | Added 3 new entries to index | Index sync |

### Files modified then reverted (no net change)

| Path | Temporary change | Final state |
|---|---|---|
| `/home/drone/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse.sdf` | Added `depth_test_box` model — never flown | Reverted to original (backup `roboverse.sdf.bak` was applied) |

### Files I did not modify (worth knowing about)

- `/home/drone/PX4-Autopilot/Tools/simulation/gz/models/x500_vision/model.sdf` — **was hand-edited before this session** to add OakD-Lite include. See Section 4.
- `/home/drone/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4005_gz_x500_vision` — **was hand-edited before this session** to enable EKF2 external-vision fusion. See Section 4.

---

## Section 3 — How I was launching the sim

Using the existing scripts in `~/`:

```bash
# 1. Always clean any running sim first (memory: "restart between scripts")
cd /home/drone && ./stop_sim.sh

# 2. Launch sim (non-interactive, tmux session named 'px4', skips QGroundControl)
./start_sim.sh --no-qgc

# 3. Wait for "Startup script returned successfully" in /tmp/px4_sim.log
for i in $(seq 1 30); do
  sleep 2
  if grep -q "Startup script returned successfully" /tmp/px4_sim.log; then break; fi
done

# 4. Set EKF origin via tmux (required — VIO needs an origin before is_armable goes true)
tmux send-keys -t px4 'commander set_ekf_origin 47.397742 8.545594 488.0' Enter

# 5. Run mission / diagnostic with the protobuf env-var workaround (see Section 5)
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 mission.py
```

If your team's preferred launch is `start_px4.sh` (interactive menu) instead, the in-sim behaviour should be identical — both end up calling `PX4_GZ_WORLD=roboverse make px4_sitl gz_x500_vision`. The only difference is automation friendliness.

**Important:** after every flight (takeoff and land/abort), restart the sim before the next run. The saved memory note `restart-between-scripts` was confirmed multiple times this session — the drone ends up in an invalid pose if you skip this.

---

## Section 4 — The local edits to PX4 (predate this session)

The `x500_vision` setup on this VM is **not stock PX4**. Two files were hand-modified before this session began:

### `Tools/simulation/gz/models/x500_vision/model.sdf`

Upstream PX4's `x500_vision` is just the base x500 + an odometry-publisher plugin. Someone added an OakD-Lite include for the depth+RGB camera:

```xml
<include merge='true'>
  <uri>model://OakD-Lite</uri>
  <pose>.12 .03 .242 0 0 0</pose>
</include>
<joint name="CameraJoint" type="fixed">
  <parent>base_link</parent>
  <child>camera_link</child>
  <pose relative_to="base_link">.12 .03 .242 0 0 0</pose>
</joint>
```

This graft is **parseable** — the SDF loads without errors. But it's also likely the source of the depth issue (Section 6).

### `ROMFS/px4fmu_common/init.d-posix/airframes/4005_gz_x500_vision`

Modified to enable EKF2 external-vision fusion:

```bash
param set-default EKF2_EV_CTRL 3      # enable EV fusion for position + yaw
param set-default EKF2_HGT_REF 3       # vision as primary height source
param set-default EKF2_GPS_CTRL 0      # disable GPS, vision only
```

This is necessary because the qualifier is GNSS-denied. Without these, EKF defaults to GPS and won't fuse VIO.

**These edits are intentional and load-bearing. Do NOT revert them.** They are how `x500_vision` becomes the GNSS-denied competition platform.

---

## Section 5 — The protobuf workaround

Mission scripts that import `gz.msgs10` crash with:
```
TypeError: Descriptors cannot be created directly.
If this call came from a _pb2.py file, your generated code is out of date and must be regenerated with protoc >= 3.19.0.
```

Cause: a pip install (likely torch/ultralytics) upgraded `~/.local/lib/.../google/protobuf` to a version that rejects the C++-descriptor protos generated for `gz.msgs10`. `~/.local/lib/python3.10/site-packages` takes priority over system Python, so a single pip install silently shadows `/usr/lib/python3/dist-packages` protobuf.

Workaround: prefix every script that imports `gz.msgs10` with:

```bash
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 mission.py
```

The pure-Python parser is slower but works. Permanent fixes (downgrading protobuf, isolating envs) would likely break ultralytics — env-var is the safest path.

---

## Section 6 — Why depth is broken (full investigation)

### What I ran (chronological)

1. **Verification flight #1** (with custom YOLO wired in, `mission.py`, 90s cap):
   - Drone armed and took off normally
   - Coverage stuck at 3.3% — drone circling spawn
   - 60 RGB images saved (proof RGB works)
   - 0 barrel detections
   - Console: `L=nan C=nan R=nan` from t=0 to t=90s
   - First clue: depth never gave valid data, even at the very start

2. **Raw depth probe (mid-flight)**:
   ```python
   d = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
   print(f"finite={np.isfinite(d).sum()}/{d.size}")
   ```
   - Result: `finite=0/307200` — every pixel inf
   - Discounted because sim may have been in post-flight broken state

3. **Sim restart + pre-flight depth probe**: same result, `finite=0/307200`. Confirms depth is broken before any flight, not just after VIO blowup.

4. **Lidar probe**: x500_vision SDF declares `lidar` and `lidar_2d_v2` sensors. Both topics exist in `gz topic -l`. Both produce zero LaserScan messages.

5. **Topic provenance check**: `/depth_camera` IS published (TCP port 34945, type `gz.msgs.Image`, format `R_FLOAT32`). So a renderer IS running and producing output — it just outputs all-inf.

6. **SDF audit**:
   - `x500_vision/model.sdf` includes OakD-Lite at pose `(0.12, 0.03, 0.242)` (forward, slightly right, above base_link)
   - OakD-Lite/model.sdf declares `StereoOV7251` depth sensor with `<topic>depth_camera</topic>`, clip range 0.2–19.1 m, format R_FLOAT32
   - World plugin `gz-sim-sensors-system` is loaded in roboverse.sdf (line 22)
   - Only benign `gz_frame_id` warnings at startup — sensor loads cleanly

7. **Depth diagnostic flight** (`depth_diag.py`, 5 stations):
   - Stations: (0,0) yaw=0°, (0,0) yaw=90°, (0,0) yaw=180°, (3,0) yaw=0°, (0,3) yaw=90°
   - **All five returned `307200/307200 = +inf`**
   - Pose-independent → not a "drone is looking at sky" issue

8. **Reference-code audit** (`/home/drone/BrainHack-26/references/codes/*`):
   - Spawned an Explore agent to read every avoidance file
   - Finding: reference code expects depth to work but has **zero defense against all-inf**
   - `AvoidancePlanner.compute_clearance` returns `(inf, inf, inf)` → `detect_blocked` returns False → drone flies forward at max speed
   - This matches your existing "SITL VIO blowup" memory: drone flies into wall → VIO corrupts → millions of metres

9. **Web research** (agent did this in parallel): multiple confirmed Gazebo bugs where camera sensors don't see meshes that the GUI renders fine. Particularly common with `.glb` via `file://`. Issues cited:
   - https://github.com/ignitionrobotics/ign-rendering/issues/101
   - https://github.com/gazebosim/gz-sim/issues/1435
   - https://github.com/gazebosim/gz-rendering/issues/968
   - https://github.com/PX4/PX4-Autopilot/issues/25992

10. **RUNBOOK check**: gate 1.5 explicitly requires "Live depth image visible, changes as you fly the drone around" (lines 131-148). Either skipped during setup or world has regressed.

### Diagnostic verdict

Depth is broken **at the sensor-scene level**, not at the topic plumbing level. The most likely cause is that `base6.glb` is invisible to the depth-camera sensor scene. Confirmation requires one final test (Section 7).

---

## Section 7 — The verification test that would close the loop

**Goal:** drop a primitive `<box>` into roboverse.sdf at a position where the drone can see it, then check if depth returns finite values for the box specifically.

### Step-by-step

```bash
# 1. Backup the world
cd /home/drone/PX4-Autopilot/Tools/simulation/gz/worlds
cp roboverse.sdf roboverse.sdf.bak

# 2. Edit roboverse.sdf — insert the block below BEFORE the <spherical_coordinates> tag
```

Insert this block:

```xml
<model name='depth_test_box'>
  <static>true</static>
  <pose>2 0 1.0 0 0 0</pose>
  <link name='link'>
    <visual name='visual'>
      <geometry><box><size>1 1 1</size></box></geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
    <collision name='collision'>
      <geometry><box><size>1 1 1</size></box></geometry>
    </collision>
  </link>
</model>
```

Then:

```bash
# 3. Launch sim (see Section 3 for the full launch sequence)
cd /home/drone
./stop_sim.sh
./start_sim.sh --no-qgc
# wait for "Startup script returned successfully"
tmux send-keys -t px4 'commander set_ekf_origin 47.397742 8.545594 488.0' Enter

# 4. Run the diagnostic — it already flies to (3, 0) yaw=0 which faces the box
cd /home/drone/Desktop/codes
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 depth_diag.py

# 5. After the flight ends, restore the world
cd /home/drone/PX4-Autopilot/Tools/simulation/gz/worlds
mv roboverse.sdf.bak roboverse.sdf

# 6. ALWAYS stop sim before the next test
cd /home/drone && ./stop_sim.sh
```

### Reading the result

Station 4 (`N=3.0 E=0.0 yaw=0.0°`) is the key one — drone hovers at 1.5 m altitude, faces north, the box is 2 m ahead at z=1.0 (centered 0.5 m below drone altitude, with top at z=1.5).

| Diagnostic output for station 4 | Interpretation | Next step |
|---|---|---|
| `FINITE px=N/307200` where N > 0 | Box detected → depth scene works for primitives. GLB-specific hypothesis confirmed. | Convert `base6.glb` to `.stl` or `.dae`. See Section 8. |
| `all-inf (307200 px)` | Even primitives invisible to depth → sensor itself is broken in this graft | Switch to upstream `x500_depth` model, or replace depth with lidar/RGB. See Section 8. |

---

## Section 8 — Suggested next steps in priority order

### Priority 1 — Confirm root cause (10 min)

Run Section 7's box test. **Do this before anything else.** The fix branches dramatically depending on outcome.

### Priority 2A — IF GLB-specific (most likely)

The fix tree, easiest first:

1. **Try `.stl` conversion** (highest reliability for Gazebo sensor scenes):
   - Open `/home/drone/worlds/groundmodel/meshes/base6.glb` in Blender (`apt install blender` if not present)
   - File → Export → Stl (.stl), name it `base6.stl`, save to same directory
   - Edit `roboverse.sdf`: change both `<uri>file:///home/drone/worlds/groundmodel/meshes/base6.glb</uri>` lines (the `<visual>` one AND the `<collision>` one) to point at `base6.stl`
   - Restart sim, run `depth_diag.py` from clean state
   - If depth returns finite values around the spaceport geometry, you're done

2. **If `.stl` doesn't work, try `.dae`** (Collada):
   - Same Blender export but as Collada → `base6.dae`
   - Update roboverse.sdf URIs
   - Test

3. **If neither works, rebuild the world from primitives** as a stopgap:
   - Create a new world file with `<box>` walls forming a 40×40×8m enclosure
   - Add `<cylinder>` barrels at known positions (since we know what to detect, primitive geometry suffices for development)
   - This is a 1-2 hour task but unblocks all downstream work
   - File to create: `/home/drone/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse_primitives.sdf`
   - Launch with `WORLD=roboverse_primitives ./start_sim.sh --no-qgc` (need to update start_sim.sh or pass `--world=roboverse_primitives`)

### Priority 2B — IF depth sensor broken globally

1. **Try upstream `x500_depth` model**:
   - This contradicts the saved memory `[[drone-model-decision]]` that says x500_vision is canonical per slides
   - BUT a broken canonical is worse than a working alternate
   - Launch: `./start_sim.sh --model=x500_depth --no-qgc`
   - Run `depth_diag.py` — if depth returns finite values, switch the team to x500_depth
   - Verify the EKF2 EV settings still apply (they're set in `4005_gz_x500_vision`; you may need to copy them into `4002_gz_x500_depth`)

2. **Try lidar-only avoidance** (alternative architecture):
   - Both lidar topics also returned no data in my probe — likely shares root cause
   - But if the box test reveals the lidar's geometry-load issue is fixable with the same `.stl` conversion, lidar becomes viable
   - The `lidar_2d_v2/scan` is the cleaner indoor avoidance signal
   - Would need to rewrite `AvoidancePlanner.py` to consume LaserScan instead of depth Image — substantial change

3. **Try YOLO-based pseudo-depth** (last resort):
   - Use YOLO bounding box size as a crude distance proxy
   - Treat detected barrels as the only obstacles
   - Won't avoid walls or other geometry — only the things YOLO detects
   - Major rewrite of avoidance code

### Priority 3 — Validation flight (after depth works)

1. Edit `mission_config.py`: set `BAILOUT_SECONDS = 480` (back to 8 min full mission)
2. Clean stale outputs: `rm -f /mnt/hgfs/Shared/barrels.json && rm -rf /mnt/hgfs/Shared/yolo_detections`
3. Run a full mission. Watch for:
   - `[mission] t=... L=X.XX C=Y.YY R=Z.ZZ` with finite numbers — proves depth works
   - `[detector] loaded /home/drone/Desktop/codes/barrel_yolo.pt` — proves model loaded
   - `[barrel] new fuel_barrel_yellow @ (...)` — proves detection → NED projection pipeline works
   - Final line `[mission] barrel detections: total=N by_class={...}` — proves dedup works
4. Inspect `/mnt/hgfs/Shared/barrels.json` and a few annotated images from `yolo_detections/` to sanity-check positions and confidence

### Priority 4 — Strategy comparison (after we have a working baseline)

Per `[[lawnmower-allowed]]`, both `visited_grid` (current) and `lawnmower` are valid exploration strategies per the rules:
- Add an `EXPLORATION_MODE = "lawnmower"` mode to mission.py
- Lawnmower waypoint list driving the same `AvoidancePlanner.compute_position_ned_with_goal()`
- Run both strategies for 480 s, compare `barrels.json` total scores
- Keep the better one for the competition

---

## Section 9 — Files to create when the next person picks this up

Recommend creating these as work progresses:

| File path | Why | When to create |
|---|---|---|
| `/home/drone/BrainHack-26/HANDOVER_DEPTH_RESULT.md` | Box test outcome — finite or inf, and what was tried next | Right after Section 7's box test |
| `/home/drone/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse_primitives.sdf` | Primitive-geometry backup world | If priority 2A path 3 is chosen |
| `/home/drone/Desktop/codes/lawnmower.py` | Lawnmower exploration mode | Priority 4 |
| `/home/drone/Desktop/codes/lidar_receiver.py` | LaserScan subscriber | If priority 2B path 2 is chosen |
| `/home/drone/Desktop/codes/AvoidancePlanner_lidar.py` | Lidar-based avoidance | If priority 2B path 2 is chosen |
| `/home/drone/.claude/projects/-home-drone-BrainHack-26/memory/project_<root-cause>.md` | Save the confirmed root cause as memory once known | After box test |

---

## Section 10 — Open questions I couldn't answer

1. **Did depth ever work in this setup?** No mission log I found shows finite depth values. The pattern is consistent with "all-inf since the OakD-Lite was grafted onto x500_vision," but I can't prove it.
2. **Why are both lidars also silent?** May share root cause with depth, may be unrelated. Worth a 5-min check after depth is fixed: probe `LaserScan` again and see if those topics start producing data once `.stl` conversion is in place.
3. **What does the team's training-images session look like?** The user has 400+ screenshots showing barrels in the world. Those barrels are visible in RGB. Whether they're separate `<model>` entries or baked into base6.glb is still unclear — `grep "fuel_barrel" *.sdf` returns nothing, so probably baked. If baked, converting `.glb` → `.stl` should preserve them.
4. **Is the OakD-Lite include pose physically correct?** `(0.12, 0.03, 0.242)` puts the camera ahead of and above the drone center. Looks reasonable but never visually verified.

---

## Section 11 — Saved memory pointers

In `/home/drone/.claude/projects/-home-drone-BrainHack-26/memory/`:

- `MEMORY.md` — index, read first
- `project_depth_world_issues.md` — depth all-inf finding (this session)
- `project_protobuf_workaround.md` — protobuf env-var workaround (this session)
- `project_lawnmower_allowed.md` — lawnmower research outcome (this session)
- `project_sitl_vio_blowup.md` — VIO blowup pattern. **Now likely understood as a downstream symptom** of all-inf depth → drone flies into wall.
- `project_drone_model.md` — x500_vision canonical per slides. May need revisiting if path 2B is taken.
- `feedback_restart_between_scripts.md` — confirmed critical this session
- `project_ekf_settle_gotcha.md` — confirmed: set EKF origin then wait for is_armable

---

## Section 12 — A 60-second sanity check before doing anything

Before any new work, run this to confirm state matches this document:

```bash
# 1. World file untouched
md5sum /home/drone/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse.sdf
# expect 4963 bytes

# 2. Custom YOLO is in place
ls -la /home/drone/Desktop/codes/barrel_yolo.pt
# expect 5758970 bytes

# 3. Mission config is updated
grep -E "YOLO_MODEL_PATH|BAILOUT_SECONDS" /home/drone/Desktop/codes/mission_config.py
# expect YOLO_MODEL_PATH = ".../barrel_yolo.pt" and BAILOUT_SECONDS = 90

# 4. Diagnostic script is in place
ls -la /home/drone/Desktop/codes/depth_diag.py

# 5. No sim running
pgrep -f "gz sim\|px4_sitl" || echo "clean — no sim running"

# 6. Protobuf workaround proves out (script will print "subscribe ok=True"; then exit)
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python timeout 3 python3 -c "
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
print('protobuf ok')
"
# expect "protobuf ok"
```

If any of those fail, fix before proceeding.

---

## Section 13 — A note on uncertainty

I am confident that:
- Depth returns all-inf at every pose tested (5 stations, fresh sim, both pre-flight and airborne)
- The custom YOLO model is correctly wired and loads
- The reference code has no defense against all-inf depth
- This is consistent with the "VIO blowup" pattern

I am NOT confident that:
- The cause is specifically GLB-incompatibility (high prior, not proven — that's what the box test settles)
- The fix is specifically `.stl` conversion (most likely, but Blender export quality may vary)
- The lidar silence is related to depth (correlation, no proof of mechanism)

The box test in Section 7 is the cheapest, most decisive experiment available. **Run it first.** Everything downstream branches on its result.

Good luck.
