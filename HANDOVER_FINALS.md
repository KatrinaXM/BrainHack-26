# Handover — Finals prep mid-session

**Date:** 2026-06-07
**Context:** BrainHack-26 made it through Qualifiers; team is now prepping for Finals. This document is the **resume-point** for a fresh agent so they can continue without re-reading the whole repo or prior chat history.

If you are a new agent picking this up: read this top to bottom (~10 min) before anything else.

---

## 1. Where we are in one paragraph

The team passed the Qualifier (1st-place-tier 5 validated detections in our last test run, full mission code wired and committed on `main`). The Finals competition is **real hardware, not Gazebo SITL** — different mission, different sensors, different libraries. We've spent the post-qualifier time on (a) documenting the Finals' vocabulary in `TUTORIAL.md` Part 2, (b) drafting a phased prep plan in `RUNBOOK.md` Part 2, (c) dissecting the workshop's reference UWB navigation script in `docs/kolomee_dissection.md`, and (d) building the first piece of integration infrastructure — a "fake UWB" bridge (`codes/sim_uwb_bridge.py`) that turns the existing Qualifier Gazebo sim into a Finals testing rig.

Net status: **infrastructure laid, no Finals code tested end-to-end yet.** The next coherent step is to verify the bridge against a running Gazebo sim, then adapt `kolomee.py` to use it.

---

## 2. The Finals competition, in 30 seconds

Two-stage real-hardware drone mission. See `materials/RoboVerse 2026 Finals.pdf` for the official rules.

- **Stage 1 (University teams only — Reconnaissance):** fly a single **Mapping Drone** (Intel RealSense stereo + Rockchip NPU + UWB tag) over an arena. Detect **ArUco markers** beside each landing pad. Produce a top-down depth map + valid/invalid landing-zone list.
- **Stage 2 (Deployment & Ambush):** based on Stage 1 results, launch 3× **Highgreat HULA** swarm drones from the C2 Terminal via the `pyhulax` library. They must land accurately on 3 chosen pads. Then 5× **RoboMaster ground robots** enter as a "convoy"; the HULAs detect and snapshot them.

The Mapping Drone uses **UWB anchors** for positioning (real position, no broken EKF problem we had in Qualifier). The reference navigation skeleton is `references/finalist_codes/uwb_mavsdk/kolomee.py` — fully dissected in `docs/kolomee_dissection.md`.

---

## 3. Repo state — read this carefully

### 3.1 Current branch

`prep/finals` (formerly `experiment/ekf-tuning`, renamed to reflect what's actually on it).

Local branches:
```
* prep/finals                        (HEAD — Finals docs + bridge)
  experiment/wall-follow-reactive    (qualifier wallfollow experiment)
  experiment/high-altitude-survey    (qualifier high-alt experiment)
  main                               (Qualifier final, pushed; same as origin/main)
```

Recent commits on `prep/finals`:
```
7322371 docs: extend TUTORIAL + RUNBOOK for Finals; dissect kolomee.py
249a729 ekf-tuning: airframe param patch + drift measurement tool   ← stale, see §3.3
c752a01 update: files and references for finals                      ← from team via origin/main pull
43386aa launcher: interactive strategy picker                        ← qualifier
```

There are uncommitted changes on `prep/finals`: `codes/sim_uwb_bridge.py` (new, compiles) and the deleted `HANDOVER_DEPTH.md` (stale, see §6). **Commit these together with this handover before stopping.**

### 3.2 What lives where

| Path | Purpose | Status |
|---|---|---|
| `README.md` | Top-level orientation, points at Tutorial/Runbook/docs | Updated this session |
| `TUTORIAL.md` Part 1 (Ch 1-17) | Qualifier concepts (NED, MAVSDK, async, intrinsics, YOLO basics) | Still relevant for Finals |
| `TUTORIAL.md` Part 2 (Ch 18-29) | Finals: UWB, RealSense, ArUco, RKNN, pyhulax, ROS2, control loop, glossary | New this session |
| `RUNBOOK.md` Part 1 (Phases 0-6) | Qualifier execution gates | Historical (done) |
| `RUNBOOK.md` Part 2 (Phases 7-15) | Finals execution gates: prep, NoMachine, kolomee, ArUco, RKNN, Stage 1, Stage 2 | New this session |
| `docs/kolomee_dissection.md` | Line-by-line dissection of the workshop's UWB reference script | New this session |
| `codes/sim_uwb_bridge.py` | Gazebo ground-truth → ROS2 `/uwb_tag` bridge with toggleable noise | New this session, compiles, **untested** |
| `codes/mission.py`, `mission_config.py`, `barrel_tracker.py`, etc. | Qualifier mission code (wallfollow strategy) | Still works; useful as Gazebo bench |
| `codes/measure_drift.py` | EKF-drift measurement tool (Qualifier EKF tuning) | Stale (Qualifier-only) |
| `codes/run_competition.sh`, `install.sh` | Qualifier launchers | Still work; need Finals analogues |
| `scripts/start_sim.sh`, `stop_sim.sh`, `set_ekf_origin.py` | Gazebo sim helpers | Still work |
| `px4-patches/` | Qualifier EKF airframe tuning | Stale (Qualifier sim only — Finals uses real hardware) |
| `sdf-patches/OakD-Lite.model.sdf` | Qualifier camera model patch | Stale (Qualifier sim only) |
| `references/qualifier_codes/` | Workshop's Qualifier code | Reference, untouched |
| `references/finalist_codes/` | Workshop's Finals code: uwb_mavsdk, realsense_cam, aruco_detection, rknn_detect, hula_swarm, model_convert | Reference, untouched. The Finals starting point. |
| `materials/RoboVerse 2026 Finals.pdf` | Official Finals rules | New this session via origin/main pull |
| `materials/RoboVerse 2026 Qualifier.pdf` | Qualifier rules (historical) | Reference |
| `materials/LearningMaterial*.pdf`, `Supplementary*.pdf` | Workshop slide decks | Reference |

### 3.3 Stale files (do not delete unless asked)

- `codes/measure_drift.py` — Qualifier EKF debugging tool. Irrelevant to Finals real hardware. Kept because the gz/rclpy pattern was useful and similar code lives in `sim_uwb_bridge.py`.
- `px4-patches/` — Qualifier EKF tuning. Real hardware uses real VIO; these params don't apply.
- `sdf-patches/` — Qualifier sim only.
- The branch name `prep/finals` formerly was `experiment/ekf-tuning`; the EKF tuning files (`px4-patches/`, `codes/measure_drift.py`) are the historical reason for the original name. Now we're using the branch for general Finals prep.

### 3.4 Things that are genuinely missing

| Missing | Why it matters | Next-agent action |
|---|---|---|
| Tested `sim_uwb_bridge.py` | We don't know if it actually publishes to a `kolomee.py` subscriber | Run it against a Gazebo sim — see §5 |
| `kolomee.py` SITL-adapted | The reference uses `serial://`; for sim we'd need `udp://` | Make a copy, swap connection string |
| `stage1_mission.py` | Stage 1 mission orchestrator | Sketch is in `docs/kolomee_dissection.md` §10 |
| RealSense pipeline integration | Needed for ArUco + top-down map | Adapt `references/finalist_codes/realsense_cam/getSyncDepthColor.py` |
| ArUco logger module | Stage 1 deliverable | Use `references/finalist_codes/aruco_detection/aruco_detection.py` |
| pyhulax mock | Stage 2 testing without HULAs | Stub the API surface |

---

## 4. The `sim_uwb_bridge.py` design — read before testing

**File:** `codes/sim_uwb_bridge.py`. Compiles. Not yet run end-to-end.

### 4.1 What it does

1. Subscribes to Gazebo's `/model/<drone>_0/pose` (Pose_V message via `gz.transport13`).
2. Optionally applies noise (Gaussian σ, dropout, NLOS spikes).
3. Re-publishes as `geometry_msgs/PoseStamped` on `/uwb_tag` via `rclpy` at ~10 Hz.

This makes the existing Qualifier Gazebo sim a usable testbed for kolomee-style UWB navigation. **Real fake-UWB design rationale** is in the commit message + research-agent transcript (see git log).

### 4.2 Env-var configuration

| Var | Default | Meaning |
|---|---|---|
| `BH26_UWB_NOISE` | `0` | `0` = perfect, `1` = realistic LOS (σ_xy=10cm, σ_z=15cm, 5% drop), `2` = torture (σ_xy=25cm, σ_z=40cm, bias, 15% drop, NLOS spikes) |
| `BH26_UWB_SEED` | `0` (system entropy) | Set non-zero for reproducible noise sequences. Vital for debugging brittle controllers. |
| `BH26_UWB_DRONE` | `x500_vision` | Gazebo model name. Automatically appends `_0` if absent. |
| `BH26_UWB_HZ` | `10.0` | Publish rate, matching real UWB infrastructure. |

### 4.3 Coordinate convention — the trap

`sim_uwb_bridge.py` mirrors the **real venue's UWB convention** so that `kolomee.py` works against it unchanged:

- Gazebo world pose: `pose.x = north`, `pose.y = east`, `pose.z = up`.
- Bridge publishes: `msg.pose.position.x = east  ←  Gazebo y`, `msg.pose.position.y = north ←  Gazebo x`.
- `kolomee.py` reads: `self.n = msg.pose.position.y` (north), `self.e = msg.pose.position.x` (east). See `kolomee.py:67-68`.

If the bridge or kolomee shows the drone flying in a wrong direction during integration: **the swap is the first place to check**. Real venues may use different conventions — sanity-test by physically (or in sim) moving the drone north and watching which value increases.

### 4.4 Known gotcha

The bridge expects ROS2 (`rclpy`) to be importable. On the dev VM, that's only true if ROS2 was installed. The Qualifier sim didn't need ROS2 at all. **First time the next agent runs this**, expect possibly:
```
ModuleNotFoundError: No module named 'rclpy'
```
Fix: install ROS2 Humble or Jazzy on the dev VM (see `https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html`) and `source /opt/ros/humble/setup.bash` before running the bridge.

If ROS2 isn't desired on the dev VM, alternative: use the non-ROS2 UWB path — the Finals materials promise a "provided Python UWB class" that bypasses ROS2. That class isn't in our hands yet (organiser will provide). The bridge would then need a different output mechanism (named pipe / socket / shared memory) — but that's premature engineering for now.

---

## 5. How to verify the bridge (next-agent first task)

This is the **highest-leverage 60-minute task** for the next session.

### Step 1: Ensure ROS2 is available

```bash
python3 -c "import rclpy; print(rclpy.__version__)"
# If ModuleNotFoundError: install ROS2 Humble (Ubuntu 22.04) per the docs link in §4.4.
```

### Step 2: Start the Qualifier Gazebo sim

```bash
/home/drone/stop_sim.sh         # clean any stale
/home/drone/start_sim.sh --no-qgc
python3 /home/drone/set_ekf_origin.py
```

Wait for the drone to be visible in Gazebo. (You don't need to arm or fly — the bridge subscribes to ground-truth pose regardless of armed state.)

### Step 3: Verify Gazebo pose topic is alive

```bash
gz topic -e -t /model/x500_vision_0/pose -n 1 | head -20
```

You should see a `Pose_V` message with `name: "x500_vision_0"`. If the topic doesn't exist, the sim isn't fully up.

### Step 4: Run the bridge

```bash
cd /home/drone/BrainHack-26
source /opt/ros/humble/setup.bash      # if you installed ROS2
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 codes/sim_uwb_bridge.py
```

Expected output:
```
[bridge] noise level=0, seed=system
[bridge] drone='x500_vision', rate=10.0 Hz
[bridge] gz subscribed: /model/x500_vision_0/pose
[bridge] ros2 publisher up on /uwb_tag at 10.0 Hz
[bridge] noise model: σ_xy=0.00 m, σ_z=0.00 m, bias_xy=0.00 m, dropout=0.0%, nlos_prob/tick=0.0%
```

### Step 5: Verify ROS2 topic is alive

In another terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /uwb_tag                  # expect ~10 Hz
ros2 topic echo /uwb_tag --once         # expect PoseStamped with the drone's GT
```

If `ros2 topic list` shows `/uwb_tag` but `echo` blocks → QoS mismatch. The bridge uses `BEST_EFFORT`; subscriber must match.

### Step 6: Run with noise

Once perfect mode works:
```bash
BH26_UWB_NOISE=1 BH26_UWB_SEED=42 \
  PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
  python3 codes/sim_uwb_bridge.py
```

You should see the same output but with noise applied. Confirm by `ros2 topic echo` showing values that wobble around the GT.

### Step 7: Drive it with kolomee.py (or adapted version)

`kolomee.py` connects to PX4 via `serial:///dev/ttyS6:921600`. For SITL we need `udpin://0.0.0.0:14540`. Make a SITL-friendly copy first:

```bash
mkdir -p codes/finals
cp references/finalist_codes/uwb_mavsdk/kolomee.py codes/finals/kolomee_sitl.py
sed -i 's|serial:///dev/ttyS6:921600|udpin://0.0.0.0:14540|' codes/finals/kolomee_sitl.py
```

Then with the bridge running:
```bash
cd codes/finals
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 kolomee_sitl.py
```

Expected behaviour: drone arms (after `y` confirmation), takes off, navigates two waypoints, lands. Watch the bridge logs and ROS2 topic to confirm the loop is closed.

### Step 8: Fix the bugs

While testing, fix the bugs we documented in `docs/kolomee_dissection.md §5.4` — missing `await`s, CPU-spin in failure path, dead `KP_SCALE`. These will cause silent failures in the noise-1 case.

---

## 6. Stale-file decisions made this session

- **Deleted:** `HANDOVER_DEPTH.md`. It documented a 2026-05-23 depth-camera all-inf bug that was resolved before the Qualifier finished. The resolution (lightweight OakD-Lite SDF patch) is already captured in `sdf-patches/OakD-Lite.model.sdf` + memory file `project_oakd_lite_lightweight.md`. Keeping the stale handover misdirected readers.
- **Kept** (despite being Qualifier-only): `codes/measure_drift.py`, `px4-patches/`, `sdf-patches/`, `codes/mission.py` and friends. None are deleted — they're useful for sim-based bench tests and code reading.

---

## 7. Branches we've been carrying

| Branch | Status | Recommendation |
|---|---|---|
| `main` | Qualifier final, pushed | **Leave alone** — this is the artefact we shipped. |
| `prep/finals` | Current, Finals docs + bridge WIP | **Use this for active work.** |
| `experiment/wall-follow-reactive` | Qualifier experiment, merged to main | Keep around briefly, can delete after Finals |
| `experiment/high-altitude-survey` | Qualifier experiment, didn't merge | Keep around briefly, can delete after Finals |

Next-agent: do not start new branches for individual files; keep working on `prep/finals` for the next few hours. Spin a new branch only when the integration is big enough to warrant isolation.

---

## 8. Memory pointers

Auto-memory at `/home/drone/.claude/projects/-home-drone-BrainHack-26/memory/`. Key Qualifier-era notes that are **still relevant** for Finals:

- `feedback_restart_between_scripts.md` — sim must be restarted between Python script runs; the bridge will hit this too if it shares state with `mission.py`.
- `project_protobuf_workaround.md` — every script that imports `gz.msgs10` needs the `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` env-var. Applies to `sim_uwb_bridge.py`.
- `project_oakd_lite_lightweight.md` — the SDF patch that fixed depth.
- `project_drone_model.md` — Discord ruling on drone model (Qualifier-specific).
- `project_qualifier_rules.md` — Qualifier scoring (historical).
- `project_barrel_truth.md` — ground-truth barrel counts (Qualifier-specific).
- `project_camera_params.md` — Qualifier intrinsics; Finals will have different RealSense intrinsics.

No Finals-specific memory entries yet. Worth adding once we know the venue's UWB topic name, coordinate convention, ArUco scheme, etc.

---

## 9. The shortest path to "Finals integration sanity-checked"

If you have only one session of work, do these in order:

1. **Verify the bridge** — §5 steps 1-6. Expected time: 30-60 min if ROS2 is preinstalled, +60 min if you need to install ROS2.
2. **Adapt kolomee.py for SITL** — `serial://` → `udpin://`. ~10 min.
3. **Run kolomee against the bridge** — §5 step 7. Expected: drone takes off, navigates 2 waypoints, lands. ~30 min including troubleshooting.
4. **Fix the kolomee bugs** documented in `docs/kolomee_dissection.md §5.4`. ~15 min.
5. **Commit `prep/finals` and push** — so the next agent has a real checkpoint. ~5 min.

After that, the next major thread is `stage1_mission.py` — sketch is in `docs/kolomee_dissection.md §10`.

---

## 10. What to read first if you've never seen this repo

1. This document (you just did).
2. `materials/RoboVerse 2026 Finals.pdf` — the Finals rules.
3. `TUTORIAL.md` Part 2 (chapters 18-29). Mandatory for the Finals vocabulary.
4. `docs/kolomee_dissection.md` — the workshop reference's architecture.
5. `RUNBOOK.md` Part 2 (Phases 7-15) — the prep schedule with gates.

After that you're current. Total reading time: ~90 min for the engineer-quality read.

---

*End of handover. The current state of the repo is just-paused, fully committable. Next agent should commit `sim_uwb_bridge.py` + this handover + the `HANDOVER_DEPTH.md` deletion as their first action.*
