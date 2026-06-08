# START HERE — BrainHack-26 Finals (Pre-University)

**Welcome.** This document is the single entry point for the team. If you've never seen this project before — or never used a Linux terminal — read this top to bottom. It's long, but each section is short and you only need to do the things in the sections that apply to *right now*.

**Last updated:** 2026-06-09

---

## Table of contents

1. [What this project is, in plain English](#1-what-this-project-is-in-plain-english)
2. [Where we are right now (status)](#2-where-we-are-right-now-status)
3. [The Finals competition explained simply](#3-the-finals-competition-explained-simply)
4. [Terminal basics (if you've never used one)](#4-terminal-basics-if-youve-never-used-one)
5. [Running the test mission on this machine](#5-running-the-test-mission-on-this-machine)
6. [What every file does](#6-what-every-file-does)
7. [Before competition day — what you can do at home](#7-before-competition-day--what-you-can-do-at-home)
8. [At the competition site — step by step](#8-at-the-competition-site--step-by-step)
9. [Troubleshooting common problems](#9-troubleshooting-common-problems)
10. [Questions to ask the organizers](#10-questions-to-ask-the-organizers)
11. [Glossary of unfamiliar terms](#11-glossary-of-unfamiliar-terms)
12. [Going deeper (optional reading)](#12-going-deeper-optional-reading)

---

## 1. What this project is, in plain English

This project is the team's software for the **BrainHack-26 RoboVerse Finals**, an autonomous-drone competition. Our team is in the **Pre-University** category, which means we only do **Challenge 2** (the second half of the competition — explained in §3 below).

**What the software does**, in one sentence: *runs on a laptop, connects to 3 small drones over Wi-Fi, tells them to fly to landing pads, then watches their cameras to take snapshots of ground robots that drive into the arena.*

That's it. There's no neural network on the drone, no obstacle avoidance to write — the small drones (called **HULAs**) handle their own positioning. Our job is to **orchestrate**: tell them where to go, then process the camera feed.

---

## 2. Where we are right now (status)

**Software:** essentially complete. The entire mission orchestrator is written, tested, and runs end-to-end on this laptop with a **fake-drone simulator** built into the code. You can confirm this for yourself in §5 below — takes about 25 seconds.

**Hardware:** untested. We've never actually connected to a real HULA drone. This is normal — we don't have HULAs in our possession. The work that remains is **at the competition site**: confirming a few things on real hardware and tuning a few numbers. Section 8 walks through exactly what to do there.

**What's been done:**
- Mission orchestrator (`codes/finals/stage2_mission.py`) — single Python program, runs the whole mission
- A **fake-drone simulator** (`codes/finals/mocks/pyhulax_mock.py`) — lets us test without real drones
- A **RoboMaster detector** that fires when a red-armor ground robot enters the camera view
- A **snapshot saver** that writes annotated JPEG + JSON every time a robot is spotted
- **16 automated tests** that all pass in under 1 second — these catch bugs before competition
- A **launcher script** (`codes/finals/run_stage2.sh`) — one command starts everything

**What still needs to happen:**
- At the venue: confirm the Wi-Fi lets drones discover each other (§8 step 2)
- At the venue: confirm Direction.FORWARD on a HULA matches our assumption (§8 step 3)
- At the venue: tune the RoboMaster color detector to the actual lighting (§8 step 5)
- That's it for software. The rest is flying and scoring points.

---

## 3. The Finals competition explained simply

Source of truth: `materials/RoboVerse 2026 Finals.pdf`. What follows is the Pre-U-only version in 6 short bullets.

**Setting:** indoor cage, ~10m × 10m. Several "landing pads" marked on the ground.

**Stage A — Landing (we do this):**

1. Organizers hand us a list of pads with a "valid" / "invalid" flag for each. We do **not** have to figure out which pads are valid ourselves — that's the University teams' job.
2. We pick 3 pads from the valid ones.
3. We launch 3 small flying drones (called **HULAs**) from our laptop. They are pre-programmed to know how to fly; we just tell them where to go.
4. Each HULA flies to its assigned pad and lands. We're scored on accuracy + total time.

**Stage B — Ambush (we do this immediately after):**

5. After our HULAs are landed, 5 small ground robots (called **RoboMasters**) drive into the arena.
6. Each landed HULA has a forward-facing camera. Our software watches all 3 video feeds. When a RoboMaster enters a feed, we save a snapshot (an image of it). Scored on number of correct snapshots + accuracy + time.

**Total time budget:** about 5 minutes from start to finish, in one continuous run.

**That's the whole thing.** Three drones, three pads, then watch for red robots and snap pictures.

---

## 4. Terminal basics (if you've never used one)

Skip this section if you're already comfortable in a Linux terminal.

**Opening a terminal on this VM (Ubuntu):**
- Look for "Terminal" in the application menu, or
- Press `Ctrl+Alt+T` (hold all three keys together).

A black window appears with text like `drone@hostname:~$` and a blinking cursor. That's the **prompt** — you type commands here and press Enter.

**The 5 commands you need:**

| Command | What it does | Example |
|---|---|---|
| `cd ~/BrainHack-26` | Change Directory — moves you into the project folder. `~` means "your home folder". | After running this, the prompt shows you're inside `BrainHack-26`. |
| `ls` | List the files in the current folder. | Shows the project's files. |
| `cat <filename>` | Show the contents of a file. | `cat README.md` prints README. |
| `./run_stage2.sh --short` | Run a script. The `./` at the start means "run the file that's right here". | Used in §5. |
| `Ctrl+C` | Stop a running program. Hold Ctrl, press C. | If something's stuck, this stops it. |

**Conventions used below:**

- A box like this:
  ```bash
  cd ~/BrainHack-26
  ```
  means: type the line into the terminal, then press Enter.

- Anything written like `THIS_TEXT_IN_CAPS` is a placeholder you replace with a real value (a file path, a number, etc).

- Output you should expect is shown like:
  ```
  === MISSION SUMMARY ===
    plane1: state=complete ...
  ```

---

## 5. Running the test mission on this machine

This is the most useful thing to do first. **It proves the software works** and takes 25 seconds.

**Step 1 — Open a terminal** (see §4 if you don't know how).

**Step 2 — Go to the Stage 2 folder:**
```bash
cd ~/BrainHack-26/codes/finals
```

**Step 3 — Run the short smoke test:**
```bash
./run_stage2.sh --short --output /tmp/test_snapshots
```

**What you should see** (scrolling output for ~25 seconds, then a summary):
```
[main] backend=mock cv2=yes output_dir=/tmp/test_snapshots ambush_window_s=8
[main] loaded 6 pads, chose ['P1', 'P3', 'P5']
[main] discovered 3 HULAs: ['plane1', 'plane2', 'plane3']
[main] plane1 (192.168.1.100) -> pad P1
[main] plane2 (192.168.1.101) -> pad P3
[main] plane3 (192.168.1.102) -> pad P5
[plane1] idle -> takeoff
[plane2] idle -> takeoff
[plane3] idle -> takeoff
[plane1] takeoff -> fly_to_pad
  [navigate_to_pad] P1: from (0.00,0.00,1.00) -> (-3.00,2.00,0.00)
  [navigate_to_pad] DOWN 1.00 m
  [navigate_to_pad] BACK 3.00 m
  [navigate_to_pad] RIGHT 2.00 m
  [navigate_to_pad] P1 arrival
[plane1] fly_to_pad -> landing
[plane1] landing -> ambush_watch
[plane1] snapshot 1 -> /tmp/test_snapshots/plane1_001_xxx.jpg (1 bbox)
... (more snapshots)
[plane1] ambush_watch -> complete
[plane2] ambush_watch -> complete
[plane3] ambush_watch -> complete

=== MISSION SUMMARY ===
  plane1: state=complete pad=P1 snapshots=4 err=-
  plane2: state=complete pad=P3 snapshots=3 err=-
  plane3: state=complete pad=P5 snapshots=4 err=-
```

**What this means:**
- "backend=mock" — using the fake-drone simulator (correct for testing)
- "discovered 3 HULAs" — the fake drones were "found"
- Each plane went through the full mission (takeoff → fly → land → watch → complete)
- Each plane saved 3-4 snapshots of fake red robots that the simulator paints in the video

**Step 4 — Look at the saved snapshots:**
```bash
ls /tmp/test_snapshots
```
You'll see pairs of `.jpg` and `.json` files.

To open one image:
```bash
xdg-open /tmp/test_snapshots/plane1_001_*.jpg
```
You should see a dark gray frame with a **red rectangle** (the fake RoboMaster) and a **green box drawn around it** (what the detector found). That's a snapshot.

**Step 5 — Run the unit tests** (this proves the code is correct):
```bash
cd ~/BrainHack-26/codes/finals
python3 -m unittest tests.test_stage2 -v
```
You should see 16 tests run, all ending in `ok`, finishing in under 1 second with `OK` on the last line.

**If anything in steps 3–5 didn't work**, jump to §9 (Troubleshooting).

---

## 6. What every file does

The repo has a lot of files because it grew out of the Qualifier (which we passed). For Pre-U Finals you mostly only care about the small `codes/finals/` folder. Here's the map.

### 6.1 What matters for Pre-U Finals (the active stack)

| File | Purpose | When to look at it |
|---|---|---|
| `codes/finals/stage2_mission.py` | The main program. ~500 lines. Contains the mission state machine, navigation, RoboMaster detector, snapshot saver. | If you need to change behavior. |
| `codes/finals/mocks/pyhulax_mock.py` | Fake-drone simulator for offline testing. ~285 lines. | Almost never — it just works. |
| `codes/finals/tests/test_stage2.py` | 16 automated tests. | Run after any change to `stage2_mission.py`. |
| `codes/finals/run_stage2.sh` | Launcher script. | Every time you run the mission. |
| `codes/finals/pads_example.json` | Sample pad list (for testing). The real one comes from organizers on competition day. | When you need to test with custom pads. |
| `materials/RoboVerse 2026 Finals.pdf` | The official rules. | Before competition day; if rules are unclear. |
| `references/finalist_codes/hula_swarm/huladola.py` | The workshop's reference code (65 lines). Shows the basic pyhulax pattern we copied. | If `stage2_mission.py` does something confusing — check the simpler reference. |

### 6.2 Reference material (read if curious, ignore otherwise)

| Folder/File | What's in it |
|---|---|
| `references/finalist_codes/aruco_detection/`, `realsense_cam/`, `rknn_detect/`, `uwb_mavsdk/`, `model_convert/` | University-team-only reference code. We don't use these for Pre-U. |
| `references/qualifier_codes/` | Workshop reference for the Qualifier. We finished that already. |
| `materials/LearningMaterial*.pdf`, `Supplmentary_LearningMaterial*.pdf` | Workshop slides. |
| `materials/RoboVerse 2026 Qualifier.pdf` | Qualifier rules (history). |
| `TUTORIAL.md` | ~1000 lines explaining the concepts. Chapter 24 ("HULA swarm via pyhulax") is the relevant section for Pre-U. The rest is Qualifier or University-Stage-1. |
| `RUNBOOK.md` | A step-by-step prep plan. Phases 0-6 are Qualifier (done); Phases 7-15 are Finals (mostly University-team). |
| `docs/kolomee_dissection.md` | Deep dive on a University-team reference script. Pre-U doesn't touch this — but the *patterns* (threading, mocks, bug-hunting) are educational. |

### 6.3 Older code (Qualifier era — left as reference, you don't run it)

| Folder/File | Why it's still here |
|---|---|
| `codes/mission.py`, `barrel_tracker.py`, `Detector.py`, `AvoidancePlanner.py`, `lawnmower.py`, `drone_control.py`, etc. | The Qualifier mission. Still works in the simulator. |
| `codes/run_competition.sh`, `install.sh`, `RUN.txt`, `JUDGE_SUMMARY.md` | Qualifier launchers + writeups. |
| `codes/barrel_yolo.pt` | Qualifier YOLO model (5.5 MB). Not used for Finals. |
| `codes/sim_uwb_bridge.py`, `codes/finals/kolomee_sitl.py` | Stage 1 SITL experiments from earlier prep. Not used for Pre-U. |
| `codes/measure_drift.py`, `codes/depth_diag.py`, `codes/depth_receiver.py`, `codes/show_camera.py`, `codes/get_position_with_task.py` | Qualifier debug tools. |
| `codes/mission_config.py` | Qualifier configuration. |
| `scripts/start_sim.sh`, `stop_sim.sh`, `set_ekf_origin.py`, `set_ekf_origin.sh` | Qualifier simulator helpers. |
| `px4-patches/`, `sdf-patches/` | Qualifier simulator patches. |

**Rule of thumb:** if it's in `codes/finals/`, it's Pre-U-relevant. Anything else, you can mostly ignore unless you're curious.

---

## 7. Before competition day — what you can do at home

There isn't much critical software work left. Things you *could* do, in priority order:

### 7.1 Practice running the mission (highly recommended)

Run §5 once a day until competition. It's 25 seconds and proves nothing has broken. Familiarity = confidence on the day.

### 7.2 Read `materials/RoboVerse 2026 Finals.pdf` carefully (highly recommended)

Especially the "Challenge Two" section. Note:
- The pad-list format (PDF doesn't specify exactly — confirm with organizers)
- Whether RoboMasters are red, blue, or mixed colors
- The scoring formula

### 7.3 Practice typing commands in a terminal (recommended)

On competition day you'll be under time pressure. Type these blind a few times:

```bash
cd ~/BrainHack-26/codes/finals
./run_stage2.sh --real --pads pads.json
```

```bash
ls /tmp/test_snapshots
```

```bash
python3 -m unittest tests.test_stage2 -v
```

### 7.4 Read the questions list for organizers (§10) (recommended)

Send these to the organizers before the day if possible. Answers shape what you need to bring.

### 7.5 Optional improvements (low priority)

- Add a blue-armor RoboMaster detector mask (in case RoboMasters can be blue). Open `codes/finals/stage2_mission.py`, find the `detect_robomaster` function — duplicate the red HSV ranges with blue ones.
- Pre-calibrate the detector on stock RoboMaster images (search online for sample images).
- Build a YOLO RoboMaster detector as backup. Reuse the Qualifier `barrel_yolo.pt` training pipeline.

**None of these are required.** The current detector works on synthetic data; calibration on real footage is fastest at the venue with real lighting.

---

## 8. At the competition site — step by step

This is the most important section. Read it before you arrive.

**What to bring:**
- The laptop with this repo on it
- Power cable + adapter
- A long network cable just in case (some venues require wired)
- A small notebook + pen to write down env-var values you tune

**Pre-flight on arrival:**

### 8.1 Set up your laptop

1. Open a terminal (§4).
2. Verify the repo is still here: `cd ~/BrainHack-26 && ls`. You should see `codes/`, `materials/`, etc.
3. Run the smoke test (§5) one more time to confirm nothing broke during transport.

### 8.2 Connect to the venue Wi-Fi

The HULAs talk to the laptop over Wi-Fi using something called **multicast** (a way for devices on the same network to find each other automatically).

1. Join the Wi-Fi network the organizers give you.
2. Verify you have an IP address: type `ip addr` in the terminal — look for something like `192.168.x.x` under a section labeled `wlan` or `eno`.
3. **Critical check:** ask the organizers whether their Wi-Fi allows **multicast traffic between client devices**. Some venue networks block this. If blocked, the drones won't be discoverable. If they say "no" or don't know, ask if there's an unmanaged switch / hotspot you can use instead.

### 8.3 Install pyhulax (if not already)

You need the real pyhulax library on the laptop. The organizers should provide install instructions on the day. The general pattern (do this only when they tell you):

```bash
pip install pyhulax
```

To verify it worked:
```bash
python3 -c "import pyhulax; import dola; print('OK')"
```
If you see `OK`, you're good. If you see `ModuleNotFoundError`, the install didn't succeed — ask an organizer for help.

### 8.4 First-flight calibration (very important)

Before flying the full mission, do these three quick tests **with just ONE HULA powered on**:

**Test A — Discovery works.** With the HULA powered on and connected to the same Wi-Fi:
```bash
cd ~/BrainHack-26/codes/finals
BH26_MOCK= python3 -c "
from dola import Dola
d = Dola(); d.start()
print(d.get_all_ips(listen_seconds=5))
d.stop()
"
```
You should see a dict like `{'plane1': '192.168.x.y'}` printed. **If you see `{}`, the multicast network is blocking discovery.** Talk to the organizers about §8.2.

**Test B — Direction calibration.** This tells you whether `Direction.FORWARD` actually moves the drone in the direction we assumed (+x in arena coordinates).

Run a one-HULA test mission:
- Create a pad file with one valid pad at a known location, e.g. `(2, 0, 0)` — 2 m in the +x direction. Save as `test_pad.json`:
  ```json
  [{"id": "T1", "x": 2.0, "y": 0.0, "z": 0.0, "valid": true}]
  ```
- Edit `codes/finals/stage2_mission.py` line where `NUM_DRONES = 3` and change it temporarily to `NUM_DRONES = 1`.
- Run: `./run_stage2.sh --real --pads test_pad.json --ambush 5`
- Watch the drone. If it flies in the direction we marked as "+x" on the arena floor, the convention is correct.
- If it flies the **wrong direction**, flip the env vars:
  - If it flew the *opposite* +x direction (i.e., -x): `export BH26_AXIS_FORWARD=-x`
  - If it flew +y or -y: `export BH26_AXIS_FORWARD=+y` (or `-y`)
- Re-run the test. Repeat until the drone flies the right way.
- **Write down the env-var values you settled on** — you'll need them for the real mission.
- Don't forget to set `NUM_DRONES = 3` back when you're done.

**Test C — Detector calibration.** Stand a RoboMaster in front of the HULA's camera (or land the HULA and walk a robot in front of it):
- During the test mission, check `/tmp/snapshots/` — are snapshots being saved with green boxes correctly around the robot?
- If the detector misses the robot (no snapshots), or detects junk (snapshots with green boxes around nothing), tune these env vars:
  - `BH26_DETECT_MIN_AREA` (default 400) — lower this if real robots appear small in the camera
  - `BH26_DETECT_MAX_AREA` (default 60000) — raise this if the robot looks huge up close
  - `BH26_DETECT_MIN_ASPECT`, `BH26_DETECT_MAX_ASPECT` (default 0.3 to 3.0) — relax if robots are at odd angles
- **Write down the values you settled on.**

### 8.5 Run the actual mission

When the organizers give you the official pad file:

1. Save the pad file as `competition_pads.json` in `~/BrainHack-26/codes/finals/`.
2. Open the terminal and run:
   ```bash
   cd ~/BrainHack-26/codes/finals
   # Apply your calibration env vars (replace the placeholder values with what you wrote down in §8.4)
   export BH26_AXIS_FORWARD=+x
   export BH26_AXIS_RIGHT=+y
   export BH26_DETECT_MIN_AREA=400
   export BH26_DETECT_MAX_AREA=60000

   # Run the mission for real (default ambush window 120 seconds)
   ./run_stage2.sh --real --pads competition_pads.json
   ```
3. Watch the output. State transitions print live (`takeoff -> fly_to_pad -> ...`). When the mission completes, you'll see the `=== MISSION SUMMARY ===` block.
4. Snapshots are saved to `./snapshots/`. Inspect them:
   ```bash
   ls snapshots
   ```
5. **Submit per the organizers' instructions** (they'll tell you the format — likely a USB stick or upload form).

### 8.6 If something goes wrong mid-mission

- **Stop the program**: press `Ctrl+C` in the terminal. This signals the orchestrator to safely land all drones and exit.
- **Re-run**: most issues are recoverable — try the mission again if rules allow.
- **Common mid-mission problems:** see §9.

---

## 9. Troubleshooting common problems

### `bash: ./run_stage2.sh: Permission denied`
The script isn't marked as executable. Fix:
```bash
chmod +x ~/BrainHack-26/codes/finals/run_stage2.sh
```

### `python3: No module named 'cv2'` (or `numpy`)
OpenCV isn't installed. Install:
```bash
pip install opencv-python numpy
```

### `ModuleNotFoundError: No module named 'pyhulax'`
Only matters in `--real` mode. The organizers will provide install instructions on the day.

### "pyhulax / dola not installed on this machine"
Same as above — the `--real` mode needs the real library. For testing without HULAs, use `--mock` or `--short`.

### `RuntimeError: need 3 HULAs, only found N`
The discovery only found N drones. Causes:
- Drones aren't powered on
- Drones aren't connected to the same Wi-Fi as your laptop
- Wi-Fi blocks multicast (§8.2) — ask the organizers
- One of the drones has a low battery and didn't boot

### `RuntimeError: need 3 valid pads, got N`
The pad file has fewer than 3 valid pads. Either:
- The organizers' pad list legitimately has < 3 valid pads (unlikely)
- The file is in a different format than expected. Open the JSON file and check field names — should match the `pads_example.json` format.

### Mission runs but no snapshots are saved
The detector isn't firing. Try:
- Lower `BH26_DETECT_MIN_AREA` (e.g., 100 instead of 400)
- Check the video feed manually — is anything visible? Maybe the HULA camera failed.

### Mission runs but snapshots are full of false positives
The detector is too sensitive. Try:
- Raise `BH26_DETECT_MIN_AREA` (e.g., 1000)
- Tighten `BH26_DETECT_MAX_ASPECT` (e.g., 2.0)

### Drone flies the wrong direction
Calibrate the axis convention — §8.4 Test B.

### Drone arrives at the wrong altitude
Adjust `BH26_TAKEOFF_ALT_M` to match where the HULA actually hovers after `.takeoff()`. Default is 1.0 m.

### "I made a code change and now the tests fail"
Run the tests, look at the failure message:
```bash
cd ~/BrainHack-26/codes/finals
python3 -m unittest tests.test_stage2 -v
```
Each test name tells you which feature broke. Revert your change with `git diff` (to see what changed) and `git checkout codes/finals/stage2_mission.py` (to undo).

### "The terminal feels stuck"
Press `Ctrl+C`. If still stuck, close the terminal window and open a new one.

---

## 10. Questions to ask the organizers

Send these in advance (email / Discord / whatever channel) if you can.

**About the network:**
1. Does the venue Wi-Fi allow **multicast** traffic between client devices (needed for HULA discovery via Dola)?
2. Is there a wired backup if Wi-Fi misbehaves?
3. What's the SSID + password?

**About pyhulax:**
4. Where do we install `pyhulax` and `dola` from on competition day? (`pip install`? Local wheel?)
5. Does pyhulax expose a `.goto(x, y, z)` method, or only `.move(direction, distance)`? (Our code assumes only `.move()` — if `.goto()` exists, we can simplify.)
6. What unit is the second argument to `.move()`? (Assumed metres — please confirm.)
7. Is `Direction.FORWARD` body-frame (relative to drone heading) or world-frame? (Assumed body-frame.)
8. What altitude does `.takeoff()` reach? (Assumed 1.0 m.)

**About the mission:**
9. What's the exact format of the pad list file? (JSON? CSV? Field names?)
10. Are the RoboMasters red, blue, or mixed colors? (Detector currently looks for red only.)
11. Do RoboMasters have any markings (ArUco tags, QR codes) we could use as a more reliable detection signal?
12. How long do RoboMasters loiter in the cage during the ambush phase?
13. Is there pre-competition hardware time? Even 30 minutes to do §8.4 calibration would be invaluable.

**About scoring:**
14. What's the snapshot scoring rule? (Are duplicate snapshots of the same robot penalized?)
15. Do we lose points if our HULAs don't land on the exact pad center?
16. Time limit for the full mission?

---

## 11. Glossary of unfamiliar terms

Terms you'll see in this doc, the code, or the rules.

| Term | Meaning |
|---|---|
| **Pre-U / Pre-University** | Our competition category. We do Challenge 2 only. |
| **HULA** | Highgreat HULA — the small flying drones we control. 3 of them. |
| **RoboMaster** | Small ground robots (made by DJI). They drive around. Our HULAs spot them and snap photos. |
| **C2 / C2 Terminal** | "Command and Control terminal" — fancy term for "the laptop running our software". |
| **pyhulax** | The Python library that lets us send commands to HULA drones. Made by the HULA manufacturer. |
| **Dola** | A small Python tool (`dola` library) that finds HULAs on the local Wi-Fi network. Stands for "drone explorer" loosely. |
| **multicast** | A way for devices on the same network to send a message "to whoever's listening". HULA discovery uses it. Some Wi-Fi networks block it. |
| **landing pad** | A marked spot on the arena floor where HULAs should land. |
| **ambush window** | The period after landing during which HULAs watch for ground robots. Default 120 seconds in our code. |
| **state machine** | A program pattern where each drone is "in a state" (idle, taking off, flying, etc.) and only does the actions allowed by that state. Our orchestrator uses this. |
| **mock** | A fake version of something (here, fake HULAs and fake video) used for offline testing. |
| **bbox / bounding box** | A rectangle drawn around something detected in an image. Our snapshot files have these. |
| **HSV** | A way of describing colors (Hue, Saturation, Value) that's better than RGB for detecting "things that look red" robustly across different lighting. |
| **env var / environment variable** | A setting you change before running a program, like `BH26_AMBUSH_S=30`. Persists for the rest of the terminal session. |
| **terminal** | The black window with a text prompt. Where you type commands. |
| **stdout / log** | The text the program prints while running. Scrolls in the terminal. |
| **NED / body-frame / world-frame** | Ways of describing direction. "Body-frame forward" = "forward relative to where the drone is currently facing". "World-frame forward" = "forward in the room", which might be different. |
| **`.goto()` vs `.move()`** | Two possible drone commands. `.goto(x, y, z)` would tell the drone "go to this exact position". `.move(direction, distance)` says "go this many meters in this direction from where you are". We only know about `.move()` from the workshop reference. |
| **mavsdk, PX4, Gazebo, EKF, kolomee, UWB, RealSense, RKNN, ArUco** | Things from Stage 1 (University-team only). You can completely ignore these. |

---

## 12. Going deeper (optional reading)

You don't need any of this to compete. Read it if you're curious or want to understand the code more deeply.

**Pyhulax patterns:** `TUTORIAL.md` Chapter 24 (lines ~793-871). Explains why we use one thread per drone, lists the open questions about pyhulax, and shows the basic API surface.

**The workshop's reference code:** `references/finalist_codes/hula_swarm/huladola.py`. Just 65 lines. Our orchestrator is structurally an expansion of this script.

**The competition rules in full:** `materials/RoboVerse 2026 Finals.pdf`.

**The Qualifier code we shipped:** `codes/mission.py` and friends. We won't run this at Finals but it's a clean example of the same patterns (state machines, threading, detection) on a different problem.

**Stage 1 (University-only) reference scripts:** `references/finalist_codes/aruco_detection/`, `realsense_cam/`, `rknn_detect/`, `uwb_mavsdk/`. Pre-U doesn't run any of these, but they're well-written examples if you ever want to learn about UWB nav, ArUco detection, etc.

**Why we set up the simulator bridge:** `docs/kolomee_dissection.md` and `codes/sim_uwb_bridge.py`. This was University-Stage-1 prep that we later discovered Pre-U doesn't need. We kept the work because the patterns it exercises (async/threading, mocking, finding bugs in workshop code) are educational. Reading the dissection is a master class in "what bugs hide in reference code we trust".

**The architecture decision log:** Our git history. `git log --oneline` shows the high-level story; `git show <commit>` shows the details. Recent commits are the most relevant; older ones are Qualifier-era.

---

## A final note

This codebase is intentionally small and obvious. There's one orchestrator program, one mock for testing it, one launcher to run it, and 16 tests to guarantee it works. Everything else in the repo is either reference material, the rules, or Qualifier history.

**On competition day, you're in the orchestrator-pilot seat.** The drones know how to fly. The tests prove the code works. All that's between you and a passing run is plugging in the actual hardware and reading the output.

**Good luck.** You've got this.
