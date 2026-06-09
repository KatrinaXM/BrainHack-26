# START HERE — BrainHack-26 Finals (Pre-University)

**Welcome.** This document is the single entry point for the team. If you've never seen this project before — or never used a Linux terminal — read this top to bottom. It's long, but each section is short and you only need to do the things in the sections that apply to *right now*.

**Last updated:** 2026-06-10 (synced with `materials/Finals brief.pptx`)

---

## Table of contents

1. [What this project is, in plain English](#1-what-this-project-is-in-plain-english)
2. [Where we are right now (status)](#2-where-we-are-right-now-status)
3. [The Finals competition explained simply](#3-the-finals-competition-explained-simply)
4. [Schedule (when does what happen)](#4-schedule-when-does-what-happen)
5. [Terminal basics (if you've never used one)](#5-terminal-basics-if-youve-never-used-one)
6. [Running the test mission on this machine](#6-running-the-test-mission-on-this-machine)
7. [What every file does](#7-what-every-file-does)
8. [Before competition day — what you can do at home](#8-before-competition-day--what-you-can-do-at-home)
9. [At the competition site — step by step](#9-at-the-competition-site--step-by-step)
10. [Bonus task (don't skip — free points)](#10-bonus-task-dont-skip--free-points)
11. [Troubleshooting common problems](#11-troubleshooting-common-problems)
12. [Questions still open for organizers](#12-questions-still-open-for-organizers)
13. [Glossary of unfamiliar terms](#13-glossary-of-unfamiliar-terms)
14. [Going deeper (optional reading)](#14-going-deeper-optional-reading)

---

## 1. What this project is, in plain English

This project is the team's software for the **BrainHack-26 RoboVerse Finals**, an autonomous-drone competition. Our team is in the **Pre-University** category, which means we only do **Challenge 2** (the second half of the competition — explained in §3).

**What the software does**, in one sentence: *runs on a laptop, connects to 3 small drones over Wi-Fi, tells them to fly to designated landing pads, then takes off again and uses the drones' cameras to detect ArUco markers on small ground robots driving around the arena.*

There's no neural network on the drone, no obstacle avoidance to write — the HULA drones handle their own positioning. Our job is to **orchestrate**: tell them where to go, then process the camera feed to spot ArUco markers (printed black-and-white squares with unique IDs).

---

## 2. Where we are right now (status)

**Software:** **complete and aligned with the official Finals brief.** Mission orchestrator runs end-to-end on this laptop using a fake-drone simulator. The detector finds real ArUco markers (not a stub or a colour heuristic). The state machine implements the brief's two-scoring-item flow (land first, then take off again to search). You can confirm this in 25 seconds — see §6.

**Hardware:** untested. We've never connected to a real HULA. That's expected; we don't have HULAs at home. The remaining work happens **at the competition site**: confirming Wi-Fi discovery works, calibrating direction conventions, tuning the ArUco detector to actual lighting. Section 9 walks through exactly what to do there.

**What's been built:**
- Mission orchestrator (`codes/finals/stage2_mission.py`) — full lifecycle: takeoff → fly to pad → land → pause → take off again → hover → ArUco search → final land
- Fake-drone simulator (`codes/finals/mocks/pyhulax_mock.py`) — including a video stream that paints real ArUco markers we can detect
- Real ArUco detector using OpenCV (DICT_6X6_250)
- Snapshot saver: writes annotated JPEG (marker outlined + ID label) + JSON sidecar with decoded ID
- Launcher script (`codes/finals/run_stage2.sh`) — one command starts everything
- **18 automated tests** that all pass in ~1 second

**What still needs to happen:**
- At the venue: confirm Wi-Fi allows drone discovery (§9 Test A)
- At the venue: confirm `Direction.FORWARD` matches our assumption (§9 Test B)
- At the venue: tune ArUco dictionary + detector params to actual lighting (§9 Test C)
- That's it for software. The rest is flying and scoring points.

---

## 3. The Finals competition explained simply

Source of truth: `materials/Finals brief.pptx` (read in PowerPoint or run `libreoffice --view "materials/Finals brief.pptx"`). The 6-bullet Pre-U version:

### Stage A — Landing (Scoring Item 1, ≈ 44% Pre-U total)

1. Organizers post **pad coordinates on Discord** with valid/invalid flags announced before assessment.
2. We pick **3 valid pads** out of the available 5.
3. From the **C2 Terminal** (our laptop), we launch **3 Highgreat HULA drones** to fly to the chosen pads and land.
4. Scored on: **Number of landings within hoop** + **timing**.

### Stage B — Ambush (Scoring Item 2, ≈ 44% Pre-U total)

5. Five **RoboMaster ground robots** are released into the cage. They drive around for a period. **Three robots are autonomous; two are remote-controlled by another team** (see the "RoboMaster Opponent" column in the brief schedule — this is your competitive matchup).
6. We **launch the HULAs again** (they take off from the pads), hover at 1.1 m, and use their cameras to detect ArUco markers printed on each ground robot. Then land.
7. Scored on: **Number of ArUco detections** + **timing**. The decoded marker IDs are the load-bearing artefact — judges check them.

### Bonus (≈ 4 + 8 = 12% Pre-U total)

8. **Counter UAS Tech Showcase** task at a separate booth (§10). Don't skip this — it's free points.
9. **Overall concept explanation** to judges. Be ready with a 2-minute pitch.

### Hard rules (violating these invalidates your score)

- HULAs **may not fly over obstacles**. Recommended height is **1.1 m**. Anything taller risks invalidation.
- HULA **max speed 0.5 m/s**.
- **No re-attempts** if the drone crashes. Be conservative.
- Stage 2 mission has a hard ceiling of **8 minutes**. Our code defaults to 9-minute safety timeout but you should aim to finish well under 8.

**Total mission time:** about 5-8 minutes from start to finish, in one continuous run (no Ctrl+C between phases unless something goes wrong).

---

## 4. Schedule (when does what happen)

From the brief's Schedule page. Pre-U highlights:

| Day | Time | What |
|---|---|---|
| Day 1 | 0930-1030 | **Briefing** (attend with the whole team) |
| Day 1 | 1030-1300 | Testing (Pre-U can test the HULA in the Drone Cage) |
| Day 1 | 1330-1800 | More testing time |
| Day 2 | 0900-1230 | Final testing |
| Day 2 | 1330-1600 | **Challenge 2** — the actual competition |

**Testing rules:**
- 5 minutes per testing session in the cage.
- **20-minute cooldown** between sessions (no re-queue during cooldown).
- First-come-first-serve queue; join only when ready to test.
- "Testing operates on a First-Come-First-Serve (FCFS) queue system. No prior online booking."
- At most 2 teams in 1 cage simultaneously.

**Important:** Plan testing slots back-to-back with **20-minute cooldowns**. You can probably get 3-4 sessions in over Day 1 if you queue smartly. Use the cooldown windows to tune env vars based on what you saw.

**Our slot in Challenge 2:** Check the brief's "Order of Assessment (Challenge 2)" page for our team's sequence number — the team facing us as RoboMaster Opponent is in the same row.

---

## 5. Terminal basics (if you've never used one)

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
| `./run_stage2.sh --short` | Run a script. The `./` at the start means "run the file that's right here". | Used in §6. |
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

## 6. Running the test mission on this machine

This is the most useful thing to do first. **It proves the software works** and takes 25 seconds.

**Step 1 — Open a terminal** (see §5 if you don't know how).

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
[run_stage2] mode=mock phase=both pads=.../pads_example.json ambush=8s ...
[main] backend=mock cv2=yes phase=both aerial_search=True
[main] output_dir=/tmp/test_snapshots ambush_window_s=8.0 takeoff_alt_m=1.1 search_alt_m=1.1
[main] loaded 5 pads, chose ['P1', 'P3', 'P5']
[main] discovered 3 HULAs: ['plane1', 'plane2', 'plane3']
[main] plane1 (192.168.1.100) -> pad P1
... (state transitions for all 3 drones in parallel)
[plane3] pad_hold -> search_takeoff
[plane3] search_takeoff -> ambush_watch
[plane3] snapshot 1 -> /tmp/test_snapshots/plane3_001_xxx.jpg (ids=[2])
...
[plane1] ambush_watch -> final_land
[plane1] final_land -> complete

=== MISSION SUMMARY ===
  plane1: state=complete pad=P1 snapshots=3 marker_ids=[2, 3] err=-
  plane2: state=complete pad=P3 snapshots=3 marker_ids=[2, 3] err=-
  plane3: state=complete pad=P5 snapshots=3 marker_ids=[2, 3] err=-
  TOTAL unique ArUco IDs detected: [2, 3] (count=2)
```

**What this means:**
- "backend=mock" — using the fake-drone simulator (correct for testing)
- Each plane went through the **two-phase** lifecycle: land → pad_hold → take off again → search → final land
- The detector found real ArUco markers (IDs 2 and 3) the simulator drew into the video
- The final "TOTAL unique ArUco IDs" line is the scoring-relevant number

**Step 4 — Look at the saved snapshots:**
```bash
ls /tmp/test_snapshots
```
You'll see pairs of `.jpg` and `.json` files.

To open one image:
```bash
xdg-open /tmp/test_snapshots/plane1_001_*.jpg
```
You should see a grey background with a **black-and-white ArUco marker**, a **green outline around it**, and a **yellow ID label** above it. That's a scored snapshot — judges will look for these IDs.

To peek at the JSON:
```bash
cat /tmp/test_snapshots/plane1_001_*.json
```
The `"marker_ids"` field is what gets scored.

**Step 5 — Run the unit tests** (this proves the code is correct):
```bash
cd ~/BrainHack-26/codes/finals
python3 -m unittest tests.test_stage2 -v
```
You should see 18 tests run, all ending in `ok`, finishing in ~1 second with `OK` on the last line.

**If anything in steps 3–5 didn't work**, jump to §11 (Troubleshooting).

---

## 7. What every file does

The repo has a lot of files because it grew out of the Qualifier (which we passed). For Pre-U Finals you mostly only care about the small `codes/finals/` folder. Here's the map.

### 7.1 What matters for Pre-U Finals (the active stack)

| File | Purpose | When to look at it |
|---|---|---|
| `codes/finals/stage2_mission.py` | **The main program.** ~600 lines. State machine, navigation, ArUco detector, snapshot saver. | If you need to change behavior. |
| `codes/finals/mocks/pyhulax_mock.py` | Fake-drone simulator for offline testing. ~310 lines. Includes real ArUco markers in synthetic video. | Almost never — it just works. |
| `codes/finals/tests/test_stage2.py` | 18 automated tests. | Run after any change to `stage2_mission.py`. |
| `codes/finals/run_stage2.sh` | Launcher script. | Every time you run the mission. |
| `codes/finals/pads_example.json` | Sample pad list (for testing). The real coordinates come **from Discord** on competition day. | When you need to test with custom pads. |
| `materials/Finals brief.pptx` | **The official rules document.** Read this before competition day. | Before the competition; whenever rules are unclear. |
| `materials/RoboVerse 2026 Finals.pdf` | Earlier rules version. The brief.pptx supersedes it where they differ. | Cross-reference only. |
| `references/finalist_codes/hula_swarm/huladola.py` | Workshop's reference code (65 lines). The basic pyhulax pattern we copied. | If `stage2_mission.py` does something confusing — check the simpler reference. |
| `references/finalist_codes/aruco_detection/aruco_detection.py` | Reference for ArUco detection patterns. | If you want to extend the detector. |

### 7.2 Reference material (read if curious, ignore otherwise)

| Folder/File | What's in it |
|---|---|
| `references/finalist_codes/realsense_cam/`, `rknn_detect/`, `uwb_mavsdk/`, `model_convert/` | University-team-only reference code. Pre-U doesn't use these. |
| `references/qualifier_codes/` | Workshop reference for the Qualifier. We finished that already. |
| `materials/LearningMaterial*.pdf`, `Supplmentary_LearningMaterial*.pdf` | Workshop slides. |
| `materials/RoboVerse 2026 Qualifier.pdf` | Qualifier rules (history). |
| `TUTORIAL.md` | ~1000 lines explaining concepts. Chapter 24 ("HULA swarm via pyhulax") is the Pre-U-relevant section. |
| `RUNBOOK.md` | A step-by-step prep plan from earlier sessions. Phases 0-6 are Qualifier (done); Phases 7-15 are Finals (mostly University-team). |
| `docs/kolomee_dissection.md` | Deep dive on a University-team reference script. Pre-U doesn't touch it — but the *patterns* (threading, mocks, bug-hunting) are educational. |

### 7.3 Older code (Qualifier era — left as reference, you don't run it)

| Folder/File | Why it's still here |
|---|---|
| `codes/mission.py`, `barrel_tracker.py`, `Detector.py`, `AvoidancePlanner.py`, etc. | The Qualifier mission. Still works in the simulator. |
| `codes/run_competition.sh`, `install.sh`, `RUN.txt`, `JUDGE_SUMMARY.md` | Qualifier launchers + writeups. |
| `codes/barrel_yolo.pt` | Qualifier YOLO model (5.5 MB). Not used for Finals. |
| `codes/sim_uwb_bridge.py`, `codes/finals/kolomee_sitl.py` | Stage 1 SITL experiments from earlier prep. Not used for Pre-U. |
| `codes/measure_drift.py`, `codes/depth_diag.py`, etc. | Qualifier debug tools. |
| `scripts/start_sim.sh`, `stop_sim.sh`, `set_ekf_origin.py`, `set_ekf_origin.sh` | Qualifier simulator helpers. |
| `px4-patches/`, `sdf-patches/` | Qualifier simulator patches. |

**Rule of thumb:** if it's in `codes/finals/`, it's Pre-U-relevant. Anything else, you can mostly ignore unless you're curious.

---

## 8. Before competition day — what you can do at home

There isn't much critical software work left. Things you *could* do, in priority order:

### 8.1 Practice running the mission (highly recommended)

Run §6 once a day until competition. It's 25 seconds and proves nothing has broken. Familiarity = confidence on the day.

### 8.2 Read `materials/Finals brief.pptx` carefully (highly recommended)

Especially:
- Challenge Two gameplay (the load-bearing rules)
- Scoring rubrics
- The hard rules ("Strictly no flying over obstacles. The recommended height is 1.1m.")
- The Order of Assessment for Challenge 2 — find our team and our RoboMaster Opponent
- The Logistics page — note what they provide (1 HULA for testing — not 3)

### 8.3 Practice typing commands in a terminal (recommended)

On competition day you'll be under time pressure. Type these blind a few times until they're muscle memory:

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

### 8.4 Read §9 (Competition site) at least twice

§9 is the most important section. Memorize the three calibration tests.

### 8.5 Pre-print materials (low priority)

If your team is presenting the concept explanation, print 2 backup copies. Have the brief PDF ready offline on the laptop.

### 8.6 Optional code improvements (very low priority)

- Set `BH26_ARUCO_DICT` to a different dictionary if organizers confirm something other than DICT_6X6_250.
- Add a YOLO RoboMaster detector as a secondary signal (low value — ArUco is what's scored).

**None of these are required.** The current code matches the brief.

---

## 9. At the competition site — step by step

This is the most important section. Read it before you arrive.

**What to bring:**
- The laptop with this repo on it
- Power cable + adapter
- A small notebook + pen to write down env-var values you tune
- Phone for taking the Counter UAS bonus photo (§10)

The organizers provide: 1 HULA drone, 1 sample landing pad, 1 sample ArUco pad, charger, mouse, the laptop (you may use yours instead).

### 9.1 Day 1 morning: briefing + setup

1. Attend the **0930-1030 briefing**. Take notes — they'll likely answer most of our open questions (§12).
2. Set up your laptop:
   - Open a terminal (§5).
   - Verify the repo is still here: `cd ~/BrainHack-26 && ls`. You should see `codes/`, `materials/`, etc.
   - Run the smoke test (§6) to confirm nothing broke during transport.

### 9.2 Connect to the venue Wi-Fi

The HULAs talk to the laptop over Wi-Fi using **multicast** (a way for devices on the same network to find each other automatically).

1. Join the Wi-Fi network the organizers give you.
2. Verify you have an IP address: type `ip addr` in the terminal — look for something like `192.168.x.x` under a section labeled `wlan` or `eno`.
3. **Critical check:** ask the organizers whether their Wi-Fi allows **multicast traffic between client devices**. Some venue networks block this. If blocked, the drones won't be discoverable.

### 9.3 Install pyhulax

You need the real pyhulax library on the laptop. The organizers should provide install instructions on the day. The general pattern (do this only when they tell you):

```bash
pip install pyhulax
```

To verify it worked:
```bash
python3 -c "import pyhulax; import dola; print('OK')"
```
If you see `OK`, you're good. If you see `ModuleNotFoundError`, the install didn't succeed — ask an organizer.

### 9.4 Three calibration tests (do these in your first testing slot)

You have **5 minutes per test session, 20-minute cooldown after**. Plan to do all three calibrations in your first slot — it's tight but doable. Use the cooldown to update env vars based on what you saw.

#### Test A — Discovery works

```bash
cd ~/BrainHack-26/codes/finals
python3 -c "
from dola import Dola
d = Dola(); d.start()
print(d.get_all_ips(listen_seconds=5))
d.stop()
"
```
You should see a dict like `{'plane1': '192.168.x.y'}` printed (you'll have one HULA for testing). **If you see `{}`, the multicast network is blocking discovery.** Talk to the organizers.

#### Test B — Direction calibration

This tells you whether `Direction.FORWARD` actually moves the drone in the direction we assumed (+x in arena coordinates).

Create a one-pad file `test_pad.json`:
```bash
cat > /tmp/test_pad.json <<'EOF'
[{"id": "T1", "x": 2.0, "y": 0.0, "z": 0.0, "valid": true}]
EOF
```

Temporarily edit `NUM_DRONES = 3` in `codes/finals/stage2_mission.py` to `NUM_DRONES = 1` (use the GUI text editor — search for `NUM_DRONES`).

Run a one-HULA test:
```bash
./run_stage2.sh --real --pads /tmp/test_pad.json --ambush 5 --phase land
```

Watch the drone. Mark "+x" on the arena floor with tape **before** the test (the brief calls this out — know the arena's coordinate convention).

- If it flies in the +x direction: convention correct, no env vars needed.
- If it flies in the -x direction (opposite): `export BH26_AXIS_FORWARD=-x`
- If it flies in the +y direction (perpendicular): `export BH26_AXIS_FORWARD=+y`
- If it flies in the -y direction: `export BH26_AXIS_FORWARD=-y`
- Then repeat the test until it flies the right way.
- **Write down the env-var values you settled on** — you'll need them for the real mission.
- Don't forget to set `NUM_DRONES = 3` back when calibration is done.

#### Test C — ArUco detector calibration

The brief uses ArUco markers on the ground robots. Our code defaults to `DICT_6X6_250` (the most common). The sample ArUco pad the organizers provide will tell you the dictionary.

1. Hold the sample ArUco pad in front of the powered-on HULA's camera (organizer help may be needed to view the video feed).
2. Run a short mission that lets you check snapshots:
   ```bash
   ./run_stage2.sh --real --pads /tmp/test_pad.json --ambush 10 --output /tmp/calib --phase search
   ```
   (`--phase search` skips landing — the drone hovers near the C2 and watches.)
3. Look at the saved snapshots:
   ```bash
   ls /tmp/calib
   xdg-open /tmp/calib/*.jpg
   ```
4. If a snapshot was taken with the correct ID visible (matching the sample pad), the dictionary is right.
5. If **no snapshots were saved**: try other dictionaries:
   ```bash
   export BH26_ARUCO_DICT=DICT_4X4_50
   # or DICT_5X5_100, DICT_APRILTAG_36h11
   ```
   Re-run.
6. **Write down the dictionary name that worked.**

### 9.5 Day 2: run the actual mission

When the organizers give you the official pad coordinates **via Discord**:

1. Save them as `competition_pads.json` in `~/BrainHack-26/codes/finals/`. Match this format (just write it into a text editor):
   ```json
   [
     {"id": "P1", "x": 1.5, "y": 2.0, "z": 0.0, "valid": true},
     {"id": "P2", "x": -1.0, "y": 0.5, "z": 0.0, "valid": false},
     ...
   ]
   ```
   If the Discord format is different (CSV, text), translate it by hand into this JSON — it's only 5 pads.

2. Open the terminal and run:
   ```bash
   cd ~/BrainHack-26/codes/finals

   # Apply your calibration env vars (replace placeholder values with what you wrote down in §9.4)
   export BH26_AXIS_FORWARD=+x
   export BH26_AXIS_RIGHT=+y
   export BH26_ARUCO_DICT=DICT_6X6_250

   # Run the mission for real
   ./run_stage2.sh --real --pads competition_pads.json
   ```

3. Watch the output. State transitions print live. When the mission completes, you'll see the `=== MISSION SUMMARY ===` block.

4. Snapshots are saved to `./snapshots/`. Inspect them:
   ```bash
   ls snapshots
   # The total marker IDs detected is what gets scored:
   grep -h marker_ids snapshots/*.json | sort -u
   ```

5. **Inform the judge that the mission is complete** (per the brief: "Teams to inform the judge to indicate that the mission is completed").

6. **Submit snapshots** per the organizers' instructions on the day.

### 9.6 If something goes wrong mid-mission

- **Stop the program**: press `Ctrl+C` in the terminal. This signals the orchestrator to safely land all drones and exit.
- **No re-attempts on crash** — per the brief, a crashed drone forfeits that scoring. Be conservative.
- **Common mid-mission problems:** see §11.

---

## 10. Bonus task (don't skip — free points)

Worth **4%** for Pre-U. Easy and unrelated to code.

**Steps** (per the brief):
1. **Head to the Counter UAS Tech Showcase booth** — in the first zone, "Above & Beyond: Skies & Space" area.
2. **Complete the task** on the Brainhack Frontier Exploration System.
3. **Snap a picture of the drone** at the Counter UAS booth.
4. **Screenshot the "zone explored" page** on the Frontier Exploration System.

Submit both per the organizers' instructions.

There's also an additional **8%** for "Overall concept explanation" — be ready with a 2-minute team pitch about your approach. Have at least one team member rehearse it.

---

## 11. Troubleshooting common problems

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
Same as above — `--real` mode needs the real library. For testing without HULAs, use `--mock` or `--short`.

### `RuntimeError: need 3 HULAs, only found N`
The discovery only found N drones. Causes:
- Drones aren't powered on
- Drones aren't connected to the same Wi-Fi as your laptop
- Wi-Fi blocks multicast (§9.2) — ask the organizers
- One of the drones has a low battery and didn't boot

### `RuntimeError: need 3 valid pads, got N`
The pad file has fewer than 3 valid pads. Either:
- The organizers' pad list legitimately has < 3 valid pads (unlikely; brief says 5 pads, some valid some invalid).
- The file is in a different format than expected. Check field names match `pads_example.json`.

### Mission runs but no snapshots are saved
The detector isn't firing. Try:
- Verify ArUco dictionary matches the sample pad (Test C in §9.4).
- Check that the HULA camera is producing frames at all (organizer can help with their video display).

### Mission runs but snapshots are full of false positives
The detector is too sensitive. The ArUco detector is pretty robust — false positives are rare unless the sample pad happens to match a marker pattern in the background. Usually safe to ignore unless the count is wildly high.

### Drone flies the wrong direction
Calibrate the axis convention — §9.4 Test B.

### Drone arrives at the wrong altitude
Adjust `BH26_TAKEOFF_ALT_M` to match where the HULA actually hovers after `.takeoff()`. Default is 1.1 m (per brief). If the HULA only reaches 0.8 m, try `export BH26_TAKEOFF_ALT_M=0.8`.

### Mission completes but only Stage A scored (no ArUco detections logged)
The `aerial_search` phase didn't fire. Check:
- Was `BH26_DO_AERIAL_SEARCH=1` (the default)?
- Did the search-takeoff fail? Check the log for `SEARCH_TAKEOFF` → `EXCEPTION`. If pyhulax doesn't allow re-takeoff after landing on real HULAs, you'll need to combine the phases differently — fall back to `--no-aerial` and watch from landed position.

### "Drone took off again but mid-air it crashed / failsafed"
- Check that `BH26_SEARCH_ALT_M` is at or below the brief's recommended 1.1 m.
- If the HULA has its own safety logic that prevents back-to-back takeoffs, you may need to add a longer `BH26_PAD_HOLD_S` (try 10 or 20 seconds) so the drone fully settles before takeoff.

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

## 12. Questions still open for organizers

Many of our earlier questions are now answered by the brief. These remain:

**About the network:**
1. Does the venue Wi-Fi allow **multicast** traffic between client devices (needed for HULA discovery via Dola)? If not, is there an unmanaged switch / hotspot we can use?

**About pyhulax:**
2. Where do we install `pyhulax` and `dola` from on competition day? (`pip install`? Local wheel?)
3. Does pyhulax expose a `.goto(x, y, z)` method, or only `.move(direction, distance)`?
4. What unit is the second argument to `.move()`? (Assumed metres — please confirm.)
5. Is `Direction.FORWARD` body-frame (relative to drone heading) or world-frame? (Assumed body-frame.)
6. What altitude does `.takeoff()` reach by default? (We assume 1.1 m per brief recommendation.)
7. **Can a HULA take off again after landing?** Our mission requires this (land on pad for Scoring Item 1, then take off again for Scoring Item 2). What's the safe interval between land and takeoff?

**About the mission:**
8. Which ArUco dictionary do the ground robots use? (We default to DICT_6X6_250.)
9. What's the exact Discord format for the pad list? Text? Table? JSON?
10. What signals the start of Stage B (convoy entry)? An organizer signal? A timer? Do we Ctrl+C between phases?

**Already answered by brief — recorded for completeness:**
- ✅ 5 pads, pick 3
- ✅ 5 RoboMaster ground robots, 2 controlled by opponents
- ✅ Marker = ArUco printed on robot
- ✅ Max speed 0.5 m/s
- ✅ Recommended height 1.1 m
- ✅ No flying over obstacles
- ✅ No re-attempts on crash
- ✅ 8-minute max mission time
- ✅ Inform judge when mission complete

---

## 13. Glossary of unfamiliar terms

Terms you'll see in this doc, the code, or the rules.

| Term | Meaning |
|---|---|
| **Pre-U / Pre-University** | Our competition category. We do Challenge 2 only. |
| **HULA** | Highgreat HULA — the small flying drones we control. 3 of them. |
| **RoboMaster** | Small ground robots (made by DJI). They drive around with ArUco markers attached. Our HULAs spot the markers. |
| **C2 / C2 Terminal** | "Command and Control terminal" — fancy term for "the laptop running our software". |
| **pyhulax** | The Python library that lets us send commands to HULA drones. Made by the HULA manufacturer. |
| **Dola** | A small Python tool (`dola` library) that finds HULAs on the local Wi-Fi network. Stands for "drone explorer" loosely. |
| **multicast** | A way for devices on the same network to send a message "to whoever's listening". HULA discovery uses it. Some Wi-Fi networks block it. |
| **ArUco** | A type of black-and-white square marker (like a small QR code) with a printed ID number. OpenCV detects them reliably. The brief uses them on ground robots. |
| **ArUco dictionary** | A specific set of valid markers. DICT_6X6_250 means "6x6 pixel patterns, 250 unique IDs". Different dictionaries can't read each other's markers. |
| **landing pad / hoop** | A marked spot on the arena floor where HULAs should land. Brief calls it "hoop" because it's likely a physical ring. |
| **ambush window** | The period after landing during which HULAs take off again and watch for ground robots. Default 120 seconds in our code. |
| **state machine** | A program pattern where each drone is "in a state" (idle, taking off, flying, etc.) and only does the actions allowed by that state. Our orchestrator uses this. |
| **mock** | A fake version of something (here, fake HULAs and fake video) used for offline testing. |
| **env var / environment variable** | A setting you change before running a program, like `BH26_AMBUSH_S=30`. Persists for the rest of the terminal session. |
| **terminal** | The black window with a text prompt. Where you type commands. |
| **stdout / log** | The text the program prints while running. Scrolls in the terminal. |
| **body-frame / world-frame** | Ways of describing direction. "Body-frame forward" = "forward relative to where the drone is currently facing". "World-frame forward" = "forward in the room", which might be different. |
| **`.goto()` vs `.move()`** | Two possible drone commands. `.goto(x, y, z)` would tell the drone "go to this exact position". `.move(direction, distance)` says "go this many meters in this direction from where you are". We only know `.move()` from the workshop reference. |
| **PAD_HOLD** | A short state in our state machine after landing — the drone sits on the pad for a few seconds before re-taking off to search. |
| **SEARCH_TAKEOFF / AMBUSH_WATCH / FINAL_LAND** | States in our state machine for the search phase. |
| **mavsdk, PX4, Gazebo, EKF, kolomee, UWB, RealSense, RKNN** | Things from Stage 1 (University-team only). You can completely ignore these. |

---

## 14. Going deeper (optional reading)

You don't need any of this to compete. Read it if you're curious or want to understand the code more deeply.

**Pyhulax patterns:** `TUTORIAL.md` Chapter 24. Explains why we use one thread per drone, lists the open questions about pyhulax.

**The workshop's reference code:** `references/finalist_codes/hula_swarm/huladola.py`. Just 65 lines. Our orchestrator is structurally an expansion of this script.

**ArUco detection reference:** `references/finalist_codes/aruco_detection/aruco_detection.py`. Shows the workshop's pattern for ArUco detection.

**The competition rules in full:** `materials/Finals brief.pptx`. Read in PowerPoint / LibreOffice Impress.

**The Qualifier code we shipped:** `codes/mission.py` and friends. We won't run this at Finals but it's a clean example of the same patterns (state machines, threading, detection) on a different problem.

**Why we set up the simulator bridge:** `docs/kolomee_dissection.md` and `codes/sim_uwb_bridge.py`. This was University-Stage-1 prep that we later discovered Pre-U doesn't need. The patterns it exercises (async/threading, mocking, finding bugs in workshop code) are educational.

**The architecture decision log:** Our git history. `git log --oneline` shows the high-level story.

---

## A final note

This codebase is intentionally small and aligned to the brief. There's one orchestrator program, one mock for testing it, one launcher to run it, and 18 tests to guarantee it works. Everything else in the repo is either reference material, the rules, or Qualifier history.

**On competition day, you're in the orchestrator-pilot seat.** The drones know how to fly. The tests prove the code works. All that's between you and a passing run is plugging in the actual hardware, doing the 3 calibration tests, and reading the output.

**Good luck.** You've got this.
