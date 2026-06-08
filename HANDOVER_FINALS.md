# Handover — Finals prep, Pre-U scope

**Date:** 2026-06-08
**Context:** BrainHack-26 made it through the Qualifier and is now prepping for the Finals. **Team is in the Pre-University category, which does Challenge 2 only** (HULA swarm landing + ambush). Challenge 1 (Mapping Drone + UWB nav) is University-only.

If you are a new agent picking this up: read this top to bottom (~10 min) before anything else. The bottom-line story is in §1; the actionable list is in §6.

---

## 1. One-paragraph status

The Pre-U Stage 2 software stack is **end-to-end testable on the dev VM**. The orchestrator (`codes/finals/stage2_mission.py`), a thread-per-drone state machine, runs cleanly against a faithful pyhulax mock (`codes/finals/mocks/pyhulax_mock.py`). The HSV color-based RoboMaster detector fires on synthetic frames and saves annotated JPEGs + JSON sidecars. The launcher (`run_stage2.sh`) and a 16-test unit suite (`tests/test_stage2.py`) cover the happy path and edge cases. The bridge + kolomee SITL work from earlier in the session is **not on the Pre-U critical path** (it was useful learning + remains as reference; the EKF drift documented in `docs/kolomee_dissection.md §11` is unfixable on this VM for kolomee-style velocity nav). Remaining work is mostly hardware-gated: pyhulax navigation API confirmation, RoboMaster detector calibration against real footage, and one venue session.

---

## 2. Pre-U Finals competition, in 30 seconds

Per `materials/RoboVerse 2026 Finals.pdf`, Challenge Two:

1. Organizers publish a landing-zone map (valid/invalid flags). **No Mapping Drone needed by Pre-U.**
2. We pick 3 valid zones, launch 3× Highgreat HULA drones from a C2 laptop running pyhulax.
3. HULAs land accurately on the chosen pads.
4. 5× RoboMaster ground robots enter the cage as a "convoy"; HULAs detect + snapshot them with their onboard cameras.

Scored on landing accuracy + time, then snapshot count + accuracy + time. Prize structure identical to University category.

---

## 3. Repo state

### 3.1 Branch

`prep/finals`, currently **4 commits ahead of origin** (push needs your terminal — no SSH/PAT/gh in this shell):

```
dbd8942 stage2: pyhulax mock + colour-based detector + cv2 snapshot writer
883d25b stage2: pre-u finals orchestrator skeleton
e5745fb docs: EKF patch tried for kolomee SITL, made things worse — reverted
437fdb6 finals integration: verify sim_uwb_bridge end-to-end against kolomee_sitl
0d9a1ac finals prep: sim UWB bridge + handover doc; drop stale qualifier handover
7322371 docs: extend TUTORIAL + RUNBOOK for Finals; dissect kolomee.py
```

### 3.2 Pre-U Stage 2 files (the active stack)

| Path | Status |
|---|---|
| `codes/finals/stage2_mission.py` | Orchestrator. Threaded per-drone state machine. `navigate_to_pad` implemented with documented body-frame axis convention (env-overridable for venue calibration). HSV RoboMaster detector with area + aspect filters. cv2 snapshot writer with cooldown. |
| `codes/finals/mocks/pyhulax_mock.py` | Faithful pyhulax + dola mock. `DroneAPI` enforces a strict state machine; `VideoStream` injects red RoboMaster patches at tunable intervals; `Dola.get_all_ips` returns `BH26_MOCK_DRONES` drones. |
| `codes/finals/tests/test_stage2.py` | 16 unit tests — all green in <1 s. Covers pad parsing, validity filtering, detector positive/negative + speckle + aspect rejection, snapshot JPEG + JSON + cooldown, navigate_to_pad direction sequences for all four quadrants + zero-axis + sub-epsilon, mock state-machine invariants. |
| `codes/finals/run_stage2.sh` | Launcher. `--mock` (default) / `--real`, `--pads PATH`, `--ambush S`, `--robos N`, `--short` (fast smoke). |
| `codes/finals/pads_example.json` | 6-pad example with 3 valid + 3 invalid. |

### 3.3 Reference / earlier-session artefacts (kept, not on critical path)

| Path | Why kept |
|---|---|
| `codes/sim_uwb_bridge.py` | Gazebo ground-truth → ROS2 `/uwb_tag` bridge with noise model. Working. Useful general async + ROS2 reference. Not used for Pre-U. |
| `codes/finals/kolomee_sitl.py` | SITL-adapted kolomee with 5 documented bugs fixed. Works end-to-end up to PX4 EKF drift (sim-side, unfixable on this VM). Useful pattern reference. Not used for Pre-U. |
| `docs/kolomee_dissection.md` | Full dissection of the workshop's UWB nav script. §5.4 bug table + §11 SITL verification log are the load-bearing parts. |
| `codes/mission.py`, `barrel_tracker.py`, etc. | Qualifier mission. Still works in Gazebo. |
| `px4-patches/`, `sdf-patches/`, `codes/measure_drift.py` | Qualifier EKF tuning. Stale for Finals real hardware. |

### 3.4 What's missing for Pre-U end-to-end

| Missing | Hardware-gated? | Estimated effort |
|---|---|---|
| pyhulax navigation API confirmation (.goto vs body-frame .move) | **Yes** | 30 minutes of bench time |
| RoboMaster detector calibration on real footage | **Yes** (footage) | 30 min |
| One venue integration session: network multicast for Dola, takeoff convention, landing accuracy | **Yes** | ~2 h |

**Software-only work remaining: zero non-trivial items.** The codebase is at a clean "ready for hardware day" checkpoint.

---

## 4. How to run things

### 4.1 Mock end-to-end (no hardware needed)

```bash
cd ~/BrainHack-26/codes/finals
./run_stage2.sh --short --output /tmp/bh26_snapshots
ls /tmp/bh26_snapshots
```

Expect: 3 drones complete their lifecycle in ~25 s, 9-12 snapshots saved (3 per drone, depending on robomaster spawn timing).

### 4.2 Unit tests

```bash
cd ~/BrainHack-26/codes/finals
python3 -m unittest tests.test_stage2 -v
```

Expect: `Ran 16 tests in <1s`, `OK`.

### 4.3 Real hardware day

```bash
cd ~/BrainHack-26/codes/finals
./run_stage2.sh --real --pads <organizers' pad file>
```

The script will refuse to run if `pyhulax` / `dola` aren't importable. Install per pyhulax.xenops.ae docs first.

### 4.4 Calibration env vars (use these at the venue)

| Env var | Default | What to flip if calibration shows it's off |
|---|---|---|
| `BH26_AXIS_FORWARD` | `+x` | If Direction.FORWARD sends drone arena-east, set to `+y` (etc) |
| `BH26_AXIS_RIGHT`   | `+y` | Same, for Direction.RIGHT |
| `BH26_TAKEOFF_ALT_M` | `1.0` | Whatever altitude HULAs hover at after `.takeoff()` |
| `BH26_DETECT_MIN_AREA`, `_MAX_AREA`, `_MIN_ASPECT`, `_MAX_ASPECT` | sane defaults | Tune to real RoboMaster armor plate size at flight distance |
| `BH26_AMBUSH_S` | 120 | Length of ambush-watch window |
| `BH26_SNAPSHOT_COOLDOWN_S` | 2.0 | Min seconds between snapshots of the same scene |

---

## 5. Open questions for hardware day

The skeleton documents each clearly with TODO comments; for context:

1. **Does pyhulax expose `.goto(x, y, z)` or similar absolute-position API?** If yes, prefer it over the body-frame decomposition in `navigate_to_pad` — drift across multiple sequential moves becomes a non-issue. Check `pyhulax.xenops.ae`.
2. **Is `Direction.FORWARD` body-frame or world-frame?** Almost certainly body. If body, our assumption that all HULAs face the same direction at takeoff (because they launch from the same C2 pad) needs a quick sanity check.
3. **Unit of `.move(direction, distance)` second arg?** `huladola.py` comment claims metres; verify with a tape measure for the first move.
4. **Does Dola multicast work on the venue Wi-Fi?** Some managed networks block client-to-client multicast. Test discovery on day 1.
5. **Are RoboMasters all red, all blue, or mixed?** The HSV ranges in the detector currently only catch red. If blue is needed, add a second mask.

---

## 6. Shortest path forward (next session)

Don't pile on more software — the codebase is in a clean checkpoint. The right next move depends on whether you have hardware time:

**If you have hardware time soon:**
1. Run `./run_stage2.sh --short --output /tmp/smoke` to confirm everything still works locally.
2. Push the 4 unpushed commits from your terminal.
3. At the venue: run the script with `--real`, calibrate the axis convention + detector thresholds via the env vars in §4.4.

**If hardware time isn't scheduled yet:**
1. **Ask the organizers for any pre-competition hardware time, even 60-90 min** — it unblocks 60% of the open questions in §5.
2. While waiting, polish: collect stock RoboMaster images and pre-calibrate `BH26_DETECT_*` thresholds; write a `RUNBOOK.md` Phase 7 entry that's specifically the venue checklist.

**If you genuinely want to write more software here:**
1. Add a blue-armor HSV mask alongside the red one (defensive: rules don't specify color).
2. Add a YOLO RoboMaster detector as a fallback path, switchable via env var. Reuse the Qualifier `barrel_yolo.pt` training pipeline.
3. Build a `pad_input_parser.py` that tolerates 3-4 likely organizer JSON formats so day-zero confusion is impossible.

None of these unblock anything critical. Pre-U end-to-end is achievable as-is given hardware time.

---

## 7. Memory pointers

`/home/drone/.claude/projects/-home-drone-BrainHack-26/memory/`:

- `project_preu_scope.md` — **Pre-U does Challenge 2 only**. This was the load-bearing scope correction this session.
- `project_sitl_ekf_kolomee.md` — Why SITL kolomee testing has a hard ceiling. Don't retry the EKF patch.
- `project_protobuf_workaround.md` — Still applies to the bridge if reused.
- `feedback_restart_between_scripts.md` — Sim restart pattern, relevant only if running the bridge.

The Qualifier-era memories are still on disk and informative for context, but Pre-U Stage 2 doesn't touch any of them.

---

## 8. What to read first if you've never seen this repo

1. This document.
2. `materials/RoboVerse 2026 Finals.pdf` — focus on the "Challenge Two" section.
3. `codes/finals/stage2_mission.py` top docstring + `navigate_to_pad` docstring.
4. `codes/finals/mocks/pyhulax_mock.py` top docstring.
5. `TUTORIAL.md` Ch 24 (pyhulax) — answers "why threading not asyncio" and lists open API questions.

Total time: ~30 min for a confident pickup. The codebase is small, well-commented, and tested.

---

*End of handover. Repo is at a clean checkpoint, working tree clean, 4 unpushed commits on `prep/finals`. Push from your terminal then either move to hardware day or pause until hardware time is available.*
