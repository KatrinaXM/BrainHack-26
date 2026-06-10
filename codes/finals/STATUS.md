# STATUS — BrainHack-26 Finals, Pre-U Challenge 2 (Stage 2)

**Running project-status doc.** Keep this current; it's the fast way for any
session (human or AI) to know where things stand. Supersedes the old
`HANDOVER_camera_livestream.md` (deleted) with the full picture.

**Last updated:** 2026-06-10 · Branch `prep/finals` · Windows 11 + Git Bash · pyhulax 0.2.0, cv2 4.13

---

## 1. One-line status

Software is **complete and verified in simulation** (24 unit tests pass, mock
mission runs end-to-end both nav modes). **Hardware is partially proven**: drones
connect and fly, but a real flight had **2 of 3 drones drop** — root cause being
nailed down (see §4). Camera livestream bring-up is in progress (§5).

## 2. Software state (verified in mock)

- **24/24 unit tests pass** (`python -m unittest tests.test_stage2`), ~1.5 s.
- **Mock mission** completes the two-phase flow (land → re-takeoff → ArUco search
  → land) in both `move` and `move_to` nav modes.
- **ArUco** (confirmed by organizers, Discord 2026-06-10): dictionary
  `DICT_7X7_1000`, marker IDs `11, 45, 51, 67, 101`, on the **ground robots**
  (not the landing pads). Mission summary reports `EXPECTED IDs … found …`.
- **Units**: pyhulax distances are **centimetres**. Pad files are metres →
  multiplied by `BH26_DIST_SCALE` (default 100). `takeoff(height_cm=110)` for the
  brief's 1.1 m.
- **move() firmware limit (verified from pyhulax source)**: distance must be an
  **integer in [5, 500] cm**. `_split_move_cm()` splits long legs into ≤500 cm
  integer hops (e.g. 750 → [375, 375]); moves issue at conservative speed level
  SLOW (300). **This was almost certainly the cause of the 2/3 drop — see §4.**
- **Navigation** climbs only, never descends mid-flight; `land()` does the final
  descent (avoids dragging across the floor).
- **Obstacle avoidance**: firmware barrier mode enabled at connect (`set_barrier_mode(True)`,
  `BH26_AVOIDANCE=1` default). The drone side-steps/halts rather than climbing over.
- **Rover search = yaw-scan ambush** (`BH26_SEARCH_YAW_SCAN=1` default): during the
  120 s ambush each drone step-and-stares — dwell `BH26_YAW_INTERVAL_S` watching,
  rotate `BH26_YAW_STEP_DEG`, repeat — sweeping the camera ~360° around its pad
  for far better coverage than a static hover, with zero translation risk. A
  failed rotate is logged but never aborts the search. Tilt the camera down for
  the ground rovers via `BH26_CAMERA_ANGLE` (find the angle on the ground with
  `dronecheck.py --camera-angle N` — needs ANGLED, not straight-down, or yaw does
  nothing). **Only sim-validated** (no propellers to fly-test).
- **Closed-loop arrival**: after the open-loop legs, `get_position()` is polled to
  confirm the drone reached the pad (within `BH26_ARRIVE_TOL_M`, default 0.20 m)
  and nudge it if not, BEFORE landing (`BH26_NAV_FEEDBACK=1` default; auto-skips
  if no `get_position`).
- **Fail-safe command layer**: every `takeoff`/`move`/`land` goes through `_cmd`,
  which logs it and checks the `CommandResult`. A rejected command raises instead
  of being silently ignored, and the handler does a **hover-then-land safe stop**
  (a `land()` from a confused state can trip the firmware emergency-stop). A
  pre-flight **battery gate** (`BH26_MIN_BATTERY_PCT`, default 25) refuses to fly
  a weak drone. All actions log with a `[planeN]` prefix for easy debugging.
- **Discovery**: drone **IPs** come from `BH26_HULA_IPS` → `drones.json` → `dola`
  auto-discovery (UDP 8668 broadcast), in that priority. pyhulax connects by IP,
  so dola is optional. **Drone arena positions are NOT discoverable** — the HULA
  broadcasts only IP/identity and self-positions by optical flow; a shared
  absolute frame needs the QR mat (`move_to` + `BH26_QR_LOCALIZATION=1`).
- **Pad config**: up to 5 pads, each with an **availability flag**
  (`available`/`valid`). Only available pads are assigned; need ≥3. Edit with
  `configure.py` (flag-setter) or by hand. **ArUco IDs are logged per detection**
  (`>>> ArUco detected IDs […]`) and summarised vs the expected set.

## 3. Competition facts (locked)

- **Pre-U does Challenge 2 / Stage 2 only.** 3 HULAs land on pads (Scoring Item 1),
  re-launch and detect ArUco on ground robots (Scoring Item 2).
- **Landing points** (`competition_pads.json`, metres, z=0): `7`→(2.5, 5.5),
  `10`→(5.5, 5.5), `12`→(7.5, 5.5). All 3 used (Pre-U lands 3). See its README.
- **Hard rules**: max 0.5 m/s · height 1.1 m · no flying over obstacles ·
  **no re-attempts on crash** (be conservative).
- **Drone network**: each HULA is its own Wi-Fi AP at **`192.168.100.1:8888`**.
  The laptop MUST be on the drone's Wi-Fi (a `192.168.100.x` IP) to connect.

## 4. ⚠ OPEN: 2 of 3 drones dropped on a real flight

**Root cause + fixes (now in code):**
1. The pads are at 5.5–7.5 m = **550–750 cm**, but pyhulax `move()` **rejects any
   distance > 500 cm**. The pre-fix code sent single 550–750 cm moves → firmware
   rejected them. → `_split_move_cm()` splits every leg to ≤500 cm integer hops.
2. The **rejection became a crash** because the old error handler called `land()`
   mid-air from a confused/moving state, which trips the firmware emergency-stop
   (props off) — *that's* the "flop". → commands now go through `_cmd` (checks
   `CommandResult`), and the error path is a **hover-then-land safe stop**.
3. Weak batteries failsafe-drop → **pre-flight battery gate** refuses takeoff.
4. Open-loop drift → **position feedback** confirms arrival before landing.

These should turn the previous crash into, at worst, a controlled land-in-place.
**Still confirm on hardware** with a single-drone re-test before flying all three.

**Other candidates to rule out** (need the run's `=== MISSION SUMMARY ===` /
`[planeX] EXCEPTION` lines, and the telemetry in `logs/drone_*.jsonl`):
- **Battery** — short flight time; a 120 s ambush hover can drain to failsafe.
  Mitigation: full charge, `--ambush 30`, and a pre-flight `get_battery()` check (TODO).
- **No keepalive** during the long hover — if the HULA needs `send_app_heartbeat()`
  to stay aloft, add a heartbeat thread (TODO).
- **Multi-drone origin** — all 3 navigate assuming start `(0,0)`; if they launch
  from different spots, 2 fly wrong. Mitigation: same launch origin, or
  `BH26_NAV_MODE=move_to` + `BH26_QR_LOCALIZATION=1` on a QR mat.

**Next action:** re-test ONE drone to a far pad (e.g. 7) and confirm the split
moves work, before risking all three (no re-attempts rule).

## 5. Camera livestream

The orchestrator is **headless by design** — it never shows video, only saves a
snapshot JPG to `snapshots/` when an ArUco marker is detected. To *see* the feed
(and check everything else, non-flying) use **`dronecheck.py`**:
```bash
python dronecheck.py --ips 192.168.100.1               # camera grid + ArUco + telemetry
python dronecheck.py --ips 192.168.100.1 --telemetry   # text telemetry (by-hand frame test)
python dronecheck.py --ips 192.168.100.1 --check       # headless proof-of-life, exit
python dronecheck.py --ips 192.168.100.1 --camera-angle 90   # tilt camera down
```
Find the IP after joining the drone's Wi-Fi AP: it's your Wi-Fi adapter's
**Default Gateway** (`ipconfig`), usually `192.168.100.1`.

> NOTE: the test drone has **no propellers** — flight (takeoff/move/land) can't be
> tested on it; only the connection + perception + telemetry stack.

## 6. File map (`codes/finals/`)

| File | Role |
|------|------|
| `stage2_mission.py` | The orchestrator (flies, headless, 3 drones). |
| `run_stage2.sh` | Launcher (`--real`/`--mock`/`--short`, `--ips`, `--no-aerial`, `--phase`). |
| `configure.py` | Easy editor + **availability flag-setter** for pads + drone IPs. |
| `competition_pads.json` (+`.README.md`) | Landing points + availability flags (7/10/12 known). |
| `drones.json` | Optional `{plane_id: ip}` map (created by `configure.py drone …`). |
| `dronecheck.py` | **TEMP** all-in-one NON-FLYING check: auto-detect/connect + camera grid + ArUco + telemetry overlay; `--telemetry` (by-hand frame test), `--check` (proof-of-life), `--web`, `--camera-angle N`. Delete after use. |
| `dola.py` | Vendor UDP discovery listener (port 8668). Optional — `--ips`/`drones.json` are fallbacks. |
| `find_drones.py` | Passive UDP discovery probe. |
| `mocks/pyhulax_mock.py` | Offline mock (`BH26_MOCK=1`). |
| `tests/test_stage2.py` | 24 unit tests. |
| `requirements.txt` | `numpy`, `opencv-python`. pyhulax needs `pip install "pyhulax[video,vision]"`; dola is vendor-provided. |
| `logs/` | Runtime telemetry from real runs (git-ignored). |

## 7. Open items / next steps

- [ ] **Diagnose the 2/3 drop** — read `logs/drone_*.jsonl`, get the MISSION
      SUMMARY; confirm the move-split fix resolves it with a single-drone re-test.
- [ ] (Code) Add pre-flight `get_battery()` check + `send_app_heartbeat()` keepalive.
- [ ] Confirm camera livestream once the laptop is on `192.168.100.x`.
- [ ] Venue calibration (START_HERE §9.4): re-takeoff after land (Test D), units
      (Test E), direction (Test B).
- [ ] Set valid/invalid flags in `competition_pads.json` if organizers change them.

## 8. How to run (Windows + Git Bash)

```bash
cd ~/BrainHack-26/codes/finals
python -m unittest tests.test_stage2          # 24 tests
bash run_stage2.sh --short                    # mock smoke (~25 s)
bash run_stage2.sh --real --pads competition_pads.json --ips IP1,IP2,IP3   # real
```
