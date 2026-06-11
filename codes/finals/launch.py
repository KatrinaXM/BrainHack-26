#!/usr/bin/env python3
"""
launch.py - BrainHack-26 Finals Stage-2 launcher (interactive menu).

The single, friendly entry point for the whole project - no flags to remember.
Run it and pick from the menu:

    python launch.py

It drives the other tools for you:
  - dronecheck.py   (camera / ArUco / telemetry, non-flying)
  - stage2_mission.py (the mock and real missions)
  - configure.py    (landing pads + drone IPs)
  - the unit tests

JUDGE QUICK START: pick [2] for a safe simulator demo, or [1] to view a drone's
camera. [3] flies the real drones (asks for confirmation first).
"""
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PY = sys.executable


# ---------- small input helpers ----------

def ask(prompt, default=""):
    d = f" [{default}]" if default != "" else ""
    try:
        v = input(f"{prompt}{d}: ").strip()
    except EOFError:
        v = ""
    return v or default


def ask_yn(prompt, default=True):
    d = "Y/n" if default else "y/N"
    v = ask(f"{prompt} ({d})", "").lower()
    if not v:
        return default
    return v.startswith("y")


def menu(title, items):
    """items: list of (key, label). Returns the chosen key (or None)."""
    print(f"\n=== {title} ===")
    for key, label in items:
        print(f"  {key}) {label}")
    choice = ask("Choose", "").lower()
    keys = {k.lower() for k, _ in items}
    return choice if choice in keys else None


def run(args, env_overrides=None, shell=False):
    env = {**os.environ, **(env_overrides or {})}
    print(f"\n>>> {' '.join(args) if not shell else args}\n")
    try:
        subprocess.run(args, cwd=str(HERE), env=env, shell=shell)
    except KeyboardInterrupt:
        print("\n(interrupted - back to menu)")
    except Exception as e:
        print(f"\n[launch] command failed: {e!r}")
    input("\n(press Enter to return to the menu) ")


# ---------- actions ----------

def action_check():
    ip = ask("Drone IP (join its Wi-Fi first; usually 192.168.100.1)", "192.168.100.1")
    what = menu("Check drone (does NOT fly)", [
        ("1", "Camera grid + live ArUco detection"),
        ("2", "Telemetry (battery/position) - by-hand frame test"),
        ("3", "Proof-of-life (connected/battery/firmware/frames), then exit"),
        ("4", "Camera with a tilt angle (test what it can see)"),
    ])
    base = [PY, "dronecheck.py", "--ips", ip]
    if what == "1":
        run(base)
    elif what == "2":
        run(base + ["--telemetry"])
    elif what == "3":
        run(base + ["--check"])
    elif what == "4":
        ang = ask("Camera angle 0(forward)..90(down)", "45")
        run(base + ["--camera-angle", ang])


def action_mock():
    print("\nMOCK mission = full simulator demo. Safe, no hardware needed.")
    quick = ask_yn("Quick demo (~25 s)?", True)
    phase = ask("Phase: both / land / search", "both")
    env = {"BH26_MOCK": "1"}
    if quick:
        env.update({"BH26_MOCK_TAKEOFF_S": "0.8", "BH26_MOCK_LAND_S": "0.5",
                    "BH26_MOCK_SPEED_MPS": "3.0", "BH26_MOCK_ROBO_FIRST": "1.0",
                    "BH26_MOCK_ROBO_PERIOD": "3.0", "BH26_PAD_HOLD_S": "1",
                    "BH26_AMBUSH_S": "8"})
    else:
        env["BH26_AMBUSH_S"] = ask("Ambush window seconds", "30")
    run([PY, "stage2_mission.py", "--pads", "pads_example.json", "--phase", phase], env)


def action_real():
    print("\n*** REAL MISSION - this will FLY the HULA drones. ***")
    print("Pre-flight: drones charged, on the venue Wi-Fi, props clear, area open.")
    if ask("Type FLY to continue (anything else cancels)", "") != "FLY":
        print("cancelled.")
        input("\n(press Enter) ")
        return

    pads = ask("Pad file", "competition_pads.json")
    ips = ask("Drone IPs comma-separated (blank = dola auto-discovery)", "")
    phase = ask("Phase: both / land / search", "both")
    env = {}
    if ips:
        env["BH26_HULA_IPS"] = ips
    print("\n-- Calibration (Enter keeps the default) --")
    env["BH26_DIST_SCALE"] = ask("Distance scale (100=cm, 1=metres)", "100")
    env["BH26_AXIS_FORWARD"] = ask("Forward axis (+x/-x/+y/-y)", "+x")
    env["BH26_AXIS_RIGHT"] = ask("Right axis (+x/-x/+y/-y)", "+y")
    env["BH26_AMBUSH_S"] = ask("Ambush window seconds", "120")
    cam = ask("Camera tilt angle 0..90 (blank = leave default)", "")
    if cam:
        env["BH26_CAMERA_ANGLE"] = cam
    if not ask_yn("Re-takeoff after landing for the search? (no = watch from pad)", True):
        env["BH26_DO_AERIAL_SEARCH"] = "0"
    if not ask_yn("Obstacle avoidance ON?", True):
        env["BH26_AVOIDANCE"] = "0"

    print("\n-- About to run with --")
    print(f"   pads={pads}  phase={phase}  ips={ips or '(dola)'}")
    for k, v in env.items():
        print(f"   {k}={v}")
    if not ask_yn("Launch now?", False):
        print("cancelled.")
        input("\n(press Enter) ")
        return
    run([PY, "stage2_mission.py", "--pads", pads, "--phase", phase], env)


def action_configure():
    while True:
        what = menu("Configure pads & drones", [
            ("1", "Show current config"),
            ("2", "Check (>=3 available pads?)"),
            ("3", "Set a pad's coordinates"),
            ("4", "Flag a pad available / unavailable"),
            ("5", "Set a drone's IP"),
            ("0", "Back"),
        ])
        if what in (None, "0"):
            return
        if what == "1":
            run([PY, "configure.py", "show"])
        elif what == "2":
            run([PY, "configure.py", "check"])
        elif what == "3":
            pid = ask("Pad id", "")
            x = ask("x (m)", "")
            y = ask("y (m)", "")
            run([PY, "configure.py", "pad", pid, "--x", x, "--y", y])
        elif what == "4":
            pid = ask("Pad id", "")
            flag = "--available" if ask_yn("Make it AVAILABLE?", True) else "--unavailable"
            run([PY, "configure.py", "pad", pid, flag])
        elif what == "5":
            pid = ask("Drone/plane id", "1")
            ip = ask("IP", "")
            run([PY, "configure.py", "drone", pid, "--ip", ip])


def action_tests():
    run([PY, "-m", "unittest", "tests.test_stage2", "-v"])


def action_results():
    out = Path(os.environ.get("BH26_OUTPUT_DIR", str(HERE / "snapshots")))
    print(f"\nSnapshots dir: {out}")
    if not out.is_dir():
        print("(no snapshots yet - run a mission first)")
    else:
        jpgs = sorted(out.glob("*.jpg"))
        jsons = sorted(out.glob("*.json"))
        print(f"  {len(jpgs)} JPEG(s), {len(jsons)} JSON sidecar(s)")
        ids = set()
        import json
        for j in jsons:
            try:
                ids |= set(json.loads(j.read_text()).get("marker_ids", []))
            except Exception:
                pass
        print(f"  ArUco IDs across snapshots: {sorted(ids)}")
        if jpgs and ask_yn("Open the snapshots folder?", False):
            try:
                os.startfile(str(out))            # Windows
            except AttributeError:
                subprocess.run(["xdg-open", str(out)])
    input("\n(press Enter) ")


def action_help():
    print("""
--- Help & network tips ---
* Each HULA is its OWN Wi-Fi access point by default. Join the drone's Wi-Fi
  (password 12345678). Its IP is then your Wi-Fi adapter's Default Gateway:
      Windows CMD:  ipconfig | findstr /i "Gateway"
  - usually 192.168.100.1.
* For a SWARM + auto-discovery (dola), put all drones in NETWORKING mode (press
  the power button 3x) joined to one router, via the Hula App.
* Confirm the drone responds:  python dronecheck.py --ips <IP> --check
* The mission saves snapshots to ./snapshots/ (JPEG + JSON with the ArUco IDs).
* Full details: STATUS.md and START_HERE.md.
""")
    input("(press Enter) ")


def main():
    actions = {
        "1": action_check, "2": action_mock, "3": action_real,
        "4": action_configure, "5": action_tests, "6": action_results,
        "7": action_help,
    }
    while True:
        choice = menu("BrainHack-26 Finals - Stage 2 Launcher", [
            ("1", "Check a drone (camera / ArUco / telemetry) - NO flight"),
            ("2", "Run MOCK mission (simulator demo - safe)"),
            ("3", "Run REAL mission (fly the HULAs)"),
            ("4", "Configure landing pads & drone IPs"),
            ("5", "Run the tests (verify the code)"),
            ("6", "View results / snapshots"),
            ("7", "Help & network tips"),
            ("0", "Quit"),
        ])
        if choice in (None, "0"):
            print("bye.")
            return 0
        actions[choice]()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        print("\nbye.")
