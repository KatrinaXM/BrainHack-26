#!/usr/bin/env python3
"""
dronecheck.py - TEMPORARY all-in-one NON-FLYING drone check (delete after use).

This drone has no propellers: NOTHING here flies. It validates the connection +
perception + telemetry stack on the ground, all in one script. The drone stays
put the whole time.

What it does (auto-detect / connect first, then one of):
  default      live camera GRID (auto-scales to N drones) with ArUco detection
               (DICT_7X7_1000) AND a telemetry overlay (battery + position) on
               each tile.
  --telemetry  text telemetry loop for the BY-HAND FRAME TEST: pick the drone up
               and slide it FORWARD -> position.x rises; RIGHT -> position.y rises
               (confirms the nav-feedback axis assumption). Also shows battery.
  --check      headless proof-of-life (connected/battery/firmware/frames), exit.
  --web        serve each drone's feed in a browser (ports 5000, 5001, ...).
  --camera-angle N   tilt the camera 0(forward)..90(down) before viewing - use
               this to confirm the camera can SEE markers where a rover would be.

Find the drone's IP first (after joining its Wi-Fi AP): it is your Wi-Fi
adapter's Default Gateway, usually 192.168.100.1.

Usage:
    python dronecheck.py --ips 192.168.100.1                 # camera grid + ArUco + telemetry
    python dronecheck.py --ips 192.168.100.1 --telemetry     # text telemetry (by-hand test)
    python dronecheck.py --ips 192.168.100.1 --check         # quick proof-of-life
    python dronecheck.py --ips 192.168.100.1 --camera-angle 90
    python dronecheck.py                                     # try dola auto-discovery
    BH26_MOCK=1 python dronecheck.py --ips 1.2.3.4 --headless --frames 20   # offline

Press 'q' in the window (or Ctrl+C) to stop.
"""
import argparse
import math
import os
import sys
import time


def _read(drone, name, *a):
    fn = getattr(drone, name, None)
    if fn is None:
        return None
    try:
        return fn(*a)
    except Exception:
        return None


def _pos_str(drone):
    p = _read(drone, "get_position")
    if p is None:
        return "n/a"
    try:
        return f"({p.x:.0f},{p.y:.0f},{p.z:.0f})"
    except Exception:
        return str(p)


def run_grid(entries, aruco=True, telemetry=True, headless=False,
             max_frames=0, cell=(480, 360)):
    """entries: list of (label, stream, drone). Auto-scaling tiled view with
    ArUco overlay and battery/position overlay. headless prints + returns."""
    import cv2
    import numpy as np
    detect = draw = None
    if aruco:
        try:
            from stage2_mission import detect_aruco_markers as detect
            from stage2_mission import draw_markers as draw
        except Exception as e:
            print(f"[grid] ArUco unavailable ({e!r}) - raw video")
            aruco = False

    n = len(entries)
    cols = max(1, math.ceil(math.sqrt(n)))
    rows = max(1, math.ceil(n / cols))
    cw, ch = cell
    print(f"[grid] {n} drone(s) -> {rows}x{cols}"
          + (" +ArUco" if aruco else "") + (" [headless]" if headless else ""))
    seen = {label: set() for label, _, _ in entries}
    done = 0
    while True:
        tiles = []
        for label, stream, drone in entries:
            f = getattr(stream, "latest_frame", None) if stream is not None else None
            if f is not None:
                rgb = f.to_rgb()
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                if aruco:
                    ok, markers = detect(rgb)
                    if ok:
                        draw(bgr, markers)
                        ids = sorted({int(mk["id"]) for mk in markers})
                        new = [i for i in ids if i not in seen[label]]
                        if new:
                            seen[label].update(new)
                            print(f"[{label}] ArUco IDs {ids}  (new {new})")
            else:
                bgr = np.full((ch, cw, 3), 40, np.uint8)
                cv2.putText(bgr, "waiting for frames...", (20, ch // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 200), 2)
            bgr = cv2.resize(bgr, (cw, ch))
            cv2.putText(bgr, str(label), (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(bgr, str(label), (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            if telemetry:
                txt = f"batt={_read(drone, 'get_battery')}% pos={_pos_str(drone)}"
                cv2.putText(bgr, txt, (10, ch - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
                cv2.putText(bgr, txt, (10, ch - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            tiles.append(bgr)
        while len(tiles) < rows * cols:
            tiles.append(np.zeros((ch, cw, 3), np.uint8))
        grid = np.vstack([np.hstack(tiles[r * cols:(r + 1) * cols])
                          for r in range(rows)])
        if headless:
            done += 1
            if max_frames and done >= max_frames:
                print(f"[grid] headless: {done} frames; "
                      f"IDs = { {k: sorted(v) for k, v in seen.items()} }")
                return seen
            time.sleep(0.2)
            continue
        cv2.imshow("dronecheck (press q to quit)", grid)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    return seen


def telemetry_text(entries, frames=0):
    print("[telemetry] by-hand frame test: slide drone FORWARD -> pos.x rises; "
          "RIGHT -> pos.y rises\n")
    n = 0
    while True:
        for label, _stream, drone in entries:
            print(f"[{label}] battery={_read(drone, 'get_battery')}% "
                  f"pos(cm)={_pos_str(drone)} "
                  f"alt={_read(drone, 'get_altitude')} "
                  f"obstacles={_read(drone, 'get_obstacles')}")
        n += 1
        if frames and n >= frames:
            return
        time.sleep(0.5)


def proof_of_life(conns):
    rc = 0
    for plane_id, ip, drone, stream in conns:
        print(f"\n=== {plane_id} @ {ip} ===")
        for label, fn in (("connected", lambda: _read(drone, "is_connected")),
                          ("battery", lambda: _read(drone, "get_battery")),
                          ("firmware", lambda: _read(drone, "get_firmware_version")),
                          ("drone_id", lambda: _read(drone, "get_drone_id"))):
            print(f"  {label}: {fn()}")
        t0 = time.time()
        while time.time() - t0 < 6.0:
            time.sleep(0.5)
        fc = getattr(stream, "frame_count", 0) if stream is not None else 0
        print(f"  >>> {ip}: {'LIVESTREAM OK' if fc else 'NO FRAMES'} (frame_count={fc})")
        rc = rc or (0 if fc else 1)
    return rc


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ips", help="comma-separated drone IP(s); a single drone's "
                                  "AP is usually 192.168.100.1")
    ap.add_argument("--need", type=int, default=1, help="min drones (default 1)")
    ap.add_argument("--telemetry", action="store_true",
                    help="text telemetry (by-hand frame test) instead of camera")
    ap.add_argument("--check", action="store_true",
                    help="headless proof-of-life, then exit")
    ap.add_argument("--web", action="store_true",
                    help="serve feeds in a browser (ports 5000, 5001, ...)")
    ap.add_argument("--camera-angle", type=int, default=None,
                    help="tilt camera 0(forward)..90(down) before viewing")
    ap.add_argument("--no-aruco", action="store_true", help="camera: no overlay")
    ap.add_argument("--headless", action="store_true", help="no window/loop cap")
    ap.add_argument("--frames", type=int, default=0, help="headless: stop after N")
    args = ap.parse_args()

    if args.ips:
        os.environ["BH26_HULA_IPS"] = args.ips

    import stage2_mission as S

    print(f"[dronecheck] backend={S.PYHULAX_BACKEND} - detecting drone(s) "
          f"(need >= {args.need}) ...")
    try:
        ips = S.discover_hulas(need=args.need)
    except Exception as e:
        print(f"[dronecheck] discovery FAILED: {e}")
        print("[dronecheck] tip: --ips 192.168.100.1 (your drone's gateway), and "
              "join the drone's Wi-Fi AP.")
        return 1
    conns = S.connect_hulas(ips)
    print(f"[dronecheck] connected {len(conns)} drone(s)")

    if args.camera_angle is not None:
        for plane_id, ip, drone, video in conns:
            if _read(drone, "set_camera_angle", args.camera_angle) is None \
                    and not hasattr(drone, "set_camera_angle"):
                print(f"[{plane_id}] set_camera_angle not supported")
            else:
                print(f"[{plane_id}] camera angle -> {args.camera_angle} deg")

    if args.check:
        return proof_of_life(conns)

    if args.telemetry:
        entries = [(pid, video, drone) for pid, ip, drone, video in conns]
        telemetry_text(entries, frames=args.frames if args.headless else 0)
        return 0

    if args.web:
        from pyhulax.video import WebStreamServer
        servers = []
        for i, (pid, ip, drone, video) in enumerate(conns):
            srv = WebStreamServer(video, port=5000 + i)
            srv.start()
            print(f"[{pid}] http://localhost:{5000 + i}")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        return 0

    entries = [(pid, video, drone) for pid, ip, drone, video in conns]
    run_grid(entries, aruco=not args.no_aruco, telemetry=True,
             headless=args.headless, max_frames=args.frames)
    print("[dronecheck] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
