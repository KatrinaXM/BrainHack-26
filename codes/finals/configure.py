#!/usr/bin/env python3
"""
configure.py — edit the mission config WITHOUT hand-editing JSON.

Manages two files in this folder:
  competition_pads.json   the landing pads (up to 5) + an AVAILABILITY flag each
  drones.json             the drone {plane_id: ip} map (optional — dola can
                          auto-discover IPs instead; see STATUS.md)

The mission (stage2_mission.py) ONLY assigns AVAILABLE pads to drones, and
Pre-U flies 3 HULAs, so you need at least 3 pads flagged available.

Why this exists
---------------
- Landing-pad coordinates are posted by organizers — not auto-discoverable.
- Drone arena positions are NOT broadcast by the HULAs (optical-flow, relative
  to each drone's own takeoff). Only the IP is discoverable (dola). So pad
  coords + availability and (optionally) drone IPs are configured here.

Examples
--------
  python configure.py show                          # print pads + drones
  python configure.py pad 7  --x 2.5 --y 5.5        # add/update pad 7's coords
  python configure.py pad 11 --unavailable          # flag pad 11 NOT available
  python configure.py pad 11 --available            # flag it available again
  python configure.py pad 11 --remove               # delete pad 11 entirely
  python configure.py drone 1 --ip 192.168.1.101    # set drone 1's IP
  python configure.py drone 1 --remove              # delete drone 1
  python configure.py check                         # validate (>=3 available)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PADS_FILE = HERE / "competition_pads.json"
DRONES_FILE = HERE / "drones.json"
NUM_DRONES = 3   # Pre-U flies 3 HULAs (mirrors stage2_mission.NUM_DRONES)


# ---------- pads ----------

def _load_pads() -> list[dict]:
    if not PADS_FILE.is_file():
        return []
    return json.loads(PADS_FILE.read_text())


def _save_pads(pads: list[dict]) -> None:
    # One pad per line (compact, hand-editable) rather than fully expanded.
    lines = [json.dumps(p, separators=(", ", ": ")) for p in pads]
    PADS_FILE.write_text("[\n  " + ",\n  ".join(lines) + "\n]\n")


def _is_available(p: dict) -> bool:
    return bool(p.get("available", p.get("valid", True)))


def _set_available(p: dict, value: bool) -> None:
    # Normalise on the "available" key; drop the legacy "valid" alias.
    p["available"] = bool(value)
    p.pop("valid", None)


def cmd_pad(args) -> int:
    pads = _load_pads()
    pad_id = str(args.id)
    pad = next((p for p in pads if str(p["id"]) == pad_id), None)

    if args.remove:
        if pad is None:
            print(f"pad {pad_id} not found")
            return 1
        pads = [p for p in pads if str(p["id"]) != pad_id]
        _save_pads(pads)
        print(f"removed pad {pad_id}")
        return 0

    if pad is None:
        pad = {"id": pad_id, "x": 0.0, "y": 0.0, "z": 0.0, "available": True}
        pads.append(pad)
        print(f"added pad {pad_id}")

    if args.x is not None:
        pad["x"] = float(args.x)
    if args.y is not None:
        pad["y"] = float(args.y)
    if args.z is not None:
        pad["z"] = float(args.z)
    if args.available:
        _set_available(pad, True)
    if args.unavailable:
        _set_available(pad, False)
    # Ensure the normalised flag exists.
    _set_available(pad, _is_available(pad))

    _save_pads(pads)
    print(f"pad {pad_id}: x={pad['x']} y={pad['y']} z={pad.get('z', 0.0)} "
          f"available={_is_available(pad)}")
    return 0


# ---------- drones ----------

def _load_drones() -> dict:
    if not DRONES_FILE.is_file():
        return {}
    return json.loads(DRONES_FILE.read_text())


def _save_drones(drones: dict) -> None:
    DRONES_FILE.write_text(json.dumps(drones, indent=2) + "\n")


def cmd_drone(args) -> int:
    drones = _load_drones()
    key = str(args.id)
    if args.remove:
        if key not in drones:
            print(f"drone {key} not found")
            return 1
        drones.pop(key)
        _save_drones(drones)
        print(f"removed drone {key}")
        return 0
    if args.ip is None:
        print("nothing to do — pass --ip IP or --remove")
        return 1
    drones[key] = str(args.ip)
    _save_drones(drones)
    print(f"drone {key} -> {args.ip}")
    return 0


# ---------- show / check ----------

def cmd_show(_args) -> int:
    pads = _load_pads()
    print(f"PADS  ({PADS_FILE.name}):")
    if not pads:
        print("  (none)")
    avail = 0
    for p in pads:
        a = _is_available(p)
        avail += a
        print(f"  {str(p['id']):>4}  x={float(p['x']):6.2f}  y={float(p['y']):6.2f}"
              f"  available={'YES' if a else 'no '}")
    print(f"  -> {avail} available (need >= {NUM_DRONES})")

    drones = _load_drones()
    print(f"\nDRONES  ({DRONES_FILE.name}):")
    if not drones:
        print("  (none - will use dola auto-discovery or BH26_HULA_IPS)")
    for k in sorted(drones):
        print(f"  {k} -> {drones[k]}")
    return 0


def cmd_check(_args) -> int:
    pads = _load_pads()
    available = [p for p in pads if _is_available(p)]
    ok = True
    if len(available) < NUM_DRONES:
        print(f"FAIL: only {len(available)} pads available, need {NUM_DRONES}. "
              f"Mark more available: configure.py pad <id> --available")
        ok = False
    else:
        print(f"OK: {len(available)} pads available "
              f"(first {NUM_DRONES} used: {[str(p['id']) for p in available[:NUM_DRONES]]})")
    drones = _load_drones()
    if drones:
        n = sum(1 for v in drones.values() if v)
        print(f"OK: drones.json has {n} IP(s)" if n >= NUM_DRONES
              else f"NOTE: drones.json has {n} IP(s) (<{NUM_DRONES}); dola/BH26_HULA_IPS can fill the rest")
    else:
        print("NOTE: no drones.json - IPs will come from dola or BH26_HULA_IPS")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_pad = sub.add_parser("pad", help="add/update/flag a landing pad")
    p_pad.add_argument("id", help="pad id, e.g. 7")
    p_pad.add_argument("--x", type=float)
    p_pad.add_argument("--y", type=float)
    p_pad.add_argument("--z", type=float)
    p_pad.add_argument("--available", action="store_true", help="flag AVAILABLE")
    p_pad.add_argument("--unavailable", action="store_true", help="flag NOT available")
    p_pad.add_argument("--remove", action="store_true", help="delete this pad")
    p_pad.set_defaults(func=cmd_pad)

    p_dr = sub.add_parser("drone", help="set/remove a drone's IP")
    p_dr.add_argument("id", help="plane id, e.g. 1")
    p_dr.add_argument("--ip", help="drone IP, e.g. 192.168.1.101")
    p_dr.add_argument("--remove", action="store_true", help="delete this drone")
    p_dr.set_defaults(func=cmd_drone)

    sub.add_parser("show", help="print current config").set_defaults(func=cmd_show)
    sub.add_parser("check", help="validate (>=3 available pads)").set_defaults(func=cmd_check)

    args = ap.parse_args()
    if getattr(args, "available", False) and getattr(args, "unavailable", False):
        ap.error("--available and --unavailable are mutually exclusive")
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
