#!/usr/bin/env python3
"""
depth_diag.py — fast standalone depth probe.

No flight, no MAVSDK. Just subscribes to /depth_camera and prints how many
pixels are finite. Use this to decide whether to commit to a mission run.

Run with:
  PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 depth_diag.py
"""

import sys
import time

import numpy as np

from depth_receiver import DepthReceiver
import mission_config as cfg


def main():
    print(f"[depth_diag] subscribing to {cfg.DEPTH_TOPIC}")
    receiver = DepthReceiver(cfg.DEPTH_TOPIC)

    deadline = time.time() + 15.0
    last_log = 0.0
    saw_finite = False

    while time.time() < deadline:
        d = receiver.get_frame()
        now = time.time()
        if d is None:
            if now - last_log > 1.0:
                print("[depth_diag] no frame yet…")
                last_log = now
        else:
            finite = np.isfinite(d).sum()
            total = d.size
            valid = ((d > cfg.DEPTH_MIN) & (d < cfg.DEPTH_MAX) & np.isfinite(d)).sum()
            mn = np.nanmin(d[np.isfinite(d)]) if finite else float("nan")
            mx = np.nanmax(d[np.isfinite(d)]) if finite else float("nan")
            print(f"[depth_diag] shape={d.shape} finite={finite}/{total} "
                  f"in_range={valid} min={mn:.2f} max={mx:.2f}")
            if finite > 0:
                saw_finite = True
                break
            last_log = now
        time.sleep(0.5)

    if saw_finite:
        print("[depth_diag] PASS — depth has finite pixels; mission may proceed")
        return 0
    print("[depth_diag] FAIL — depth is silent or all-inf. Mission will fly blind.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
