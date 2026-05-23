#!/usr/bin/env python3
"""
set_ekf_origin.py — send SET_GPS_GLOBAL_ORIGIN over MAVLink.

Bypasses the PX4 commander shell entirely. Useful when interactive control
of the pxh> prompt isn't available (no tmux, screen stuffs lost in repaint
storms, etc.).

Run after start_sim.sh, before mission.py:
  python3 set_ekf_origin.py
"""

import sys
import time

from pymavlink import mavutil


LAT_E7 = int(47.397742 * 1e7)
LON_E7 = int(8.545594 * 1e7)
ALT_MM = int(488.0 * 1000)   # millimetres above MSL


def main():
    print("[set_ekf_origin] connecting to udpin:0.0.0.0:14540 …")
    m = mavutil.mavlink_connection("udpin:0.0.0.0:14540")
    m.wait_heartbeat(timeout=20)
    print(f"[set_ekf_origin] heartbeat from sys {m.target_system} comp {m.target_component}")

    msg = m.mav.set_gps_global_origin_encode(
        m.target_system,
        LAT_E7, LON_E7, ALT_MM,
        int(time.time() * 1e6),    # time_usec (MAVLink 2+)
    )
    m.mav.send(msg)
    print(f"[set_ekf_origin] sent SET_GPS_GLOBAL_ORIGIN lat={LAT_E7/1e7} "
          f"lon={LON_E7/1e7} alt={ALT_MM/1000}m")

    # Wait a couple of seconds, then poll EKF status / home position OK.
    print("[set_ekf_origin] waiting for EKF convergence …")
    deadline = time.time() + 30
    last_log = 0.0
    while time.time() < deadline:
        sys_status = m.recv_match(type="SYS_STATUS", timeout=1)
        gps = m.recv_match(type="HOME_POSITION", timeout=0.1)
        now = time.time()
        if now - last_log > 2:
            print(f"  EKF probe: home_position={'OK' if gps else 'pending'}")
            last_log = now
        if gps is not None:
            print(f"[set_ekf_origin] HOME_POSITION received "
                  f"(lat={gps.latitude/1e7:.6f}, lon={gps.longitude/1e7:.6f}). EKF OK.")
            return 0

    print("[set_ekf_origin] no HOME_POSITION after 30 s — EKF may not have converged")
    return 1


if __name__ == "__main__":
    sys.exit(main())
