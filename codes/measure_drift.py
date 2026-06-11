#!/usr/bin/env python3
"""
measure_drift.py — quantify EKF position drift vs Gazebo ground truth.

Subscribes to both:
  - /model/x500_vision_0/pose            (Gazebo ground truth)
  - vehicle_local_position via MAVSDK    (PX4 EKF estimate)

and logs the difference. Use this to A/B test EKF tuning: run before+after
the airframe patch and compare drift magnitude.

Run with:
  PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 measure_drift.py
"""

import asyncio
import math
import threading
import time

from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from mavsdk import System


class GroundTruth:
    """Latest drone pose from Gazebo. Topic publishes Pose_V (vector of poses
    for every named entity); we filter for the drone."""
    DRONE_NAME = "x500_vision_0"

    def __init__(self):
        self.lock = threading.Lock()
        self.x = None
        self.y = None
        self.z = None
        self.t = 0.0
        self.node = Node()
        ok = self.node.subscribe(Pose_V, "/model/x500_vision_0/pose", self._cb)
        if not ok:
            raise RuntimeError("subscribe failed — is sim running?")
        print("[gt] subscribed: /model/x500_vision_0/pose (Pose_V)")

    def _cb(self, msg):
        for p in msg.pose:
            if p.name == self.DRONE_NAME:
                with self.lock:
                    self.x = p.position.x
                    self.y = p.position.y
                    self.z = p.position.z
                    self.t = time.time()
                return

    def get(self):
        with self.lock:
            return (self.x, self.y, self.z, self.t)


async def main():
    gt = GroundTruth()
    drone = System()
    print("[ekf] connecting MAVSDK …")
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for st in drone.core.connection_state():
        if st.is_connected:
            break
    print("[ekf] connected")

    # Stream PX4 NED.
    samples = []
    last_log = 0.0
    n = 0
    async for pv in drone.telemetry.position_velocity_ned():
        gx, gy, gz, _ = gt.get()
        if gx is None:
            await asyncio.sleep(0.1)
            continue
        ekf_n = pv.position.north_m
        ekf_e = pv.position.east_m
        ekf_d = pv.position.down_m   # NED: down positive
        # Gazebo world frame: x = north, y = east, z = up. So:
        truth_n = gx
        truth_e = gy
        truth_d = -gz
        err_n = ekf_n - truth_n
        err_e = ekf_e - truth_e
        err_d = ekf_d - truth_d
        err = math.sqrt(err_n*err_n + err_e*err_e + err_d*err_d)
        samples.append(err)
        n += 1
        now = time.time()
        if now - last_log > 1.0:
            recent = samples[-30:]
            print(f"[drift] EKF=({ekf_n:+6.2f},{ekf_e:+6.2f},{ekf_d:+6.2f})  "
                  f"GT=({truth_n:+6.2f},{truth_e:+6.2f},{truth_d:+6.2f})  "
                  f"err=({err_n:+6.2f},{err_e:+6.2f},{err_d:+6.2f}) |err|={err:.2f}m  "
                  f"mean_30s={sum(recent)/len(recent):.2f}m")
            last_log = now
        if n >= 1200:   # ~20 s of samples
            mean = sum(samples) / len(samples)
            mx = max(samples)
            print(f"\n[drift] DONE n={n} mean={mean:.2f}m max={mx:.2f}m")
            return


if __name__ == "__main__":
    asyncio.run(main())
