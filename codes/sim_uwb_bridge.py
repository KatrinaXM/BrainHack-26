#!/usr/bin/env python3
"""
sim_uwb_bridge.py — bridge Gazebo ground-truth pose into a fake UWB
ROS2 topic, so kolomee.py (and friends) can be tested against the
existing PX4 SITL + Gazebo Harmonic simulator without the real venue.

Subscribes (gz.transport):  /model/<drone_model>/pose      (Pose_V)
Publishes (rclpy):          /uwb_tag                       (PoseStamped)

Publish rate: ~10 Hz, matching real UWB infrastructure.

Configuration via env vars:

    BH26_UWB_NOISE   0  perfect ground truth (default; for clean tuning)
                     1  realistic LOS  (σ_xy = 0.10 m, σ_z = 0.15 m, 5% dropout)
                     2  torture        (σ_xy = 0.25 m, σ_z = 0.40 m, +10 cm bias,
                                        15% dropout, occasional +0.5-1.5 m NLOS
                                        spikes lasting 0.5-2 s)

    BH26_UWB_SEED    integer; sets RNG seed so failure traces are replayable.
                     Default: 0 = non-deterministic (uses system entropy).

    BH26_UWB_DRONE   drone model name in Gazebo, default "x500_vision".

    BH26_UWB_HZ      publish rate, default 10.0. Matches real UWB.

Run with the protobuf workaround:
    PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 sim_uwb_bridge.py

Ctrl-C cleans up the ROS2 node and gz subscription.
"""

from __future__ import annotations

import math
import os
import random
import threading
import time
from dataclasses import dataclass
from typing import Optional

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

import rclpy
from rclpy.node import Node as RosNode
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped


# =====================================================================
#  Noise model — three escalating levels per research recommendation.
# =====================================================================

@dataclass
class NoiseModel:
    sigma_xy:    float    # m, Gaussian per-axis stddev on north/east
    sigma_z:     float    # m, Gaussian stddev on up
    bias_xy:     float    # m, constant offset added to north/east (NLOS-ish)
    dropout:     float    # 0..1, probability of skipping this publish tick
    nlos_prob:   float    # 0..1 per tick, chance of starting an NLOS spike event
    nlos_min_s:  float    # s, minimum spike duration
    nlos_max_s:  float    # s, maximum spike duration
    nlos_min_m:  float    # m, minimum spike magnitude (one-sided, positive only)
    nlos_max_m:  float    # m, maximum spike magnitude

    @classmethod
    def from_level(cls, level: int) -> "NoiseModel":
        if level <= 0:
            return cls(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if level == 1:
            # Realistic LOS — matches literature σ≈10-17 cm for DWM1000 in LOS.
            return cls(sigma_xy=0.10, sigma_z=0.15, bias_xy=0.0,
                       dropout=0.05,
                       nlos_prob=0.0, nlos_min_s=0.0, nlos_max_s=0.0,
                       nlos_min_m=0.0, nlos_max_m=0.0)
        # Level 2 — torture mode: realistic NLOS conditions.
        return cls(sigma_xy=0.25, sigma_z=0.40, bias_xy=0.10,
                   dropout=0.15,
                   nlos_prob=0.02,         # ~2% per tick at 10 Hz ≈ one spike per 5 s
                   nlos_min_s=0.5, nlos_max_s=2.0,
                   nlos_min_m=0.5, nlos_max_m=1.5)


# =====================================================================
#  Gazebo ground-truth subscriber.
# =====================================================================

class GazeboPose:
    """Subscribes to /model/<drone>/pose (Pose_V) and stores the latest
    pose for the named drone. Thread-safe single-reader getter."""

    def __init__(self, drone_name: str):
        self._lock = threading.Lock()
        self._drone_name = drone_name
        self._x: Optional[float] = None
        self._y: Optional[float] = None
        self._z: Optional[float] = None
        self._t_last: float = 0.0
        self._frame_id_checked: bool = False
        self._node = GzNode()

        topic = f"/model/{drone_name}/pose"
        ok = self._node.subscribe(Pose_V, topic, self._cb)
        if not ok:
            raise RuntimeError(f"gz subscribe failed: {topic}. Is Gazebo running?")
        print(f"[bridge] gz subscribed: {topic}")

    def _cb(self, msg: Pose_V):
        # /model/<name>/pose emits a single entry with an empty `name` field;
        # the model identity lives in header.data[frame_id] = "<name>/odom".
        # We subscribed to a model-specific topic, so the first entry is ours —
        # but verify frame_id once, so a wrong drone name fails loudly.
        if not msg.pose:
            return
        p = msg.pose[0]
        if not self._frame_id_checked:
            fid = ""
            for d in p.header.data:
                if d.key == "frame_id" and d.value:
                    fid = d.value[0]
                    break
            expected = f"{self._drone_name}/odom"
            if fid and fid != expected:
                print(f"[bridge] WARNING: pose frame_id={fid!r} but expected {expected!r}; "
                      f"check BH26_UWB_DRONE")
            self._frame_id_checked = True
        with self._lock:
            self._x = p.position.x
            self._y = p.position.y
            self._z = p.position.z
            if self._t_last == 0.0:
                print(f"[bridge] first gz pose: x={self._x:.3f} y={self._y:.3f} z={self._z:.3f}")
            self._t_last = time.time()

    def get(self) -> Optional[tuple]:
        """Returns (x, y, z) in Gazebo world frame, or None if no sample yet."""
        with self._lock:
            if self._x is None:
                return None
            return (self._x, self._y, self._z)


# =====================================================================
#  ROS2 publisher.
# =====================================================================

class UwbPublisher(RosNode):
    """ROS2 node that publishes PoseStamped on /uwb_tag with the
    requested noise / dropout / NLOS model applied."""

    def __init__(self, gt: GazeboPose, noise: NoiseModel, rng: random.Random,
                 publish_hz: float):
        super().__init__("sim_uwb_bridge")
        self._gt = gt
        self._noise = noise
        self._rng = rng

        # Publishers on sensor topics should match the publisher's QoS used by
        # real UWB stacks — BEST_EFFORT with shallow depth. kolomee.py
        # subscribes with the same QoS (see references/.../kolomee.py:54).
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self._pub = self.create_publisher(PoseStamped, "uwb_tag", qos)

        period = 1.0 / publish_hz
        self._timer = self.create_timer(period, self._tick)

        # NLOS spike state machine.
        self._nlos_ends_at: float = 0.0     # epoch time when current spike ends
        self._nlos_offset_x: float = 0.0    # m, the spike's east offset
        self._nlos_offset_y: float = 0.0    # m, the spike's north offset

        self._tick_count = 0
        self._publish_count = 0
        self._drop_count = 0
        self._nlos_count = 0
        print(f"[bridge] ros2 publisher up on /uwb_tag at {publish_hz:.1f} Hz")
        print(f"[bridge] noise model: σ_xy={noise.sigma_xy:.2f} m, "
              f"σ_z={noise.sigma_z:.2f} m, bias_xy={noise.bias_xy:.2f} m, "
              f"dropout={noise.dropout*100:.1f}%, "
              f"nlos_prob/tick={noise.nlos_prob*100:.1f}%")

    def _tick(self):
        self._tick_count += 1
        gt = self._gt.get()
        if gt is None:
            return    # no Gazebo data yet
        x_gt, y_gt, z_gt = gt

        # Dropout: skip this tick entirely.
        if self._noise.dropout > 0.0 and self._rng.random() < self._noise.dropout:
            self._drop_count += 1
            return

        # NLOS spike state machine — start a new spike, or apply ongoing one.
        now = time.time()
        if (self._noise.nlos_prob > 0.0 and now >= self._nlos_ends_at and
                self._rng.random() < self._noise.nlos_prob):
            duration = self._rng.uniform(self._noise.nlos_min_s, self._noise.nlos_max_s)
            magnitude = self._rng.uniform(self._noise.nlos_min_m, self._noise.nlos_max_m)
            angle = self._rng.uniform(0.0, 2.0 * math.pi)
            self._nlos_offset_x = magnitude * math.cos(angle)
            self._nlos_offset_y = magnitude * math.sin(angle)
            self._nlos_ends_at = now + duration
            self._nlos_count += 1
            print(f"[bridge] NLOS spike: +{magnitude:.2f} m for {duration:.1f} s")

        in_spike = now < self._nlos_ends_at
        spike_dx = self._nlos_offset_x if in_spike else 0.0
        spike_dy = self._nlos_offset_y if in_spike else 0.0

        # Apply Gaussian noise + bias (bias on horizontal only).
        noise_dx = self._rng.gauss(0.0, self._noise.sigma_xy) if self._noise.sigma_xy > 0 else 0.0
        noise_dy = self._rng.gauss(0.0, self._noise.sigma_xy) if self._noise.sigma_xy > 0 else 0.0
        noise_dz = self._rng.gauss(0.0, self._noise.sigma_z)  if self._noise.sigma_z  > 0 else 0.0

        # Publish in UWB convention: real venues publish
        #   msg.pose.position.x = east  ←  Gazebo y
        #   msg.pose.position.y = north ←  Gazebo x
        # kolomee.py (line 67-68) swaps back to NED. We mirror the real
        # venue's convention here so kolomee.py works unchanged against
        # this bridge.
        east_arena  = y_gt + noise_dx + spike_dx + self._noise.bias_xy
        north_arena = x_gt + noise_dy + spike_dy + self._noise.bias_xy
        up_arena    = z_gt + noise_dz

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "arena"
        msg.pose.position.x = east_arena
        msg.pose.position.y = north_arena
        msg.pose.position.z = up_arena
        # Orientation left as identity — UWB tags don't measure rotation.
        msg.pose.orientation.w = 1.0

        self._pub.publish(msg)
        self._publish_count += 1

    def stats(self) -> str:
        return (f"ticks={self._tick_count} published={self._publish_count} "
                f"dropped={self._drop_count} nlos_spikes={self._nlos_count}")


# =====================================================================
#  Entrypoint.
# =====================================================================

def main():
    level = int(os.environ.get("BH26_UWB_NOISE", "0"))
    seed  = int(os.environ.get("BH26_UWB_SEED",  "0"))
    drone = os.environ.get("BH26_UWB_DRONE", "x500_vision")
    hz    = float(os.environ.get("BH26_UWB_HZ",    "10.0"))

    rng = random.Random(seed) if seed != 0 else random.Random()
    noise = NoiseModel.from_level(level)

    print(f"[bridge] noise level={level}, seed={seed if seed else 'system'}")
    print(f"[bridge] drone='{drone}', rate={hz:.1f} Hz")

    # Append _0 — Gazebo's pose topic uses the spawned-instance name, which
    # for the default single-vehicle SITL is "<model>_0".
    drone_topic_name = drone if drone.endswith("_0") else f"{drone}_0"
    gt = GazeboPose(drone_topic_name)

    rclpy.init()
    pub_node = UwbPublisher(gt, noise, rng, hz)

    try:
        rclpy.spin(pub_node)
    except KeyboardInterrupt:
        print("\n[bridge] stopping…")
    finally:
        print(f"[bridge] final: {pub_node.stats()}")
        pub_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
