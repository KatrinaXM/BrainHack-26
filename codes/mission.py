#!/usr/bin/env python3
"""
mission.py — BrainHack-26 RoboVerse Qualifier mission orchestrator.

Strategy (rotate-scan-step, optimised for the forward-facing OakD-Lite and
the x500_vision EKF that doesn't tolerate long-range position setpoints):

  1. Takeoff to low altitude.
  2. Rotate 360° in place — sweep yaw in small steps, dwelling 0.5 s at each
     so depth + YOLO get clean readings in every direction. This is also
     where most detections happen because the drone is still and the
     camera is steady.
  3. Pick the heading with the best forward clearance from the scan.
  4. Fly FORWARD ONLY (body-frame velocity) for up to a few seconds or
     until the depth-based avoidance declares "blocked". Never sidestep —
     the camera can't see sideways or backwards.
  5. When blocked: rotate to the next clear direction (re-scan if needed)
     and repeat.
  6. Periodically climb to high altitude for red-barrel detection, then
     descend back to low.
  7. At BAILOUT_SECONDS, descend back near origin and land.

Background tasks (DO NOT block the main loop, per organiser guidance):
  - asyncio telemetry task: streams NED pose + yaw into SharedState.
  - DepthReceiver: subscribes to /depth_camera (gz transport thread).
  - VisionReceiver: subscribes to RGB topic (gz transport thread) and feeds
    a 2-worker Detector pool. Detection callback projects bboxes into NED
    and dedupes via BarrelTracker.

Run with the protobuf workaround:
  PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 mission.py
"""

from __future__ import annotations

import asyncio
import math
import os
import signal
import sys
import threading
import time
from typing import Optional

import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as GzImage

import mission_config as cfg
from drone_control import Drone
from depth_receiver import DepthReceiver
from AvoidancePlanner import AvoidancePlanner
from Detector import Detector
from barrel_tracker import BarrelTracker


# =====================================================================
#  Shared state — written by telemetry, read by main loop & detector.
# =====================================================================

class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self.north: Optional[float] = None
        self.east:  Optional[float] = None
        self.down:  Optional[float] = None
        self.yaw_deg: Optional[float] = None

    def update_position(self, n: float, e: float, d: float) -> None:
        with self._lock:
            self.north, self.east, self.down = n, e, d

    def update_yaw(self, yaw_deg: float) -> None:
        with self._lock:
            self.yaw_deg = yaw_deg

    def snapshot(self) -> Optional[dict]:
        with self._lock:
            if self.north is None or self.yaw_deg is None:
                return None
            yaw_deg = self.yaw_deg
            return {
                "north":   self.north,
                "east":    self.east,
                "down":    self.down,
                "yaw_deg": yaw_deg,
                "yaw":     math.radians(yaw_deg),
            }


# =====================================================================
#  Background tasks
# =====================================================================

async def telemetry_task(drone: Drone, state: SharedState, stop: asyncio.Event):
    async def stream_pos():
        async for pv in drone.drone.telemetry.position_velocity_ned():
            if stop.is_set():
                return
            p = pv.position
            state.update_position(p.north_m, p.east_m, p.down_m)

    async def stream_yaw():
        async for att in drone.drone.telemetry.attitude_euler():
            if stop.is_set():
                return
            state.update_yaw(att.yaw_deg)

    try:
        await asyncio.gather(stream_pos(), stream_yaw())
    except asyncio.CancelledError:
        pass
    except Exception as exc:
        print(f"[telemetry] error: {exc}")


class VisionReceiver:
    """Subscribes to the RGB topic, converts to BGR, hands frames to a callback.
    Runs on a gz transport thread — main loop is never blocked by image I/O."""

    def __init__(self, topic: str, on_frame):
        self._lock = threading.Lock()
        self._node = Node()
        self._frame: Optional[np.ndarray] = None
        self._on_frame = on_frame
        self._frame_count = 0
        ok = self._node.subscribe(GzImage, topic, self._cb)
        if not ok:
            raise RuntimeError(f"VisionReceiver failed to subscribe to {topic}")
        print(f"[vision] subscribed: {topic}")

    def _cb(self, msg: GzImage):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            buf = buf.reshape((msg.height, msg.width, 3))
            bgr = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
        except Exception as exc:
            print(f"[vision] decode error: {exc}")
            return
        with self._lock:
            self._frame = bgr
            self._frame_count += 1
        try:
            self._on_frame(bgr)
        except Exception as exc:
            print(f"[vision] callback error: {exc}")

    def frame_count(self) -> int:
        with self._lock:
            return self._frame_count


# =====================================================================
#  Mission
# =====================================================================

class Mission:
    def __init__(self):
        self.drone = Drone()
        self.state = SharedState()
        self.depth_receiver = DepthReceiver(cfg.DEPTH_TOPIC)

        self.planner = AvoidancePlanner(
            K=cfg.K_CAM,
            width=cfg.IMAGE_WIDTH,
            height=cfg.IMAGE_HEIGHT,
            safe_distance=cfg.SAFE_DISTANCE,
            critical_distance=cfg.CRITICAL_DISTANCE,
            num_bins=cfg.NUM_BINS,
            smoothing_alpha=cfg.SMOOTHING_ALPHA,
        )

        self.tracker = BarrelTracker()

        os.makedirs(cfg.DETECTIONS_DIR, exist_ok=True)
        self.detector = Detector(
            model_path=cfg.YOLO_MODEL_PATH,
            confidence_threshold=cfg.DETECTION_CONF,
            callback=self._on_detection,
            num_workers=cfg.DETECTOR_WORKERS,
            device=cfg.DETECTOR_DEVICE,
            save_dir=cfg.DETECTIONS_DIR,
            enable_display=cfg.DETECTOR_DISPLAY,
        )

        self.vision = VisionReceiver(cfg.RGB_TOPIC, self._on_rgb_frame)

        self._stop = asyncio.Event()
        self._mission_t0: Optional[float] = None
        self._last_log_t = 0.0

        # Track current altitude target so the loop doesn't make sudden jumps.
        self._target_down = cfg.ALTITUDE_LOW_DOWN
        self._alt_phase = "low"
        self._alt_switch_t = 0.0

    # -----------------------------------------------------------------
    # Detector path
    # -----------------------------------------------------------------
    def _on_rgb_frame(self, bgr: np.ndarray) -> None:
        pose = self.state.snapshot()
        depth = self.depth_receiver.get_frame()
        ctx = {
            "timestamp": time.time(),
            "pose": pose,
            "depth": depth,
        }
        self.detector.submit_image(bgr, context=ctx)

    def _on_detection(self, detections, annotated_image, context) -> None:
        pose = context.get("pose")
        depth = context.get("depth")
        if pose is None:
            return
        image_path = context.get("saved_path")
        touched = self.tracker.consume(detections, depth, pose, image_path)
        for b in touched:
            print(f"[barrel] {b.class_name} @ ({b.north:+.2f}, {b.east:+.2f}, "
                  f"{b.down:+.2f}) conf={b.confidence:.2f} hits={b.hits} "
                  f"(total: {len(self.tracker)})")

    # -----------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------
    @staticmethod
    def _yaw_diff(a: float, b: float) -> float:
        """Shortest signed angular difference a→b (deg, -180..180)."""
        return (b - a + 540.0) % 360.0 - 180.0

    async def _send_hover(self, pose: dict, yaw_deg: Optional[float] = None) -> None:
        """Send a position setpoint at the drone's *current* pose to hold in
        place — used during rotation / scan, where horizontal motion is
        actively suppressed."""
        target_yaw = pose["yaw_deg"] if yaw_deg is None else yaw_deg
        await self.drone.send_position_setpoint(
            pose["north"], pose["east"], self._target_down, target_yaw)

    async def _wait_for_pose(self, timeout_s: float = 30.0) -> None:
        deadline = time.time() + timeout_s
        while self.state.snapshot() is None:
            if time.time() > deadline:
                raise TimeoutError("No NED pose telemetry within timeout")
            await asyncio.sleep(0.1)

    async def _wait_for_depth(self, timeout_s: float = 20.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            d = self.depth_receiver.get_frame()
            if d is not None:
                finite = np.isfinite(d).sum()
                print(f"[depth] frame: shape={d.shape} finite_px={finite}/{d.size}")
                if finite > 0:
                    return True
            await asyncio.sleep(0.5)
        return False

    async def _wait_for_armable(self, timeout_s: float = 30.0) -> None:
        deadline = time.time() + timeout_s
        async for health in self.drone.drone.telemetry.health():
            home_ok = getattr(health, "is_home_position_ok", False)
            local_ok = getattr(health, "is_local_position_ok", False)
            armable = getattr(health, "is_armable", False)
            print(f"[health] armable={armable} local_ok={local_ok} home_ok={home_ok}")
            if armable or (home_ok and local_ok):
                return
            if time.time() > deadline:
                raise TimeoutError(
                    "Drone never became armable. Run set_ekf_origin.py first.")

    def _maybe_switch_altitude_phase(self, elapsed: float) -> None:
        """Toggle between low (yellow) and high (red) altitude passes."""
        if self._alt_phase == "low" and elapsed >= cfg.PHASE_LOW_SECONDS:
            self._alt_phase = "high"
            self._target_down = cfg.ALTITUDE_HIGH_DOWN
            self._alt_switch_t = elapsed
            print(f"[mission] switching to HIGH altitude phase at t={elapsed:.0f}s")
        elif self._alt_phase == "high" and (elapsed - self._alt_switch_t) >= cfg.PHASE_HIGH_SECONDS:
            self._alt_phase = "low"
            self._target_down = cfg.ALTITUDE_LOW_DOWN
            self._alt_switch_t = elapsed
            print(f"[mission] switching back to LOW altitude phase at t={elapsed:.0f}s")

    def _is_ekf_runaway(self, pose: dict) -> bool:
        # Only flag horizontal runaway. EKF Z is the most drift-prone axis
        # on x500_vision and Gazebo ground-truth often shows the drone is
        # fine even when EKF down is wildly off. Z is handled by PX4's
        # altitude hold on body-velocity commands.
        return (abs(pose["north"]) > cfg.MAX_DRIFT_FROM_ORIGIN
                or abs(pose["east"]) > cfg.MAX_DRIFT_FROM_ORIGIN)

    # -----------------------------------------------------------------
    # Rotate-and-scan: spin 360° in place, sample depth+RGB in every direction
    # -----------------------------------------------------------------
    async def _rotate_scan(self, num_steps: int = 8, dwell_s: float = 0.6) -> list:
        """Rotate the drone in num_steps increments using yaw-rate velocity
        commands ONLY — no NED position setpoint. This is critical: position
        setpoints depend on the EKF, which is drift-prone on x500_vision.
        Yaw-rate commands let PX4's onboard attitude controller spin the
        body without consulting the drifted world frame.

        The drone holds horizontal position via body-velocity (0, 0) and
        gently bias toward the target altitude in body-down.
        """

        pose0 = self.state.snapshot()
        if pose0 is None:
            print("[scan] no pose; skipping scan")
            return []
        start_yaw = pose0["yaw_deg"]

        step_deg = 360.0 / num_steps
        readings = []

        for i in range(num_steps):
            target_yaw = ((start_yaw + (i + 1) * step_deg + 540.0) % 360.0) - 180.0

            # Spin to target yaw using yaw-rate. Don't use position setpoint —
            # we want body-relative motion only.
            await self._yaw_only_rotate_to(target_yaw, tolerance=8.0, timeout_s=4.0)

            # Dwell so YOLO + depth settle. Hold zero body velocity but allow
            # a gentle altitude correction in body-down.
            t_dwell_end = time.time() + dwell_s
            while time.time() < t_dwell_end:
                pose = self.state.snapshot()
                v_down = self._altitude_correction_velocity(pose)
                await self.drone.send_body_velocity(0.0, 0.0, v_down, 0.0)
                await asyncio.sleep(0.1)
                if self._stop.is_set():
                    return readings

            pose = self.state.snapshot()
            depth = self.depth_receiver.get_frame()
            if pose is not None and depth is not None:
                try:
                    left, center, right = self.planner.compute_clearance(depth)
                except Exception:
                    left = center = right = float("nan")
            else:
                left = center = right = float("nan")

            actual_yaw = pose["yaw_deg"] if pose is not None else target_yaw
            readings.append((actual_yaw, float(center), float(left), float(right)))
            print(f"[scan] yaw={actual_yaw:+6.1f}°  L={left:.2f} C={center:.2f} R={right:.2f}")

            if pose is not None and self._is_ekf_runaway(pose):
                print("[scan] EKF horizontal runaway during scan — aborting")
                self._stop.set()
                return readings

        return readings

    def _altitude_correction_velocity(self, pose: Optional[dict]) -> float:
        """Body-down velocity to converge toward self._target_down. Clamped."""
        if pose is None:
            return 0.0
        dz = self._target_down - pose["down"]
        return max(-cfg.MAX_VERTICAL_SPEED, min(cfg.MAX_VERTICAL_SPEED, dz * 0.3))

    async def _yaw_only_rotate_to(self, target_yaw: float,
                                  tolerance: float = 5.0,
                                  timeout_s: float = 5.0) -> None:
        """Rotate to target_yaw using yaw-rate body-velocity commands. No
        position setpoint. Trust PX4's attitude/position hold to keep the
        drone stationary in body XY."""
        deadline = time.time() + timeout_s
        while time.time() < deadline and not self._stop.is_set():
            pose = self.state.snapshot()
            if pose is None:
                await asyncio.sleep(0.1)
                continue
            diff = self._yaw_diff(pose["yaw_deg"], target_yaw)
            if abs(diff) < tolerance:
                break
            # Proportional yaw rate, capped at ±60 deg/s for faster scans.
            yaw_rate = max(-60.0, min(60.0, diff * 2.0))
            v_down = self._altitude_correction_velocity(pose)
            await self.drone.send_body_velocity(0.0, 0.0, v_down, yaw_rate)
            await asyncio.sleep(0.1)
        # Settle: zero rate for a moment so the drone stabilises before
        # sampling depth.
        for _ in range(3):
            pose = self.state.snapshot()
            v_down = self._altitude_correction_velocity(pose)
            await self.drone.send_body_velocity(0.0, 0.0, v_down, 0.0)
            await asyncio.sleep(0.05)

    def _pick_best_heading(self, readings: list, exclude_recent: list = None) -> Optional[float]:
        """Among the rotate_scan readings, pick the yaw with the largest
        center clearance, preferring values > SAFE_DISTANCE. Optionally
        exclude headings near recently-tried directions to avoid loops."""
        exclude_recent = exclude_recent or []

        candidates = []
        for yaw, center, left, right in readings:
            if not math.isfinite(center):
                continue
            # Penalise headings within 30° of any "recent" heading that was
            # blocked, so we don't immediately try the same dead end.
            penalty = 0.0
            for prev in exclude_recent:
                if abs(self._yaw_diff(yaw, prev)) < 30:
                    penalty = 99.0
                    break
            score = center - penalty
            candidates.append((score, yaw, center))

        if not candidates:
            return None
        candidates.sort(reverse=True)
        score, yaw, center = candidates[0]
        if score < cfg.CRITICAL_DISTANCE:
            print(f"[scan] best clearance is only {center:.2f} m — all blocked")
            return None
        print(f"[scan] best heading {yaw:+.1f}° (center clearance {center:.2f} m)")
        return yaw

    async def _rotate_to(self, target_yaw: float, tolerance: float = 5.0, timeout_s: float = 8.0) -> None:
        """Spin in place to target_yaw using slow yaw-only velocity commands."""
        pose0 = self.state.snapshot()
        if pose0 is None:
            return
        anchor_n, anchor_e = pose0["north"], pose0["east"]
        deadline = time.time() + timeout_s

        while time.time() < deadline and not self._stop.is_set():
            pose = self.state.snapshot()
            if pose is None:
                await asyncio.sleep(0.1)
                continue
            diff = self._yaw_diff(pose["yaw_deg"], target_yaw)
            if abs(diff) < tolerance:
                break
            # Yaw-only via position setpoint — set anchor as current pose to
            # avoid translation. PX4 will rotate to target_yaw.
            await self.drone.send_position_setpoint(
                anchor_n, anchor_e, self._target_down, target_yaw)
            await asyncio.sleep(0.15)

    # -----------------------------------------------------------------
    # Fly forward until blocked or step distance reached
    # -----------------------------------------------------------------
    async def _fly_forward(self, max_distance: float, max_seconds: float = 8.0) -> str:
        """Send body-frame forward velocity until either the depth-based
        clearance drops below SAFE_DISTANCE, the time budget elapses, or
        max_distance has been covered. Returns the reason for stopping."""
        pose0 = self.state.snapshot()
        if pose0 is None:
            return "no_pose"
        start_n, start_e = pose0["north"], pose0["east"]
        t_end = time.time() + max_seconds
        loop_dt = 1.0 / cfg.LOOP_HZ

        v_forward = cfg.FORWARD_SPEED
        reason = "max_distance"

        while time.time() < t_end and not self._stop.is_set():
            pose = self.state.snapshot()
            if pose is None:
                await asyncio.sleep(loop_dt)
                continue

            if self._is_ekf_runaway(pose):
                reason = "ekf_runaway"
                self._stop.set()
                break

            dist = math.hypot(pose["north"] - start_n, pose["east"] - start_e)
            if dist >= max_distance:
                reason = "max_distance"
                break

            depth = self.depth_receiver.get_frame()
            if depth is not None:
                try:
                    left, center, right = self.planner.compute_clearance(depth)
                except Exception:
                    left = center = right = float("inf")
            else:
                left = center = right = float("inf")

            # Slow down as obstacles approach; stop hard at critical.
            if not math.isfinite(center) or center > cfg.SAFE_DISTANCE:
                v = v_forward
            elif center > cfg.CRITICAL_DISTANCE:
                # Linearly ramp from full speed at SAFE down to zero at CRITICAL.
                v = v_forward * (center - cfg.CRITICAL_DISTANCE) / max(
                    cfg.SAFE_DISTANCE - cfg.CRITICAL_DISTANCE, 1e-3)
            else:
                v = 0.0
                reason = "blocked"

            # Soft altitude correction in body frame (down is positive down).
            dz = self._target_down - pose["down"]
            v_down = max(-cfg.MAX_VERTICAL_SPEED,
                         min(cfg.MAX_VERTICAL_SPEED, dz * 0.5))

            await self.drone.send_body_velocity(
                forward_m_s=v,
                right_m_s=0.0,
                down_m_s=v_down,
                yawspeed_deg_s=0.0,
            )

            if v == 0.0:
                break

            await asyncio.sleep(loop_dt)

        # Hover briefly to settle before returning.
        for _ in range(3):
            await self.drone.send_body_velocity(0.0, 0.0, 0.0, 0.0)
            await asyncio.sleep(0.05)

        return reason

    # -----------------------------------------------------------------
    # Main loop — "spin at spots"
    #
    # Strategy: x500_vision's fake VIO drifts catastrophically during
    # extended motion in tight spaces. Spin-in-place keeps the drone
    # stationary (no translation = no drift compounding) while still
    # sweeping the depth + YOLO over 360° at each spot. Between spots, a
    # short bounded forward burst translates ~5 m in the safest direction.
    #
    # At each spot we do BOTH low-alt (yellow barrels) and high-alt (red
    # barrels) sweeps before moving on.
    # -----------------------------------------------------------------
    async def _main_loop_spin(self) -> None:
        spot_idx = 0
        max_spots = 12   # try many; bailout will end us early

        # Track previous transit directions to bias AWAY from where we came from.
        prev_transit_yaws = []

        while not self._stop.is_set() and spot_idx < max_spots:
            elapsed = time.time() - self._mission_t0
            if elapsed > cfg.BAILOUT_SECONDS:
                print(f"[mission] bailout at t={elapsed:.1f} s")
                break

            pose = self.state.snapshot()
            if pose is None:
                await asyncio.sleep(0.2)
                continue
            if self._is_ekf_runaway(pose):
                print(f"[mission] EKF horizontal runaway: "
                      f"pos=({pose['north']:.1f},{pose['east']:.1f}) — aborting")
                break

            print(f"[mission] === SPOT {spot_idx+1} t={elapsed:.0f}s "
                  f"pos=({pose['north']:+.1f},{pose['east']:+.1f},{pose['down']:+.1f}) ===")

            # Single rotational scan at this spot. Skip the dedicated high
            # pass — red barrels on stacks are visible via the upper rows of
            # the depth image even from low altitude (verified in v8: red at
            # NED down=-5.17 detected from LOW pose at down=-1.2 by pixel-Y
            # projection upward).
            self._target_down = cfg.ALTITUDE_LOW_DOWN
            self._alt_phase = "low"
            print(f"[mission] scanning at spot {spot_idx+1}")
            readings = await self._slow_360_spin(
                duration_s=cfg.SPIN_DURATION_S,
                yaw_rate_deg_s=cfg.SPIN_YAW_RATE)

            # Pick best transit direction. Bias against the reverse of the
            # most recent transit so we don't immediately retrace.
            best_yaw = self._pick_outward_heading(readings, prev_transit_yaws)
            if best_yaw is None:
                print(f"[mission] spot {spot_idx+1} — no escape; ending exploration")
                break

            # Sprint forward to next spot — long bounded position setpoints.
            print(f"[mission] sprint toward yaw={best_yaw:+.1f}°")
            await self._yaw_only_rotate_to(best_yaw, tolerance=8.0, timeout_s=4.0)
            reason = await self._fly_forward_via_position(
                max_distance=cfg.TRANSIT_DISTANCE_M,
                max_seconds=cfg.TRANSIT_TIMEOUT_S)
            print(f"[mission] sprint stopped: {reason}")
            if reason == "ekf_runaway":
                break

            prev_transit_yaws.append(best_yaw)
            if len(prev_transit_yaws) > 3:
                prev_transit_yaws.pop(0)
            spot_idx += 1

    def _pick_outward_heading(self,
                              readings: list,
                              recent_transit_yaws: list) -> Optional[float]:
        """Pick the heading with best clearance, penalising headings near
        the reverse of any recent transit direction (so we don't backtrack)."""
        candidates = []
        for yaw, center, left, right in readings:
            if not math.isfinite(center) or center < cfg.CRITICAL_DISTANCE * 1.2:
                continue
            penalty = 0.0
            for prev in recent_transit_yaws:
                reverse_of_prev = ((prev + 180.0 + 540.0) % 360.0) - 180.0
                if abs(self._yaw_diff(yaw, reverse_of_prev)) < 40:
                    penalty = 50.0
                    break
            score = center - penalty
            candidates.append((score, yaw, center))
        if not candidates:
            return None
        candidates.sort(reverse=True)
        score, yaw, center = candidates[0]
        print(f"[mission] outward heading {yaw:+.1f}° (clearance {center:.2f} m)")
        return yaw

    async def _slow_360_spin(self, duration_s: float,
                             yaw_rate_deg_s: float) -> list:
        """Spin in place at the current spot, sampling depth + clearance
        periodically. Returns a list of (yaw_deg, center, left, right)
        readings interleaved with the rotation."""
        readings = []
        t_end = time.time() + duration_s
        sample_interval = 1.5
        next_sample = time.time() + 0.5  # initial settle

        while time.time() < t_end and not self._stop.is_set():
            pose = self.state.snapshot()
            v_down = self._altitude_correction_velocity(pose)
            await self.drone.send_body_velocity(0.0, 0.0, v_down, yaw_rate_deg_s)

            if time.time() >= next_sample:
                depth = self.depth_receiver.get_frame()
                if pose is not None and depth is not None:
                    try:
                        left, center, right = self.planner.compute_clearance(depth)
                    except Exception:
                        left = center = right = float("nan")
                else:
                    left = center = right = float("nan")
                if pose is not None:
                    readings.append((pose["yaw_deg"], float(center),
                                    float(left), float(right)))
                    print(f"[spin] yaw={pose['yaw_deg']:+6.1f}° "
                          f"L={left:.2f} C={center:.2f} R={right:.2f}")
                next_sample = time.time() + sample_interval

                if pose is not None and self._is_ekf_runaway(pose):
                    print("[spin] EKF horizontal runaway — aborting spin")
                    self._stop.set()
                    return readings

            await asyncio.sleep(0.1)

        # Settle to zero yaw rate before exiting.
        for _ in range(5):
            pose = self.state.snapshot()
            v_down = self._altitude_correction_velocity(pose)
            await self.drone.send_body_velocity(0.0, 0.0, v_down, 0.0)
            await asyncio.sleep(0.05)
        return readings

    async def _fly_forward_via_position(self, max_distance: float,
                                        max_seconds: float = 7.0) -> str:
        """Fly forward using bounded position-offset setpoints (NED). At each
        step the target is at most STEP_SIZE away from the drone's current
        EKF pose, so even a runaway EKF can't drive PX4 to a wild target.
        Stops on max_distance covered (EKF-measured), depth blocked, timeout,
        or EKF runaway."""
        pose0 = self.state.snapshot()
        if pose0 is None:
            return "no_pose"
        start_n, start_e = pose0["north"], pose0["east"]
        t_end = time.time() + max_seconds
        loop_dt = 1.0 / cfg.LOOP_HZ

        while time.time() < t_end and not self._stop.is_set():
            pose = self.state.snapshot()
            if pose is None:
                await asyncio.sleep(loop_dt)
                continue
            if self._is_ekf_runaway(pose):
                return "ekf_runaway"

            dist_traveled = math.hypot(pose["north"] - start_n,
                                       pose["east"] - start_e)
            if dist_traveled >= max_distance:
                return "max_distance"

            # Depth-based block check.
            depth = self.depth_receiver.get_frame()
            if depth is not None:
                try:
                    left, center, right = self.planner.compute_clearance(depth)
                except Exception:
                    left = center = right = float("inf")
            else:
                left = center = right = float("inf")
            if math.isfinite(center) and center < cfg.CRITICAL_DISTANCE:
                return "blocked"

            # Slow down when obstacles approach.
            if not math.isfinite(center) or center > cfg.SAFE_DISTANCE:
                step_ahead = cfg.STEP_SIZE
            elif center > cfg.CRITICAL_DISTANCE:
                step_ahead = cfg.STEP_SIZE * (
                    (center - cfg.CRITICAL_DISTANCE) /
                    max(cfg.SAFE_DISTANCE - cfg.CRITICAL_DISTANCE, 1e-3))
            else:
                step_ahead = 0.0

            # Compute position-offset setpoint in current heading direction.
            yaw_rad = pose["yaw"]
            target_n = pose["north"] + step_ahead * math.cos(yaw_rad)
            target_e = pose["east"]  + step_ahead * math.sin(yaw_rad)

            # Altitude: clamp delta to STEP_SIZE.
            dz = self._target_down - pose["down"]
            dz_clamped = max(-cfg.STEP_SIZE, min(cfg.STEP_SIZE, dz))
            target_d = pose["down"] + dz_clamped

            await self.drone.send_position_setpoint(
                target_n, target_e, target_d, pose["yaw_deg"])
            await asyncio.sleep(loop_dt)

        return "timeout"

    async def _climb_to_target(self, target_down: float,
                               timeout_s: float = 8.0) -> None:
        """Drive the drone toward target_down via body-frame Z velocity.
        Bounded so EKF Z drift doesn't cause runaway commands."""
        deadline = time.time() + timeout_s
        while time.time() < deadline and not self._stop.is_set():
            pose = self.state.snapshot()
            if pose is None:
                await asyncio.sleep(0.1)
                continue
            err = target_down - pose["down"]
            if abs(err) < 0.3:
                break
            v = max(-cfg.MAX_VERTICAL_SPEED,
                    min(cfg.MAX_VERTICAL_SPEED, err * 0.6))
            await self.drone.send_body_velocity(0.0, 0.0, v, 0.0)
            await asyncio.sleep(0.1)

    # -----------------------------------------------------------------
    # High-altitude survey — climb above wall tops, sweep ground from above
    # -----------------------------------------------------------------
    async def _main_loop_high_alt(self) -> None:
        """Strategy: climb to ALTITUDE_SURVEY_DOWN (well above wall tops),
        fly forward in body frame for a fixed time, then turn 90° and
        repeat. The forward-facing camera looks at a band of ground below
        and ahead, so YOLO sweeps a wide footprint per second.

        No avoidance stops, no spins. EKF stays stable because we're not
        colliding with anything."""
        # Climb to survey altitude.
        self._target_down = cfg.ALTITUDE_SURVEY_DOWN
        print(f"[mission] climbing to survey altitude (down={cfg.ALTITUDE_SURVEY_DOWN:.1f} m)")
        await self._climb_to_target(cfg.ALTITUDE_SURVEY_DOWN, timeout_s=15.0)

        sprint_idx = 0
        # Alternating turn pattern: right, right, left, left, right, right, ...
        # which produces a lawnmower coverage on a flat plane.
        turn_pattern = [+90.0, +90.0, -90.0, -90.0]

        while not self._stop.is_set():
            elapsed = time.time() - self._mission_t0
            if elapsed > cfg.BAILOUT_SECONDS:
                print(f"[mission] bailout at t={elapsed:.1f} s")
                break

            pose = self.state.snapshot()
            if pose is None:
                await asyncio.sleep(0.2)
                continue
            if self._is_ekf_runaway(pose):
                print(f"[mission] EKF horizontal runaway: "
                      f"pos=({pose['north']:.1f},{pose['east']:.1f}) — aborting")
                break

            print(f"[mission] === SPRINT {sprint_idx+1} t={elapsed:.0f}s "
                  f"pos=({pose['north']:+.1f},{pose['east']:+.1f},{pose['down']:+.1f}) "
                  f"yaw={pose['yaw_deg']:+.1f} ===")

            # Long forward sprint at high altitude. The depth-clearance check
            # is still active for safety, but at altitude 6.5 m the depth
            # image is dominated by the ground 7-14 m ahead, which is well
            # above SAFE_DISTANCE so blocking is rare.
            reason = await self._fly_forward_via_position(
                max_distance=cfg.SURVEY_SPRINT_M,
                max_seconds=cfg.SURVEY_SPRINT_TIMEOUT_S)
            print(f"[mission] sprint stopped: {reason}")
            if reason == "ekf_runaway":
                break

            # Lawnmower-style turn: pick the next angle from the pattern.
            turn_deg = turn_pattern[sprint_idx % len(turn_pattern)]
            sprint_idx += 1
            pose = self.state.snapshot()
            if pose is None:
                continue
            new_yaw = ((pose["yaw_deg"] + turn_deg + 540.0) % 360.0) - 180.0
            print(f"[mission] lawn turn {turn_deg:+.0f}° → yaw {new_yaw:+.1f}°")
            await self._yaw_only_rotate_to(new_yaw, tolerance=8.0, timeout_s=5.0)

    # -----------------------------------------------------------------
    # Old wall-follow loop kept as reference; main_loop now delegates.
    # -----------------------------------------------------------------
    async def _main_loop(self) -> None:
        if cfg.STRATEGY == "high_alt":
            await self._main_loop_high_alt()
        else:
            await self._main_loop_spin()

    async def _main_loop_wallfollow(self) -> None:
        visited_cells = set()
        consec_blocks = 0
        last_blocked_heading = None

        # Wait briefly for telemetry to stabilise after takeoff.
        await asyncio.sleep(1.0)

        while not self._stop.is_set():
            elapsed = time.time() - self._mission_t0
            if elapsed > cfg.BAILOUT_SECONDS:
                print(f"[mission] bailout at t={elapsed:.1f} s")
                break

            self._maybe_switch_altitude_phase(elapsed)
            pose = self.state.snapshot()
            if pose is None:
                await asyncio.sleep(0.2)
                continue
            if self._is_ekf_runaway(pose):
                print(f"[mission] EKF runaway: pos=({pose['north']:.1f},"
                      f"{pose['east']:.1f},{pose['down']:.1f}) — aborting")
                break

            # Record visited cell.
            cell = self._cell_key(pose["north"], pose["east"])
            visited_cells.add(cell)

            # Fly forward in the current heading.
            print(f"[mission] t={elapsed:.0f}s ph={self._alt_phase} "
                  f"pos=({pose['north']:+.2f},{pose['east']:+.2f},{pose['down']:+.2f}) "
                  f"yaw={pose['yaw_deg']:+.1f} visited={len(visited_cells)} — flying forward")
            reason = await self._fly_forward(
                max_distance=cfg.FORWARD_STEP_M,
                max_seconds=cfg.FORWARD_STEP_TIMEOUT_S)
            print(f"[mission] forward stopped: {reason}")

            if reason == "ekf_runaway":
                break

            if reason == "max_distance":
                # Path still clear ahead — keep going next iteration.
                consec_blocks = 0
                continue

            # Reason == "blocked" → wall-follow or scan.
            pose_blk = self.state.snapshot()
            if pose_blk is None:
                continue
            last_blocked_heading = pose_blk["yaw_deg"]
            consec_blocks += 1

            if consec_blocks <= cfg.WALL_FOLLOW_BLOCKS_BEFORE_SCAN:
                # Wall-follow: turn FIXED_TURN_DEG (always same sign).
                turn_to = ((pose_blk["yaw_deg"] + cfg.FIXED_TURN_DEG + 540.0) % 360.0) - 180.0
                print(f"[mission] wall-follow: turning to {turn_to:+.1f}°")
                await self._yaw_only_rotate_to(turn_to, tolerance=8.0, timeout_s=5.0)
                continue

            # Dead-end suspected — full scan, pick heading toward most
            # unvisited region with adequate clearance.
            print("[mission] dead-end suspected — running full scan")
            readings = await self._rotate_scan(
                num_steps=cfg.SCAN_STEPS, dwell_s=cfg.SCAN_DWELL_S)
            new_yaw = self._pick_unvisited_heading(
                readings, pose["north"], pose["east"], visited_cells)
            if new_yaw is None:
                # Everything visited or blocked. Pick any clear heading.
                new_yaw = self._pick_best_heading(readings)
            if new_yaw is None:
                print("[mission] no clear heading anywhere — descending 0.5 m and continuing")
                self._target_down = min(self._target_down + 0.5,
                                        cfg.MIN_ALTITUDE_DOWN - 0.5)
                consec_blocks = 0
                continue
            await self._yaw_only_rotate_to(new_yaw, tolerance=8.0, timeout_s=5.0)
            consec_blocks = 0

    @staticmethod
    def _cell_key(north: float, east: float) -> tuple:
        return (int(north // cfg.VISITED_CELL_SIZE),
                int(east // cfg.VISITED_CELL_SIZE))

    def _pick_unvisited_heading(self,
                                readings: list,
                                north: float, east: float,
                                visited_cells: set) -> Optional[float]:
        """Choose a heading that points toward an unvisited grid cell while
        having adequate clearance."""
        candidates = []
        step = cfg.FORWARD_STEP_M
        for yaw, center, left, right in readings:
            if not math.isfinite(center) or center < cfg.CRITICAL_DISTANCE * 1.5:
                continue
            # Predict the cell we'd land in.
            dn = step * math.cos(math.radians(yaw))
            de = step * math.sin(math.radians(yaw))
            cell = self._cell_key(north + dn, east + de)
            score = center + (5.0 if cell not in visited_cells else 0.0)
            candidates.append((score, yaw, center, cell in visited_cells))
        if not candidates:
            return None
        candidates.sort(reverse=True)
        score, yaw, center, was_visited = candidates[0]
        print(f"[mission] pick_unvisited best={yaw:+.1f}° clear={center:.2f}m "
              f"visited={was_visited}")
        return yaw

    async def run(self) -> None:
        print("=== BrainHack-26 mission start ===")
        await self.drone.connect()
        await asyncio.sleep(2.0)

        tel_task = asyncio.create_task(telemetry_task(self.drone, self.state, self._stop))

        await self._wait_for_pose(timeout_s=20.0)
        await self._wait_for_armable(timeout_s=30.0)

        depth_ok = await self._wait_for_depth(timeout_s=15.0)
        if not depth_ok:
            print("[mission] WARNING: depth all-inf / no frame — flying blind")

        print("[mission] arming + taking off")
        await self.drone.arm_and_takeoff()
        print("[mission] offboard active")

        self._mission_t0 = time.time()
        try:
            await self._main_loop()
        finally:
            await self._land()
            self._stop.set()
            tel_task.cancel()
            try:
                await tel_task
            except asyncio.CancelledError:
                pass

    async def _land(self) -> None:
        print("[mission] descending + landing")
        # Gradual descent — let PX4's altitude controller handle it.
        for _ in range(20):
            await self.drone.send_body_velocity(0.0, 0.0, 0.3, 0.0)
            await asyncio.sleep(0.2)

        snap = self.tracker.snapshot()
        print(f"[mission] raw detections:       total={snap['raw_total']} "
              f"by_class={snap['raw_by_class']}")
        print(f"[mission] validated detections: total={snap['validated_total']} "
              f"by_class={snap['validated_by_class']}")
        print(f"[mission] barrels.json -> {cfg.BARRELS_JSON}")

        try:
            await self.drone.land()
        except Exception as exc:
            print(f"[mission] land error: {exc}")

    async def shutdown(self) -> None:
        print("[mission] shutdown")
        self._stop.set()
        try:
            self.detector.stop()
        except Exception:
            pass


# =====================================================================
#  Entry point
# =====================================================================

async def main():
    mission = Mission()

    loop = asyncio.get_running_loop()
    stop_signal = asyncio.Event()

    def _sig_handler():
        print("\n[mission] signal — stopping")
        stop_signal.set()
        mission._stop.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _sig_handler)
        except NotImplementedError:
            pass

    run_task = asyncio.create_task(mission.run())

    done, pending = await asyncio.wait(
        {run_task, asyncio.create_task(stop_signal.wait())},
        return_when=asyncio.FIRST_COMPLETED,
    )

    if stop_signal.is_set() and not run_task.done():
        await mission.shutdown()
        try:
            await asyncio.wait_for(run_task, timeout=cfg.SHUTDOWN_GRACE_SECONDS)
        except asyncio.TimeoutError:
            print("[mission] graceful shutdown timeout")
            run_task.cancel()
            try:
                await run_task
            except asyncio.CancelledError:
                pass

    await mission.shutdown()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInterrupted")
        sys.exit(0)
