"""
lawnmower.py — two-altitude lawnmower exploration policy.

Generates a list of NED waypoints that sweep the world in a boustrophedon
pattern at one altitude, then climbs and sweeps again at a higher altitude.
The low pass finds yellow (ground-level) barrels; the high pass finds red
(elevated) barrels.

The policy is "open-loop" — it does NOT consult the depth camera itself. It
just emits a target waypoint at each step. The mission loop blends this
target with reactive avoidance from AvoidancePlanner: if the depth-based
clearance says "blocked", we either sidestep or advance to the next
waypoint, whichever the mission policy decides.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import mission_config as cfg


@dataclass
class Waypoint:
    north: float
    east: float
    down: float
    yaw_deg: float
    phase: str    # "low", "high", "return"


def _lawnmower_lanes(n_min: float, n_max: float,
                     e_min: float, e_max: float,
                     spacing: float,
                     altitude_down: float,
                     phase: str,
                     start_n: float = 0.0, start_e: float = 0.0) -> List[Waypoint]:
    """
    Build a boustrophedon (north-south lanes, alternating direction)
    sweep over the (n_min..n_max, e_min..e_max) rectangle, with `spacing`
    metres between lanes.
    """
    waypoints: List[Waypoint] = []

    # Build east coordinates of each lane. Start from whichever side of
    # `start_e` is closer.
    lanes = []
    e = e_min
    while e <= e_max + 1e-3:
        lanes.append(e)
        e += spacing
    if not lanes:
        return waypoints

    if abs(start_e - e_min) > abs(start_e - e_max):
        lanes = list(reversed(lanes))

    going_north = (start_n - n_min) < (n_max - start_n)

    for i, e_val in enumerate(lanes):
        if going_north:
            north_target = n_max
        else:
            north_target = n_min
        # First, slide east to the lane.
        yaw = 0.0 if going_north else 180.0
        waypoints.append(Waypoint(
            north=waypoints[-1].north if waypoints else start_n,
            east=e_val,
            down=altitude_down,
            yaw_deg=yaw,
            phase=phase,
        ))
        # Then sweep along this lane.
        waypoints.append(Waypoint(
            north=north_target,
            east=e_val,
            down=altitude_down,
            yaw_deg=yaw,
            phase=phase,
        ))
        going_north = not going_north

    return waypoints


class Lawnmower:
    """Stateful lawnmower waypoint manager."""

    def __init__(self,
                 north_min: float = cfg.WORLD_NORTH_MIN,
                 north_max: float = cfg.WORLD_NORTH_MAX,
                 east_min:  float = cfg.WORLD_EAST_MIN,
                 east_max:  float = cfg.WORLD_EAST_MAX,
                 spacing:   float = cfg.SWEEP_SPACING,
                 low_down:  float = cfg.ALTITUDE_LOW_DOWN,
                 high_down: float = cfg.ALTITUDE_HIGH_DOWN,
                 reached_radius: float = cfg.WAYPOINT_REACHED_RADIUS,
                 phase_low_seconds: float = cfg.PHASE_LOW_SECONDS,
                 phase_high_seconds: float = cfg.PHASE_HIGH_SECONDS):
        self.reached_radius = reached_radius
        self.phase_low_seconds = phase_low_seconds
        self.phase_high_seconds = phase_high_seconds

        low = _lawnmower_lanes(north_min, north_max, east_min, east_max,
                               spacing, low_down, phase="low")
        # Climb transition waypoint at the end of the low pass.
        climb = []
        if low:
            climb.append(Waypoint(
                north=low[-1].north, east=low[-1].east,
                down=high_down, yaw_deg=low[-1].yaw_deg, phase="climb",
            ))
        high = _lawnmower_lanes(north_min, north_max, east_min, east_max,
                                spacing, high_down, phase="high",
                                start_n=low[-1].north if low else 0.0,
                                start_e=low[-1].east if low else 0.0)

        self.waypoints: List[Waypoint] = low + climb + high

        # Return-to-launch placeholder; the mission_loop will append this
        # dynamically when bailout fires.
        self.idx = 0
        self._phase_start_t = time.time()
        self._current_phase = self.waypoints[0].phase if self.waypoints else "done"

    def current(self) -> Optional[Waypoint]:
        if self.idx >= len(self.waypoints):
            return None
        return self.waypoints[self.idx]

    def advance(self, reason: str = "reached") -> Optional[Waypoint]:
        """Move to the next waypoint. Returns the new current waypoint."""
        prev = self.current()
        self.idx = min(self.idx + 1, len(self.waypoints))
        nxt = self.current()
        if nxt is not None and prev is not None and nxt.phase != prev.phase:
            self._current_phase = nxt.phase
            self._phase_start_t = time.time()
        return nxt

    def skip_phase(self) -> Optional[Waypoint]:
        """Jump to the first waypoint of the next phase, e.g. when the time
        budget for the current pass has expired."""
        cur = self.current()
        if cur is None:
            return None
        target_phase = cur.phase
        while self.idx < len(self.waypoints) and self.waypoints[self.idx].phase == target_phase:
            self.idx += 1
        nxt = self.current()
        if nxt is not None:
            self._current_phase = nxt.phase
            self._phase_start_t = time.time()
        return nxt

    def maybe_force_phase_change(self) -> bool:
        """If the current phase has run too long, jump ahead. Returns True if
        a phase change happened."""
        cur = self.current()
        if cur is None:
            return False
        elapsed = time.time() - self._phase_start_t
        budget = self.phase_low_seconds if cur.phase == "low" else self.phase_high_seconds
        if elapsed > budget:
            print(f"[lawnmower] phase '{cur.phase}' exceeded {budget}s — skipping")
            self.skip_phase()
            return True
        return False

    def is_reached(self, north: float, east: float, down: float) -> bool:
        cur = self.current()
        if cur is None:
            return True
        dn = cur.north - north
        de = cur.east - east
        dd = cur.down - down
        # The altitude tolerance is more forgiving (drones drift in z).
        return math.hypot(dn, de) < self.reached_radius and abs(dd) < 0.6

    def goal_vector_ned(self, north: float, east: float) -> Tuple[float, float, float]:
        """Returns (north_dir, east_dir, distance) for the current goal."""
        cur = self.current()
        if cur is None:
            return (0.0, 0.0, 0.0)
        dn = cur.north - north
        de = cur.east  - east
        dist = math.hypot(dn, de)
        if dist < 1e-3:
            return (0.0, 0.0, 0.0)
        return (dn / dist, de / dist, dist)

    def remaining(self) -> int:
        return max(0, len(self.waypoints) - self.idx)
