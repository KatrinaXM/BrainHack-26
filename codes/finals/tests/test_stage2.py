#!/usr/bin/env python3
"""
test_stage2.py — unit tests for the deterministic parts of stage2_mission.py.

Aligned with the Finals brief: the detector tests cover ArUco markers
(not the old HSV RoboMaster colour detector).

Covers:
- Pad loading + validity filtering + selection
- ArUco detector on synthetic frames containing real cv2-rendered
  markers (positive + negative cases)
- save_snapshot writes both JPEG and JSON sidecar; JSON includes
  decoded ArUco IDs; mission tracks unique IDs seen
- navigate_to_pad emits the right Direction sequence (using mock DroneAPI)
- Mock DroneAPI state-machine: rejects move-before-takeoff; allows
  re-takeoff after land (required for SEARCH_TAKEOFF phase)

Run:
    BH26_MOCK=1 python3 -m unittest codes/finals/tests/test_stage2.py
or
    cd codes/finals && BH26_MOCK=1 python3 -m unittest tests/test_stage2.py
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

# Make sure the orchestrator picks up the mock backend before we import it.
os.environ["BH26_MOCK"] = "1"
# Use fast mock timings so tests are quick.
os.environ.setdefault("BH26_MOCK_TAKEOFF_S", "0.05")
os.environ.setdefault("BH26_MOCK_LAND_S",    "0.05")
os.environ.setdefault("BH26_MOCK_SPEED_MPS", "100.0")

# Add codes/finals/ to sys.path so we can import stage2_mission and mocks.
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import cv2              # noqa: E402
import numpy as np      # noqa: E402

import stage2_mission   # noqa: E402
from stage2_mission import (   # noqa: E402
    Pad, load_pads, select_pads,
    detect_aruco_markers, save_snapshot, navigate_to_pad,
    DroneMission, DroneState,
)
from mocks.pyhulax_mock import DroneAPI, Direction      # noqa: E402


# ----- ArUco rendering helper for tests -----

# Match the detector's default dictionary (DICT_7X7_1000, confirmed by
# organizers 2026-06-10) so rendered test markers are actually detectable.
_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)


def _frame_with_marker(marker_id: int, h: int = 480, w: int = 640,
                       size: int = 120, cx: int = 320, cy: int = 240) -> np.ndarray:
    """Build an RGB frame with a real ArUco marker rendered at (cx, cy)."""
    bg = np.full((h, w, 3), 96, dtype=np.uint8)
    gray = cv2.aruco.generateImageMarker(_ARUCO_DICT, marker_id, size)
    rgb_patch = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    x0, y0 = cx - size // 2, cy - size // 2
    bg[y0:y0 + size, x0:x0 + size] = rgb_patch
    return bg


def _blank_frame(h: int = 480, w: int = 640) -> np.ndarray:
    return np.full((h, w, 3), 96, dtype=np.uint8)


# ============================================================================
#  Pad loading + selection
# ============================================================================

class PadLoadingTests(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False)
        json.dump([
            {"id": "A", "x": 1.0, "y": 0.0, "z": 0.0, "valid": True},
            {"id": "B", "x": 2.0, "y": 0.0, "z": 0.0, "valid": False},
            {"id": "C", "x": 3.0, "y": 0.0, "z": 0.0, "valid": True},
            {"id": "D", "x": 4.0, "y": 0.0, "z": 0.0, "valid": True},
        ], self.tmp)
        self.tmp.close()

    def tearDown(self):
        os.unlink(self.tmp.name)

    def test_load_pads_parses_all_entries(self):
        pads = load_pads(Path(self.tmp.name))
        self.assertEqual(len(pads), 4)
        self.assertEqual(pads[0].pad_id, "A")
        self.assertTrue(pads[0].valid)
        self.assertFalse(pads[1].valid)

    def test_select_pads_filters_invalid(self):
        pads = load_pads(Path(self.tmp.name))
        chosen = select_pads(pads, 3)
        self.assertEqual([p.pad_id for p in chosen], ["A", "C", "D"])
        for p in chosen:
            self.assertTrue(p.valid)

    def test_select_pads_raises_when_not_enough(self):
        pads = [
            Pad("X", 0, 0, 0, True),
            Pad("Y", 0, 0, 0, False),
        ]
        with self.assertRaises(RuntimeError):
            select_pads(pads, 3)


# ============================================================================
#  ArUco detector
# ============================================================================

class DetectorTests(unittest.TestCase):

    def test_blank_frame_returns_no_markers(self):
        detected, markers = detect_aruco_markers(_blank_frame())
        self.assertFalse(detected)
        self.assertEqual(markers, [])

    def test_single_marker_detected_with_correct_id(self):
        frame = _frame_with_marker(marker_id=42)
        detected, markers = detect_aruco_markers(frame)
        self.assertTrue(detected)
        self.assertEqual(len(markers), 1)
        self.assertEqual(markers[0]["id"], 42)

    def test_marker_bbox_roughly_correct(self):
        # Place a 120x120 marker centred at (300, 200).
        frame = _frame_with_marker(marker_id=7, size=120, cx=300, cy=200)
        _, markers = detect_aruco_markers(frame)
        self.assertEqual(len(markers), 1)
        bbox = markers[0]["bbox"]
        # bbox top-left should be near (240, 140); width/height near 120.
        self.assertAlmostEqual(bbox["x"], 240, delta=5)
        self.assertAlmostEqual(bbox["y"], 140, delta=5)
        self.assertAlmostEqual(bbox["w"], 120, delta=10)
        self.assertAlmostEqual(bbox["h"], 120, delta=10)

    def test_multiple_markers_detected_with_distinct_ids(self):
        # Render two markers in one frame.
        frame = _blank_frame()
        for mid, cx in [(11, 160), (22, 480)]:
            patch_gray = cv2.aruco.generateImageMarker(_ARUCO_DICT, mid, 100)
            patch_rgb = cv2.cvtColor(patch_gray, cv2.COLOR_GRAY2RGB)
            frame[180:280, cx - 50:cx + 50] = patch_rgb
        detected, markers = detect_aruco_markers(frame)
        self.assertTrue(detected)
        ids = sorted(m["id"] for m in markers)
        self.assertEqual(ids, [11, 22])


# ============================================================================
#  Snapshot writer
# ============================================================================

class SnapshotTests(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()
        self._saved_output_dir = stage2_mission.OUTPUT_DIR
        stage2_mission.OUTPUT_DIR = Path(self.tmp_dir.name)

    def tearDown(self):
        stage2_mission.OUTPUT_DIR = self._saved_output_dir
        self.tmp_dir.cleanup()

    def _mission(self) -> DroneMission:
        return DroneMission(
            plane_id="ptest", ip="0.0.0.0",
            drone=DroneAPI(), video=None,
            pad=Pad("Pz", 0.0, 0.0, 0.0, True),
        )

    def test_save_writes_jpeg_and_json_with_marker_ids(self):
        frame = _frame_with_marker(marker_id=99)
        m = self._mission()
        markers = [{
            "id": 99,
            "corners": [[10, 10], [110, 10], [110, 110], [10, 110]],
            "bbox": {"x": 10, "y": 10, "w": 100, "h": 100},
        }]
        save_snapshot(frame, m, markers)
        files = sorted(Path(self.tmp_dir.name).iterdir())
        suffixes = sorted(f.suffix for f in files)
        self.assertEqual(suffixes, [".jpg", ".json"])
        # JPEG is non-empty.
        jpg = next(f for f in files if f.suffix == ".jpg")
        self.assertGreater(jpg.stat().st_size, 1000)
        # JSON has the expected metadata + marker IDs.
        sidecar = next(f for f in files if f.suffix == ".json")
        meta = json.loads(sidecar.read_text())
        self.assertEqual(meta["plane_id"], "ptest")
        self.assertEqual(meta["pad_id"], "Pz")
        self.assertEqual(meta["snapshot_num"], 1)
        self.assertEqual(meta["marker_ids"], [99])
        self.assertEqual(len(meta["markers"]), 1)
        # Mission's seen-set updated.
        self.assertIn(99, m.marker_ids_seen)

    def test_save_throttled_by_cooldown(self):
        frame = _blank_frame()
        m = self._mission()
        markers = [{"id": 1, "corners": [[0, 0]] * 4,
                    "bbox": {"x": 0, "y": 0, "w": 1, "h": 1}}]
        save_snapshot(frame, m, markers)
        save_snapshot(frame, m, markers)
        # Should be exactly 2 files: 1 jpg + 1 json from the first save.
        files = list(Path(self.tmp_dir.name).iterdir())
        self.assertEqual(len(files), 2)
        self.assertEqual(m.snapshots_saved, 1)

    def test_unique_marker_ids_aggregated_across_snapshots(self):
        m = self._mission()
        # Force cooldown to 0 for this test by manipulating last_snapshot_at.
        m.last_snapshot_at = 0.0
        save_snapshot(_blank_frame(), m, [
            {"id": 1, "corners": [[0, 0]] * 4,
             "bbox": {"x": 0, "y": 0, "w": 1, "h": 1}},
            {"id": 2, "corners": [[0, 0]] * 4,
             "bbox": {"x": 0, "y": 0, "w": 1, "h": 1}},
        ])
        # Reset cooldown to allow a second save in the same test.
        m.last_snapshot_at = 0.0
        save_snapshot(_blank_frame(), m, [
            {"id": 2, "corners": [[0, 0]] * 4,
             "bbox": {"x": 0, "y": 0, "w": 1, "h": 1}},
            {"id": 3, "corners": [[0, 0]] * 4,
             "bbox": {"x": 0, "y": 0, "w": 1, "h": 1}},
        ])
        self.assertEqual(sorted(m.marker_ids_seen), [1, 2, 3])


# ============================================================================
#  navigate_to_pad (via mock)
# ============================================================================

class _RecordingDroneAPI(DroneAPI):
    """Mock DroneAPI subclass that records movement calls for assertions.

    Distances are recorded as passed to pyhulax — i.e. in DEVICE UNITS
    (centimetres by default), after navigate_to_pad applies DIST_SCALE.
    """

    def __init__(self):
        super().__init__()
        self.moves: list[tuple[str, float]] = []
        self.move_tos: list[tuple[float, float, float]] = []
        self.takeoff_heights: list[int] = []

    def move(self, direction: Direction, distance_cm: float, **kwargs) -> None:
        self.moves.append((direction.name, float(distance_cm)))
        super().move(direction, distance_cm)

    def move_to(self, x: float, y: float, z: float, **kwargs) -> None:
        self.move_tos.append((float(x), float(y), float(z)))
        super().move_to(x, y, z)

    def takeoff(self, height_cm: int = 100, **kwargs) -> None:
        self.takeoff_heights.append(int(height_cm))
        super().takeoff(height_cm=height_cm)


class NavigateTests(unittest.TestCase):

    def _airborne_drone(self) -> _RecordingDroneAPI:
        d = _RecordingDroneAPI()
        d.connect("1.2.3.4")
        d.takeoff()
        return d

    def test_navigate_emits_forward_right_for_positive_pad(self):
        d = self._airborne_drone()
        pad = Pad("P", 3.0, 2.0, 0.0, True)  # +x, +y, pad on the ground (z=0)
        navigate_to_pad("t", d, pad)
        names = [n for n, _ in d.moves]
        # No descent leg: the drone cruises at altitude and land() drops it
        # onto the pad. Distances are INTEGER cm (metres * DIST_SCALE = 100).
        self.assertEqual(names, ["FORWARD", "RIGHT"])
        self.assertEqual(d.moves[0], ("FORWARD", 300.0))
        self.assertEqual(d.moves[1], ("RIGHT", 200.0))
        # Every distance handed to pyhulax must be a whole number of cm.
        for _, dist in d.moves:
            self.assertEqual(dist, int(dist))

    def test_navigate_splits_legs_over_500cm(self):
        d = self._airborne_drone()
        # 7.5 m forward (750 cm) and 5.5 m right (550 cm) both exceed the
        # firmware's 500 cm single-move cap and must be split into hops.
        pad = Pad("P", 7.5, 5.5, 0.0, True)
        navigate_to_pad("t", d, pad)
        fwd = [dist for name, dist in d.moves if name == "FORWARD"]
        rt = [dist for name, dist in d.moves if name == "RIGHT"]
        self.assertEqual(sum(fwd), 750.0)   # total distance preserved
        self.assertEqual(sum(rt), 550.0)
        for _, dist in d.moves:             # every hop within firmware limits
            self.assertTrue(5 <= dist <= 500, f"hop {dist} out of [5, 500]")
            self.assertEqual(dist, int(dist))

    def test_navigate_climbs_only_when_pad_above_cruise(self):
        d = self._airborne_drone()
        # Pad hover 1 m ABOVE takeoff altitude → a single UP leg precedes the
        # horizontal move; descending pads never emit a vertical leg.
        pad = Pad("P", 1.0, 0.0, stage2_mission.TAKEOFF_ALT_M + 1.0, True)
        navigate_to_pad("t", d, pad)
        names = [n for n, _ in d.moves]
        self.assertEqual(names, ["UP", "FORWARD"])
        self.assertEqual(d.moves[0], ("UP", 100.0))

    def test_navigate_move_to_mode_issues_single_absolute_call(self):
        d = self._airborne_drone()
        pad = Pad("P", 3.0, 2.0, 0.0, True)
        saved = stage2_mission.NAV_MODE
        stage2_mission.NAV_MODE = "move_to"
        try:
            navigate_to_pad("t", d, pad)
        finally:
            stage2_mission.NAV_MODE = saved
        # No decomposed body-frame moves; exactly one move_to in cm.
        self.assertEqual(d.moves, [])
        self.assertEqual(len(d.move_tos), 1)
        self.assertEqual(d.move_tos[0], (300.0, 200.0, 0.0))

    def test_dist_scale_converts_metres_to_device_units(self):
        # The unit fix: pyhulax wants centimetres, pad files are metres.
        self.assertAlmostEqual(stage2_mission._to_device(1.1), 110.0, places=6)
        self.assertAlmostEqual(stage2_mission._to_device(3.0), 300.0, places=6)

    def test_navigate_emits_back_left_for_negative_pad(self):
        d = self._airborne_drone()
        # Pad at takeoff altitude → no vertical move needed.
        pad = Pad("P", -3.0, -2.0, stage2_mission.TAKEOFF_ALT_M, True)
        navigate_to_pad("t", d, pad)
        names = [n for n, _ in d.moves]
        self.assertEqual(names, ["BACK", "LEFT"])

    def test_navigate_skips_zero_axes(self):
        d = self._airborne_drone()
        # Only +y, no vertical change.
        pad = Pad("P", 0.0, 2.0, stage2_mission.TAKEOFF_ALT_M, True)
        navigate_to_pad("t", d, pad)
        names = [n for n, _ in d.moves]
        self.assertEqual(names, ["RIGHT"])

    def test_navigate_skips_sub_epsilon_moves(self):
        d = self._airborne_drone()
        pad = Pad("P", 0.01, 0.02, stage2_mission.TAKEOFF_ALT_M, True)
        navigate_to_pad("t", d, pad)
        self.assertEqual(d.moves, [])


# ============================================================================
#  Mock state-machine invariants
# ============================================================================

class MockInvariantsTests(unittest.TestCase):

    def test_move_before_takeoff_raises(self):
        d = DroneAPI()
        d.connect("1.2.3.4")
        with self.assertRaises(RuntimeError):
            d.move(Direction.FORWARD, 1.0)

    def test_double_takeoff_raises(self):
        d = DroneAPI()
        d.connect("1.2.3.4")
        d.takeoff()
        with self.assertRaises(RuntimeError):
            d.takeoff()

    def test_land_is_idempotent_after_first_call(self):
        d = DroneAPI()
        d.connect("1.2.3.4")
        d.takeoff()
        d.land()
        # Second land call should be a no-op rather than raise.
        d.land()

    def test_retakeoff_after_land_is_allowed(self):
        # Stage 2 mission requires this: land on pad, then take off again
        # for ArUco search. Mock must allow re-takeoff after land.
        d = DroneAPI()
        d.connect("1.2.3.4")
        d.takeoff()
        d.land()
        d.takeoff()      # must succeed
        d.land()


class AvailabilityTests(unittest.TestCase):
    """Landing-pad availability flag — only AVAILABLE pads get assigned."""

    def _write(self, data) -> Path:
        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False)
        json.dump(data, tmp)
        tmp.close()
        self.addCleanup(os.unlink, tmp.name)
        return Path(tmp.name)

    def test_load_pads_accepts_available_key_and_defaults_z(self):
        path = self._write([
            {"id": "7", "x": 2.5, "y": 5.5, "available": True},        # no z, new key
            {"id": "10", "x": 5.5, "y": 5.5, "z": 0.0, "valid": False},  # legacy key
        ])
        pads = load_pads(path)
        self.assertEqual(pads[0].z, 0.0)        # z defaulted
        self.assertTrue(pads[0].valid)          # "available": true
        self.assertFalse(pads[1].valid)         # legacy "valid": false honored

    def test_select_pads_skips_unavailable(self):
        pads = [
            Pad("7", 0, 0, 0, True), Pad("8", 0, 0, 0, False),
            Pad("10", 0, 0, 0, True), Pad("11", 0, 0, 0, False),
            Pad("12", 0, 0, 0, True),
        ]
        chosen = select_pads(pads, 3)
        self.assertEqual([p.pad_id for p in chosen], ["7", "10", "12"])

    def test_select_pads_raises_when_too_few_available(self):
        pads = [Pad("7", 0, 0, 0, True), Pad("8", 0, 0, 0, False),
                Pad("9", 0, 0, 0, True)]
        with self.assertRaises(RuntimeError):
            select_pads(pads, 3)


class DiscoveryTests(unittest.TestCase):
    """Drone IPs come from BH26_HULA_IPS, drones.json, or dola — no dola needed."""

    def tearDown(self):
        os.environ.pop("BH26_HULA_IPS", None)
        os.environ.pop("BH26_DRONES_FILE", None)

    def test_manual_ips_bypass_dola(self):
        os.environ["BH26_HULA_IPS"] = "10.0.0.1, 10.0.0.2 ,10.0.0.3"
        ips = stage2_mission.discover_hulas()
        self.assertEqual(ips, {
            "plane1": "10.0.0.1",
            "plane2": "10.0.0.2",
            "plane3": "10.0.0.3",
        })

    def test_manual_ips_too_few_raises(self):
        os.environ["BH26_HULA_IPS"] = "10.0.0.1"
        with self.assertRaises(RuntimeError):
            stage2_mission.discover_hulas()

    def test_drones_json_used_when_no_env(self):
        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False)
        json.dump({"1": "10.0.0.1", "2": "10.0.0.2", "3": "10.0.0.3"}, tmp)
        tmp.close()
        self.addCleanup(os.unlink, tmp.name)
        os.environ.pop("BH26_HULA_IPS", None)
        os.environ["BH26_DRONES_FILE"] = tmp.name
        ips = stage2_mission.discover_hulas()
        self.assertEqual(ips, {"1": "10.0.0.1", "2": "10.0.0.2", "3": "10.0.0.3"})


class HardeningTests(unittest.TestCase):
    """Fail-safe behaviour: result checks, battery gate, avoidance, feedback."""

    def test_result_ok_variants(self):
        self.assertTrue(stage2_mission._result_ok(None))        # mock / void
        self.assertTrue(stage2_mission._result_ok(255))         # SUCCESS
        self.assertFalse(stage2_mission._result_ok(241))        # a FAILED code

        class OK:
            name = "SUCCESS"

        class Bad:
            name = "FAILED_241"
        self.assertTrue(stage2_mission._result_ok(OK()))
        self.assertFalse(stage2_mission._result_ok(Bad()))

    def test_cmd_raises_on_failed_result(self):
        class FailDrone:
            def move(self, *a, **k):
                class R:
                    name = "FAILED_241"
                return R()
        with self.assertRaises(stage2_mission.CommandFailed):
            stage2_mission._cmd(FailDrone(), "t", "move", Direction.FORWARD, 100)

    def _mission(self, drone) -> DroneMission:
        return DroneMission(plane_id="t", ip="0", drone=drone, video=None,
                            pad=Pad("P", 0.0, 0.0, 0.0, True))

    def test_preflight_battery_low_raises(self):
        class LowBatt(DroneAPI):
            def get_battery(self):
                return 5
        with self.assertRaises(stage2_mission.CommandFailed):
            stage2_mission._preflight_battery(self._mission(LowBatt()))

    def test_preflight_battery_ok_passes(self):
        stage2_mission._preflight_battery(self._mission(DroneAPI()))  # 100% — no raise

    def test_connect_hulas_enables_barrier_mode(self):
        conns = stage2_mission.connect_hulas({"1": "1.2.3.4"})
        drone = conns[0][2]
        self.assertTrue(getattr(drone, "_barrier_mode", False))

    def test_verify_arrival_false_when_never_reaches(self):
        from mocks.pyhulax_mock import _Vector3

        class StuckDrone(DroneAPI):
            def get_position(self):
                return _Vector3(0, 0, 100)   # never moves toward target
            def move(self, *a, **k):
                return None                  # corrections have no effect
        d = StuckDrone()
        d.connect("1.2.3.4")
        d.takeoff()
        self.assertFalse(stage2_mission._verify_arrival("t", d, 300.0, 200.0))

    def test_verify_arrival_true_when_at_target(self):
        d = DroneAPI()
        d.connect("1.2.3.4")
        d.takeoff()
        d.move(Direction.FORWARD, 300)
        d.move(Direction.RIGHT, 200)
        self.assertTrue(stage2_mission._verify_arrival("t", d, 300.0, 200.0))


class SearchTests(unittest.TestCase):
    """Yaw-scan search coverage + camera tilt."""

    def _mission(self, drone) -> DroneMission:
        return DroneMission(plane_id="t", ip="0", drone=drone, video=None,
                            pad=Pad("P", 0.0, 0.0, 0.0, True))

    def _rot_drone(self):
        class RotDrone(DroneAPI):
            def __init__(self):
                super().__init__()
                self.rotations = []

            def rotate(self, deg, **k):
                self.rotations.append(float(deg))
                return None
        return RotDrone()

    def test_yaw_scan_rotates_after_interval(self):
        d = self._rot_drone()
        m = self._mission(d)
        m.last_yaw_at = 1.0                          # far in the past
        stage2_mission._yaw_scan_tick(m)
        self.assertEqual(d.rotations, [stage2_mission.YAW_STEP_DEG])

    def test_yaw_scan_skips_within_interval(self):
        d = self._rot_drone()
        m = self._mission(d)
        m.last_yaw_at = stage2_mission.time.time()   # just rotated
        stage2_mission._yaw_scan_tick(m)
        self.assertEqual(d.rotations, [])

    def test_yaw_scan_rotate_failure_does_not_raise(self):
        class BadRot(DroneAPI):
            def rotate(self, deg, **k):
                raise RuntimeError("rotate rejected")
        m = self._mission(BadRot())
        m.last_yaw_at = 1.0
        stage2_mission._yaw_scan_tick(m)             # must not raise

    def test_camera_angle_set_at_connect(self):
        saved = stage2_mission.CAMERA_ANGLE
        stage2_mission.CAMERA_ANGLE = "45"
        try:
            conns = stage2_mission.connect_hulas({"1": "1.2.3.4"})
        finally:
            stage2_mission.CAMERA_ANGLE = saved
        drone = conns[0][2]
        self.assertEqual(getattr(drone, "_camera_angle", None), (45, 0))

    def test_all_expected_found_across_drones(self):
        saved = stage2_mission.EXPECTED_MARKER_IDS
        stage2_mission.EXPECTED_MARKER_IDS = {11, 45, 51}
        try:
            m1 = self._mission(DroneAPI())
            m2 = self._mission(DroneAPI())
            m1.marker_ids_seen = {11, 45}
            m2.marker_ids_seen = {51}
            self.assertTrue(stage2_mission._all_expected_found([m1, m2]))
            m2.marker_ids_seen = set()                 # 51 now missing
            self.assertFalse(stage2_mission._all_expected_found([m1, m2]))
        finally:
            stage2_mission.EXPECTED_MARKER_IDS = saved

    def test_all_expected_found_false_without_expected_set(self):
        saved = stage2_mission.EXPECTED_MARKER_IDS
        stage2_mission.EXPECTED_MARKER_IDS = set()
        try:
            self.assertFalse(stage2_mission._all_expected_found([self._mission(DroneAPI())]))
        finally:
            stage2_mission.EXPECTED_MARKER_IDS = saved


if __name__ == "__main__":
    unittest.main()
