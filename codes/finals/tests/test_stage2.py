#!/usr/bin/env python3
"""
test_stage2.py — unit tests for the deterministic parts of stage2_mission.py.

Covers:
- Pad loading + validity filtering + selection
- HSV RoboMaster detector on synthetic frames (positive + negative cases)
- save_snapshot writes both JPEG and JSON sidecar
- navigate_to_pad emits the right Direction sequence (using mock DroneAPI)
- Mock DroneAPI state-machine rejects illegal transitions

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

import numpy as np      # noqa: E402

import stage2_mission   # noqa: E402
from stage2_mission import (   # noqa: E402
    Pad, load_pads, select_pads,
    detect_robomaster, save_snapshot, navigate_to_pad,
    DroneMission, DroneState, OUTPUT_DIR,
)
from mocks.pyhulax_mock import DroneAPI, Direction      # noqa: E402


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
#  RoboMaster detector
# ============================================================================

class DetectorTests(unittest.TestCase):

    @staticmethod
    def _blank(h=480, w=640) -> np.ndarray:
        return np.full((h, w, 3), 80, dtype=np.uint8)

    def test_no_red_returns_false(self):
        frame = self._blank()
        detected, bboxes = detect_robomaster(frame)
        self.assertFalse(detected)
        self.assertEqual(bboxes, [])

    def test_red_patch_detected(self):
        frame = self._blank()
        # Paint a strong-red rectangle.
        frame[200:280, 300:400] = (200, 30, 30)
        detected, bboxes = detect_robomaster(frame)
        self.assertTrue(detected)
        self.assertEqual(len(bboxes), 1)
        b = bboxes[0]
        # bbox should roughly enclose the painted region
        self.assertAlmostEqual(b["x"], 300, delta=5)
        self.assertAlmostEqual(b["y"], 200, delta=5)
        self.assertAlmostEqual(b["w"], 100, delta=5)
        self.assertAlmostEqual(b["h"],  80, delta=5)

    def test_tiny_speckle_rejected_by_area_filter(self):
        frame = self._blank()
        # Single 3x3 red speckle — well under the default min-area threshold.
        frame[100:103, 100:103] = (200, 30, 30)
        detected, bboxes = detect_robomaster(frame)
        self.assertFalse(detected)

    def test_extreme_aspect_ratio_rejected(self):
        frame = self._blank()
        # A 200x5 strip — area passes (1000 >= 400), aspect 40 >> max 3.
        frame[200:205, 200:400] = (200, 30, 30)
        detected, bboxes = detect_robomaster(frame)
        self.assertFalse(detected)


# ============================================================================
#  Snapshot writer
# ============================================================================

class SnapshotTests(unittest.TestCase):

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()
        # Re-bind the module's OUTPUT_DIR to our tempdir for the test.
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

    def test_save_writes_jpeg_and_json(self):
        frame = np.full((480, 640, 3), 80, dtype=np.uint8)
        frame[100:200, 100:200] = (200, 30, 30)
        m = self._mission()
        save_snapshot(frame, m, [{"x": 100, "y": 100, "w": 100, "h": 100,
                                  "area": 10000.0, "aspect": 1.0}])
        files = sorted(Path(self.tmp_dir.name).iterdir())
        suffixes = sorted(f.suffix for f in files)
        self.assertEqual(suffixes, [".jpg", ".json"])
        # JPEG is non-empty.
        jpg = next(f for f in files if f.suffix == ".jpg")
        self.assertGreater(jpg.stat().st_size, 1000)
        # JSON has the expected metadata.
        sidecar = next(f for f in files if f.suffix == ".json")
        meta = json.loads(sidecar.read_text())
        self.assertEqual(meta["plane_id"], "ptest")
        self.assertEqual(meta["pad_id"], "Pz")
        self.assertEqual(meta["snapshot_num"], 1)
        self.assertEqual(len(meta["bboxes"]), 1)

    def test_save_throttled_by_cooldown(self):
        frame = np.full((480, 640, 3), 80, dtype=np.uint8)
        m = self._mission()
        # First save fires, immediate second save should be no-op.
        save_snapshot(frame, m, [{"x": 0, "y": 0, "w": 10, "h": 10,
                                  "area": 100.0, "aspect": 1.0}])
        save_snapshot(frame, m, [{"x": 0, "y": 0, "w": 10, "h": 10,
                                  "area": 100.0, "aspect": 1.0}])
        # Should be exactly 2 files: 1 jpg + 1 json from the first save.
        files = list(Path(self.tmp_dir.name).iterdir())
        self.assertEqual(len(files), 2)
        self.assertEqual(m.snapshots_saved, 1)


# ============================================================================
#  navigate_to_pad (via mock)
# ============================================================================

class _RecordingDroneAPI(DroneAPI):
    """Mock DroneAPI subclass that records every .move() call for assertions."""

    def __init__(self):
        super().__init__()
        self.moves: list[tuple[str, float]] = []

    def move(self, direction: Direction, distance: float) -> None:
        self.moves.append((direction.name, float(distance)))
        super().move(direction, distance)


class NavigateTests(unittest.TestCase):

    def _airborne_drone(self) -> _RecordingDroneAPI:
        d = _RecordingDroneAPI()
        d.connect("1.2.3.4")
        d.takeoff()
        return d

    def test_navigate_emits_down_forward_right_for_positive_pad(self):
        d = self._airborne_drone()
        pad = Pad("P", 3.0, 2.0, 0.0, True)  # +x, +y, descend from 1m
        navigate_to_pad(d, pad)
        names = [n for n, _ in d.moves]
        self.assertEqual(names, ["DOWN", "FORWARD", "RIGHT"])
        # distances: 1 m down, 3 m forward, 2 m right (default axis convention).
        self.assertAlmostEqual(d.moves[0][1], 1.0, places=2)
        self.assertAlmostEqual(d.moves[1][1], 3.0, places=2)
        self.assertAlmostEqual(d.moves[2][1], 2.0, places=2)

    def test_navigate_emits_back_left_for_negative_pad(self):
        d = self._airborne_drone()
        pad = Pad("P", -3.0, -2.0, 1.0, True)  # no vertical, both negative horiz
        navigate_to_pad(d, pad)
        names = [n for n, _ in d.moves]
        self.assertEqual(names, ["BACK", "LEFT"])

    def test_navigate_skips_zero_axes(self):
        d = self._airborne_drone()
        pad = Pad("P", 0.0, 2.0, 1.0, True)  # only +y
        navigate_to_pad(d, pad)
        names = [n for n, _ in d.moves]
        self.assertEqual(names, ["RIGHT"])

    def test_navigate_skips_sub_epsilon_moves(self):
        d = self._airborne_drone()
        # All deltas below default epsilon of 0.05 m.
        pad = Pad("P", 0.01, 0.02, 1.0, True)
        navigate_to_pad(d, pad)
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


if __name__ == "__main__":
    unittest.main()
