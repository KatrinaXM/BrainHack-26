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

_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)


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
        pad = Pad("P", 3.0, 2.0, 0.0, True)  # +x, +y, descend from 1.1m
        navigate_to_pad(d, pad)
        names = [n for n, _ in d.moves]
        self.assertEqual(names, ["DOWN", "FORWARD", "RIGHT"])
        # distances: 1.1 m down (from TAKEOFF_ALT_M), 3 m forward, 2 m right.
        self.assertAlmostEqual(d.moves[0][1], 1.1, places=2)
        self.assertAlmostEqual(d.moves[1][1], 3.0, places=2)
        self.assertAlmostEqual(d.moves[2][1], 2.0, places=2)

    def test_navigate_emits_back_left_for_negative_pad(self):
        d = self._airborne_drone()
        # Pad at takeoff altitude → no vertical move needed.
        pad = Pad("P", -3.0, -2.0, stage2_mission.TAKEOFF_ALT_M, True)
        navigate_to_pad(d, pad)
        names = [n for n, _ in d.moves]
        self.assertEqual(names, ["BACK", "LEFT"])

    def test_navigate_skips_zero_axes(self):
        d = self._airborne_drone()
        # Only +y, no vertical change.
        pad = Pad("P", 0.0, 2.0, stage2_mission.TAKEOFF_ALT_M, True)
        navigate_to_pad(d, pad)
        names = [n for n, _ in d.moves]
        self.assertEqual(names, ["RIGHT"])

    def test_navigate_skips_sub_epsilon_moves(self):
        d = self._airborne_drone()
        pad = Pad("P", 0.01, 0.02, stage2_mission.TAKEOFF_ALT_M, True)
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

    def test_retakeoff_after_land_is_allowed(self):
        # Stage 2 mission requires this: land on pad, then take off again
        # for ArUco search. Mock must allow re-takeoff after land.
        d = DroneAPI()
        d.connect("1.2.3.4")
        d.takeoff()
        d.land()
        d.takeoff()      # must succeed
        d.land()


if __name__ == "__main__":
    unittest.main()
