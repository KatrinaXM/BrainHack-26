"""
barrel_tracker.py — NED-space deduplication of YOLO barrel detections.

Each detection is projected from (bbox_centre, depth, drone_pose) into NED
world coordinates. Detections within DEDUP_RADIUS of an already-known barrel
of the same class are merged (highest-confidence wins).

The tracker is designed to be called from the Detector callback, which runs
on a worker thread. All public methods are thread-safe.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass, asdict, field
from typing import Dict, List, Optional, Tuple

import numpy as np

import mission_config as cfg


@dataclass
class Barrel:
    class_name: str
    north: float
    east: float
    down: float
    confidence: float
    first_seen_t: float
    last_seen_t: float
    hits: int = 1
    bboxes: List[List[float]] = field(default_factory=list)
    image_paths: List[str] = field(default_factory=list)

    def update(self, north: float, east: float, down: float,
               confidence: float, bbox: List[float], image_path: Optional[str], t: float) -> None:
        # Running mean of position, weighted by confidence.
        w = max(confidence, 1e-3)
        total_w = self.confidence * self.hits + w
        self.north = (self.north * self.confidence * self.hits + north * w) / total_w
        self.east  = (self.east  * self.confidence * self.hits + east  * w) / total_w
        self.down  = (self.down  * self.confidence * self.hits + down  * w) / total_w
        self.confidence = max(self.confidence, confidence)
        self.last_seen_t = t
        self.hits += 1
        self.bboxes.append([float(x) for x in bbox])
        if image_path:
            self.image_paths.append(image_path)


class BarrelTracker:
    """Thread-safe NED-deduped barrel registry."""

    def __init__(self,
                 K: np.ndarray = cfg.K_CAM,
                 dedup_radius: float = cfg.DEDUP_RADIUS,
                 image_w: int = cfg.IMAGE_WIDTH,
                 image_h: int = cfg.IMAGE_HEIGHT,
                 depth_patch: int = cfg.DEPTH_PATCH,
                 det_min_range: float = cfg.DET_MIN_RANGE,
                 det_max_range: float = cfg.DET_MAX_RANGE,
                 output_path: str = cfg.BARRELS_JSON):
        self.fx = float(K[0, 0])
        self.fy = float(K[1, 1])
        self.cx = float(K[0, 2])
        self.cy = float(K[1, 2])
        self.image_w = image_w
        self.image_h = image_h
        self.depth_patch = depth_patch
        self.dedup_radius = dedup_radius
        self.det_min_range = det_min_range
        self.det_max_range = det_max_range
        self.output_path = output_path

        self.barrels: List[Barrel] = []
        self._lock = threading.Lock()

    # -----------------------------------------------------------------
    # Pixel + depth + pose → NED world coordinates
    # -----------------------------------------------------------------
    def _sample_depth(self, depth_map: np.ndarray, u: float, v: float) -> Optional[float]:
        h, w = depth_map.shape
        # Scale pixel coordinates if RGB and depth resolutions differ.
        u_d = int(round(u * w / self.image_w))
        v_d = int(round(v * h / self.image_h))
        u_d = max(0, min(w - 1, u_d))
        v_d = max(0, min(h - 1, v_d))

        half = self.depth_patch // 2
        u_lo, u_hi = max(0, u_d - half), min(w, u_d + half + 1)
        v_lo, v_hi = max(0, v_d - half), min(h, v_d + half + 1)
        patch = depth_map[v_lo:v_hi, u_lo:u_hi]

        valid = patch[np.isfinite(patch) & (patch > self.det_min_range) & (patch < self.det_max_range)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _project_to_ned(self,
                        u: float, v: float, depth_m: float,
                        pose: Dict[str, float]) -> Tuple[float, float, float]:
        """
        Convert pixel (u, v) + depth (m) into NED world coordinates given the
        drone pose. Assumes camera is forward-facing and rigidly mounted to the
        body, optical Z axis aligned with body X (forward) when yaw=0.

        Camera optical frame:  X_cam=right, Y_cam=down, Z_cam=forward (=depth).
        Body frame (FRD):      X_body=forward, Y_body=right, Z_body=down.
        World (NED):           X=north, Y=east, Z=down.
        """
        z_cam = depth_m
        x_cam = (u - self.cx) * z_cam / self.fx
        y_cam = (v - self.cy) * z_cam / self.fy

        # camera-frame -> body-frame.
        x_body = z_cam          # forward
        y_body = x_cam          # right
        z_body = y_cam          # down

        # body-frame -> NED via yaw rotation.
        yaw = pose["yaw"]
        cy_r, sy_r = math.cos(yaw), math.sin(yaw)
        north_off = x_body * cy_r - y_body * sy_r
        east_off  = x_body * sy_r + y_body * cy_r
        down_off  = z_body

        return (pose["north"] + north_off,
                pose["east"]  + east_off,
                pose["down"]  + down_off)

    # -----------------------------------------------------------------
    # Public API
    # -----------------------------------------------------------------
    def consume(self,
                detections: List[Dict],
                depth_map: Optional[np.ndarray],
                pose: Dict[str, float],
                image_path: Optional[str]) -> List[Barrel]:
        """
        Called once per RGB frame with the YOLO detection list and a depth
        snapshot. Returns the list of barrel registry entries that were
        added or updated.
        """
        if depth_map is None or pose is None or not detections:
            return []

        touched: List[Barrel] = []
        t = time.time()

        for d in detections:
            cls = d.get("class_name", "")
            if cls not in (cfg.CLASS_YELLOW, cfg.CLASS_RED):
                continue
            x1, y1, x2, y2 = d["bbox"]
            # Skip bboxes that touch the image edge — depth at the boundary
            # is usually unreliable and the projection can spike to wrong NEDs.
            if x1 < 2 or y1 < 2 or x2 > self.image_w - 2 or y2 > self.image_h - 2:
                continue
            # Skip tiny boxes (likely false positives in noise).
            if (x2 - x1) < 6 or (y2 - y1) < 6:
                continue

            u = 0.5 * (x1 + x2)
            v = 0.5 * (y1 + y2)

            depth_m = self._sample_depth(depth_map, u, v)
            if depth_m is None:
                continue

            n, e, dwn = self._project_to_ned(u, v, depth_m, pose)
            conf = float(d.get("confidence", 0.0))

            with self._lock:
                match = self._find_match(cls, n, e)
                if match is None:
                    barrel = Barrel(
                        class_name=cls,
                        north=n, east=e, down=dwn,
                        confidence=conf,
                        first_seen_t=t, last_seen_t=t,
                        bboxes=[[float(x) for x in d["bbox"]]],
                        image_paths=[image_path] if image_path else [],
                    )
                    self.barrels.append(barrel)
                    touched.append(barrel)
                else:
                    match.update(n, e, dwn, conf, d["bbox"], image_path, t)
                    touched.append(match)

        if touched:
            self._persist_locked_outside()
        return touched

    def _find_match(self, cls: str, n: float, e: float) -> Optional[Barrel]:
        best: Optional[Barrel] = None
        best_d2 = self.dedup_radius * self.dedup_radius
        for b in self.barrels:
            if b.class_name != cls:
                continue
            dn = b.north - n
            de = b.east - e
            d2 = dn * dn + de * de
            if d2 <= best_d2:
                best_d2 = d2
                best = b
        return best

    def _persist_locked_outside(self) -> None:
        # Snapshot inside the lock, write outside it.
        with self._lock:
            payload = self._snapshot()
        try:
            with open(self.output_path, "w") as f:
                json.dump(payload, f, indent=2)
        except OSError as exc:
            print(f"[BarrelTracker] write {self.output_path} failed: {exc}")

    def _snapshot(self) -> Dict:
        by_class_raw: Dict[str, int] = {}
        by_class_val: Dict[str, int] = {}
        raw: List[Dict] = []
        validated: List[Dict] = []
        for b in self.barrels:
            by_class_raw[b.class_name] = by_class_raw.get(b.class_name, 0) + 1
            entry = asdict(b)
            raw.append(entry)
            is_validated = (b.hits >= cfg.VALIDATED_MIN_HITS
                            or b.confidence >= cfg.VALIDATED_MIN_CONF)
            entry["validated"] = is_validated
            if is_validated:
                by_class_val[b.class_name] = by_class_val.get(b.class_name, 0) + 1
                validated.append(entry)
        return {
            "schema_version": 2,
            "generated_at": time.time(),
            "raw_total": len(self.barrels),
            "raw_by_class": by_class_raw,
            "validated_total": len(validated),
            "validated_by_class": by_class_val,
            "barrels": raw,
            "validated": validated,
        }

    def snapshot(self) -> Dict:
        with self._lock:
            return self._snapshot()

    def __len__(self) -> int:
        with self._lock:
            return len(self.barrels)
