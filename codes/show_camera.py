#!/usr/bin/env python3
"""
show_camera.py — standalone OpenCV viewer for the drone's RGB camera.

Use this in a SECOND terminal alongside run_competition.sh if you want to
see the raw (unannotated) camera feed. The mission's own "YOLO Detections"
window shows the *annotated* feed with bounding boxes — that's the primary
evidence display. This script is the same feed without YOLO overlay, in
case the judge wants to verify what the model is seeing.

Run with:
  PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 show_camera.py
"""

import sys
import time

import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as GzImage

import mission_config as cfg


_frame = None


def callback(msg: GzImage):
    global _frame
    try:
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        buf = buf.reshape((msg.height, msg.width, 3))
        _frame = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
    except Exception as exc:
        print(f"[show_camera] decode error: {exc}")


def main():
    node = Node()
    ok = node.subscribe(GzImage, cfg.RGB_TOPIC, callback)
    if not ok:
        print(f"Failed to subscribe to {cfg.RGB_TOPIC}. Is Gazebo running?")
        return 1
    print(f"[show_camera] subscribed: {cfg.RGB_TOPIC}")
    print("[show_camera] press Q in the window to quit.")

    cv2.namedWindow("Drone RGB", cv2.WINDOW_AUTOSIZE)
    while True:
        if _frame is not None:
            cv2.imshow("Drone RGB", _frame)
        if cv2.waitKey(50) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
