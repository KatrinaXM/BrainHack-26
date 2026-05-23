#!/bin/bash
# set_ekf_origin.sh — send the EKF origin via MAVLink (bypasses pxh> screen).
exec python3 /home/drone/set_ekf_origin.py "$@"
