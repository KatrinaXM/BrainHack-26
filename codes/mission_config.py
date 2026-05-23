"""
mission_config.py — central configuration for the BrainHack-26 RoboVerse Qualifier mission.

Edit these values to tune the mission. mission.py reads everything from here.
"""

import os
import numpy as np

# -------------------------------------------------------------------
# Simulator / topics
# -------------------------------------------------------------------
# Organiser confirmed on Discord (May 2026): use x500_vision. The depth
# drone has GPS that cannot be disabled from code, and the Qualifier
# requires no-GNSS. x500_vision presents as no-GPS in SITL (handled by the
# hand-modified 4005_gz_x500_vision airframe on this VM).
DRONE_MODEL = "x500_vision"

# Pre-flight: type at PX4 pxh> prompt (or via QGC "Set Estimator Origin") —
# without this, the vision drone never becomes armable.
EKF_ORIGIN_CMD = "commander set_ekf_origin 47.397742 8.545594 488.0"

# Depth-camera topic. The OakD-Lite include publishes float32 depth here for
# both x500_depth and x500_vision in roboverse.
DEPTH_TOPIC = "/depth_camera"

# RGB-camera topic. The OakD-Lite IMX214 sensor publishes here. The MODEL
# segment is taken from DRONE_MODEL above.
RGB_TOPIC = f"/world/roboverse/model/{DRONE_MODEL}_0/link/camera_link/sensor/IMX214/image"

# MAVSDK connection (PX4 SITL).
MAVSDK_SYSTEM_ADDRESS = "udpin://0.0.0.0:14540"


# -------------------------------------------------------------------
# Camera intrinsics (confirmed by Supp1 p.15 example; matches OakD-Lite SDF
# at 640x480, ~74 deg HFOV).
# -------------------------------------------------------------------
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

K_CAM = np.array([
    [433.0,   0.0, 320.0],
    [  0.0, 433.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float32)

# Depth value sanity range. Values outside [DEPTH_MIN, DEPTH_MAX] are treated
# as invalid by the avoidance planner.
DEPTH_MIN = 0.2
DEPTH_MAX = 15.0


# -------------------------------------------------------------------
# Avoidance / planner
# -------------------------------------------------------------------
SAFE_DISTANCE = 2.5       # m — above this fly at full speed
CRITICAL_DISTANCE = 0.9   # m — below this stop hard
STEP_SIZE = 1.5           # m — legacy; only used by the old waypoint code
NUM_BINS = 36             # bins for the polar histogram
SMOOTHING_ALPHA = 0.6     # planner velocity smoothing
LOOP_HZ = 15.0            # mission loop rate

# Wall-following: when forward is blocked, turn this many degrees CW and
# try again. Positive = right turn (clockwise). Negative = left.
FIXED_TURN_DEG = 90.0

# After this many consecutive blocks without progress, fall back to full
# 360° scan instead of wall-following. This handles dead-ends.
WALL_FOLLOW_BLOCKS_BEFORE_SCAN = 3

# Visited-cell grid size — must be larger than typical scan / step noise so
# we don't mark every micro-position as a new cell.
VISITED_CELL_SIZE = 3.0

# Spin-at-spots strategy.
SPIN_DURATION_S = 15.0       # seconds per spin (one full 360° + margin)
SPIN_YAW_RATE = 28.0         # deg/s — 360° in ~13 s
TRANSIT_DISTANCE_M = 12.0    # how far to move between spots
TRANSIT_TIMEOUT_S = 15.0     # hard cap on transit time

# Rotate-scan parameters. Used initially and on every block / periodic resync.
SCAN_STEPS = 8                  # 8 × 45° = full 360° sweep
SCAN_DWELL_S = 0.4              # dwell per heading — long enough for YOLO

# Forward-step parameters (the only translation the drone makes).
FORWARD_SPEED = 0.8             # m/s body-frame forward velocity
FORWARD_STEP_M = 6.0            # max distance per forward burst (EKF-measured)
FORWARD_STEP_TIMEOUT_S = 12.0   # hard cap on each forward burst

# Periodic re-scans cause the drone to second-guess its direction and zig-zag
# back toward spawn. Set very high to effectively disable — only re-scan
# when actually blocked.
PERIODIC_SCAN_INTERVAL_S = 1e9

# Vertical control.
MAX_VERTICAL_SPEED = 0.4        # m/s in body-down direction; soft cap

# Safety bounds — abort if EKF goes nuts.
MAX_DRIFT_FROM_ORIGIN = 35.0    # m — beyond this is outside the 40×40 world
MAX_ALTITUDE_DOWN = -8.0        # NED down; deeper (more negative) means higher
MIN_ALTITUDE_DOWN = 1.0         # NED down; positive means below ground


# -------------------------------------------------------------------
# Exploration — lawnmower with two-altitude pass
# -------------------------------------------------------------------
# World is 40 x 40 with 4 m grid. Take-off is at the origin (0, 0).
# These bounds describe the maximum extent we'll attempt — the AvoidancePlanner
# will keep us out of walls regardless.
WORLD_NORTH_MIN = -20.0
WORLD_NORTH_MAX =  20.0
WORLD_EAST_MIN  = -20.0
WORLD_EAST_MAX  =  20.0

SWEEP_SPACING = 4.0       # m between lanes (matches world grid)
SWEEP_HEADING_DEG = 0.0   # primary lane orientation: north-south sweeps

# Two altitudes. Yellow barrels live on the ground; red barrels sit on top
# of crate stacks ~2-3 m up (qualifier cover image).
ALTITUDE_LOW_DOWN  = -1.2   # NED down; -1.2 = 1.2 m above ground
ALTITUDE_HIGH_DOWN = -3.0   # NED down; -3.0 = 3.0 m above ground
TAKEOFF_ALTITUDE   = 1.5    # action.set_takeoff_altitude (meters above ground)

# How long to spend on each altitude pass before forcing a switch.
PHASE_LOW_SECONDS  = 150
PHASE_HIGH_SECONDS = 110

# When the planner declares the path is blocked toward a waypoint, we'll
# skip-ahead this many waypoints rather than retrying indefinitely.
WAYPOINT_SKIP_ON_BLOCK = 1

# When we are within this many meters of the active waypoint, advance to the
# next one.
WAYPOINT_REACHED_RADIUS = 1.5


# -------------------------------------------------------------------
# YOLO
# -------------------------------------------------------------------
YOLO_MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "barrel_yolo.pt")
DETECTION_CONF = 0.45         # confidence threshold (false-positives unpenalised)
DETECTOR_WORKERS = 2          # parallel YOLO worker threads
DETECTOR_DEVICE = "cpu"       # set "cuda" if a GPU is available
# True opens a live OpenCV "AI View" window with every annotated detection.
# For the live competition demo, judges expect to see bounding boxes during
# the run — turn this on (or set BH26_DISPLAY=1 in the env at run time). The
# display thread drops frames if YOLO/mission are busy, so the CPU cost
# stays bounded.
DETECTOR_DISPLAY = bool(int(os.environ.get("BH26_DISPLAY", "1")))

# Classes — must match the trained model exactly (HANDOVER spec).
CLASS_YELLOW = "fuel_barrel_yellow"
CLASS_RED    = "fuel_barrel_red"


# -------------------------------------------------------------------
# Barrel deduplication
# -------------------------------------------------------------------
# Two NED detections within this radius (m) are treated as the same barrel.
# Set ≈ 1 grid cell — barrels live in distinct grid cells (≥ 4 m apart) so
# this conservatively merges drift-noise while keeping real barrels apart.
DEDUP_RADIUS = 4.0

# When finalising the output, a barrel counts as "validated" only if it has
# at least this many hits OR at least this peak confidence. Single low-conf
# detections are reported in `raw` but not in `validated`.
VALIDATED_MIN_HITS = 2
VALIDATED_MIN_CONF = 0.75

# Minimum bbox depth sample window — we sample depth at the bbox centre and
# average over a small patch to suppress noise.
DEPTH_PATCH = 5   # 5x5 px

# Ignore detections whose centre depth falls outside this range — they're
# almost certainly noise or out-of-spec.
DET_MIN_RANGE = 0.5
DET_MAX_RANGE = 12.0


# -------------------------------------------------------------------
# Timing / safety
# -------------------------------------------------------------------
# Hard bailout. The qualifier gives 10 minutes total but the time bonus only
# counts detections under 5 min — so we want to RTL by ~4:40 to bank a
# bonus while leaving headroom for landing.
BAILOUT_SECONDS = 280

# After the mission ends, we issue RTL + Land. Allow this much time before
# the python process exits.
SHUTDOWN_GRACE_SECONDS = 30


# -------------------------------------------------------------------
# Output paths
# -------------------------------------------------------------------
OUTPUT_BASE = "/mnt/hgfs/Shared"
if not os.path.isdir(OUTPUT_BASE):
    OUTPUT_BASE = os.path.expanduser("~/mission_output")
    os.makedirs(OUTPUT_BASE, exist_ok=True)

BARRELS_JSON = os.path.join(OUTPUT_BASE, "barrels.json")
DETECTIONS_DIR = os.path.join(OUTPUT_BASE, "yolo_detections")
MISSION_LOG = os.path.join(OUTPUT_BASE, "mission_log.txt")
