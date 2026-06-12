import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge
from collections import deque
from ultralytics import YOLO
import cv2
import numpy as np
import time
import json
import os
import struct
import threading
import can

# ─────────────────────────────────────────────────────────────────────────────
# WORKSPACE CONFIG (30cm x 30cm)
# ─────────────────────────────────────────────────────────────────────────────
MARKER_POSITIONS = {
    1: np.array([-0.15,  0.30]),
    2: np.array([ 0.15,  0.30]),
    3: np.array([ 0.15,  0.00]),
    4: np.array([-0.15,  0.00]),
}

DEFAULT_OFFSET_X = 0.00005
DEFAULT_OFFSET_Y = -0.001

WORKSPACE_SAVE_PATH  = os.path.expanduser(
    '~/SCARA/src/SCARA_pkg/SCARA_pkg/scara_workspace.json')
CORRECTION_SAVE_PATH = os.path.expanduser(
    '~/SCARA/src/SCARA_pkg/SCARA_pkg/scara_correction.json')
# ── NEW: where the 3 place positions are stored ──
PLACE_SAVE_PATH = os.path.expanduser(
    '~/SCARA/src/SCARA_pkg/SCARA_pkg/scara_place_positions.json')

# ─────────────────────────────────────────────────────────────────────────────
# DETECTION TARGETS  (NEW MODEL: best2.pt → classes are green / red / yellow)
# The robot will pick up ANY object whose class is in this set.
# To pick only ONE color, reduce this set, e.g. TARGET_CLASSES = {"red"}.
# ─────────────────────────────────────────────────────────────────────────────
TARGET_CLASSES = {"green", "red", "yellow"}

# Box colors (BGR) used when drawing each detected class on screen.
CLASS_DRAW_COLORS = {
    "green":  (0, 255,   0),
    "red":    (0,   0, 255),
    "yellow": (0, 255, 255),
}

# ── NEW: the order in which the user clicks the 3 place positions ──
# Click 1 → green bin, click 2 → yellow bin, click 3 → red bin.
PLACE_ORDER = ["green", "yellow", "red"]

JOG_STEP        = 0.001       # 1 mm per WASD press
STEPPER_JOG_STEP = 0.001      # 1 mm per U/I press
NUM_CALIB_POINTS = 4

# Park (shutdown) position
PARK_X       = 0.0
PARK_Y       = 0.30
PARK_TRAVEL_S = 2.5           # time to wait for arms to reach park
PARK_STEPPER_S = 1.5          # time to wait for stepper to retract up

# ─────────────────────────────────────────────────────────────────────────────
# STEPPER (MKS SERVO42D/57D) CONFIG
# ─────────────────────────────────────────────────────────────────────────────
CAN_INTERFACE  = "can0"
MKS_NODE_ID    = 3
COUNTS_PER_REV = 16384
MOTOR_REVS     = 6.0
STROKE_M       = 0.035
DIRECTION_SIGN = 1
COUNTS_PER_M   = MOTOR_REVS * COUNTS_PER_REV / STROKE_M

SPEED_RPM      = 300
ACCELERATION   = 2

POS_UP_M       = 0.000        # highest (home / safe travel)
POS_DOWN_M     = 0.035        # lowest (35 mm down to grab/place)

INT24_MIN, INT24_MAX = -8388607, 8388607

PICK_WAIT_S    = 1.0
# Stepper full-stroke (35mm @ 300rpm) takes ~1.2s travel + accel/decel ramp,
# so ~1.6s real. Use 2.0s so the stepper is ALWAYS fully arrived before the
# next action (suction ON, or moving the arm to the place position).
STEPPER_TRAVEL_S = 4
XY_TRAVEL_S    = 2.0

# ─────────────────────────────────────────────────────────────────────────────
# OUTPUT2CAN SOLENOID BOARD CONFIG  (vacuum suction gripper)
# These are the values that fired the solenoid in the keyboard test:
#   CAN id = 4, channel D1, bus running at 1 Mbit/s.
# Frame sent is exactly what can_driver.py builds: byte0 = header (0x40),
# byte1 = digital outputs (0), byte2 = solenoid bits. NO MKS CRC byte.
# ─────────────────────────────────────────────────────────────────────────────
SOLENOID_CAN_ID  = 4      # board CAN id (value that worked in your test)
SOLENOID_CHANNEL = 1      # which output: 1->D1, 2->D2, 3->D3, 4->D4
SOLENOID_HEADER  = 0x40   # command header the Output2CAN firmware expects

# ─────────────────────────────────────────────────────────────────────────────
# STATES
# ─────────────────────────────────────────────────────────────────────────────
STATE_CALIBRATING        = 0
STATE_HOMING             = 1
STATE_IDLE               = 2
STATE_ASK_RELOAD         = 4
STATE_ASK_OFFSET_CALIB   = 5
STATE_OFFSET_DETECT      = 6
STATE_OFFSET_JOG         = 7

# ── NEW: place-position selection states ──
STATE_ASK_PLACE_RELOAD   = 8   # saved place positions found → load or re-pick?
STATE_SELECT_PLACE       = 9   # user clicks 3 bins (green, yellow, red)
STATE_SELECT_PLACE_JOG   = 23  # after a click: arm moves there, WASD/UI fine-tune

# Pick-and-place sub-states
STATE_PNP_GO_TO_CAP      = 10
STATE_PNP_DOWN_AT_CAP    = 11
STATE_PNP_WAIT_AT_CAP    = 12
STATE_PNP_UP_AT_CAP      = 13
STATE_PNP_GO_HOME        = 14
STATE_PNP_DOWN_AT_HOME   = 15
STATE_PNP_UP_AT_HOME     = 16

# Park-and-exit sub-states
STATE_PARK_RETRACT       = 20  # stepper going up
STATE_PARK_MOVE          = 21  # XY moving to (0, 0.30)
STATE_PARK_DONE          = 22  # request rclpy.shutdown()

MODE_YOLO   = 0
MODE_MANUAL = 1


# ─────────────────────────────────────────────────────────────────────────────
# CAN / MKS HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def crc8_sum(can_id: int, payload: bytes) -> int:
    return (can_id + sum(payload)) & 0xFF


def encode_int24_be(v: int) -> bytes:
    if v < 0:
        v += 1 << 24
    return bytes([(v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


def build_abs_frame(ee_meters: float):
    counts = int(round(DIRECTION_SIGN * ee_meters * COUNTS_PER_M))
    counts = max(INT24_MIN, min(INT24_MAX, counts))
    speed_be = struct.pack('>H', max(0, min(3000, SPEED_RPM)))
    payload = bytes([0xF5]) + speed_be + bytes([ACCELERATION]) + encode_int24_be(counts)
    return payload, counts


def build_set_zero_frame() -> bytes:
    return bytes([0x92])


def build_estop_frame() -> bytes:
    return bytes([0xF7])


def build_solenoid_frame(on: bool) -> bytes:
    """Output2CAN digital+solenoid data payload.
       byte0 = header, byte1 = digital outputs (unused -> 0),
       byte2 = solenoid bits. Matches can_driver.py exactly."""
    sol_bits = (1 << (SOLENOID_CHANNEL - 1)) if on else 0
    return bytes([SOLENOID_HEADER, 0x00, sol_bits])


# ─────────────────────────────────────────────────────────────────────────────
# PERSISTENCE
# ─────────────────────────────────────────────────────────────────────────────
def save_workspace(points):
    with open(WORKSPACE_SAVE_PATH, 'w') as f:
        json.dump({'points': points}, f)
    print(f"[INFO] Workspace saved to {WORKSPACE_SAVE_PATH}")


def load_workspace():
    if not os.path.exists(WORKSPACE_SAVE_PATH):
        return None
    try:
        with open(WORKSPACE_SAVE_PATH, 'r') as f:
            data = json.load(f)
        points = data.get('points', None)
        if points and len(points) == 4:
            print(f"[INFO] Loaded saved workspace from {WORKSPACE_SAVE_PATH}")
            return points
    except Exception as e:
        print(f"[WARN] Failed to load workspace: {e}")
    return None


def save_correction(matrix_2x3, raw_pts, actual_pts):
    data = {
        'matrix': matrix_2x3.tolist(),
        'raw_points':    [list(p) for p in raw_pts],
        'actual_points': [list(p) for p in actual_pts],
    }
    with open(CORRECTION_SAVE_PATH, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"[INFO] Correction matrix saved to {CORRECTION_SAVE_PATH}")
    print(f"[INFO] Matrix:\n{matrix_2x3}")


def load_correction():
    if not os.path.exists(CORRECTION_SAVE_PATH):
        return None
    try:
        with open(CORRECTION_SAVE_PATH, 'r') as f:
            data = json.load(f)
        M = np.array(data['matrix'], dtype=np.float64)
        if M.shape == (2, 3):
            print(f"[INFO] Loaded correction matrix from {CORRECTION_SAVE_PATH}")
            return M
    except Exception as e:
        print(f"[WARN] Failed to load correction: {e}")
    return None


# ── NEW: save / load the 3 place positions ──
def save_place_positions(place_map, place_px):
    """place_map : {'green': (rx, ry), 'yellow': (...), 'red': (...)}
       place_px  : {'green': (cx, cy), ...}  pixel coords for redrawing."""
    data = {
        'robot': {k: list(v) for k, v in place_map.items()},
        'pixel': {k: list(v) for k, v in place_px.items()},
    }
    with open(PLACE_SAVE_PATH, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"[INFO] Place positions saved to {PLACE_SAVE_PATH}")
    for k in PLACE_ORDER:
        print(f"        {k:>6} → robot {place_map[k]}  pixel {place_px[k]}")


def load_place_positions():
    """Return (place_map, place_px) or (None, None) if not found/invalid."""
    if not os.path.exists(PLACE_SAVE_PATH):
        return None, None
    try:
        with open(PLACE_SAVE_PATH, 'r') as f:
            data = json.load(f)
        robot = data.get('robot', {})
        pixel = data.get('pixel', {})
        # make sure every required color is present
        if all(c in robot for c in PLACE_ORDER):
            place_map = {c: (float(robot[c][0]), float(robot[c][1]))
                         for c in PLACE_ORDER}
            place_px = {c: (int(pixel[c][0]), int(pixel[c][1]))
                        for c in PLACE_ORDER if c in pixel}
            print(f"[INFO] Loaded place positions from {PLACE_SAVE_PATH}")
            return place_map, place_px
    except Exception as e:
        print(f"[WARN] Failed to load place positions: {e}")
    return None, None


def compute_correction_matrix(raw_pts, actual_pts):
    """Fit 2D similarity transform: actual = M * [raw_x, raw_y, 1]^T"""
    if len(raw_pts) < 2:
        dx = actual_pts[0][0] - raw_pts[0][0]
        dy = actual_pts[0][1] - raw_pts[0][1]
        return np.array([[1.0, 0.0, dx],
                         [0.0, 1.0, dy]], dtype=np.float64)

    if len(raw_pts) == 2:
        p1, p2 = np.array(raw_pts,    dtype=np.float64)
        q1, q2 = np.array(actual_pts, dtype=np.float64)
        dp = p2 - p1
        dq = q2 - q1
        denom = dp[0]*dp[0] + dp[1]*dp[1]
        if denom < 1e-12:
            dx = q1[0] - p1[0]; dy = q1[1] - p1[1]
            return np.array([[1.0, 0.0, dx],
                             [0.0, 1.0, dy]], dtype=np.float64)
        a = (dp[0]*dq[0] + dp[1]*dq[1]) / denom
        b = (dp[0]*dq[1] - dp[1]*dq[0]) / denom
        tx = q1[0] - (a*p1[0] - b*p1[1])
        ty = q1[1] - (b*p1[0] + a*p1[1])
        return np.array([[a, -b, tx],
                         [b,  a, ty]], dtype=np.float64)

    raw    = np.array(raw_pts,    dtype=np.float64).reshape(-1, 1, 2)
    actual = np.array(actual_pts, dtype=np.float64).reshape(-1, 1, 2)
    M, _ = cv2.estimateAffinePartial2D(raw, actual)
    if M is None:
        M, _ = cv2.estimateAffine2D(raw, actual)
    return M.astype(np.float64)


def apply_correction(M, x, y):
    cx = M[0, 0]*x + M[0, 1]*y + M[0, 2]
    cy = M[1, 0]*x + M[1, 1]*y + M[1, 2]
    return float(cx), float(cy)


# ─────────────────────────────────────────────────────────────────────────────
# NODE
# ─────────────────────────────────────────────────────────────────────────────
class ScaraYoloNode(Node):
    def __init__(self):
        super().__init__('scara_yolo_node')

        # ── ROS interfaces ──
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.listener_callback, 10)
        self.ik_pub    = self.create_publisher(Float32MultiArray, '/ik_target', 10)
        self.joint_pub = self.create_publisher(Float32MultiArray, '/odrive/angle_cmd', 10)
        # ── NEW: publishes cube detections + pick/place events as JSON.
        #     Your friend subscribes to /cube_info (std_msgs/String), parses
        #     the JSON, and draws RViz markers from it.
        self.cube_info_pub = self.create_publisher(String, '/cube_info', 10)

        self.bridge = CvBridge()
        # ── NEW MODEL ──  green / red / yellow detector
        self.model = YOLO('/home/reach/SCARA/src/yolo_pick_place/models/best2.pt')
        self.coord_buffer = {}
        self.smooth_window = 10
        self.get_logger().info(f'Model classes: {self.model.names}')
        self.get_logger().info(f'Picking targets: {sorted(TARGET_CLASSES)}')

        # ── CAN bus for MKS stepper ──
        self._tx_lock = threading.Lock()
        self.can_ok = False
        try:
            self.bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
            self.can_ok = True
            self.get_logger().info(f'CAN bus opened on {CAN_INTERFACE}.')
        except OSError as e:
            self.bus = None
            self.get_logger().error(
                f"CAN open failed: {e}\n"
                f"Run: sudo ip link set {CAN_INTERFACE} up type can bitrate 250000\n"
                f"Continuing without stepper control.")

        # Lock physical startup position as ZERO (= highest).
        self.stepper_pos_m = 0.0
        self.suction_on = False   # vacuum gripper state (False = released)
        if self.can_ok:
            self.get_logger().info(
                'Locking current physical stepper position as ZERO (highest).')
            self._send_can(build_set_zero_frame(), 'SET_ZERO @ startup')
            time.sleep(0.05)
            payload, _ = build_abs_frame(0.0)
            self._send_can(payload, 'POS=0 (anchor)')

        # ── Correction matrix ──
        M = load_correction()
        if M is not None:
            self.correction_M = M
            self.get_logger().info(f'Loaded correction matrix:\n{self.correction_M}')
        else:
            self.correction_M = np.array(
                [[1.0, 0.0, DEFAULT_OFFSET_X],
                 [0.0, 1.0, DEFAULT_OFFSET_Y]], dtype=np.float64)
            self.get_logger().info(
                f'Using default correction (translation only):\n{self.correction_M}')

        # ── State ──
        self.H = None
        self.roi_locked = False
        self.manual_points = []
        self.wait_start_time = None
        self.home_sent_time = None
        self.stage_start_time = None
        self.last_target_robot = (0.0, 0.0)
        self.last_target_label = ''
        self.last_target_px = (0, 0)
        self.current_mouse_px = (0, 0)
        self.workspace_polygon = None
        self.mode = MODE_YOLO

        # ── NEW: place positions ──
        # robot-frame target for each color, and pixel for drawing.
        self.place_map = {}        # {'green': (rx, ry), 'yellow': ..., 'red': ...}
        self.place_px  = {}        # {'green': (cx, cy), ...}
        self.place_click_idx = 0   # how many bins clicked so far during selection
        self.place_jog_color = None  # color currently being fine-tuned
        self.place_jog_px    = (0, 0)  # pixel where the user clicked for this color
        self.saved_place_map = None
        self.saved_place_px  = None

        # ── Multi-cube snapshot queue ──
        self.cube_queue = []        # list of dicts: {label, rx, ry, cx, cy}
        self.cube_snapshot = []     # full snapshot (for drawing white crosses)
        self.cube_total = 0         # how many were in the original snapshot
        self.cube_index = 0         # how many we've finished so far

        # Calibration scratch
        self.calib_raw_list = []
        self.calib_actual_list = []
        self.calib_current_idx = 0
        self.calib_raw_target = (0.0, 0.0)
        self.calib_current = (0.0, 0.0)
        self.calib_detected = False
        self.calib_last_px = []

        # Shutdown request flag (read by main loop after spin)
        self.shutdown_requested = False

        # ── NEW: throttle for the live "scanning" broadcast while IDLE.
        #     Without this we'd publish on every camera frame (~30 Hz).
        self.last_scan_pub_time = 0.0
        self.SCAN_PUB_PERIOD_S  = 1.0   # publish what the camera sees ~1x/sec

        # ── NEW: idle-scan window. When cubes first appear the robot waits
        #     SCAN_WINDOW_S seconds (streaming 'scanning' messages) BEFORE it
        #     locks the queue and starts picking. None = not started yet.
        self.SCAN_WINDOW_S = 5.0
        self.scan_window_start = None

        cv2.namedWindow("SCARA YOLO Control")
        cv2.setMouseCallback("SCARA YOLO Control", self.mouse_callback)

        saved_points = load_workspace()
        if saved_points is not None:
            self.saved_points = saved_points
            self.state = STATE_ASK_RELOAD
        else:
            self.saved_points = None
            self.state = STATE_CALIBRATING

    # ─────────────────────────────────────────────────────────────────────
    # NEW: publish cube info / pick-place events to /cube_info
    # ─────────────────────────────────────────────────────────────────────
    def publish_cube_info(self, event, cubes=None, color=None, rx=None, ry=None):
        """Publish cube data + pick/place events as JSON on /cube_info.

        event = 'scanning' : live preview — every cube the camera currently
                             sees while IDLE, before picking starts.
        event = 'detected' : the locked snapshot — every cube in the work
                             queue, sent once when picking begins.
        event = 'picked'   : includes color/x/y of the cube just grabbed.
        event = 'placed'   : includes color/x/y of the bin it was dropped in.
        """
        msg = {'event': event, 'stamp': time.time()}
        if cubes is not None:
            msg['cubes'] = [
                {'color': c['label'],
                 'x': round(float(c['rx']), 4),
                 'y': round(float(c['ry']), 4)}
                for c in cubes
            ]
        if color is not None:
            msg['color'] = color
        if rx is not None and ry is not None:
            msg['x'] = round(float(rx), 4)
            msg['y'] = round(float(ry), 4)

        out = String()
        out.data = json.dumps(msg)
        self.cube_info_pub.publish(out)
        self.get_logger().info(f'[/cube_info] {out.data}')

    # ─────────────────────────────────────────────────────────────────────
    # CAN send wrapper
    # ─────────────────────────────────────────────────────────────────────
    def _send_can(self, payload: bytes, label: str):
        if not self.can_ok:
            self.get_logger().warn(f'[{label}] (CAN disabled, skipping)')
            return
        crc  = crc8_sum(MKS_NODE_ID, payload)
        data = payload + bytes([crc])
        msg  = can.Message(arbitration_id=MKS_NODE_ID,
                           data=data, is_extended_id=False)
        with self._tx_lock:
            try:
                self.bus.send(msg, timeout=0.1)
                self.get_logger().info(
                    f'[{label}] → id=0x{MKS_NODE_ID:03X} data={data.hex().upper()}')
            except can.CanError as e:
                self.get_logger().error(f'[{label}] CAN send failed: {e}')

    def set_suction(self, on: bool):
        """Energize (on=True) or vent (on=False) the vacuum gripper via the
        Output2CAN board. Sent directly on the shared CAN bus — different
        arbitration id from the stepper, and NO MKS CRC byte."""
        if not self.can_ok or self.bus is None:
            self.get_logger().warn(
                f'[SUCTION {"ON" if on else "OFF"}] (CAN disabled, skipping)')
            return
        data = build_solenoid_frame(on)
        msg = can.Message(arbitration_id=SOLENOID_CAN_ID,
                          data=data, is_extended_id=False)
        with self._tx_lock:
            try:
                # Send 3x: a single dropped frame must not drop the object.
                for _ in range(3):
                    self.bus.send(msg, timeout=0.1)
                self.suction_on = on
                self.get_logger().info(
                    f'[SUCTION {"ON" if on else "OFF"}] '
                    f'id=0x{SOLENOID_CAN_ID:03X} data={data.hex().upper()}')
            except can.CanError as e:
                self.get_logger().error(f'[SUCTION] CAN send failed: {e}')

    def stepper_move_to(self, z_m: float, label: str = ''):
        z_m = max(POS_UP_M, min(POS_DOWN_M, z_m))
        payload, counts = build_abs_frame(z_m)
        self.stepper_pos_m = z_m
        self._send_can(payload, f'Z={z_m*1000:+.1f}mm cnt={counts:+d} {label}')

    def stepper_up(self):
        self.stepper_move_to(POS_UP_M, 'UP')

    def stepper_down(self):
        self.stepper_move_to(POS_DOWN_M, 'DOWN')

    def stepper_jog(self, delta_m: float):
        new_z = self.stepper_pos_m + delta_m
        self.stepper_move_to(new_z, 'JOG')

    # ─────────────────────────────────────────────────────────────────────
    # Workspace setup
    # ─────────────────────────────────────────────────────────────────────
    def apply_workspace(self, points):
        src = np.array(points, dtype=np.float32)
        dst = np.array([MARKER_POSITIONS[i] for i in range(1, 5)], dtype=np.float32)
        self.H, _ = cv2.findHomography(src, dst)
        self.workspace_polygon = np.array(points, dtype=np.int32)
        self.manual_points = points
        self.roi_locked = True
        self.state = STATE_ASK_OFFSET_CALIB

    # ─────────────────────────────────────────────────────────────────────
    # NEW: after offset calibration finishes, decide whether to ask the user
    # to reload saved place positions or to pick new ones.
    # ─────────────────────────────────────────────────────────────────────
    def enter_place_stage(self):
        """Called once offset calibration (or skip) is done."""
        saved_map, saved_px = load_place_positions()
        if saved_map is not None:
            self.saved_place_map = saved_map
            self.saved_place_px  = saved_px
            self.state = STATE_ASK_PLACE_RELOAD
        else:
            self.begin_place_selection()

    def begin_place_selection(self):
        """Reset and enter the click-to-select-bins screen."""
        self.place_map = {}
        self.place_px  = {}
        self.place_click_idx = 0
        self.state = STATE_SELECT_PLACE
        self.get_logger().info(
            f'>>> SELECT PLACE POSITIONS: click {len(PLACE_ORDER)} points '
            f'in order → {", ".join(PLACE_ORDER)}. Clicks may be OUTSIDE the box.')

    def finish_place_selection(self):
        """All bins clicked → save and continue to homing."""
        save_place_positions(self.place_map, self.place_px)
        self.get_logger().info('Place positions set. Proceeding to HOMING.')
        self.stepper_up()
        self.state = STATE_HOMING
        self.home_sent_time = None

    # ─────────────────────────────────────────────────────────────────────
    # Mouse handling
    # ─────────────────────────────────────────────────────────────────────
    def mouse_callback(self, event, x, y, flags, param):
        self.current_mouse_px = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.state == STATE_CALIBRATING:
                self.manual_points.append([x, y])
                self.get_logger().info(
                    f'Corner {len(self.manual_points)}/4 set at ({x}, {y})')
                if len(self.manual_points) == 4:
                    save_workspace(self.manual_points)
                    self.apply_workspace(self.manual_points)

            # ── NEW: clicking to select a place position ──
            elif self.state == STATE_SELECT_PLACE:
                if self.place_click_idx < len(PLACE_ORDER):
                    color = PLACE_ORDER[self.place_click_idx]
                    rx, ry = self.pixel_to_robot(x, y)   # works outside box too
                    # Move the arm to the clicked spot, then let the user
                    # fine-tune with WASD/UI before confirming with ENTER.
                    self.place_jog_color = color
                    self.place_jog_px    = (x, y)
                    self.calib_current   = (rx, ry)   # reuse jog XY variable
                    self.ik_pub.publish(Float32MultiArray(data=[rx, ry]))
                    self.get_logger().info(
                        f'Place [{color}] clicked at pixel ({x},{y}) '
                        f'→ robot ({rx:.4f}, {ry:.4f}). Arm moving there. '
                        f'Fine-tune with WASD/UI, ENTER to confirm.')
                    self.state = STATE_SELECT_PLACE_JOG

            elif self.mode == MODE_MANUAL and self.state == STATE_IDLE:
                rx, ry = self.pixel_to_robot(x, y)
                self.start_pick_place(rx, ry, (x, y), 'manual click')

    # ─────────────────────────────────────────────────────────────────────
    # Coordinate helpers
    # ─────────────────────────────────────────────────────────────────────
    def pixel_to_robot_raw(self, px, py):
        pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
        result = cv2.perspectiveTransform(pt, self.H)
        return float(result[0][0][0]), float(result[0][0][1])

    def pixel_to_robot(self, px, py):
        rx, ry = self.pixel_to_robot_raw(px, py)
        return apply_correction(self.correction_M, rx, ry)

    def point_in_polygon(self, px, py):
        return cv2.pointPolygonTest(
            self.workspace_polygon.astype(np.float32),
            (float(px), float(py)), False) >= 0

    # ─────────────────────────────────────────────────────────────────────
    # YOLO
    # ─────────────────────────────────────────────────────────────────────
    def run_yolo_detection(self, frame, draw=True, apply_correction_flag=True):
        x, y, w, h = cv2.boundingRect(self.workspace_polygon)
        h_frame, w_frame = frame.shape[:2]
        rx1 = max(0, x); ry1 = max(0, y)
        rx2 = min(w_frame, x + w); ry2 = min(h_frame, y + h)

        roi_frame = frame[ry1:ry2, rx1:rx2]
        results = self.model(roi_frame, verbose=False, conf=0.3, imgsz=640)

        for box in results[0].boxes:
            cls_name = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            # Ignore anything that is not one of our color targets.
            if cls_name not in TARGET_CLASSES:
                if draw:
                    bx1, by1, bx2, by2 = [int(v) for v in box.xyxy[0].tolist()]
                    fx1, fy1 = bx1 + rx1, by1 + ry1
                    fx2, fy2 = bx2 + rx1, by2 + ry1
                    cv2.rectangle(frame, (fx1, fy1), (fx2, fy2), (128, 128, 128), 1)
                    cv2.putText(frame, f"{cls_name} (ignored)",
                                (fx1, fy1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
                continue

            bx1, by1, bx2, by2 = [int(v) for v in box.xyxy[0].tolist()]
            cx = (bx1 + bx2) // 2 + rx1
            cy = (by1 + by2) // 2 + ry1
            if not self.point_in_polygon(cx, cy):
                continue

            if apply_correction_flag:
                rx, ry = self.pixel_to_robot(cx, cy)
            else:
                rx, ry = self.pixel_to_robot_raw(cx, cy)

            if draw:
                box_color = CLASS_DRAW_COLORS.get(cls_name, (0, 255, 255))
                fx1, fy1 = bx1 + rx1, by1 + ry1
                fx2, fy2 = bx2 + rx1, by2 + ry1
                cv2.rectangle(frame, (fx1, fy1), (fx2, fy2), box_color, 2)
                cs = 12
                cv2.line(frame, (cx - cs, cy), (cx + cs, cy), (0, 0, 255), 2)
                cv2.line(frame, (cx, cy - cs), (cx, cy + cs), (0, 0, 255), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(frame,
                            f"{cls_name} {conf:.2f} | center:({cx},{cy}) | robot:({rx:.3f},{ry:.3f})",
                            (fx1, fy1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 2)
            return cls_name, rx, ry, cx, cy

        return None, None, None, None, None

    def detect_all_cubes(self, frame, draw=True):
        """Run YOLO once and return EVERY target cube inside the workspace.

        Returns a list of dicts: {label, rx, ry, cx, cy, conf}.
        Coordinates use the corrected pixel_to_robot transform (same as a
        normal pick). The list is NOT sorted here — caller sorts it.
        """
        x, y, w, h = cv2.boundingRect(self.workspace_polygon)
        h_frame, w_frame = frame.shape[:2]
        rx1 = max(0, x); ry1 = max(0, y)
        rx2 = min(w_frame, x + w); ry2 = min(h_frame, y + h)

        roi_frame = frame[ry1:ry2, rx1:rx2]
        results = self.model(roi_frame, verbose=False, conf=0.3, imgsz=640)

        cubes = []
        for box in results[0].boxes:
            cls_name = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            if cls_name not in TARGET_CLASSES:
                continue

            bx1, by1, bx2, by2 = [int(v) for v in box.xyxy[0].tolist()]
            cx = (bx1 + bx2) // 2 + rx1
            cy = (by1 + by2) // 2 + ry1
            if not self.point_in_polygon(cx, cy):
                continue

            rx, ry = self.pixel_to_robot(cx, cy)
            cubes.append({'label': cls_name, 'rx': rx, 'ry': ry,
                          'cx': cx, 'cy': cy, 'conf': conf})

            if draw:
                # White cross on every detected cube.
                self._draw_cross(frame, cx, cy, (255, 255, 255))
        return cubes

    def _draw_cross(self, frame, cx, cy, color, size=12, thick=2):
        """Draw a simple + cross centered at (cx, cy)."""
        cv2.line(frame, (cx - size, cy), (cx + size, cy), color, thick, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - size), (cx, cy + size), color, thick, cv2.LINE_AA)

    def draw_queue_progress(self, frame):
        """Show 'Cube N / Total' progress during a multi-cube run."""
        if self.cube_total <= 0:
            return
        h, w = frame.shape[:2]
        done = self.cube_index
        remaining = len(self.cube_queue)
        txt = f"CUBE {done + 1}/{self.cube_total}   ({remaining} left in queue)"
        cv2.putText(frame, txt, (20, h - 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

    def draw_locked_target(self, frame):
        # White cross stays on every cube still waiting in the queue.
        for cube in self.cube_queue:
            self._draw_cross(frame, cube['cx'], cube['cy'], (255, 255, 255),
                             size=12, thick=2)
        # The cube currently being picked/placed gets a RED cross.
        cx, cy = self.last_target_px
        self._draw_cross(frame, cx, cy, (0, 0, 255), size=14, thick=2)

    def draw_active_coord(self, frame):
        """Bottom-left: show ONLY the coordinate of the cube currently being
        handled — exactly the (x, y) that was published to the IK node.
        Text color matches the cube's color (green / red / yellow)."""
        rx, ry = self.last_target_robot
        col = CLASS_DRAW_COLORS.get(self.last_target_label, (255, 255, 255))
        h, w = frame.shape[:2]
        cv2.putText(frame, f"({rx:.3f}, {ry:.3f})",
                    (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    col, 2, cv2.LINE_AA)

    # ── NEW: draw the place markers (during selection AND during runs) ──
    def draw_place_markers(self, frame):
        for color, px in self.place_px.items():
            draw_col = CLASS_DRAW_COLORS.get(color, (255, 255, 255))
            cx, cy = px
            cv2.drawMarker(frame, (cx, cy), draw_col,
                           cv2.MARKER_TILTED_CROSS, 20, 2)
            cv2.circle(frame, (cx, cy), 14, draw_col, 2)

    # ─────────────────────────────────────────────────────────────────────
    # PARK-AND-EXIT  (triggered by C key outside calibration)
    # ─────────────────────────────────────────────────────────────────────
    def start_park_and_exit(self):
        """Retract stepper UP, then move XY to (0, 0.30), then shut down."""
        self.get_logger().info(
            f'PARK requested. Retracting stepper UP, then moving to '
            f'({PARK_X}, {PARK_Y}). Node will exit afterwards.')
        self.stepper_up()
        self.stage_start_time = time.time()
        self.state = STATE_PARK_RETRACT

    def tick_park(self):
        now = time.time()
        elapsed = now - self.stage_start_time

        if self.state == STATE_PARK_RETRACT:
            if elapsed >= PARK_STEPPER_S:
                self.get_logger().info(
                    f'PARK: Stepper retracted. Moving XY to ({PARK_X}, {PARK_Y}).')
                self.ik_pub.publish(Float32MultiArray(data=[float(PARK_X), float(PARK_Y)]))
                self.stage_start_time = now
                self.state = STATE_PARK_MOVE

        elif self.state == STATE_PARK_MOVE:
            if elapsed >= PARK_TRAVEL_S:
                self.get_logger().info(
                    'PARK: Arrived at park position. Shutting down node.')
                self.state = STATE_PARK_DONE
                self.shutdown_requested = True

    # ─────────────────────────────────────────────────────────────────────
    # MULTI-CUBE QUEUE
    # ─────────────────────────────────────────────────────────────────────
    def build_cube_queue(self, cubes):
        """Sort detected cubes left-to-right, then top-to-bottom, and store
        them as the work queue. Sorting is done on PIXEL coords so it matches
        what the camera sees on screen (cx = left→right, cy = top→bottom)."""
        cubes_sorted = sorted(cubes, key=lambda c: (c['cx'], c['cy']))
        self.cube_queue = cubes_sorted
        self.cube_snapshot = list(cubes_sorted)   # keep a copy for drawing crosses
        self.cube_total = len(cubes_sorted)
        self.cube_index = 0
        order = ", ".join(f"{i+1}:{c['label']}"
                          for i, c in enumerate(cubes_sorted))
        self.get_logger().info(
            f'Snapshot locked: {self.cube_total} cube(s). Pick order → {order}')
        # ── NEW: broadcast every detected cube (color + coordinate) ──
        self.publish_cube_info('detected', cubes=cubes_sorted)

    def advance_cube_queue(self):
        """Start the next cube in the queue, or finish if the queue is empty."""
        if self.cube_queue:
            cube = self.cube_queue.pop(0)
            self.get_logger().info(
                f'>>> Picking cube {self.cube_index + 1}/{self.cube_total} '
                f'[{cube["label"]}] at ({cube["rx"]:.3f}, {cube["ry"]:.3f}).')
            self.start_pick_place(cube['rx'], cube['ry'],
                                  (cube['cx'], cube['cy']), cube['label'])
        else:
            self.get_logger().info(
                f'All {self.cube_total} cube(s) done. Returning HOME.')
            self.cube_total = 0
            self.cube_index = 0
            self.cube_snapshot = []
            # Go through HOMING so the arm physically returns to home,
            # then the HOMING handler drops us back into IDLE automatically.
            self.state = STATE_HOMING
            self.home_sent_time = None

    # ─────────────────────────────────────────────────────────────────────
    # PICK-AND-PLACE SEQUENCE
    # ─────────────────────────────────────────────────────────────────────
    def start_pick_place(self, rx, ry, px_xy, label):
        self.ik_pub.publish(Float32MultiArray(data=[rx, ry]))
        self.last_target_robot = (rx, ry)
        self.last_target_label = label
        self.last_target_px = px_xy
        self.get_logger().info(
            f'PnP start → XY → ({rx:.3f}, {ry:.3f}) [{label}]')
        self.stage_start_time = time.time()
        self.state = STATE_PNP_GO_TO_CAP

    # ── NEW: look up the place target for the current cube's color ──
    def get_place_target(self, label):
        """Return (rx, ry) place position for this color, or fall back to a
        safe default if the color was never set."""
        if label in self.place_map:
            return self.place_map[label]
        # Fallback (should not happen since we force all 3): old home point.
        self.get_logger().warn(
            f'No place position for color [{label}] — falling back to '
            f'({PARK_X}, {PARK_Y}).')
        return (float(PARK_X), float(PARK_Y))

    def tick_pick_place(self):
        now = time.time()
        elapsed = now - self.stage_start_time

        if self.state == STATE_PNP_GO_TO_CAP:
            if elapsed >= XY_TRAVEL_S:
                self.get_logger().info('PnP: XY reached object → stepper DOWN.')
                self.stepper_down()
                self.stage_start_time = now
                self.state = STATE_PNP_DOWN_AT_CAP

        elif self.state == STATE_PNP_DOWN_AT_CAP:
            if elapsed >= STEPPER_TRAVEL_S:
                self.set_suction(True)        # grab the object (vacuum ON)
                # ── NEW: broadcast that this cube has been picked ──
                self.publish_cube_info('picked',
                                       color=self.last_target_label,
                                       rx=self.last_target_robot[0],
                                       ry=self.last_target_robot[1])
                self.get_logger().info(
                    f'PnP: Stepper DOWN reached → SUCTION ON, '
                    f'waiting {PICK_WAIT_S:.0f}s at object.')
                self.stage_start_time = now
                self.state = STATE_PNP_WAIT_AT_CAP

        elif self.state == STATE_PNP_WAIT_AT_CAP:
            if elapsed >= PICK_WAIT_S:
                self.get_logger().info('PnP: Pick wait done → stepper UP.')
                self.stepper_up()
                self.stage_start_time = now
                self.state = STATE_PNP_UP_AT_CAP

        elif self.state == STATE_PNP_UP_AT_CAP:
            if elapsed >= STEPPER_TRAVEL_S:
                # ── CHANGED: go to the COLOR-SPECIFIC place position, not home ──
                place_rx, place_ry = self.get_place_target(self.last_target_label)
                self.get_logger().info(
                    f'PnP: Stepper UP at object → XY going to '
                    f'[{self.last_target_label}] BIN ({place_rx:.3f}, {place_ry:.3f}).')
                self.ik_pub.publish(
                    Float32MultiArray(data=[float(place_rx), float(place_ry)]))
                self.stage_start_time = now
                self.state = STATE_PNP_GO_HOME

        elif self.state == STATE_PNP_GO_HOME:
            if elapsed >= XY_TRAVEL_S:
                self.get_logger().info('PnP: XY at BIN → stepper DOWN (release).')
                self.stepper_down()
                self.stage_start_time = now
                self.state = STATE_PNP_DOWN_AT_HOME

        elif self.state == STATE_PNP_DOWN_AT_HOME:
            if elapsed >= STEPPER_TRAVEL_S:
                self.set_suction(False)       # release the object (vacuum OFF)
                # ── NEW: broadcast that this cube has been placed in its bin ──
                place_rx, place_ry = self.get_place_target(self.last_target_label)
                self.publish_cube_info('placed',
                                       color=self.last_target_label,
                                       rx=place_rx, ry=place_ry)
                self.get_logger().info(
                    'PnP: Released at bin (SUCTION OFF) → stepper UP.')
                self.stepper_up()
                self.stage_start_time = now
                self.state = STATE_PNP_UP_AT_HOME

        elif self.state == STATE_PNP_UP_AT_HOME:
            if elapsed >= STEPPER_TRAVEL_S:
                self.cube_index += 1
                self.get_logger().info(
                    f'PnP: Cube {self.cube_index}/{self.cube_total} COMPLETE.')
                self.advance_cube_queue()

    # ─────────────────────────────────────────────────────────────────────
    # MAIN CALLBACK
    # ─────────────────────────────────────────────────────────────────────
    def listener_callback(self, msg):
        # If we're parking, just tick the park state machine and exit early
        if self.state in (STATE_PARK_RETRACT, STATE_PARK_MOVE, STATE_PARK_DONE):
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.waitKey(1)
            self.tick_park()
            self.draw_park(frame)
            cv2.imshow("SCARA YOLO Control", frame)
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ── ASK RELOAD ──
        if self.state == STATE_ASK_RELOAD:
            self.draw_ask_reload(frame)
            key = cv2.waitKey(1) & 0xFF
            if self._check_park_key(key, allow_in_calib=False):
                return
            if key == 13:
                self.apply_workspace(self.saved_points)
            elif key == ord('r') or key == ord('R'):
                self.manual_points = []
                self.state = STATE_CALIBRATING
            return

        # ── CALIBRATING WORKSPACE ──
        if not self.roi_locked:
            key = cv2.waitKey(1) & 0xFF
            # C during workspace calibration also parks & exits
            if self._check_park_key(key, allow_in_calib=False):
                return
            self.handle_stepper_jog_key(key)
            self.draw_calibration(frame)
            return

        # ── ASK OFFSET CALIB ──
        if self.state == STATE_ASK_OFFSET_CALIB:
            self.draw_ask_offset_calib(frame)
            key = cv2.waitKey(1) & 0xFF
            if self._check_park_key(key, allow_in_calib=False):
                return
            self.handle_stepper_jog_key(key)
            # Consistent with other screens: R = redo/do-fresh, ENTER = use saved.
            if key == ord('r') or key == ord('R'):
                self.calib_raw_list = []
                self.calib_actual_list = []
                self.calib_last_px = []
                self.calib_current_idx = 0
                self.start_next_calib_point()
            elif key == 13:   # ENTER → skip calibration, use saved correction
                # ── CHANGED: skip offset calib → go to place stage (not homing) ──
                self.stepper_up()
                self.enter_place_stage()
            return

        # ── OFFSET DETECT ──
        if self.state == STATE_OFFSET_DETECT:
            key = cv2.waitKey(1) & 0xFF
            self.handle_stepper_jog_key(key)

            if self.home_sent_time is not None and \
               (time.time() - self.home_sent_time) < 2.0:
                self.draw_offset_detect(frame, homing=True)
                cv2.imshow("SCARA YOLO Control", frame)
                return
            self.home_sent_time = None

            cls_name, rx, ry, cx, cy = self.run_yolo_detection(
                frame, draw=True, apply_correction_flag=False)

            if cls_name is not None and not self.calib_detected:
                self.calib_raw_target = (rx, ry)
                self.calib_current    = (rx, ry)
                self.last_target_px   = (cx, cy)
                self.ik_pub.publish(Float32MultiArray(data=[rx, ry]))
                self.get_logger().info(
                    f'[Point {self.calib_current_idx + 1}/{NUM_CALIB_POINTS}] '
                    f'{cls_name} detected at raw ({rx:.4f}, {ry:.4f}). Moving XY.')
                self.calib_detected = True
                self.wait_start_time = time.time()
            elif self.calib_detected:
                if (time.time() - self.wait_start_time) > 2.0:
                    self.state = STATE_OFFSET_JOG
                    self.get_logger().info(
                        f'[Point {self.calib_current_idx + 1}/{NUM_CALIB_POINTS}] '
                        f'JOG MODE: WASD = XY, U/I = stepper, ENTER to save.')

            self.draw_offset_detect(frame, homing=False)
            cv2.imshow("SCARA YOLO Control", frame)
            return

        # ── OFFSET JOG ──  (C here cancels calibration, NOT park-exit)
        if self.state == STATE_OFFSET_JOG:
            key = cv2.waitKey(1) & 0xFFFF
            self.handle_jog_key(key)
            self.draw_offset_jog(frame)
            cv2.imshow("SCARA YOLO Control", frame)
            return

        # ── NEW: ASK PLACE RELOAD ──
        if self.state == STATE_ASK_PLACE_RELOAD:
            self.draw_ask_place_reload(frame)
            key = cv2.waitKey(1) & 0xFF
            if self._check_park_key(key, allow_in_calib=False):
                return
            if key == 13:   # ENTER → load saved place positions
                self.place_map = dict(self.saved_place_map)
                self.place_px  = dict(self.saved_place_px) if self.saved_place_px else {}
                self.get_logger().info('Loaded saved place positions.')
                self.stepper_up()
                self.state = STATE_HOMING
                self.home_sent_time = None
            elif key == ord('r') or key == ord('R'):
                self.begin_place_selection()
            return

        # ── NEW: SELECT PLACE POSITIONS (click 3 bins) ──
        if self.state == STATE_SELECT_PLACE:
            key = cv2.waitKey(1) & 0xFF
            if self._check_park_key(key, allow_in_calib=False):
                return
            self.handle_stepper_jog_key(key)
            # Allow 'r' to restart the clicking if user mis-clicks.
            if key == ord('r') or key == ord('R'):
                self.get_logger().info('Restarting place-position selection.')
                self.begin_place_selection()
            self.draw_select_place(frame)
            cv2.imshow("SCARA YOLO Control", frame)
            return

        # ── NEW: SELECT PLACE JOG (arm moved to click; fine-tune then ENTER) ──
        if self.state == STATE_SELECT_PLACE_JOG:
            key = cv2.waitKey(1) & 0xFFFF
            self.handle_place_jog_key(key)
            self.draw_select_place_jog(frame)
            cv2.imshow("SCARA YOLO Control", frame)
            return

        # ── Common key handling for IDLE/HOMING/PnP ──
        key = cv2.waitKey(1) & 0xFF

        # PARK-AND-EXIT shortcut: C key
        if self._check_park_key(key, allow_in_calib=False):
            return

        # Stepper jog only in IDLE so it doesn't interrupt PnP
        if self.state == STATE_IDLE:
            self.handle_stepper_jog_key(key)

        if key == ord('m') or key == ord('M'):
            if self.mode == MODE_YOLO:
                self.mode = MODE_MANUAL
                self.get_logger().info('Switched to MANUAL click mode.')
            else:
                self.mode = MODE_YOLO
                self.get_logger().info('Switched to YOLO auto mode.')
            self.state = STATE_HOMING

        if key == ord('o') or key == ord('O') and self.state == STATE_IDLE:
            self.get_logger().info(
                f'Re-running {NUM_CALIB_POINTS}-point offset calibration...')
            self.calib_raw_list = []
            self.calib_actual_list = []
            self.calib_last_px = []
            self.calib_current_idx = 0
            self.start_next_calib_point()
            return

        # ── NEW: 'p' key re-selects place positions from IDLE ──
        if (key == ord('p') or key == ord('P')) and self.state == STATE_IDLE:
            self.get_logger().info('Re-selecting place positions...')
            self.begin_place_selection()
            return

        # ── State machine ──
        if self.state == STATE_HOMING:
            if self.home_sent_time is None:
                self.cube_queue = []      # clear any stale queue on (re)home
                self.cube_total = 0
                self.cube_index = 0
                self.scan_window_start = None   # fresh 5s scan next cycle
                self.stepper_up()
                self.joint_pub.publish(Float32MultiArray(data=[1.0, 0.0, 2.0, 0.0]))
                self.home_sent_time = time.time()
            if (time.time() - self.home_sent_time) > 2.0:
                self.state = STATE_IDLE
                self.home_sent_time = None
                if self.mode == MODE_YOLO:
                    self.get_logger().info('Robot Home. Scanning for objects...')
                else:
                    self.get_logger().info('Robot Home. Click to move.')

        elif self.state == STATE_IDLE:
            if self.mode == MODE_YOLO:
                # Scan the table every frame.
                cubes = self.detect_all_cubes(frame, draw=True)

                if cubes:
                    now = time.time()

                    # First frame we see cubes → start the 5s scan window.
                    if self.scan_window_start is None:
                        self.scan_window_start = now
                        self.get_logger().info(
                            f'Cubes detected. Scanning for '
                            f'{self.SCAN_WINDOW_S:.0f}s before picking...')

                    # While inside the window: stream 'scanning' (~1 Hz) and
                    # do NOT move the arm yet.
                    if (now - self.scan_window_start) < self.SCAN_WINDOW_S:
                        if (now - self.last_scan_pub_time) >= self.SCAN_PUB_PERIOD_S:
                            self.publish_cube_info('scanning', cubes=cubes)
                            self.last_scan_pub_time = now
                    else:
                        # Window elapsed → lock this snapshot and start picking.
                        self.scan_window_start = None
                        self.build_cube_queue(cubes)
                        self.advance_cube_queue()   # starts the first cube
                else:
                    # No cubes in view → reset the window so the next time
                    # cubes appear we wait a fresh 5s.
                    self.scan_window_start = None
                # If no cubes, stay in IDLE and keep scanning each frame.

        elif self.state in (STATE_PNP_GO_TO_CAP, STATE_PNP_DOWN_AT_CAP,
                            STATE_PNP_WAIT_AT_CAP, STATE_PNP_UP_AT_CAP,
                            STATE_PNP_GO_HOME, STATE_PNP_DOWN_AT_HOME,
                            STATE_PNP_UP_AT_HOME):
            self.draw_locked_target(frame)
            self.tick_pick_place()

        self.draw_ui(frame)
        cv2.imshow("SCARA YOLO Control", frame)

    # ─────────────────────────────────────────────────────────────────────
    # Helper: check for park-and-exit key (C)
    # ─────────────────────────────────────────────────────────────────────
    def _check_park_key(self, key, allow_in_calib=False):
        """If C is pressed (and not in a state where C means cancel-calib),
        start park-and-exit. Returns True if park was triggered."""
        if key == 255 or key == -1:
            return False
        if key == ord('c') or key == ord('C'):
            # In offset jog state, C cancels calibration instead (not park)
            if not allow_in_calib and self.state == STATE_OFFSET_JOG:
                return False
            self.start_park_and_exit()
            return True
        return False

    # ─────────────────────────────────────────────────────────────────────
    # Calibration flow
    # ─────────────────────────────────────────────────────────────────────
    def start_next_calib_point(self):
        self.calib_detected = False
        self.stepper_up()
        self.joint_pub.publish(Float32MultiArray(data=[1.0, 0.0, 2.0, 0.0]))
        self.home_sent_time = time.time()
        self.state = STATE_OFFSET_DETECT
        self.get_logger().info(
            f'>>> Place a colored object for point {self.calib_current_idx + 1}/'
            f'{NUM_CALIB_POINTS}. Spread points DIAGONALLY across workspace.')

    def finish_collect_current_point(self):
        self.calib_raw_list.append(self.calib_raw_target)
        self.calib_actual_list.append(self.calib_current)
        self.calib_last_px.append(self.last_target_px)
        self.get_logger().info(
            f'Point {self.calib_current_idx + 1} recorded: '
            f'raw={self.calib_raw_target}, actual={self.calib_current}')
        self.calib_current_idx += 1
        if self.calib_current_idx >= NUM_CALIB_POINTS:
            self.finalize_calibration()
        else:
            self.start_next_calib_point()

    def finalize_calibration(self):
        M = compute_correction_matrix(self.calib_raw_list, self.calib_actual_list)
        self.correction_M = M
        save_correction(M, self.calib_raw_list, self.calib_actual_list)

        scale   = float(np.sqrt(M[0, 0]**2 + M[0, 1]**2))
        rot_deg = float(np.degrees(np.arctan2(M[1, 0], M[0, 0])))
        tx, ty  = float(M[0, 2]), float(M[1, 2])
        self.get_logger().info(
            f'Calibration complete:\n'
            f'  scale       = {scale:.6f}\n'
            f'  rotation    = {rot_deg:+.4f} deg\n'
            f'  translation = ({tx:+.5f}, {ty:+.5f}) m')
        for i, (r, a_pt) in enumerate(zip(self.calib_raw_list, self.calib_actual_list)):
            pred = apply_correction(M, r[0], r[1])
            err  = np.sqrt((pred[0] - a_pt[0])**2 + (pred[1] - a_pt[1])**2)
            self.get_logger().info(
                f'  point {i+1} residual = {err*1000:.3f} mm')

        # ── CHANGED: after offset calib, go to place stage (not homing) ──
        self.stepper_up()
        self.enter_place_stage()

    # ─────────────────────────────────────────────────────────────────────
    # KEY HANDLERS
    # ─────────────────────────────────────────────────────────────────────
    def handle_stepper_jog_key(self, key):
        if key == 255 or key == -1:
            return
        k = key & 0xFF
        if k == ord('u') or k == ord('U'):
            self.stepper_jog(-STEPPER_JOG_STEP)
        elif k == ord('i') or k == ord('I'):
            self.stepper_jog(+STEPPER_JOG_STEP)

    def handle_jog_key(self, key):
        if key == 255 or key == -1 or key == 0xFFFF:
            return
        k = key & 0xFF

        if k == ord('w') or k == ord('W'):
            self._jog_xy(0.0, +JOG_STEP); return
        if k == ord('a') or k == ord('A'):
            self._jog_xy(-JOG_STEP, 0.0); return
        if k == ord('d') or k == ord('D'):
            self._jog_xy(+JOG_STEP, 0.0); return
        if k == ord('s'):
            self._jog_xy(0.0, -JOG_STEP); return

        if k == ord('u') or k == ord('U'):
            self.stepper_jog(-STEPPER_JOG_STEP); return
        if k == ord('i') or k == ord('I'):
            self.stepper_jog(+STEPPER_JOG_STEP); return

        if k == 13 or k == ord('S'):
            self.finish_collect_current_point(); return
        if k == 27 or k == ord('c') or k == ord('C'):
            self.cancel_calibration(); return
        if k == ord('r') or k == ord('R'):
            self.get_logger().info(
                f'Redoing point {self.calib_current_idx + 1}...')
            self.start_next_calib_point(); return

    def _jog_xy(self, dx, dy):
        cx_now, cy_now = self.calib_current
        new_x = cx_now + dx
        new_y = cy_now + dy
        self.calib_current = (new_x, new_y)
        self.ik_pub.publish(Float32MultiArray(data=[new_x, new_y]))
        self.get_logger().info(
            f'[Pt {self.calib_current_idx + 1}/{NUM_CALIB_POINTS}] '
            f'JOG XY → ({new_x:.4f}, {new_y:.4f}) | '
            f'delta = ({new_x - self.calib_raw_target[0]:+.4f}, '
            f'{new_y - self.calib_raw_target[1]:+.4f})')

    def cancel_calibration(self):
        self.get_logger().info(
            'Calibration CANCELLED. Keeping previous correction matrix.')
        # ── CHANGED: still proceed to place stage so a cancel doesn't skip it ──
        self.stepper_up()
        self.enter_place_stage()

    # ── NEW: jog handler for fine-tuning a place position ──
    def handle_place_jog_key(self, key):
        """During place fine-tune: WASD jogs XY, U/I jogs stepper, ENTER
        confirms this color and advances to the next (or finishes after the
        last), R re-does the current color's click."""
        if key == 255 or key == -1 or key == 0xFFFF:
            return
        k = key & 0xFF

        # XY jog (reuses _jog_xy which moves self.calib_current and publishes)
        if k == ord('w') or k == ord('W'):
            self._jog_xy(0.0, +JOG_STEP); return
        if k == ord('a') or k == ord('A'):
            self._jog_xy(-JOG_STEP, 0.0); return
        if k == ord('d') or k == ord('D'):
            self._jog_xy(+JOG_STEP, 0.0); return
        if k == ord('s'):
            self._jog_xy(0.0, -JOG_STEP); return

        # Stepper jog (lower it onto the bin to check placement height)
        if k == ord('u') or k == ord('U'):
            self.stepper_jog(-STEPPER_JOG_STEP); return
        if k == ord('i') or k == ord('I'):
            self.stepper_jog(+STEPPER_JOG_STEP); return

        # ENTER → save fine-tuned position for this color, go to next color
        if k == 13:
            color = self.place_jog_color
            final_xy = self.calib_current               # the tuned coordinate
            self.place_map[color] = (float(final_xy[0]), float(final_xy[1]))
            self.place_px[color]  = self.place_jog_px   # keep original click px
            self.get_logger().info(
                f'Place [{color}] CONFIRMED at robot '
                f'({final_xy[0]:.4f}, {final_xy[1]:.4f}).')
            # Retract so the next move travels safely up high.
            self.stepper_up()
            self.place_click_idx += 1
            if self.place_click_idx >= len(PLACE_ORDER):
                self.finish_place_selection()           # all 3 done → save+home
            else:
                self.state = STATE_SELECT_PLACE          # click the next color
                nxt = PLACE_ORDER[self.place_click_idx]
                self.get_logger().info(
                    f'>>> Now click the [{nxt}] basket position.')
            return

        # R → redo this color: go back to click mode for the same color
        if k == ord('r') or k == ord('R'):
            self.get_logger().info(
                f'Re-clicking [{self.place_jog_color}] basket position.')
            self.stepper_up()
            self.state = STATE_SELECT_PLACE
            return

        # C / ESC → abort selection entirely (park & exit)
        if k == 27 or k == ord('c') or k == ord('C'):
            self.start_park_and_exit()
            return

    # ─────────────────────────────────────────────────────────────────────
    # DRAWING
    # ─────────────────────────────────────────────────────────────────────
    def _draw_z_indicator(self, frame):
        h, w = frame.shape[:2]
        z_mm = self.stepper_pos_m * 1000.0
        color = (0, 255, 0) if abs(z_mm) < 0.5 else (0, 200, 255)
        cv2.putText(frame, f"Z stepper: {z_mm:+.1f} mm  (U=up I=down)",
                    (w - 290, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    def draw_park(self, frame):
        """Big overlay while parking & exiting."""
        h, w = frame.shape[:2]
        if self.workspace_polygon is not None:
            cv2.polylines(frame, [self.workspace_polygon], True, (255, 0, 0), 2)
        overlay = frame.copy()
        cv2.rectangle(overlay, (w//2 - 360, h//2 - 100),
                      (w//2 + 360, h//2 + 100), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, frame)
        cv2.putText(frame, "PARK & EXIT",
                    (w//2 - 130, h//2 - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 200, 255), 3, cv2.LINE_AA)
        if self.state == STATE_PARK_RETRACT:
            txt = "Retracting stepper UP..."
        elif self.state == STATE_PARK_MOVE:
            txt = f"Moving to park ({PARK_X}, {PARK_Y})..."
        else:
            txt = "Shutting down..."
        cv2.putText(frame, txt,
                    (w//2 - 280, h//2 + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        self._draw_z_indicator(frame)

    def draw_ask_reload(self, frame):
        overlay = frame.copy()
        h, w = frame.shape[:2]
        cv2.rectangle(overlay, (w//2 - 320, h//2 - 120),
                      (w//2 + 320, h//2 + 120), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
        cv2.putText(frame, "SAVED WORKSPACE FOUND",
                    (w//2 - 230, h//2 - 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, "Press  ENTER  to load saved workspace",
                    (w//2 - 270, h//2 - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "Press  R  to recalibrate new workspace",
                    (w//2 - 270, h//2 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        pts_text = "  ".join(
            [f"C{i+1}:({p[0]},{p[1]})" for i, p in enumerate(self.saved_points)])
        cv2.putText(frame, pts_text,
                    (w//2 - 300, h//2 + 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
        cv2.putText(frame, "C : park & exit",
                    (w//2 - 90, h - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1, cv2.LINE_AA)
        self._draw_z_indicator(frame)
        cv2.imshow("SCARA YOLO Control", frame)

    # ── NEW: ask whether to reload saved place positions ──
    def draw_ask_place_reload(self, frame):
        cv2.polylines(frame, [self.workspace_polygon], True, (255, 0, 0), 2)
        # show where the saved bins are
        if self.saved_place_px:
            for color, px in self.saved_place_px.items():
                draw_col = CLASS_DRAW_COLORS.get(color, (255, 255, 255))
                cv2.drawMarker(frame, tuple(px), draw_col,
                               cv2.MARKER_TILTED_CROSS, 18, 2)
                cv2.putText(frame, color.upper(), (px[0] + 14, px[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_col, 1, cv2.LINE_AA)
        overlay = frame.copy()
        h, w = frame.shape[:2]
        cv2.rectangle(overlay, (w//2 - 330, h//2 - 130),
                      (w//2 + 330, h//2 + 130), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.78, frame, 0.22, 0, frame)
        cv2.putText(frame, "SAVED PLACE POSITIONS FOUND",
                    (w//2 - 270, h//2 - 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, "Press  ENTER  to load saved bins",
                    (w//2 - 270, h//2 - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "Press  R  to re-select 3 bins",
                    (w//2 - 270, h//2 + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        bins_text = "  ".join(
            f"{c}:({self.saved_place_map[c][0]:+.3f},{self.saved_place_map[c][1]:+.3f})"
            for c in PLACE_ORDER)
        cv2.putText(frame, bins_text,
                    (w//2 - 320, h//2 + 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1, cv2.LINE_AA)
        cv2.putText(frame, "C : park & exit",
                    (w//2 - 90, h - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1, cv2.LINE_AA)
        self._draw_z_indicator(frame)
        cv2.imshow("SCARA YOLO Control", frame)

    # ── NEW: the click-to-select-bins screen ──
    def draw_select_place(self, frame):
        cv2.polylines(frame, [self.workspace_polygon], True, (255, 0, 0), 2)
        # draw any bins already placed
        self.draw_place_markers(frame)

        h, w = frame.shape[:2]
        # Small, thin backing strip only behind the two text lines so it
        # doesn't cover the workspace where you need to click.
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (360, 64), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        if self.place_click_idx < len(PLACE_ORDER):
            next_color = PLACE_ORDER[self.place_click_idx]
            nc = CLASS_DRAW_COLORS.get(next_color, (255, 255, 255))
            # Simple single-line prompt that changes per color.
            cv2.putText(frame, f"Select {next_color} basket",
                        (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.6, nc, 2, cv2.LINE_AA)

        cv2.putText(frame, "R: restart   C: exit",
                    (20, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (180, 180, 180), 1, cv2.LINE_AA)

        # live cursor robot coordinate (helpful for aiming outside the box)
        mx, my = self.current_mouse_px
        if self.H is not None:
            m_rx, m_ry = self.pixel_to_robot(mx, my)
            cv2.putText(frame, f"({m_rx:.3f}, {m_ry:.3f})",
                        (mx + 12, my + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        self._draw_z_indicator(frame)

    # ── NEW: overlay while fine-tuning a place position ──
    def draw_select_place_jog(self, frame):
        cv2.polylines(frame, [self.workspace_polygon], True, (255, 0, 0), 2)
        # show bins already confirmed
        self.draw_place_markers(frame)

        color = self.place_jog_color or ''
        col = CLASS_DRAW_COLORS.get(color, (255, 255, 255))

        # marker at the current (being-tuned) click pixel
        cx, cy = self.place_jog_px
        cv2.drawMarker(frame, (cx, cy), col, cv2.MARKER_TILTED_CROSS, 22, 2)
        cv2.circle(frame, (cx, cy), 16, col, 2)

        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (560, 170), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

        cv2.putText(frame, f"FINE-TUNE [{color.upper()}] BASKET",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)
        cv2.putText(frame, "W/A/S/D : jog XY 1mm    U/I : stepper UP/DOWN 1mm",
                    (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "ENTER : confirm this basket   R : re-click   C : exit",
                    (20, 98), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (180, 180, 180), 1, cv2.LINE_AA)
        rx, ry = self.calib_current
        cv2.putText(frame, f"XY pos: ({rx:+.4f}, {ry:+.4f}) m",
                    (20, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame,
                    f"Baskets confirmed: {self.place_click_idx} / {len(PLACE_ORDER)}",
                    (20, 158), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0), 1, cv2.LINE_AA)
        self._draw_z_indicator(frame)

    def draw_calibration(self, frame):
        for i, pt in enumerate(self.manual_points):
            cv2.circle(frame, (pt[0], pt[1]), 7, (0, 0, 255), -1)
            cv2.putText(frame, f"C{i+1}", (pt[0] + 10, pt[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        next_corner = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]
        idx = len(self.manual_points)
        if idx < 4:
            cv2.putText(frame,
                        f"CALIBRATION: Click corner {idx + 1}/4  ->  {next_corner[idx]}",
                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX,
                        0.75, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(frame,
                        "U = stepper UP (1mm)   |   I = stepper DOWN (1mm)   |   C = park & exit",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 200, 0), 1, cv2.LINE_AA)
        self._draw_z_indicator(frame)
        cv2.imshow("SCARA YOLO Control", frame)

    def draw_ask_offset_calib(self, frame):
        cv2.polylines(frame, [self.workspace_polygon], True, (255, 0, 0), 2)
        overlay = frame.copy()
        h, w = frame.shape[:2]
        cv2.rectangle(overlay, (w//2 - 380, h//2 - 160),
                      (w//2 + 380, h//2 + 160), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.78, frame, 0.22, 0, frame)
        cv2.putText(frame, f"OFFSET CALIBRATION ({NUM_CALIB_POINTS} POINTS)?",
                    (w//2 - 290, h//2 - 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, f"You will place {NUM_CALIB_POINTS} colored objects, one at a time.",
                    (w//2 - 290, h//2 - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "TIP: Spread them across the workspace (4 corners).",
                    (w//2 - 290, h//2 - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "Press  R      to calibrate (recommended)",
                    (w//2 - 290, h//2 + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, "Press  ENTER  to skip (use saved correction)",
                    (w//2 - 290, h//2 + 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "Press  C      to park & exit",
                    (w//2 - 290, h//2 + 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1, cv2.LINE_AA)
        scale = float(np.sqrt(self.correction_M[0, 0]**2 + self.correction_M[0, 1]**2))
        rot   = float(np.degrees(np.arctan2(self.correction_M[1, 0], self.correction_M[0, 0])))
        tx, ty = float(self.correction_M[0, 2]), float(self.correction_M[1, 2])
        cv2.putText(frame,
                    f"Current: scale={scale:.4f} rot={rot:+.2f}deg t=({tx:+.4f},{ty:+.4f})",
                    (w//2 - 290, h//2 + 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1, cv2.LINE_AA)
        self._draw_z_indicator(frame)
        cv2.imshow("SCARA YOLO Control", frame)

    def draw_offset_detect(self, frame, homing=False):
        cv2.polylines(frame, [self.workspace_polygon], True, (255, 0, 0), 2)
        for i, px in enumerate(self.calib_last_px):
            cv2.circle(frame, px, 8, (0, 255, 0), 2)
            cv2.putText(frame, f"#{i+1}", (px[0] + 10, px[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        header = f"OFFSET CALIB — Point {self.calib_current_idx + 1}/{NUM_CALIB_POINTS}"
        if homing:
            cv2.putText(frame, f"{header}: Homing...",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 165, 255), 2)
        else:
            cv2.putText(frame, f"{header}: Place colored object & wait for detection",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 2)
            if self.calib_detected:
                cv2.putText(frame, "Detected! Moving XY...",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2)
        self._draw_z_indicator(frame)

    def draw_offset_jog(self, frame):
        cv2.polylines(frame, [self.workspace_polygon], True, (255, 0, 0), 2)
        for i, px in enumerate(self.calib_last_px):
            cv2.circle(frame, px, 8, (0, 255, 0), 2)
            cv2.putText(frame, f"#{i+1}", (px[0] + 10, px[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if self.last_target_px != (0, 0):
            cx, cy = self.last_target_px
            cv2.line(frame, (cx - 12, cy), (cx + 12, cy), (0, 0, 255), 2)
            cv2.line(frame, (cx, cy - 12), (cx, cy + 12), (0, 0, 255), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Pt {self.calib_current_idx + 1}",
                        (cx + 15, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (600, 270), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
        cv2.putText(frame,
                    f"OFFSET CALIB — JOG  (Point {self.calib_current_idx + 1}/{NUM_CALIB_POINTS})",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "W/A/S/D : jog XY 1mm",
                    (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "U / I   : stepper UP / DOWN 1mm",
                    (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 200, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, "ENTER : save this point  |  R : redo  |  C/ESC : cancel calib",
                    (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (180, 180, 180), 1, cv2.LINE_AA)
        rx, ry = self.calib_current
        ex = rx - self.calib_raw_target[0]
        ey = ry - self.calib_raw_target[1]
        cv2.putText(frame, f"XY pos:  ({rx:+.4f}, {ry:+.4f}) m",
                    (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"Z pos:   {self.stepper_pos_m*1000:+.2f} mm",
                    (20, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 200, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, f"Delta:   ({ex:+.5f}, {ey:+.5f}) m",
                    (20, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame,
                    f"Points collected: {self.calib_current_idx} / {NUM_CALIB_POINTS}",
                    (20, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0), 1, cv2.LINE_AA)

    def draw_ui(self, frame):
        cv2.polylines(frame, [self.workspace_polygon], True, (255, 0, 0), 2)
        # ── always show the bins during normal operation ──
        self.draw_place_markers(frame)
        h, w = frame.shape[:2]

        # Top-left: just AUTO or MANUAL.
        mode_text  = "AUTO" if self.mode == MODE_YOLO else "MANUAL"
        mode_color = (0, 255, 0) if self.mode == MODE_YOLO else (0, 165, 255)
        cv2.putText(frame, mode_text,
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2, cv2.LINE_AA)

        # During pick-and-place, show ONLY the active cube's coordinate
        # bottom-left — the exact (x, y) sent to the IK node.
        if self.state in (STATE_PNP_GO_TO_CAP, STATE_PNP_DOWN_AT_CAP,
                          STATE_PNP_WAIT_AT_CAP, STATE_PNP_UP_AT_CAP,
                          STATE_PNP_GO_HOME, STATE_PNP_DOWN_AT_HOME,
                          STATE_PNP_UP_AT_HOME):
            self.draw_active_coord(frame)

    # ─────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        try:
            if self.can_ok and self.bus is not None:
                self.set_suction(False)                    # drop anything held
                self._send_can(build_estop_frame(), 'ESTOP @ shutdown')
                self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScaraYoloNode()
    try:
        # Use a hand-rolled spin so we can check the shutdown flag
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.shutdown_requested:
            node.get_logger().info('Shutdown flag set. Exiting node.')
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()