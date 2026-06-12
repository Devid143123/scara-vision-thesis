#!/usr/bin/env python3
"""
MKS Servo Test Node (End Effector Up/Down)
===========================================
Standalone ROS2 test node for the MKS SERVO42D/57D end effector
using python-can (same as your existing MKSEndEffectorNode).

Mechanical setup (from your project):
  - 6 motor revolutions → 35 mm shaft travel
  - CW  (positive counts) = UP
  - CCW (negative counts) = DOWN
  - URDF ee_joint axis (0,0,-1): positive = world -Z (down)
  - direction_sign = -1  (CCW for "down")

Usage:
  ros2 run SCARA_pkg mks_servo_test

Keyboard controls (press Enter after each):
  u  → Move UP   (+35mm, raises end effector)
  d  → Move DOWN (-35mm, lowers end effector)
  z  → Set current position as ZERO
  s  → Emergency STOP
  q  → Quit

Requirements:
  pip install python-can
  sudo ip link set can0 up type can bitrate 250000
"""

import struct
import threading
import time

import can
import rclpy
from rclpy.node import Node


# ── CAN CONFIG ────────────────────────────────────────────────────────────────
CAN_INTERFACE  = "can0"
MKS_NODE_ID    = 3            # Your servo's CAN node ID

# ── MECHANICAL CONFIG ─────────────────────────────────────────────────────────
COUNTS_PER_REV    = 16384     # MKS 14-bit encoder (0x4000 per rev)
MOTOR_REVS        = 6.0       # revolutions to travel STROKE_M meters
STROKE_M          = 0.035     # 35 mm full stroke
DIRECTION_SIGN    = 1        # -1: CCW = down (positive ee_joint = down in URDF)
COUNTS_PER_M      = MOTOR_REVS * COUNTS_PER_REV / STROKE_M  # ~2,811,428 counts/m

# ── MOTION PROFILE ────────────────────────────────────────────────────────────
SPEED_RPM    = 300            # 0..3000
ACCELERATION = 2              # 0..255

# ── TEST POSITIONS ────────────────────────────────────────────────────────────
POS_UP_M   =  0.000           # meters (home / raised position)
POS_DOWN_M =  0.035           # meters (fully lowered = 35 mm down in URDF)

# ── INT24 LIMITS ──────────────────────────────────────────────────────────────
INT24_MIN, INT24_MAX = -8388607, 8388607


# ── FRAME HELPERS ─────────────────────────────────────────────────────────────

def crc8_sum(can_id: int, payload: bytes) -> int:
    return (can_id + sum(payload)) & 0xFF


def encode_int24_be(v: int) -> bytes:
    if v < 0:
        v += 1 << 24  # two's complement in 24 bits
    return bytes([(v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


def build_abs_frame(ee_meters: float):
    """Build F5 absolute position frame. Returns (payload, counts)."""
    counts = int(round(DIRECTION_SIGN * ee_meters * COUNTS_PER_M))
    counts = max(INT24_MIN, min(INT24_MAX, counts))
    speed_be = struct.pack('>H', max(0, min(3000, SPEED_RPM)))
    payload = bytes([0xF5]) + speed_be + bytes([ACCELERATION]) + encode_int24_be(counts)
    return payload, counts


def build_set_zero_frame() -> bytes:
    return bytes([0x92])


def build_estop_frame() -> bytes:
    return bytes([0xF7])


# ── TEST NODE ─────────────────────────────────────────────────────────────────

class MksServoTestNode(Node):
    def __init__(self):
        super().__init__('mks_servo_test_node')

        # Open CAN bus
        try:
            self.bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
        except OSError as e:
            self.get_logger().fatal(
                f"Cannot open CAN '{CAN_INTERFACE}': {e}\n"
                f"Run: sudo ip link set {CAN_INTERFACE} up type can bitrate 250000")
            raise

        self._tx_lock = threading.Lock()

        self.get_logger().info('MKS Servo Test Node started.')
        self.get_logger().info(f'CAN Interface  : {CAN_INTERFACE}')
        self.get_logger().info(f'MKS Node ID    : {MKS_NODE_ID}')
        self.get_logger().info(f'Speed          : {SPEED_RPM} RPM')
        self.get_logger().info(f'Counts/m       : {int(COUNTS_PER_M)}')
        self.get_logger().info(
            f'UP  position   : {POS_UP_M*1000:.1f} mm → '
            f'{int(round(DIRECTION_SIGN * POS_UP_M * COUNTS_PER_M))} counts')
        self.get_logger().info(
            f'DOWN position  : {POS_DOWN_M*1000:.1f} mm → '
            f'{int(round(DIRECTION_SIGN * POS_DOWN_M * COUNTS_PER_M))} counts')

        # Start keyboard thread
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

    # ── CAN SEND ──────────────────────────────────────────────────────────────

    def _send(self, payload: bytes, label: str):
        crc  = crc8_sum(MKS_NODE_ID, payload)
        data = payload + bytes([crc])
        msg  = can.Message(arbitration_id=MKS_NODE_ID,
                           data=data, is_extended_id=False)
        with self._tx_lock:
            try:
                self.bus.send(msg, timeout=0.1)
                self.get_logger().info(
                    f'[{label}] → id=0x{MKS_NODE_ID:03X}  data={data.hex().upper()}')
            except can.CanError as e:
                self.get_logger().error(f'[{label}] CAN send failed: {e}')

    # ── COMMANDS ──────────────────────────────────────────────────────────────

    def move_up(self):
        payload, counts = build_abs_frame(POS_UP_M)
        self._send(payload, f'UP   {POS_UP_M*1000:.1f}mm counts={counts:+d}')

    def move_down(self):
        payload, counts = build_abs_frame(POS_DOWN_M)
        self._send(payload, f'DOWN {POS_DOWN_M*1000:.1f}mm counts={counts:+d}')

    def set_zero(self):
        self._send(build_set_zero_frame(), 'SET_ZERO')

    def estop(self):
        self._send(build_estop_frame(), 'ESTOP')

    # ── KEYBOARD LOOP ─────────────────────────────────────────────────────────

    def _keyboard_loop(self):
        print("\n[KEYBOARD] Controls:")
        print("  u + Enter  →  Move UP   (0 mm)")
        print("  d + Enter  →  Move DOWN (35 mm)")
        print("  z + Enter  →  Set current position as ZERO")
        print("  s + Enter  →  Emergency STOP")
        print("  q + Enter  →  Quit\n")

        while rclpy.ok():
            try:
                key = input().strip().lower()
            except EOFError:
                break

            if key == 'u':
                self.move_up()
            elif key == 'd':
                self.move_down()
            elif key == 'z':
                self.set_zero()
            elif key == 's':
                self.estop()
            elif key == 'q':
                print('[KEYBOARD] Quitting...')
                self.estop()
                rclpy.shutdown()
                break
            else:
                print(f"[KEYBOARD] Unknown key '{key}'. Use: u / d / z / s / q")

    # ── CLEANUP ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MksServoTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print('[SHUTDOWN] Sending emergency stop...')
        node.estop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()