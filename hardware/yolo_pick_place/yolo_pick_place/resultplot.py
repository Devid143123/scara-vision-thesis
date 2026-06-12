# """
# Physical SCARA Test Node
# ========================================================================
# Author: SAR Seihak Reach

# PURPOSE
# -------
# Drive the REAL SCARA robot through a square trajectory with a spindle dip
# at each corner, and record the commanded joint state over time to a CSV
# file so the physical-robot result can be plotted and compared against the
# simulation result.

# IMPORTANT - WHAT THIS LOGS
# --------------------------
# The existing SCARA software only ever SENDS commands; nothing publishes the
# real motor encoder positions back to ROS 2. Therefore this node records the
# COMMANDED joint values (the same values the robot is driven with), not values
# read from sensors. The two arm angles are produced by the same inverse
# kinematics used by the main control node (reach_ik + to_urdf), and the
# prismatic displacement is the commanded stepper position. This is the same
# kind of data the simulation plots are built from, and must be described as
# "commanded" in the thesis, not "measured".

# HOW IT WORKS
# ------------
#   * Publishes Cartesian targets [x, y] to  /ik_target  (Float32MultiArray),
#     exactly like the main node, so the arm moves through your real pipeline.
#   * Drives the prismatic stepper directly over CAN, exactly like the main node.
#   * Every LOG_PERIOD_S seconds it appends one row to the CSV:
#         t, x, y, theta1, theta2, d3, phase
#   * Velocities are NOT stored here; they are computed later from the positions
#     by plot_physical.py (numerical differentiation), matching the simulation.

# RUN
# ---
#   1. Make sure the IK node (the one that subscribes to /ik_target) is running,
#      and the CAN bus is up:
#         sudo ip link set can0 up type can bitrate 250000   (use your bitrate)
#   2. Then:
#         python3 physical_test_node.py
#   3. The robot homes, traces the square with four corner dips, then parks.
#      A CSV is written to  ./scara_physical_square.csv
# """

# import math
# import csv
# import time
# import struct
# import threading

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray

# import can


# # =====================================================================
# # SQUARE / MOTION CONFIGURATION
# # ---------------------------------------------------------------------
# # These mirror the simulation square_demo so the physical and simulated
# # results are directly comparable. Adjust CENTER/SIDE if your reachable
# # workspace needs it.
# # =====================================================================
# CENTER_X = 0.0
# CENTER_Y = 0.25          # metres, in front of the base
# SIDE     = 0.10          # 100 mm square
# SAMPLES_PER_EDGE = 20    # how many intermediate targets per edge

# # Timing (seconds) - generous so the real robot fully arrives at each step
# MOVE_FROM_HOME_S = 1.0
# EDGE_DURATION_S  = 6.0
# DIP_DOWN_S       = 4.0   # real stepper travel is slow; match STEPPER_TRAVEL_S
# DIP_DWELL_S      = 0.5
# DIP_UP_S         = 4.0

# # Spindle dip depth for the test (metres). Kept small and within the real
# # 35 mm stroke. 0.020 = 20 mm, same as the simulation demo.
# SPINDLE_UP   = 0.000
# SPINDLE_DOWN = 0.020

# # Logging period
# LOG_PERIOD_S = 0.05      # 20 Hz, plenty for smooth velocity curves
# CSV_PATH     = "scara_physical_square.csv"


# # =====================================================================
# # INVERSE KINEMATICS  (copied verbatim from your main node so the logged
# # angles are exactly what the robot is commanded with)
# # =====================================================================
# A1 = 0.15
# A2 = 0.15
# X_MIN, X_MAX = -0.159, 0.159
# Y_MIN, Y_MAX = 0.100, 0.390
# JOINT_1_VISUAL_OFFSET = math.radians(12.0)


# def reach_ik(x, y):
#     if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX):
#         return None
#     r2 = x * x + y * y
#     c2 = max(-1.0, min(1.0, (r2 - A1 * A1 - A2 * A2) / (2 * A1 * A2)))
#     t2 = math.acos(c2)
#     t1 = math.atan2(y, x) - math.atan2(A2 * math.sin(t2), A1 + A2 * math.cos(t2))
#     return t1, t2


# def to_urdf(theta1, theta2):
#     return math.pi / 2.0 - theta1, theta2 + JOINT_1_VISUAL_OFFSET


# def joints_at(x, y):
#     """Return (j0, j1) URDF angles for a workspace point, or None if unreachable."""
#     sol = reach_ik(x, y)
#     if sol is None:
#         return None
#     return to_urdf(*sol)


# # =====================================================================
# # CAN / STEPPER CONFIG  (copied from your main node)
# # =====================================================================
# CAN_INTERFACE  = "can0"
# MKS_NODE_ID    = 3
# COUNTS_PER_REV = 16384
# MOTOR_REVS     = 6.0
# STROKE_M       = 0.035          # real physical stroke = 35 mm
# DIRECTION_SIGN = 1
# COUNTS_PER_M   = MOTOR_REVS * COUNTS_PER_REV / STROKE_M

# SPEED_RPM      = 300
# ACCELERATION   = 2
# INT24_MIN, INT24_MAX = -8388607, 8388607

# POS_UP_M   = 0.000
# POS_DOWN_M = 0.035


# def crc8_sum(can_id, payload):
#     return (can_id + sum(payload)) & 0xFF


# def encode_int24_be(v):
#     if v < 0:
#         v += 1 << 24
#     return bytes([(v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


# def build_abs_frame(ee_meters):
#     counts = int(round(DIRECTION_SIGN * ee_meters * COUNTS_PER_M))
#     counts = max(INT24_MIN, min(INT24_MAX, counts))
#     speed_be = struct.pack('>H', max(0, min(3000, SPEED_RPM)))
#     payload = bytes([0xF5]) + speed_be + bytes([ACCELERATION]) + encode_int24_be(counts)
#     return payload, counts


# def build_set_zero_frame():
#     return bytes([0x92])


# def build_estop_frame():
#     return bytes([0xF7])


# # =====================================================================
# # WAYPOINT / SCHEDULE BUILDER
# # ---------------------------------------------------------------------
# # We build a time-ordered schedule of (time_from_start, x, y, spindle, phase)
# # entries. The node then plays this schedule against wall-clock time: when a
# # new entry's time arrives it sends the XY target and the stepper command.
# # Between entries the logger keeps recording the most recent commanded values
# # so the CSV is dense and smooth.
# # =====================================================================
# def square_corners():
#     h = SIDE / 2.0
#     return [
#         (CENTER_X - h, CENTER_Y - h),   # bottom-left
#         (CENTER_X + h, CENTER_Y - h),   # bottom-right
#         (CENTER_X + h, CENTER_Y + h),   # top-right
#         (CENTER_X - h, CENTER_Y + h),   # top-left
#     ]


# def interpolate_edge(start, end, n):
#     pts = []
#     for s in range(1, n + 1):
#         t = s / n
#         x = start[0] + t * (end[0] - start[0])
#         y = start[1] + t * (end[1] - start[1])
#         pts.append((x, y))
#     return pts


# def build_schedule():
#     """Return a list of (t_from_start, x, y, spindle_m, phase_label).

#     Returns None if any waypoint is outside the reachable workspace.
#     """
#     corners = square_corners()
#     cycle = corners + [corners[0]]
#     sched = []
#     t = 0.0

#     # Phase 1: home -> first corner (spindle up)
#     c0 = corners[0]
#     if joints_at(*c0) is None:
#         print(f"[ERROR] Corner 0 {c0} is out of workspace.")
#         return None
#     t += MOVE_FROM_HOME_S
#     sched.append((t, c0[0], c0[1], SPINDLE_UP, "move_from_home"))

#     # Phase 2: at each corner, dip the spindle, then trace the edge.
#     for k in range(4):
#         corner = corners[k]
#         nxt = cycle[k + 1]
#         if joints_at(*corner) is None:
#             print(f"[ERROR] Corner {k} out of workspace.")
#             return None

#         # spindle DOWN
#         t += DIP_DOWN_S
#         sched.append((t, corner[0], corner[1], SPINDLE_DOWN, f"dip_down_c{k}"))
#         # dwell
#         t += DIP_DWELL_S
#         sched.append((t, corner[0], corner[1], SPINDLE_DOWN, f"dwell_c{k}"))
#         # spindle UP
#         t += DIP_UP_S
#         sched.append((t, corner[0], corner[1], SPINDLE_UP, f"dip_up_c{k}"))

#         # trace edge to next corner (spindle up)
#         edge_pts = interpolate_edge(corner, nxt, SAMPLES_PER_EDGE)
#         for i, (x, y) in enumerate(edge_pts):
#             if joints_at(x, y) is None:
#                 print(f"[ERROR] Edge {k} waypoint {i} ({x:.3f},{y:.3f}) unreachable.")
#                 return None
#             t_step = (t - 0) + (i + 1) * (EDGE_DURATION_S / SAMPLES_PER_EDGE)
#             sched.append((t_step, x, y, SPINDLE_UP, f"edge_c{k}"))
#         t += EDGE_DURATION_S

#     return sched


# # =====================================================================
# # NODE
# # =====================================================================
# class PhysicalTestNode(Node):
#     def __init__(self):
#         super().__init__("physical_test_node")

#         self.ik_pub = self.create_publisher(Float32MultiArray, "/ik_target", 10)

#         # CAN bus for the stepper
#         self._tx_lock = threading.Lock()
#         self.can_ok = False
#         try:
#             self.bus = can.interface.Bus(channel=CAN_INTERFACE, bustype="socketcan")
#             self.can_ok = True
#             self.get_logger().info(f"CAN bus opened on {CAN_INTERFACE}.")
#         except OSError as e:
#             self.bus = None
#             self.get_logger().error(
#                 f"CAN open failed: {e}. Continuing WITHOUT stepper "
#                 f"(prismatic column will be logged as commanded but not moved).")

#         # Anchor the stepper at zero (highest) like the main node does
#         self.stepper_pos_m = 0.0
#         if self.can_ok:
#             self._send_can(build_set_zero_frame(), "SET_ZERO @ startup")
#             time.sleep(0.05)
#             payload, _ = build_abs_frame(0.0)
#             self._send_can(payload, "POS=0 (anchor)")

#         # Build the motion schedule
#         self.schedule = build_schedule()
#         if self.schedule is None:
#             self.get_logger().error("Schedule build failed. Shutting down.")
#             self.failed = True
#             return
#         self.failed = False
#         self.total_time = self.schedule[-1][0]
#         self.get_logger().info(
#             f"Schedule built: {len(self.schedule)} steps, "
#             f"total {self.total_time:.1f} s.")

#         # Current commanded state (updated as the schedule plays)
#         self.cur_x = CENTER_X
#         self.cur_y = CENTER_Y
#         self.cur_spindle = SPINDLE_UP
#         self.cur_phase = "start"

#         # CSV
#         self.rows = []   # collected in memory, written at the end
#         self.start_wall = None
#         self.next_step_idx = 0

#         # Send the robot to a defined home/first point before timing starts.
#         # The IK home in your main node is published as joint cmd, but here we
#         # just command the first corner as the start so the arm is in a known
#         # place. We give it MOVE_FROM_HOME_S inside the schedule.
#         self.get_logger().info("Starting in 2 s. Make sure the IK node is running.")
#         time.sleep(2.0)
#         self.start_wall = time.time()

#         # Timers: one fast logger, one scheduler (can share, but keep clear)
#         self.log_timer = self.create_timer(LOG_PERIOD_S, self._log_tick)
#         self.sched_timer = self.create_timer(0.02, self._sched_tick)

#         self.done = False

#     # -----------------------------------------------------------------
#     def _send_can(self, payload, label):
#         if not self.can_ok:
#             return
#         crc = crc8_sum(MKS_NODE_ID, payload)
#         data = payload + bytes([crc])
#         msg = can.Message(arbitration_id=MKS_NODE_ID, data=data,
#                           is_extended_id=False)
#         with self._tx_lock:
#             try:
#                 self.bus.send(msg, timeout=0.1)
#             except can.CanError as e:
#                 self.get_logger().error(f"[{label}] CAN send failed: {e}")

#     def _stepper_move_to(self, z_m):
#         z_m = max(POS_UP_M, min(POS_DOWN_M, z_m))
#         payload, _ = build_abs_frame(z_m)
#         self.stepper_pos_m = z_m
#         self._send_can(payload, f"Z={z_m*1000:+.1f}mm")

#     # -----------------------------------------------------------------
#     def _sched_tick(self):
#         """Send the next scheduled XY/spindle command when its time arrives."""
#         if self.done or self.start_wall is None:
#             return
#         elapsed = time.time() - self.start_wall

#         # Fire every schedule step whose time has passed
#         while (self.next_step_idx < len(self.schedule)
#                and self.schedule[self.next_step_idx][0] <= elapsed):
#             t_step, x, y, spindle, phase = self.schedule[self.next_step_idx]

#             # XY target -> /ik_target (same as the main node)
#             self.ik_pub.publish(Float32MultiArray(data=[float(x), float(y)]))

#             # Spindle -> CAN, only if it changed (avoid spamming the bus)
#             if abs(spindle - self.cur_spindle) > 1e-6:
#                 self._stepper_move_to(spindle)

#             self.cur_x = x
#             self.cur_y = y
#             self.cur_spindle = spindle
#             self.cur_phase = phase
#             self.next_step_idx += 1

#         # Finished?
#         if elapsed >= self.total_time and not self.done:
#             self.done = True
#             self.get_logger().info("Motion complete. Writing CSV and parking.")
#             self._finish()

#     # -----------------------------------------------------------------
#     def _log_tick(self):
#         """Record one CSV row of the current commanded state."""
#         if self.start_wall is None or self.done:
#             return
#         t = time.time() - self.start_wall

#         sol = reach_ik(self.cur_x, self.cur_y)
#         if sol is None:
#             return
#         j0, j1 = to_urdf(*sol)          # URDF joint angles (radians)
#         d3 = self.cur_spindle            # prismatic displacement (metres)

#         self.rows.append({
#             "t": round(t, 4),
#             "x": round(self.cur_x, 5),
#             "y": round(self.cur_y, 5),
#             "theta1": round(j0, 6),
#             "theta2": round(j1, 6),
#             "d3": round(d3, 6),
#             "phase": self.cur_phase,
#         })

#     # -----------------------------------------------------------------
#     def _finish(self):
#         # Write CSV
#         if self.rows:
#             with open(CSV_PATH, "w", newline="") as f:
#                 writer = csv.DictWriter(
#                     f, fieldnames=["t", "x", "y", "theta1", "theta2", "d3", "phase"])
#                 writer.writeheader()
#                 writer.writerows(self.rows)
#             self.get_logger().info(
#                 f"Wrote {len(self.rows)} rows to {CSV_PATH}")
#         else:
#             self.get_logger().warn("No rows recorded - CSV not written.")

#         # Park: retract stepper and move to a safe spot
#         self._stepper_move_to(POS_UP_M)
#         time.sleep(0.2)
#         self.ik_pub.publish(Float32MultiArray(data=[0.0, 0.30]))
#         self.get_logger().info("Parked. You can Ctrl+C now.")

#     # -----------------------------------------------------------------
#     def destroy_node(self):
#         try:
#             if self.can_ok and self.bus is not None:
#                 self._send_can(build_estop_frame(), "ESTOP @ shutdown")
#                 self.bus.shutdown()
#         except Exception:
#             pass
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = PhysicalTestNode()
#     if getattr(node, "failed", False):
#         node.destroy_node()
#         rclpy.shutdown()
#         return
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()



























# #!/usr/bin/env python3
# """
# ODrive encoder -> ROS 2, reading angle back and converting to DEGREES.

# Inverts the exact linear angle->position map used by odrive_angle_can_node,
# so the angles read here match the convention the arm was commanded in.

# Topics:
#     /scara/measured_angles_deg   Float32MultiArray  [theta1_deg, theta2_deg]
#     /scara/measured_angles_rad   Float32MultiArray  [theta1_rad, theta2_rad]
# """

# import math
# import struct
# import threading

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray

# import can

# CAN_INTERFACE = "can0"
# ENCODER_CMD   = 0x09

# # Node IDs decoded from your command code: 0x02C->node1, 0x04C->node2
# NODE_M1 = 1
# NODE_M2 = 2
# M1_ENC_ID = (NODE_M1 << 5) | ENCODER_CMD   # 0x029
# M2_ENC_ID = (NODE_M2 << 5) | ENCODER_CMD   # 0x049

# # ── Inverse of the maps in odrive_angle_can_node ──
# FACTOR_M1 = 50.0 / math.pi
# OFFSET_M1 = -25.0
# FACTOR_M2 = 50.0 / math.pi
# OFFSET_M2 = -25.0 - FACTOR_M2 * (-math.pi / 2.0)

# def pos_to_rad_m1(position):
#     return (position - OFFSET_M1) / FACTOR_M1

# def pos_to_rad_m2(position):
#     return (position - OFFSET_M2) / FACTOR_M2


# class OdriveAngleReader(Node):
#     def __init__(self):
#         super().__init__("odrive_angle_reader")

#         self.pub_deg = self.create_publisher(
#             Float32MultiArray, "/scara/measured_angles_deg", 10)
#         self.pub_rad = self.create_publisher(
#             Float32MultiArray, "/scara/measured_angles_rad", 10)

#         try:
#             self.bus = can.interface.Bus(channel=CAN_INTERFACE, bustype="socketcan")
#             self.get_logger().info(
#                 f"Listening on {CAN_INTERFACE}: M1 id 0x{M1_ENC_ID:03X}, "
#                 f"M2 id 0x{M2_ENC_ID:03X}.")
#         except OSError as e:
#             self.get_logger().error(f"CAN open failed: {e}")
#             raise

#         self.theta1_rad = 0.0
#         self.theta2_rad = 0.0

#         self._stop = threading.Event()
#         self._rx = threading.Thread(target=self._rx_loop, daemon=True)
#         self._rx.start()

#     def _rx_loop(self):
#         while not self._stop.is_set():
#             msg = self.bus.recv(timeout=0.1)
#             if msg is None or len(msg.data) < 8:
#                 continue

#             if msg.arbitration_id == M1_ENC_ID:
#                 pos, _vel = struct.unpack('<ff', bytes(msg.data))
#                 self.theta1_rad = pos_to_rad_m1(pos)
#                 self._publish()

#             elif msg.arbitration_id == M2_ENC_ID:
#                 pos, _vel = struct.unpack('<ff', bytes(msg.data))
#                 self.theta2_rad = pos_to_rad_m2(pos)
#                 self._publish()

#     def _publish(self):
#         d1 = math.degrees(self.theta1_rad)
#         d2 = math.degrees(self.theta2_rad)
#         self.pub_deg.publish(Float32MultiArray(data=[d1, d2]))
#         self.pub_rad.publish(
#             Float32MultiArray(data=[self.theta1_rad, self.theta2_rad]))

#     def destroy_node(self):
#         self._stop.set()
#         try:
#             self._rx.join(timeout=1.0)
#             self.bus.shutdown()
#         except Exception:
#             pass
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = OdriveAngleReader()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()


















#!/usr/bin/env python3
"""
ODrive encoder -> ROS 2 angle reader (DEGREES), with ALL-ZERO frame rejection.

ROOT CAUSE (confirmed by candump can0,029:7FF)
----------------------------------------------
ID 0x029 carries TWO kinds of 8-byte frame, interleaved:
  * real encoder estimate, e.g.  C1 86 2B C1 00 A0 8C BD  (position ~ -10.7)
  * an all-zero frame:           00 00 00 00 00 00 00 00  (position = 0.0)
The all-zero frame arrives ~2x as often as the real one. A position of 0.0
maps through the motor-1 inverse to exactly pi/2 rad = 90.0 deg, which is the
phantom "90" that was corrupting joint 1.

These zero frames are not measurements (the arm is physically at ~64 deg, not
0 position), so they are discarded. This is frame validation, not smoothing.

Topics:
    /scara/measured_angles_deg   Float32MultiArray  [theta1_deg, theta2_deg]
    /scara/measured_angles_rad   Float32MultiArray  [theta1_rad, theta2_rad]

Run:
    sudo ip link set can0 up type can bitrate 250000   # if not already up
    python3 odrive_angle_reader_filtered.py
    ros2 topic echo /scara/measured_angles_deg --field data
"""

import math
import struct
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import can


# ── CAN / ODrive ──
CAN_INTERFACE = "can0"
ENCODER_CMD   = 0x09
NODE_M1 = 1
NODE_M2 = 2
M1_ENC_ID = (NODE_M1 << 5) | ENCODER_CMD   # 0x029
M2_ENC_ID = (NODE_M2 << 5) | ENCODER_CMD   # 0x049

# ── inverse of the command maps in odrive_angle_can_node ──
FACTOR_M1 = 50.0 / math.pi
OFFSET_M1 = -25.0
FACTOR_M2 = 50.0 / math.pi
OFFSET_M2 = -25.0 - FACTOR_M2 * (-math.pi / 2.0)


def pos_to_rad_m1(position):
    return (position - OFFSET_M1) / FACTOR_M1


def pos_to_rad_m2(position):
    return (position - OFFSET_M2) / FACTOR_M2


# ── secondary spike guard (kept as a backstop) ──
ANGLE_MIN_DEG = -180.0
ANGLE_MAX_DEG =  180.0
MAX_STEP_DEG  =  10.0

# Bytes that mark a junk frame (all zeros).
ZERO_FRAME = bytes(8)


class OdriveAngleReaderFiltered(Node):
    def __init__(self):
        super().__init__("odrive_angle_reader_filtered")

        self.pub_deg = self.create_publisher(
            Float32MultiArray, "/scara/measured_angles_deg", 10)
        self.pub_rad = self.create_publisher(
            Float32MultiArray, "/scara/measured_angles_rad", 10)

        self.theta1_rad = None
        self.theta2_rad = None
        self.zeros_m1 = 0
        self.zeros_m2 = 0
        self.spikes_m1 = 0
        self.spikes_m2 = 0

        try:
            self.bus = can.interface.Bus(
                channel=CAN_INTERFACE, bustype="socketcan")
            self.get_logger().info(
                f"Listening on {CAN_INTERFACE}: M1 0x{M1_ENC_ID:03X}, "
                f"M2 0x{M2_ENC_ID:03X}. Rejecting all-zero frames + "
                f"jumps > {MAX_STEP_DEG} deg.")
        except OSError as e:
            self.get_logger().error(
                f"CAN open failed: {e}\n"
                f"  sudo ip link set {CAN_INTERFACE} up type can bitrate 250000")
            raise

        self._stop = threading.Event()
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()

    # -----------------------------------------------------------------
    def _spike_ok(self, new_deg, last_rad):
        if not math.isfinite(new_deg):
            return False
        if not (ANGLE_MIN_DEG <= new_deg <= ANGLE_MAX_DEG):
            return False
        if last_rad is not None and \
           abs(new_deg - math.degrees(last_rad)) > MAX_STEP_DEG:
            return False
        return True

    # -----------------------------------------------------------------
    def _rx_loop(self):
        while not self._stop.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg is None or len(msg.data) != 8:
                continue

            data = bytes(msg.data)

            # PRIMARY FILTER: drop the all-zero junk frame outright.
            if data == ZERO_FRAME:
                if msg.arbitration_id == M1_ENC_ID:
                    self.zeros_m1 += 1
                elif msg.arbitration_id == M2_ENC_ID:
                    self.zeros_m2 += 1
                continue

            if msg.arbitration_id == M1_ENC_ID:
                try:
                    pos, _vel = struct.unpack('<ff', data)
                except struct.error:
                    continue
                new_rad = pos_to_rad_m1(pos)
                new_deg = math.degrees(new_rad)
                if self._spike_ok(new_deg, self.theta1_rad):
                    self.theta1_rad = new_rad
                    self._publish()
                else:
                    self.spikes_m1 += 1

            elif msg.arbitration_id == M2_ENC_ID:
                try:
                    pos, _vel = struct.unpack('<ff', data)
                except struct.error:
                    continue
                new_rad = pos_to_rad_m2(pos)
                new_deg = math.degrees(new_rad)
                if self._spike_ok(new_deg, self.theta2_rad):
                    self.theta2_rad = new_rad
                    self._publish()
                else:
                    self.spikes_m2 += 1
            # all other ids ignored

    # -----------------------------------------------------------------
    def _publish(self):
        if self.theta1_rad is None or self.theta2_rad is None:
            return
        d1 = math.degrees(self.theta1_rad)
        d2 = math.degrees(self.theta2_rad)
        self.pub_deg.publish(Float32MultiArray(data=[d1, d2]))
        self.pub_rad.publish(
            Float32MultiArray(data=[self.theta1_rad, self.theta2_rad]))

    # -----------------------------------------------------------------
    def destroy_node(self):
        self._stop.set()
        self.get_logger().info(
            f"Rejected zero frames: M1={self.zeros_m1}, M2={self.zeros_m2}. "
            f"Rejected spikes: M1={self.spikes_m1}, M2={self.spikes_m2}.")
        try:
            self._rx.join(timeout=1.0)
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdriveAngleReaderFiltered()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()