"""
ik_target / angle_cmd -> RViz Controller   v4 - with spindle dip
=================================================================
Author: SARIN Chandevid

For the new URDF (Robot_v2 clean export). Subscribes to two input topics
from Reach's hardware:

  1. /ik_target            (Float32MultiArray [x, y])     - Cartesian targets
  2. /odrive/angle_cmd     (Float32MultiArray [1.0, t1, 2.0, t2])  - raw angles

For each input, the arm moves to the (x, y) position and the spindle
(joint_2) performs a quick 30 mm dip - down then back up - as if picking
or placing a cube.

Frame transform for new URDF:
    joint_0 = pi/2 - theta1
    joint_1 = theta2 + visual offset
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# =====================================================================
# CONFIGURATION
# =====================================================================

A1 = 0.15
A2 = 0.15
X_MIN, X_MAX = -0.159, 0.159
Y_MIN, Y_MAX = 0.100, 0.390
THETA_LIMIT = math.radians(140)

TRAJECTORY_TOPIC = "/scara_arm_controller/joint_trajectory"
JOINT_NAMES = ["joint_0", "joint_1", "joint_2"]

IK_TARGET_TOPIC = "/ik_target"
ANGLE_CMD_TOPIC = "/odrive/angle_cmd"

# --- Spindle dip parameters ---
SPINDLE_UP   = 0.0       # joint_2 fully up (0 m)
SPINDLE_DOWN = 0.030     # 30 mm dip (your spec)

# --- Timing for the spindle dip - matches the real robot ---
# After the arm reaches the target, the spindle:
#   - goes DOWN over 3 seconds
#   - stays at the bottom for 2 seconds (gripper actuates here on real robot)
#   - goes back UP over 3 seconds
T_REACH         = 2.0    # arm reaches the target
T_AT_BOTTOM     = T_REACH + 3.0       # 3 s down  -> t = 5.0 s
T_DWELL_END     = T_AT_BOTTOM + 2.0   # 2 s dwell -> t = 7.0 s
T_BACK_UP       = T_DWELL_END + 3.0   # 3 s up    -> t = 10.0 s

# Visual cosmetic offset for joint_1 mesh
JOINT_1_VISUAL_OFFSET = math.radians(12.0)

ELBOW_SIGN = +1.0


# =====================================================================
# IK + frame transform
# =====================================================================

def reach_ik(x, y):
    if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX):
        return None
    r2 = x * x + y * y
    c2 = (r2 - A1*A1 - A2*A2) / (2.0 * A1 * A2)
    c2 = max(-1.0, min(1.0, c2))
    theta2 = ELBOW_SIGN * math.acos(c2)
    theta1 = math.atan2(y, x) - math.atan2(
        A2 * math.sin(theta2), A1 + A2 * math.cos(theta2)
    )
    return theta1, theta2


def to_urdf_angles(theta1, theta2):
    """joint_0 = pi/2 - theta1, joint_1 = theta2 + visual offset."""
    return math.pi / 2.0 - theta1, theta2 + JOINT_1_VISUAL_OFFSET


def within_limits(theta1, theta2):
    return abs(theta1) <= THETA_LIMIT and abs(theta2) <= THETA_LIMIT


def make_duration(seconds):
    return Duration(
        sec=int(seconds),
        nanosec=int((seconds - int(seconds)) * 1e9),
    )


# =====================================================================
# Node
# =====================================================================

class IkToRvizController(Node):

    def __init__(self):
        super().__init__("ik_to_rviz_controller")

        self._traj_pub = self.create_publisher(
            JointTrajectory, TRAJECTORY_TOPIC, 10
        )
        self.create_subscription(
            Float32MultiArray, IK_TARGET_TOPIC, self._on_ik_target, 10
        )
        self.create_subscription(
            Float32MultiArray, ANGLE_CMD_TOPIC, self._on_angle_cmd, 10
        )

        self._count_ik = 0
        self._count_angle = 0

        self.get_logger().info("=" * 64)
        self.get_logger().info("ik_target / angle_cmd -> RViz Controller v4")
        self.get_logger().info(f"  Input 1: {IK_TARGET_TOPIC}  (Cartesian x, y)")
        self.get_logger().info(f"  Input 2: {ANGLE_CMD_TOPIC}  (raw theta1, theta2 in RAD)")
        self.get_logger().info(f"  Output:  {TRAJECTORY_TOPIC}")
        self.get_logger().info(f"  Joints:  {JOINT_NAMES}")
        self.get_logger().info(f"  Spindle motion: down 3 s -> dwell 2 s -> up 3 s "
                               f"(dip = {SPINDLE_DOWN*1000:.0f} mm)")
        self.get_logger().info("=" * 64)

    # ---------- Input 1: /ik_target ----------
    def _on_ik_target(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn(
                f"/ik_target needs [x, y] - got {len(msg.data)} values"
            )
            return

        x = float(msg.data[0])
        y = float(msg.data[1])
        self._count_ik += 1

        sol = reach_ik(x, y)
        if sol is None:
            self.get_logger().warn(
                f"[IK #{self._count_ik}] ({x:.3f},{y:.3f}) outside workspace"
            )
            return

        theta1, theta2 = sol
        self._dispatch(theta1, theta2, source=f"IK #{self._count_ik}",
                       extra=f"from ({x:+.3f},{y:.3f})")

    # ---------- Input 2: /odrive/angle_cmd ----------
    def _on_angle_cmd(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn(
                f"/odrive/angle_cmd needs [1.0, t1, 2.0, t2] - "
                f"got {len(msg.data)} values"
            )
            return

        theta1 = float(msg.data[1])
        theta2 = float(msg.data[3])
        self._count_angle += 1

        if not within_limits(theta1, theta2):
            self.get_logger().warn(
                f"[ANG #{self._count_angle}] angles outside safety limits"
            )
            return

        self._dispatch(theta1, theta2, source=f"ANG #{self._count_angle}",
                       extra="direct angles")

    # ---------- Common dispatch: 4-point trajectory matching real robot ----------
    def _dispatch(self, theta1, theta2, source, extra):
        j0, j1 = to_urdf_angles(theta1, theta2)

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        # Point 1: arm reaches target, spindle UP (t = 2s)
        p1 = JointTrajectoryPoint()
        p1.positions = [j0, j1, SPINDLE_UP]
        p1.time_from_start = make_duration(T_REACH)
        traj.points.append(p1)

        # Point 2: spindle DOWN 30 mm (3 s after reach -> t = 5s)
        p2 = JointTrajectoryPoint()
        p2.positions = [j0, j1, SPINDLE_DOWN]
        p2.time_from_start = make_duration(T_AT_BOTTOM)
        traj.points.append(p2)

        # Point 3: stay at bottom (dwell 2 s -> t = 7s)
        p3 = JointTrajectoryPoint()
        p3.positions = [j0, j1, SPINDLE_DOWN]
        p3.time_from_start = make_duration(T_DWELL_END)
        traj.points.append(p3)

        # Point 4: spindle back UP (3 s after dwell -> t = 10s)
        p4 = JointTrajectoryPoint()
        p4.positions = [j0, j1, SPINDLE_UP]
        p4.time_from_start = make_duration(T_BACK_UP)
        traj.points.append(p4)

        self._traj_pub.publish(traj)

        self.get_logger().info(
            f"[{source}] {extra}  "
            f"Reach: t1={math.degrees(theta1):+6.2f} t2={math.degrees(theta2):+6.2f}   "
            f"URDF: j0={math.degrees(j0):+6.2f} j1={math.degrees(j1):+6.2f}   "
            f"(+ spindle dip)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IkToRvizController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f"Stopped. IK: {node._count_ik}, angle: {node._count_angle}"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
