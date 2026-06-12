"""
Pick-and-Place Demo for new scara_sim URDF.  (v2 - markers in world frame)
==========================================================================
Author: SARIN Chandevid

Same as v1 but marker positions are corrected for the shoulder offset
of the new URDF (joint_0 at xyz=0.0125, -0.1035, 0.11125 from base_link).

Reach's IK frame has its origin at the SHOULDER, so to draw markers at the
correct world position we add the shoulder offset.
"""

import math

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


# =====================================================================
# CONFIGURATION
# =====================================================================

# Object pick location  (in Reach's IK frame: origin at shoulder, +Y forward)
PICK_X = -0.08
PICK_Y =  0.25

# Place location (drop-off zone)
PLACE_X =  0.08
PLACE_Y =  0.20

# Spindle heights (joint_2, range 0..0.105)
SPINDLE_UP   = 0.00
SPINDLE_DOWN = 0.08

# Phase durations (seconds)
T_APPROACH  = 3.0
T_DESCEND   = 1.5
T_DWELL     = 0.5
T_LIFT      = 1.5
T_TRANSPORT = 3.0
T_PLACE     = 1.5
T_RELEASE   = 0.5
T_RETREAT   = 1.5
T_HOME      = 3.0

# Home pose
HOME_J0 = 0.0
HOME_J1 = 0.0

# IK link lengths (match Reach's ikpos.py)
A1 = 0.15
A2 = 0.15

# Workspace box
X_MIN, X_MAX = -0.159, 0.159
Y_MIN, Y_MAX = 0.100, 0.390

# === SHOULDER offset (from new URDF, joint_0 origin) ===
# joint_0 origin in base_link/world frame is (0.0125, -0.1035, 0.11125)
# Reach's IK is in a frame with origin at the SHOULDER.
# So to convert an IK (x,y) point to world frame for marker drawing:
#     world_x = x + SHOULDER_X
#     world_y = y + SHOULDER_Y
SHOULDER_X =  0.0125
SHOULDER_Y = -0.1035
SHOULDER_Z =  0.11125

# Topics
TRAJECTORY_TOPIC = "/scara_arm_controller/joint_trajectory"
GRIPPER_TOPIC    = "/gripper_command"
MARKER_TOPIC     = "/workspace_markers"
JOINT_NAMES = ["joint_0", "joint_1", "joint_2"]


# =====================================================================
# IK + frame transform
# =====================================================================

def reach_ik(x, y):
    if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX):
        return None
    r2 = x * x + y * y
    c2 = (r2 - A1*A1 - A2*A2) / (2.0 * A1 * A2)
    c2 = max(-1.0, min(1.0, c2))
    theta2 = math.acos(c2)
    theta1 = math.atan2(y, x) - math.atan2(
        A2*math.sin(theta2), A1 + A2*math.cos(theta2)
    )
    return theta1, theta2


def to_urdf(theta1, theta2):
    return math.pi/2.0 - theta1, theta2


def joints_for_xy(x, y):
    sol = reach_ik(x, y)
    if sol is None:
        return None
    return to_urdf(*sol)


def ik_to_world(x_ik, y_ik):
    """Convert a point in Reach's IK frame to world frame for marker drawing."""
    return x_ik + SHOULDER_X, y_ik + SHOULDER_Y


# =====================================================================
# Node
# =====================================================================

class PickPlaceDemo(Node):

    def __init__(self):
        super().__init__("pick_place_demo")
        self._traj_pub = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)
        self._grip_pub = self.create_publisher(Bool, GRIPPER_TOPIC, 10)
        self._marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, 10)

        traj = self._build_trajectory()
        if traj is None:
            self.get_logger().error("Pick or place location unreachable.")
            return

        self._traj = traj
        self._published = False
        self._timer = self.create_timer(1.0, self._publish_once)
        self._marker_timer = self.create_timer(1.0, self._publish_markers)

    def _build_trajectory(self):
        pick_joints = joints_for_xy(PICK_X, PICK_Y)
        place_joints = joints_for_xy(PLACE_X, PLACE_Y)
        if pick_joints is None or place_joints is None:
            return None
        pj0, pj1 = pick_joints
        qj0, qj1 = place_joints

        sequence = []
        t = 0.0
        t += T_APPROACH;  sequence.append((pj0, pj1, SPINDLE_UP,   t, None))
        t += T_DESCEND;   sequence.append((pj0, pj1, SPINDLE_DOWN, t, None))
        t += T_DWELL;     sequence.append((pj0, pj1, SPINDLE_DOWN, t, "close"))
        t += T_LIFT;      sequence.append((pj0, pj1, SPINDLE_UP,   t, None))
        t += T_TRANSPORT; sequence.append((qj0, qj1, SPINDLE_UP,   t, None))
        t += T_PLACE;     sequence.append((qj0, qj1, SPINDLE_DOWN, t, None))
        t += T_RELEASE;   sequence.append((qj0, qj1, SPINDLE_DOWN, t, "open"))
        t += T_RETREAT;   sequence.append((qj0, qj1, SPINDLE_UP,   t, None))
        t += T_HOME;      sequence.append((HOME_J0, HOME_J1, SPINDLE_UP, t, None))

        self._gripper_events = [
            (time_s, action) for (_,_,_, time_s, action) in sequence
            if action is not None
        ]

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        for (j0, j1, j2, time_s, _) in sequence:
            pt = JointTrajectoryPoint()
            pt.positions = [j0, j1, j2]
            pt.time_from_start = Duration(
                sec=int(time_s),
                nanosec=int((time_s - int(time_s)) * 1e9),
            )
            traj.points.append(pt)

        # Compute world-frame positions for the markers
        pwx, pwy = ik_to_world(PICK_X, PICK_Y)
        qwx, qwy = ik_to_world(PLACE_X, PLACE_Y)
        self.get_logger().info(
            f"Pick IK ({PICK_X:+.3f},{PICK_Y:.3f}) -> world ({pwx:+.3f},{pwy:.3f})"
        )
        self.get_logger().info(
            f"Place IK ({PLACE_X:+.3f},{PLACE_Y:.3f}) -> world ({qwx:+.3f},{qwy:.3f})"
        )
        self.get_logger().info(
            f"Total motion: {t:.1f} s, {len(traj.points)} waypoints."
        )
        return traj

    def _publish_once(self):
        if self._published:
            return
        self._traj_pub.publish(self._traj)
        self._published = True
        self.get_logger().info("Trajectory published. The robot is moving.")
        for time_s, action in self._gripper_events:
            self.create_timer(
                time_s + 1.0,
                lambda a=action, t=time_s: self._fire_gripper_once(a, t),
            )

    def _fire_gripper_once(self, action, t):
        attr = f"_fired_{int(t*1000)}"
        if getattr(self, attr, False):
            return
        setattr(self, attr, True)
        msg = Bool()
        msg.data = (action == "close")
        self._grip_pub.publish(msg)
        word = "CLOSE (grasp)" if action == "close" else "OPEN (release)"
        self.get_logger().info(f"  t={t:5.1f}s  gripper -> {word}")

    def _publish_markers(self):
        ma = MarkerArray()

        # Convert IK positions to world for drawing
        pwx, pwy = ik_to_world(PICK_X, PICK_Y)
        qwx, qwy = ik_to_world(PLACE_X, PLACE_Y)

        # Red cube at PICK
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "pick_object"; m.id = 0
        m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.position.x = pwx
        m.pose.position.y = pwy
        m.pose.position.z = 0.015
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03; m.scale.y = 0.03; m.scale.z = 0.03
        m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 1.0
        ma.markers.append(m)

        # Green circle at PLACE
        m2 = Marker()
        m2.header.frame_id = "world"
        m2.header.stamp = self.get_clock().now().to_msg()
        m2.ns = "place_zone"; m2.id = 1
        m2.type = Marker.CYLINDER; m2.action = Marker.ADD
        m2.pose.position.x = qwx
        m2.pose.position.y = qwy
        m2.pose.position.z = 0.001
        m2.pose.orientation.w = 1.0
        m2.scale.x = 0.06; m2.scale.y = 0.06; m2.scale.z = 0.002
        m2.color.r = 0.2; m2.color.g = 0.9; m2.color.b = 0.3; m2.color.a = 0.7
        ma.markers.append(m2)

        self._marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceDemo()
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
