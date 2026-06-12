"""
Move SCARA to Home Position
============================
Author: SARIN Chandevid

Sends one JointTrajectory to move the simulated SCARA arm to its home pose:
    joint_0 = 0.0   (arm pointing straight forward, along +Y)
    joint_1 = 0.0   (elbow straight)
    joint_2 = 0.0   (spindle fully up)

Run with the sim already launched:
    python3 home_position.py
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# =====================================================================
# CONFIGURATION
# =====================================================================

# Home pose (radians for revolute, meters for prismatic)
HOME_J0 = 0.0
HOME_J1 = 0.0
HOME_J2 = 0.0

# Time to reach home (seconds)
MOVE_TIME = 3.0

# Sim controller + joint names (must match scara_sim controllers.yaml)
TRAJECTORY_TOPIC = "/scara_arm_controller/joint_trajectory"
JOINT_NAMES = ["joint_0", "joint_1", "joint_2"]


class HomeMover(Node):

    def __init__(self):
        super().__init__("home_mover")
        self._pub = self.create_publisher(
            JointTrajectory, TRAJECTORY_TOPIC, 10
        )
        # Small delay so subscribers attach before we publish
        self._timer = self.create_timer(1.0, self._publish_once)
        self._published = False

        self.get_logger().info("Home mover started.")
        self.get_logger().info(
            f"Will send the arm to home (j0={HOME_J0}, j1={HOME_J1}, "
            f"j2={HOME_J2}) over {MOVE_TIME:.1f} s..."
        )

    def _publish_once(self):
        if self._published:
            return

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = [HOME_J0, HOME_J1, HOME_J2]
        pt.time_from_start = Duration(
            sec=int(MOVE_TIME),
            nanosec=int((MOVE_TIME - int(MOVE_TIME)) * 1e9),
        )
        traj.points.append(pt)

        self._pub.publish(traj)
        self._published = True
        self.get_logger().info(
            "Home trajectory published. The arm should be returning to home now."
        )
        self.get_logger().info(
            f"You can stop this node any time with Ctrl+C "
            f"(motion continues for {MOVE_TIME:.1f} s)."
        )


def main(args=None):
    rclpy.init(args=args)
    node = HomeMover()
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
