"""
Square Demo v3.2 - starts at HOME, with spindle dips at corners
=================================================================
Author: SARIN Chandevid

Motion plan:
    1. Robot starts at HOME (joint_0=0, joint_1=0, joint_2=0).
    2. Moves smoothly from home to the first corner (0.5 s).
    3. At each of the 4 corners: spindle dips down 20 mm
       (1 s down, 0.5 s dwell, 1 s up = 2.5 s per dip).
    4. Between corners: arm traces the edge (5 s per edge).
    5. Total motion: 0.5 + 4 dips * 2.5 + 4 edges * 5 = 30.5 s

This makes joint_2 (d3) actively move so it appears in the graph.

Run with the sim launched:
    python3 square_demo_v3.py
"""

import math
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# =====================================================================
# CONFIGURATION
# =====================================================================

CENTER_X = 0.0
CENTER_Y = 0.25
SIDE     = 0.10
SAMPLES_PER_EDGE = 20

# Timing
MOVE_FROM_HOME_S = 0.5
EDGE_DURATION_S  = 5.0     # time to trace one edge
DIP_DOWN_S       = 1.0
DIP_DWELL_S      = 0.5
DIP_UP_S         = 1.0

# Spindle
SPINDLE_UP   = 0.0
SPINDLE_DOWN = 0.020   # 20 mm

# Home pose
HOME_J0 = 0.0
HOME_J1 = 0.0
HOME_J2 = 0.0

# IK
A1 = 0.15
A2 = 0.15
X_MIN, X_MAX = -0.159, 0.159
Y_MIN, Y_MAX = 0.100, 0.390
JOINT_1_VISUAL_OFFSET = math.radians(12.0)

TRAJECTORY_TOPIC = "/scara_arm_controller/joint_trajectory"
JOINT_NAMES = ["joint_0", "joint_1", "joint_2"]


def reach_ik(x, y):
    if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX):
        return None
    r2 = x*x + y*y
    c2 = max(-1.0, min(1.0, (r2 - A1*A1 - A2*A2)/(2*A1*A2)))
    t2 = math.acos(c2)
    t1 = math.atan2(y, x) - math.atan2(A2*math.sin(t2), A1 + A2*math.cos(t2))
    return t1, t2


def to_urdf(theta1, theta2):
    return math.pi/2.0 - theta1, theta2 + JOINT_1_VISUAL_OFFSET


def joints_at(x, y):
    """Return (j0, j1) for a square point, or None if out of workspace."""
    sol = reach_ik(x, y)
    if sol is None:
        return None
    return to_urdf(*sol)


def make_duration(seconds):
    return Duration(
        sec=int(seconds),
        nanosec=int((seconds - int(seconds)) * 1e9),
    )


def square_corners():
    h = SIDE / 2.0
    return [
        (CENTER_X - h, CENTER_Y - h),   # corner 0 (bottom-left)
        (CENTER_X + h, CENTER_Y - h),   # corner 1 (bottom-right)
        (CENTER_X + h, CENTER_Y + h),   # corner 2 (top-right)
        (CENTER_X - h, CENTER_Y + h),   # corner 3 (top-left)
    ]


def interpolate_edge(start, end, n_samples):
    """Return n_samples points from start to end, NOT including start."""
    pts = []
    for s in range(1, n_samples + 1):
        t = s / n_samples
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        pts.append((x, y))
    return pts


class SquareDemoV3(Node):

    def __init__(self):
        super().__init__("square_demo_v3")
        self._pub = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)
        traj = self._build()
        if traj is None:
            self.get_logger().error("Build failed.")
            return
        self._traj = traj
        self._published = False
        self._timer = self.create_timer(1.0, self._fire)

    def _build(self):
        corners = square_corners()
        cycle = corners + [corners[0]]    # close the loop -> 5 points
        dip_total = DIP_DOWN_S + DIP_DWELL_S + DIP_UP_S
        total = MOVE_FROM_HOME_S + 4 * dip_total + 4 * EDGE_DURATION_S

        self.get_logger().info("=" * 64)
        self.get_logger().info("Square Demo v3.2 - HOME start + corner dips")
        self.get_logger().info(
            f"  Square centre   : ({CENTER_X:.3f}, {CENTER_Y:.3f}) m"
        )
        self.get_logger().info(f"  Square side     : {SIDE*1000:.0f} mm")
        self.get_logger().info(
            f"  Spindle dip     : {SPINDLE_DOWN*1000:.0f} mm "
            f"({DIP_DOWN_S}s down + {DIP_DWELL_S}s dwell + {DIP_UP_S}s up)"
        )
        self.get_logger().info(f"  Edge duration   : {EDGE_DURATION_S}s per edge")
        self.get_logger().info(f"  Total motion    : {total:.1f}s")
        self.get_logger().info("=" * 64)

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        t_now = 0.0

        # --- Phase 1: home -> corner 0 (no dip yet) ---
        c0 = corners[0]
        j = joints_at(*c0)
        if j is None:
            self.get_logger().error(f"Corner 0 {c0} out of workspace.")
            return None
        t_now += MOVE_FROM_HOME_S
        pt = JointTrajectoryPoint()
        pt.positions = [j[0], j[1], SPINDLE_UP]
        pt.time_from_start = make_duration(t_now)
        traj.points.append(pt)

        # --- Phase 2: at each of the 4 corners, dip the spindle, then trace
        #              the next edge ---
        for k in range(4):
            corner = corners[k]
            next_corner = cycle[k + 1]

            j_here = joints_at(*corner)
            if j_here is None:
                self.get_logger().error(f"Corner {k} out of workspace.")
                return None

            # 2a) spindle DOWN
            t_now += DIP_DOWN_S
            pt = JointTrajectoryPoint()
            pt.positions = [j_here[0], j_here[1], SPINDLE_DOWN]
            pt.time_from_start = make_duration(t_now)
            traj.points.append(pt)

            # 2b) dwell at the bottom
            t_now += DIP_DWELL_S
            pt = JointTrajectoryPoint()
            pt.positions = [j_here[0], j_here[1], SPINDLE_DOWN]
            pt.time_from_start = make_duration(t_now)
            traj.points.append(pt)

            # 2c) spindle UP
            t_now += DIP_UP_S
            pt = JointTrajectoryPoint()
            pt.positions = [j_here[0], j_here[1], SPINDLE_UP]
            pt.time_from_start = make_duration(t_now)
            traj.points.append(pt)

            # 2d) trace the edge to the next corner (spindle UP throughout)
            edge_pts = interpolate_edge(corner, next_corner, SAMPLES_PER_EDGE)
            for i, (x, y) in enumerate(edge_pts):
                jj = joints_at(x, y)
                if jj is None:
                    self.get_logger().error(
                        f"Edge {k} waypoint {i} ({x:.3f},{y:.3f}) out of workspace."
                    )
                    return None
                t_step = t_now + (i + 1) * (EDGE_DURATION_S / SAMPLES_PER_EDGE)
                pt = JointTrajectoryPoint()
                pt.positions = [jj[0], jj[1], SPINDLE_UP]
                pt.time_from_start = make_duration(t_step)
                traj.points.append(pt)
            t_now += EDGE_DURATION_S

        self.get_logger().info(
            f"Built {len(traj.points)} way-points, total duration {t_now:.1f}s."
        )
        return traj

    def _fire(self):
        if self._published:
            return
        self._pub.publish(self._traj)
        self._published = True
        self.get_logger().info(
            "Trajectory published. The arm will trace the square, "
            "dipping the spindle at each of the four corners."
        )


def main(args=None):
    rclpy.init(args=args)
    node = SquareDemoV3()
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
