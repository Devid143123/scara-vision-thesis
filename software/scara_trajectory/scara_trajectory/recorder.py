"""
Joint State Recorder for the new URDF
======================================
Logs /joint_states to scara_log.csv while the robot moves.
For each sample stores: t, joint angles, joint velocities, end-effector (x,y,z)
computed via forward kinematics.

For the new URDF (+Y forward, joint_0 axis (0,0,-1), joint_1 has +12 deg visual
offset), the recorded URDF joint angles are converted BACK to Reach's IK
frame (theta1, theta2) so the x,y from forward kinematics matches the square_demo
reference.

Run:
    python3 recorder.py
Press Ctrl+C when motion finishes -> scara_log.csv saved.
"""
import csv, math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

A1 = 0.15
A2 = 0.15
JOINT_1_VISUAL_OFFSET = math.radians(12.0)
OUT_FILE = "scara_log.csv"


def fk_from_reach(t1, t2):
    """Forward kinematics in Reach's IK frame."""
    x = A1*math.cos(t1) + A2*math.cos(t1 + t2)
    y = A1*math.sin(t1) + A2*math.sin(t1 + t2)
    return x, y


def urdf_to_reach(j0, j1):
    """Inverse of the controller's transform: undo the visual offset and the +pi/2."""
    theta1 = math.pi/2.0 - j0
    theta2 = j1 - JOINT_1_VISUAL_OFFSET
    return theta1, theta2


class Recorder(Node):
    def __init__(self):
        super().__init__("scara_recorder")
        self.f = open(OUT_FILE, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t","j0","j1","j2","v0","v1","v2","theta1","theta2","x_ee","y_ee"])
        self.t0 = None
        self.count = 0
        self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.get_logger().info(f"Recording to {OUT_FILE} - Ctrl+C to stop.")

    def cb(self, msg):
        if self.t0 is None: self.t0 = time.time()
        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        j0 = name_to_pos.get("joint_0", 0.0)
        j1 = name_to_pos.get("joint_1", 0.0)
        j2 = name_to_pos.get("joint_2", 0.0)
        v0 = name_to_vel.get("joint_0", 0.0)
        v1 = name_to_vel.get("joint_1", 0.0)
        v2 = name_to_vel.get("joint_2", 0.0)
        theta1, theta2 = urdf_to_reach(j0, j1)
        x, y = fk_from_reach(theta1, theta2)
        t = time.time() - self.t0
        self.w.writerow([f"{t:.4f}",f"{j0:.6f}",f"{j1:.6f}",f"{j2:.6f}",
                         f"{v0:.6f}",f"{v1:.6f}",f"{v2:.6f}",
                         f"{theta1:.6f}",f"{theta2:.6f}",
                         f"{x:.6f}",f"{y:.6f}"])
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(f"recorded {self.count} samples ({t:.1f}s)")

    def close(self):
        self.f.close()
        self.get_logger().info(f"Saved {self.count} samples to {OUT_FILE}")


def main():
    rclpy.init()
    node = Recorder()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
