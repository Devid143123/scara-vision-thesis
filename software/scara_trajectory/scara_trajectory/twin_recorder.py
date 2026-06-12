"""
Digital Twin Recorder (real robot + sim) - v2 for new topic format
====================================================================
Author: SARIN Chandevid

Subscribes to two topics simultaneously:
    /scara/measured_angles_deg  - real robot's measured angles [t1_deg, t2_deg]
    /joint_states               - sim robot's actual angles    [j0, j1 in RAD]

NOTE on the new format: /scara/measured_angles_deg is a Float32MultiArray
containing exactly two values in DEGREES:
    data: [theta1_degrees, theta2_degrees]

Both streams are recorded with timestamps into twin_log.csv at 20 Hz.
The sim's joint_0 is converted BACK to theta1 (Reach's frame) with the
inverse of the controller's transform:
    theta1_sim = pi/2 - joint_0
    theta2_sim = joint_1 - 12 deg

Run:
    python3 twin_recorder.py
Ctrl+C to stop -> twin_log.csv saved.
"""

import csv
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

JOINT_1_VISUAL_OFFSET = math.radians(12.0)
OUT_FILE = "twin_log.csv"

# NEW topic from your friend's updated code
REAL_TOPIC = "/scara/measured_angles_deg"


class TwinRecorder(Node):

    def __init__(self):
        super().__init__("twin_recorder")

        # Latest measurements (in radians internally for consistency)
        self._real_theta1 = None
        self._real_theta2 = None
        self._sim_theta1  = None
        self._sim_theta2  = None
        self._sim_d3      = 0.0

        self._f = open(OUT_FILE, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow([
            "t",
            "real_theta1", "real_theta2",
            "sim_theta1",  "sim_theta2",
            "sim_d3",
        ])

        self._t0 = None
        self._n_real = 0
        self._n_sim  = 0

        # Real robot - DEGREES
        self.create_subscription(
            Float32MultiArray, REAL_TOPIC,
            self._on_real_deg, 10,
        )
        # Sim - radians
        self.create_subscription(
            JointState, "/joint_states",
            self._on_joint_states, 10,
        )

        # Log row every 50 ms (20 Hz)
        self.create_timer(0.05, self._log_row)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Twin Recorder v2 started")
        self.get_logger().info(f"  Real angles  <- {REAL_TOPIC}  (in DEGREES)")
        self.get_logger().info(f"  Sim  angles  <- /joint_states  (in RAD)")
        self.get_logger().info(f"  Output       -> {OUT_FILE}")
        self.get_logger().info("  Ctrl+C to stop and save.")
        self.get_logger().info("=" * 60)

    def _on_real_deg(self, msg):
        """Format: data = [theta1_deg, theta2_deg]"""
        if len(msg.data) >= 2:
            # Convert degrees -> radians for storage consistency
            self._real_theta1 = math.radians(float(msg.data[0]))
            self._real_theta2 = math.radians(float(msg.data[1]))
            self._n_real += 1

    def _on_joint_states(self, msg):
        names = dict(zip(msg.name, msg.position))
        if "joint_0" in names and "joint_1" in names:
            j0 = float(names["joint_0"])
            j1 = float(names["joint_1"])
            # Undo the controller's frame transform:
            #   joint_0 = pi/2 - theta1   ->  theta1 = pi/2 - joint_0
            #   joint_1 = theta2 + 12 deg ->  theta2 = joint_1 - 12 deg
            self._sim_theta1 = math.pi/2.0 - j0
            self._sim_theta2 = j1 - JOINT_1_VISUAL_OFFSET
            if "joint_2" in names:
                self._sim_d3 = float(names["joint_2"])
            self._n_sim += 1

    def _log_row(self):
        if self._real_theta1 is None or self._sim_theta1 is None:
            return
        if self._t0 is None:
            self._t0 = time.time()
        t = time.time() - self._t0
        self._w.writerow([
            f"{t:.4f}",
            f"{self._real_theta1:.6f}", f"{self._real_theta2:.6f}",
            f"{self._sim_theta1:.6f}",  f"{self._sim_theta2:.6f}",
            f"{self._sim_d3:.6f}",
        ])

    def close(self):
        self._f.close()
        self.get_logger().info(
            f"Saved {OUT_FILE}: {self._n_real} real samples received, "
            f"{self._n_sim} sim samples received."
        )


def main():
    rclpy.init()
    node = TwinRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
