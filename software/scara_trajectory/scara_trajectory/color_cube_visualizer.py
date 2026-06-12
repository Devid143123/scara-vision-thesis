"""
Color Cube Visualizer (from /cube_info)  v2 - removes picked cubes
==================================================================
Author: SARIN Chandevid

Subscribes to /cube_info and shows the workspace state in RViz:

    "detected"  -> ADD cubes to the workspace (camera saw them)
    "picked"    -> REMOVE the picked cube from the workspace (robot has it)
    "placed"    -> ADD the cube at its new drop location (with its color)
    "manual click" -> ADD as a gray test target

A cube is matched by its (x, y) position (within a small threshold) so we
remove the correct one when picked.

To see in RViz:
    Add -> By topic -> /detected_cubes -> MarkerArray
To clear everything:
    ros2 topic pub --once /clear_cubes std_msgs/msg/Empty "{}"
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from visualization_msgs.msg import Marker, MarkerArray


# CONFIGURATION
SHOULDER_X =  0.0
SHOULDER_Y = -0.016

CUBE_SIZE_X = 0.030    # 30 mm wide
CUBE_SIZE_Y = 0.030    # 30 mm deep
CUBE_SIZE_Z = 0.010    # 10 mm tall (flat tile shape)
CUBE_Z = 0.005         # half of height, so cube sits ON the floor (z=0)
LABEL_HEIGHT = CUBE_Z + 0.04
LABEL_SIZE = 0.02

# Two cubes within this distance (m) are considered the same cube
MATCH_THRESHOLD = 0.02   # 2 cm

WORLD_FRAME = "world"

COLOR_TABLE = {
    "red":    (0.95, 0.15, 0.15),
    "green":  (0.20, 0.85, 0.30),
    "yellow": (0.95, 0.90, 0.10),
    "blue":   (0.20, 0.40, 0.95),
    "manual click": (0.60, 0.60, 0.60),
}
DEFAULT_COLOR = (0.50, 0.50, 0.50)


def color_rgb(name):
    return COLOR_TABLE.get(name.lower(), DEFAULT_COLOR)


def ik_to_world(x, y):
    return x + SHOULDER_X, y + SHOULDER_Y


class ColorCubeVisualizer(Node):

    def __init__(self):
        super().__init__("color_cube_visualizer")

        self._marker_pub = self.create_publisher(
            MarkerArray, "/detected_cubes", 10
        )
        self.create_subscription(String, "/cube_info", self._on_info, 10)
        self.create_subscription(Empty, "/clear_cubes", self._on_clear, 10)

        # Each cube: dict with wx, wy, color, marker_id (unique, never reused)
        self._cubes = []
        self._next_id = 0          # always increases - unique marker IDs

        # Pickup state (so a "placed" event can know which cube to re-add)
        self._carried = None       # dict {wx, wy, color} or None

        self._timer = self.create_timer(0.5, self._publish_markers)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Color Cube Visualizer v2 (pick/place tracking)")
        self.get_logger().info("  /cube_info       -> events from hardware")
        self.get_logger().info("  /clear_cubes     -> reset")
        self.get_logger().info("  /detected_cubes  -> MarkerArray to RViz")
        self.get_logger().info("=" * 60)

    # =================================================================
    # /cube_info handler
    # =================================================================
    def _on_info(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Bad JSON in /cube_info: {e}")
            return

        event = data.get("event", "unknown")

        if event == "detected":
            cubes = data.get("cubes", [])
            added = 0
            for c in cubes:
                color = c.get("color", "unknown")
                x_ik = float(c.get("x", 0.0))
                y_ik = float(c.get("y", 0.0))
                if self._add_cube(x_ik, y_ik, color):
                    added += 1
            self.get_logger().info(
                f"[detected] {len(cubes)} cube(s) reported, "
                f"{added} new added (workspace now has {len(self._cubes)})"
            )

        elif event == "picked":
            color = data.get("color", "unknown")
            x_ik = float(data.get("x", 0.0))
            y_ik = float(data.get("y", 0.0))
            removed = self._remove_cube_at(x_ik, y_ik)
            # Remember what the arm is carrying so we can re-place it
            self._carried = {
                "wx": x_ik + SHOULDER_X,
                "wy": y_ik + SHOULDER_Y,
                "color": color,
            }
            if removed:
                self.get_logger().info(
                    f"[picked]  {color} cube at IK ({x_ik:+.3f},{y_ik:.3f}) "
                    f"-> REMOVED from workspace ({len(self._cubes)} left)"
                )
            else:
                self.get_logger().info(
                    f"[picked]  {color} at IK ({x_ik:+.3f},{y_ik:.3f}) "
                    f"(no matching cube in workspace - still carried)"
                )

        elif event == "placed":
            color = data.get("color", "unknown")
            x_ik = float(data.get("x", 0.0))
            y_ik = float(data.get("y", 0.0))
            self._add_cube(x_ik, y_ik, color)
            self._carried = None
            self.get_logger().info(
                f"[placed]  {color} cube placed at IK ({x_ik:+.3f},{y_ik:.3f}) "
                f"(workspace has {len(self._cubes)})"
            )

        elif event == "manual click":
            x_ik = float(data.get("x", 0.0))
            y_ik = float(data.get("y", 0.0))
            self._add_cube(x_ik, y_ik, "manual click")
            self.get_logger().info(
                f"[manual]  test target at IK ({x_ik:+.3f},{y_ik:.3f})"
            )

        else:
            self.get_logger().info(f"[event: {event}]  raw: {msg.data[:80]}...")

    # =================================================================
    # State helpers
    # =================================================================
    def _add_cube(self, x_ik, y_ik, color):
        wx, wy = ik_to_world(x_ik, y_ik)
        # Skip duplicates (same cube already in workspace)
        if self._find_index_at(wx, wy) is not None:
            return False
        cube = {
            "wx": wx, "wy": wy, "color": color,
            "marker_id": self._next_id,
        }
        self._next_id += 1
        self._cubes.append(cube)
        return True

    def _remove_cube_at(self, x_ik, y_ik):
        wx, wy = ik_to_world(x_ik, y_ik)
        idx = self._find_index_at(wx, wy)
        if idx is None:
            return False
        cube = self._cubes.pop(idx)

        # Tell RViz to delete this specific marker (both shape and label)
        ma = MarkerArray()
        for ns in ("detected_cubes", "cube_labels"):
            m = Marker()
            m.header.frame_id = WORLD_FRAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = cube["marker_id"]
            m.action = Marker.DELETE
            ma.markers.append(m)
        self._marker_pub.publish(ma)
        return True

    def _find_index_at(self, wx, wy):
        for i, c in enumerate(self._cubes):
            if abs(c["wx"] - wx) < MATCH_THRESHOLD and \
               abs(c["wy"] - wy) < MATCH_THRESHOLD:
                return i
        return None

    # =================================================================
    # /clear_cubes
    # =================================================================
    def _on_clear(self, msg):
        count = len(self._cubes)
        self._cubes = []
        self._carried = None

        ma = MarkerArray()
        for ns in ("detected_cubes", "cube_labels"):
            m = Marker()
            m.header.frame_id = WORLD_FRAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.action = Marker.DELETEALL
            ma.markers.append(m)
        self._marker_pub.publish(ma)
        self.get_logger().info(f"*** CLEARED {count} cubes ***")

    # =================================================================
    # Republish all current cubes (so they stay visible in RViz)
    # =================================================================
    def _publish_markers(self):
        if not self._cubes:
            return

        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for c in self._cubes:
            r, g, b = color_rgb(c["color"])

            cube = Marker()
            cube.header.frame_id = WORLD_FRAME
            cube.header.stamp = stamp
            cube.ns = "detected_cubes"
            cube.id = c["marker_id"]
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = c["wx"]
            cube.pose.position.y = c["wy"]
            cube.pose.position.z = CUBE_Z
            cube.pose.orientation.w = 1.0
            cube.scale.x = CUBE_SIZE_X
            cube.scale.y = CUBE_SIZE_Y
            cube.scale.z = CUBE_SIZE_Z
            cube.color.r = r; cube.color.g = g; cube.color.b = b; cube.color.a = 1.0
            ma.markers.append(cube)

            label = Marker()
            label.header.frame_id = WORLD_FRAME
            label.header.stamp = stamp
            label.ns = "cube_labels"
            label.id = c["marker_id"]
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = c["wx"]
            label.pose.position.y = c["wy"]
            label.pose.position.z = LABEL_HEIGHT
            label.scale.z = LABEL_SIZE
            label.color.r = 1.0; label.color.g = 1.0; label.color.b = 1.0; label.color.a = 1.0
            label.text = c["color"]
            ma.markers.append(label)

        self._marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ColorCubeVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Stopped. Cubes left: {len(node._cubes)}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
