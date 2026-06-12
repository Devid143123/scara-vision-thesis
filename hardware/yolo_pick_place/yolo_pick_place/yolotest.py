# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String, Float32MultiArray
# from geometry_msgs.msg import Point
# from cv_bridge import CvBridge
# from collections import deque
# import cv2
# import numpy as np
# from ultralytics import YOLO

# MARKER_POSITIONS = {
#     1: np.array([-0.1185,  0.2685]),
#     2: np.array([ 0.1185,  0.2685]),
#     3: np.array([-0.1185,  0.0315]),
#     4: np.array([ 0.1185,  0.0315]),
# }

# class YoloNode(Node):
#     def __init__(self):
#         super().__init__('yolo_node')

#         self.subscription = self.create_subscription(
#             Image, '/image_raw', self.listener_callback, 10)

#         self.publisher = self.create_publisher(String, '/yolo/detections', 10)
#         self.image_pub = self.create_publisher(Image, '/yolo/annotated', 10)
#         self.coord_pub = self.create_publisher(Point, '/object_position', 10)
#         self.ik_pub = self.create_publisher(Float32MultiArray, '/ik_target', 10)

#         self.reset_sub = self.create_subscription(
#             String, '/yolo/reset_roi', self.reset_callback, 10)

#         self.bridge = CvBridge()
#         self.model = YOLO('/home/reach/SCARA/src/yolo_pick_place/yolov8n.pt')

#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
#         self.aruco_params = cv2.aruco.DetectorParameters()
#         self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

#         self.H = None
#         self.marker_half_px = None
#         self.workspace_polygon = None
#         self.roi = None
#         self.coord_buffer = {}
#         self.smooth_window = 10
#         self.roi_locked = False
#         self.frame_count = 0
#         self.process_every_n = 2
#         self.raw_corners = {}

#         self.get_logger().info('Yolo node started!')

#     def reset_callback(self, msg):
#         self.roi = None
#         self.roi_locked = False
#         self.H = None
#         self.marker_half_px = None
#         self.workspace_polygon = None
#         self.raw_corners = {}
#         self.get_logger().info('ROI reset — searching for markers again')

#     def calibrate(self, frame):
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         corners, ids, _ = self.detector.detectMarkers(gray)

#         if ids is None:
#             self.H = None
#             return False, {}

#         image_points = []
#         robot_points = []
#         marker_centers = {}
#         marker_sizes_px = []
#         raw_corners = {}

#         for i, marker_id in enumerate(ids.flatten()):
#             if marker_id in MARKER_POSITIONS:
#                 c = corners[i][0]
#                 cx = float(np.mean(c[:, 0]))
#                 cy = float(np.mean(c[:, 1]))
#                 image_points.append([cx, cy])
#                 robot_points.append(MARKER_POSITIONS[marker_id])
#                 marker_centers[marker_id] = (int(cx), int(cy))
#                 raw_corners[marker_id] = c

#                 # Log each marker's corners for debugging
#                 self.get_logger().info(
#                     f'Marker {marker_id} center=({int(cx)},{int(cy)}) '
#                     f'c0={tuple(c[0].astype(int))} c1={tuple(c[1].astype(int))} '
#                     f'c2={tuple(c[2].astype(int))} c3={tuple(c[3].astype(int))}'
#                 )

#                 w = float(np.max(c[:, 0]) - np.min(c[:, 0]))
#                 h = float(np.max(c[:, 1]) - np.min(c[:, 1]))
#                 marker_sizes_px.append((w + h) / 2.0)

#         if len(image_points) < 4:
#             self.H = None
#             return False, marker_centers

#         self.marker_half_px = float(np.mean(marker_sizes_px)) / 2.0
#         self.raw_corners = raw_corners

#         image_points = np.array(image_points, dtype=np.float32)
#         robot_points = np.array(robot_points, dtype=np.float32)

#         self.H, _ = cv2.findHomography(image_points, robot_points, cv2.RANSAC)
#         return True, marker_centers

#     def compute_workspace_polygon(self):
#         """
#         Pick the single outermost corner from each marker.
#         For a camera slightly off-center, we find the true outer corner
#         by selecting the corner farthest from the workspace center.
#         """
#         rc = self.raw_corners
#         if not all(k in rc for k in [1, 2, 3, 4]):
#             return None

#         # Compute image center of all 4 marker centers combined
#         all_centers = np.array([
#             np.mean(rc[mid], axis=0) for mid in [1, 2, 3, 4]
#         ])
#         cx = float(np.mean(all_centers[:, 0]))
#         cy = float(np.mean(all_centers[:, 1]))

#         def outer_corner(mid):
#             """Return the corner of marker `mid` farthest from workspace center."""
#             c = rc[mid]
#             dists = [np.linalg.norm(np.array([cx, cy]) - pt) for pt in c]
#             return c[np.argmax(dists)]

#         tl = outer_corner(1)
#         tr = outer_corner(2)
#         br = outer_corner(4)
#         bl = outer_corner(3)

#         return np.array([tl, tr, br, bl], dtype=np.int32)

#     def compute_bounding_roi(self, polygon):
#         x, y, w, h = cv2.boundingRect(polygon)
#         return (x, y, x + w, y + h)

#     def pixel_to_robot(self, px, py):
#         pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
#         result = cv2.perspectiveTransform(pt, self.H)
#         return float(result[0][0][0]), float(result[0][0][1])

#     def smooth_coords(self, cls_name, x_m, y_m):
#         if cls_name not in self.coord_buffer:
#             self.coord_buffer[cls_name] = deque(maxlen=self.smooth_window)
#         self.coord_buffer[cls_name].append((x_m, y_m))
#         xs = [p[0] for p in self.coord_buffer[cls_name]]
#         ys = [p[1] for p in self.coord_buffer[cls_name]]
#         return float(np.mean(xs)), float(np.mean(ys))

#     def point_in_polygon(self, px, py, polygon):
#         return cv2.pointPolygonTest(polygon.astype(np.float32), (float(px), float(py)), False) >= 0

#     def draw_info_panel(self, frame, detections):
#         panel_x = 10
#         panel_y = 60
#         line_height = 30
#         for i, (cls_name, conf, cx, cy, x_m, y_m) in enumerate(detections):
#             text = f"[{i+1}] {cls_name} | px:({cx},{cy}) | m:({x_m:.3f},{y_m:.3f})"
#             cv2.putText(frame, text,
#                 (panel_x, panel_y + i * line_height),
#                 cv2.FONT_HERSHEY_SIMPLEX,
#                 0.55, (0, 255, 0), 2, cv2.LINE_AA)

#     def listener_callback(self, msg):
#         self.frame_count += 1
#         if self.frame_count % self.process_every_n != 0:
#             return

#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         h_frame, w_frame = frame.shape[:2]

#         # ── PHASE 1: Find all 4 markers ONCE and lock workspace ──────────────
#         if not self.roi_locked:
#             calibrated, marker_centers = self.calibrate(frame)

#             debug_frame = frame.copy()
#             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#             corners, ids, _ = self.detector.detectMarkers(gray)
#             if ids is not None:
#                 cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)

#             if calibrated and len(marker_centers) == 4:
#                 self.workspace_polygon = self.compute_workspace_polygon()
#                 if self.workspace_polygon is not None:
#                     self.roi = self.compute_bounding_roi(self.workspace_polygon)
#                     self.roi_locked = True
#                     self.get_logger().info(f'Workspace locked: {self.roi}')
#             else:
#                 found = len(marker_centers) if marker_centers else 0
#                 cv2.putText(debug_frame,
#                     f'Searching for markers... {found}/4 found',
#                     (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
#                     0.7, (0, 0, 255), 2)
#                 annotated_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
#                 self.image_pub.publish(annotated_msg)
#                 return

#         # ── PHASE 2: Workspace locked — detect objects inside ROI ────────────
#         rx1, ry1, rx2, ry2 = self.roi

#         rx1 = max(0, rx1)
#         ry1 = max(0, ry1)
#         rx2 = min(w_frame, rx2)
#         ry2 = min(h_frame, ry2)

#         roi_frame = frame[ry1:ry2, rx1:rx2]
#         results = self.model(roi_frame, verbose=False, conf=0.5, imgsz=320)

#         annotated_frame = frame.copy()
#         detections = []

#         # Draw perspective-correct polygon
#         cv2.polylines(annotated_frame,
#                       [self.workspace_polygon],
#                       isClosed=True,
#                       color=(0, 255, 0),
#                       thickness=2)
#         cv2.putText(annotated_frame, "Workspace",
#             (self.workspace_polygon[0][0], self.workspace_polygon[0][1] - 10),
#             cv2.FONT_HERSHEY_SIMPLEX,
#             0.6, (0, 255, 0), 2, cv2.LINE_AA)

#         for box in results[0].boxes:
#             bx1, by1, bx2, by2 = [int(v) for v in box.xyxy[0].tolist()]

#             fx1, fy1 = bx1 + rx1, by1 + ry1
#             fx2, fy2 = bx2 + rx1, by2 + ry1

#             cx = int((fx1 + fx2) / 2)
#             cy = int((fy1 + fy2) / 2)

#             if not self.point_in_polygon(cx, cy, self.workspace_polygon):
#                 continue

#             cls = int(box.cls[0])
#             conf = float(box.conf[0])
#             cls_name = self.model.names[cls]

#             x_m, y_m = self.pixel_to_robot(cx, cy)
#             x_m, y_m = self.smooth_coords(cls_name, x_m, y_m)

#             cv2.rectangle(annotated_frame, (fx1, fy1), (fx2, fy2),
#                           (0, 255, 255), 2)
#             cv2.circle(annotated_frame, (cx, cy), 6, (0, 255, 255), -1)

#             detections.append((cls_name, conf, cx, cy, x_m, y_m))

#             detection_str = (f"class:{cls}, conf:{conf:.2f}, "
#                             f"box:[{fx1},{fy1},{fx2},{fy2}], center:({cx},{cy}), "
#                             f"robot:({x_m:.3f}m,{y_m:.3f}m)")
#             self.publisher.publish(String(data=detection_str))

#             point = Point()
#             point.x = x_m
#             point.y = y_m
#             point.z = 0.0
#             self.coord_pub.publish(point)

#             if -0.1185 <= x_m <= 0.1185 and 0.0315 <= y_m <= 0.2685:
#                 ik_msg = Float32MultiArray()
#                 ik_msg.data = [x_m, y_m]
#                 self.ik_pub.publish(ik_msg)
#                 self.get_logger().info(
#                     f'Sent to IK: ({x_m:.3f}, {y_m:.3f}) m'
#                 )

#         cv2.putText(annotated_frame,
#             f'Workspace Locked | Objects: {len(detections)}',
#             (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
#             0.7, (0, 255, 0), 2)

#         if detections:
#             self.draw_info_panel(annotated_frame, detections)

#         annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
#         self.image_pub.publish(annotated_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


















# import rclpy
# import torch
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from sensor_msgs.msg import Image
# from std_msgs.msg import String, Float32MultiArray
# from geometry_msgs.msg import Point
# from cv_bridge import CvBridge
# from collections import deque
# import cv2
# import numpy as np
# from ultralytics import YOLO

# # Marker positions in robot frame (meters)
# # Blue box coordinate system:
# #   top of blue box    = y = 0.30m
# #   bottom of blue box = y = 0.00m
# #   robot base (0,0)   = y = -0.075m (below the blue box)
# #   x: -0.1185m (left) to +0.1185m (right)
# MARKER_POSITIONS = {
#     1: np.array([-0.1185,  0.30]),   # top-left
#     2: np.array([ 0.1185,  0.30]),   # top-right
#     3: np.array([-0.1185,  0.00]),   # bottom-left
#     4: np.array([ 0.1185,  0.00]),   # bottom-right
# }

# IK_X_MIN = -0.1185
# IK_X_MAX =  0.1185
# IK_Y_MIN =  0.00
# IK_Y_MAX =  0.30

# ROBOT_BASE_OFFSET_M = 0.075

# class YoloNode(Node):
#     def __init__(self):
#         super().__init__('yolo_node')

#         self.subscription = self.create_subscription(
#             CompressedImage, '/camera/realsense_driver/color/image_raw/compressed', self.listener_callback, 10)

#         self.publisher = self.create_publisher(String, '/yolo/detections', 10)
#         self.image_pub = self.create_publisher(Image, '/yolo/annotated', 10)
#         self.coord_pub = self.create_publisher(Point, '/object_position', 10)
#         self.ik_pub = self.create_publisher(Float32MultiArray, '/ik_target', 10)

#         self.reset_sub = self.create_subscription(
#             String, '/yolo/reset_roi', self.reset_callback, 10)

#         self.bridge = CvBridge()
#         self.model = YOLO('/home/reach/SCARA/src/yolo_pick_place/yolov8n.pt')
#         if torch.cuda.is_available():
#             self.model.to('cuda')
#             self.get_logger().info("YOLO model moved to GPU")
#         else:
#             self.get_logger().warn("CUDA not available, running on CPU")

#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
#         self.aruco_params = cv2.aruco.DetectorParameters()
#         self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

#         self.H = None
#         self.marker_half_px = None
#         self.workspace_polygon = None
#         self.reachable_polygon = None
#         self.roi = None
#         self.coord_buffer = {}
#         self.smooth_window = 10
#         self.roi_locked = False
#         self.frame_count = 0
#         self.process_every_n = 2
#         self.raw_corners = {}

#         self.get_logger().info('Yolo node started!')

#     def reset_callback(self, msg):
#         self.roi = None
#         self.roi_locked = False
#         self.H = None
#         self.marker_half_px = None
#         self.workspace_polygon = None
#         self.reachable_polygon = None
#         self.raw_corners = {}
#         self.get_logger().info('ROI reset — searching for markers again')

#     def calibrate(self, frame):
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         corners, ids, _ = self.detector.detectMarkers(gray)

#         if ids is None:
#             self.H = None
#             return False, {}

#         image_points = []
#         robot_points = []
#         marker_centers = {}
#         marker_sizes_px = []
#         raw_corners = {}

#         for i, marker_id in enumerate(ids.flatten()):
#             if marker_id in MARKER_POSITIONS:
#                 c = corners[i][0]
#                 cx = float(np.mean(c[:, 0]))
#                 cy = float(np.mean(c[:, 1]))
#                 image_points.append([cx, cy])
#                 robot_points.append(MARKER_POSITIONS[marker_id])
#                 marker_centers[marker_id] = (int(cx), int(cy))
#                 raw_corners[marker_id] = c

#                 w = float(np.max(c[:, 0]) - np.min(c[:, 0]))
#                 h = float(np.max(c[:, 1]) - np.min(c[:, 1]))
#                 marker_sizes_px.append((w + h) / 2.0)

#         if len(image_points) < 4:
#             self.H = None
#             return False, marker_centers

#         self.marker_half_px = float(np.mean(marker_sizes_px)) / 2.0
#         self.raw_corners = raw_corners

#         image_points = np.array(image_points, dtype=np.float32)
#         robot_points = np.array(robot_points, dtype=np.float32)

#         self.H, _ = cv2.findHomography(image_points, robot_points, cv2.RANSAC)
#         return True, marker_centers

#     def compute_workspace_polygon(self):
#         rc = self.raw_corners
#         if not all(k in rc for k in [1, 2, 3, 4]):
#             return None

#         all_centers = np.array([
#             np.mean(rc[mid], axis=0) for mid in [1, 2, 3, 4]
#         ])
#         cx = float(np.mean(all_centers[:, 0]))
#         cy = float(np.mean(all_centers[:, 1]))

#         def outer_corner(mid):
#             c = rc[mid]
#             dists = [np.linalg.norm(np.array([cx, cy]) - pt) for pt in c]
#             return c[np.argmax(dists)]

#         tl = outer_corner(1)
#         tr = outer_corner(2)
#         br = outer_corner(4)
#         bl = outer_corner(3)

#         return np.array([tl, tr, br, bl], dtype=np.int32)

#     def compute_reachable_polygon(self, workspace_polygon):
#         """
#         Shift the workspace polygon DOWNWARD in pixel space by 7.5cm.
#         Down in image = increasing pixel y.
#         Box height in pixels = 30cm total.
#         offset = box_height_px * (7.5 / 30)
#         """
#         all_y = workspace_polygon[:, 1]
#         top_y = int(np.min(all_y))
#         bottom_y = int(np.max(all_y))
#         box_height_px = bottom_y - top_y

#         offset_px = int(box_height_px * (ROBOT_BASE_OFFSET_M / 0.30))

#         self.get_logger().info(
#             f'box_height_px={box_height_px}, offset_px={offset_px}')

#         shifted = workspace_polygon.copy()
#         shifted[:, 1] += offset_px
#         return shifted.astype(np.int32)

#     def compute_bounding_roi(self, polygon):
#         x, y, w, h = cv2.boundingRect(polygon)
#         return (x, y, x + w, y + h)

#     def pixel_to_robot(self, px, py):
#         pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
#         result = cv2.perspectiveTransform(pt, self.H)
#         return float(result[0][0][0]), float(result[0][0][1])

#     def smooth_coords(self, cls_name, x_m, y_m):
#         if cls_name not in self.coord_buffer:
#             self.coord_buffer[cls_name] = deque(maxlen=self.smooth_window)
#         self.coord_buffer[cls_name].append((x_m, y_m))
#         xs = [p[0] for p in self.coord_buffer[cls_name]]
#         ys = [p[1] for p in self.coord_buffer[cls_name]]
#         return float(np.mean(xs)), float(np.mean(ys))

#     def point_in_polygon(self, px, py, polygon):
#         return cv2.pointPolygonTest(
#             polygon.astype(np.float32), (float(px), float(py)), False) >= 0

#     def draw_info_panel(self, frame, detections):
#         panel_x = 10
#         panel_y = 60
#         line_height = 30
#         for i, (cls_name, conf, cx, cy, x_m, y_m) in enumerate(detections):
#             text = f"[{i+1}] {cls_name} | px:({cx},{cy}) | m:({x_m:.3f},{y_m:.3f})"
#             cv2.putText(frame, text,
#                 (panel_x, panel_y + i * line_height),
#                 cv2.FONT_HERSHEY_SIMPLEX,
#                 0.55, (0, 255, 0), 2, cv2.LINE_AA)

#     def listener_callback(self, msg):
#         self.frame_count += 1
#         if self.frame_count % self.process_every_n != 0:
#             return

#         frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         h_frame, w_frame = frame.shape[:2]

#         # Always start with a copy
#         annotated_frame = frame.copy()


#         # ── PHASE 1: Find all 4 markers ONCE and lock workspace ──────────────
#         if not self.roi_locked:
#             calibrated, marker_centers = self.calibrate(frame)

#             debug_frame = frame.copy()
#             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#             corners, ids, _ = self.detector.detectMarkers(gray)
#             if ids is not None:
#                 cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)

#             if calibrated and len(marker_centers) == 4:
#                 self.workspace_polygon = self.compute_workspace_polygon()
#                 if self.workspace_polygon is not None:
#                     self.reachable_polygon = self.compute_reachable_polygon(
#                         self.workspace_polygon)
#                     self.roi = self.compute_bounding_roi(self.workspace_polygon)
#                     self.roi_locked = True
#                     self.get_logger().info(f'Workspace locked: {self.roi}')
#             else:
#                 found = len(marker_centers) if marker_centers else 0
#                 cv2.putText(debug_frame,
#                     f'Searching for markers... {found}/4 found',
#                     (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
#                     0.7, (0, 0, 255), 2)
                
#                 annotated_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')

#                 self.image_pub.publish(annotated_msg)
#                 return

#         # ── PHASE 2: Workspace locked — detect objects inside ROI ────────────
#         rx1, ry1, rx2, ry2 = self.roi

#         rx1 = max(0, rx1)
#         ry1 = max(0, ry1)
#         rx2 = min(w_frame, rx2)
#         ry2 = min(h_frame, ry2)

#         roi_frame = frame[ry1:ry2, rx1:rx2]
#         results = self.model(roi_frame, verbose=False, conf=0.5, imgsz=192)

#         annotated_frame = frame.copy()
#         detections = []

#         # Green = full workspace (ArUco marker boundary)
#         cv2.polylines(annotated_frame,
#                       [self.workspace_polygon],
#                       isClosed=True,
#                       color=(0, 255, 0),
#                       thickness=2)
#         cv2.putText(annotated_frame, "Workspace",
#             (self.workspace_polygon[0][0], self.workspace_polygon[0][1] - 10),
#             cv2.FONT_HERSHEY_SIMPLEX,
#             0.5, (0, 255, 0), 2, cv2.LINE_AA)

#         # Blue = reachable area (shifted down 7.5cm)
#         if self.reachable_polygon is not None:
#             cv2.polylines(annotated_frame,
#                           [self.reachable_polygon],
#                           isClosed=True,
#                           color=(255, 100, 0),
#                           thickness=2)
#             cv2.putText(annotated_frame, "Reachable",
#                 (self.reachable_polygon[0][0],
#                  self.reachable_polygon[0][1] - 10),
#                 cv2.FONT_HERSHEY_SIMPLEX,
#                 0.5, (255, 100, 0), 2, cv2.LINE_AA)

#         for box in results[0].boxes:
#             bx1, by1, bx2, by2 = [int(v) for v in box.xyxy[0].tolist()]

#             fx1, fy1 = bx1 + rx1, by1 + ry1
#             fx2, fy2 = bx2 + rx1, by2 + ry1

#             cx = int((fx1 + fx2) / 2)
#             cy = int((fy1 + fy2) / 2)

#             # Only detect inside the blue reachable polygon
#             if self.reachable_polygon is None:
#                 continue
#             if not self.point_in_polygon(cx, cy, self.reachable_polygon):
#                 continue

#             cls = int(box.cls[0])
#             conf = float(box.conf[0])
#             cls_name = self.model.names[cls]

#             # Pixel → robot frame (blue box coordinate system)
#             x_m, y_m = self.pixel_to_robot(cx, cy)
#             x_m, y_m = self.smooth_coords(cls_name, x_m, y_m)

#             cv2.rectangle(annotated_frame, (fx1, fy1), (fx2, fy2),
#                           (0, 255, 255), 2)
#             cv2.circle(annotated_frame, (cx, cy), 6, (0, 255, 255), -1)

#             detections.append((cls_name, conf, cx, cy, x_m, y_m))

#             detection_str = (f"class:{cls}, conf:{conf:.2f}, "
#                             f"box:[{fx1},{fy1},{fx2},{fy2}], center:({cx},{cy}), "
#                             f"robot:({x_m:.3f}m,{y_m:.3f}m)")
#             self.publisher.publish(String(data=detection_str))

#             point = Point()
#             point.x = x_m
#             point.y = y_m
#             point.z = 0.0
#             self.coord_pub.publish(point)

#             if IK_X_MIN <= x_m <= IK_X_MAX and IK_Y_MIN <= y_m <= IK_Y_MAX:
#                 ik_msg = Float32MultiArray()
#                 ik_msg.data = [x_m, y_m]
#                 self.ik_pub.publish(ik_msg)
#                 self.get_logger().info(
#                     f'Sent to IK: ({x_m:.3f}, {y_m:.3f}) m'
#                 )

#         cv2.putText(annotated_frame,
#             f'Workspace Locked | Objects: {len(detections)}',
#             (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
#             0.7, (0, 255, 0), 2)

#         if detections:
#             self.draw_info_panel(annotated_frame, detections)

#         annotated_msg = self.bridge.cv2_to_compressed_imgmsg(annotated_frame, dst_format='jpeg')
#         self.image_pub.publish(annotated_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()












# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String, Float32MultiArray
# from geometry_msgs.msg import Point
# from cv_bridge import CvBridge
# from collections import deque
# import cv2
# import numpy as np
# from ultralytics import YOLO
# import threading

# # ── WORKSPACE CONFIGURATION (30cm x 30cm) ──────────────────────────────────
# # Origin (0,0) is center-bottom. Dimensions in meters.
# MARKER_POSITIONS = {
#     1: np.array([-0.15,  0.30]), # Top-Left
#     2: np.array([ 0.15,  0.30]), # Top-Right
#     3: np.array([ 0.15,  0.00]), # Bottom-Right
#     4: np.array([-0.15,  0.00]), # Bottom-Left
# }

# WINDOW_NAME = "YOLO Workspace Mapper | Click TL->TR->BR->BL | R=Reset"

# class YoloWorkspaceNode(Node):
#     def __init__(self):
#         super().__init__('yolo_workspace_node')

#         # ROS Communication
#         self.subscription = self.create_subscription(
#             Image, '/image_raw', self.listener_callback, 10)
#         self.publisher = self.create_publisher(String, '/yolo/detections', 10)
#         self.image_pub = self.create_publisher(Image, '/yolo/annotated', 10)
#         self.coord_pub = self.create_publisher(Point, '/object_position', 10)
#         self.ik_pub = self.create_publisher(Float32MultiArray, '/ik_target', 10)

#         # State Variables
#         self.bridge = CvBridge()
#         self.model = YOLO('/home/reach/SCARA/src/yolo_pick_place/yolov8n.pt')
#         self.H = None
#         self.roi_locked = False
#         self.manual_points = []
#         self.coord_buffer = {}
#         self.smooth_window = 10
        
#         # OpenCV Window Setup
#         cv2.namedWindow(WINDOW_NAME)
#         cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

#         self.get_logger().info('Node started! Please click the 4 corners of your 30x30cm workspace.')

#     def mouse_callback(self, event, x, y, flags, param):
#         if event == cv2.EVENT_LBUTTONDOWN:
#             if not self.roi_locked:
#                 self.manual_points.append([x, y])
#                 self.get_logger().info(f"Corner {len(self.manual_points)} set at: ({x}, {y})")
#                 if len(self.manual_points) == 4:
#                     self.compute_homography()

#     def compute_homography(self):
#         src = np.array(self.manual_points, dtype=np.float32)
#         dst = np.array([MARKER_POSITIONS[i] for i in range(1, 5)], dtype=np.float32)
#         self.H, _ = cv2.findHomography(src, dst)
        
#         # Compute bounding ROI for YOLO cropping
#         polygon = np.array(self.manual_points, dtype=np.int32)
#         x, y, w, h = cv2.boundingRect(polygon)
#         self.roi = (x, y, x + w, y + h)
#         self.workspace_polygon = polygon
        
#         self.roi_locked = True
#         self.get_logger().info("Workspace mapped and YOLO processing started!")

#     def pixel_to_robot(self, px, py):
#         pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
#         result = cv2.perspectiveTransform(pt, self.H)
#         return float(result[0][0][0]), float(result[0][0][1])

#     def smooth_coords(self, cls_name, x_m, y_m):
#         if cls_name not in self.coord_buffer:
#             self.coord_buffer[cls_name] = deque(maxlen=self.smooth_window)
#         self.coord_buffer[cls_name].append((x_m, y_m))
#         xs = [p[0] for p in self.coord_buffer[cls_name]]
#         ys = [p[1] for p in self.coord_buffer[cls_name]]
#         return float(np.mean(xs)), float(np.mean(ys))

#     def listener_callback(self, msg):
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         h_frame, w_frame = frame.shape[:2]

#         if not self.roi_locked:
#             # Calibration Display
#             vis_frame = frame.copy()
#             for i, pt in enumerate(self.manual_points):
#                 cv2.circle(vis_frame, (pt[0], pt[1]), 5, (0, 0, 255), -1)
#             cv2.putText(vis_frame, f"Click 4 corners: {len(self.manual_points)}/4", 
#                         (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
#             cv2.imshow(WINDOW_NAME, vis_frame)
#             cv2.waitKey(1)
#             return

#         # Phase 2: YOLO Detection
#         rx1, ry1, rx2, ry2 = self.roi
#         rx1, ry1 = max(0, rx1), max(0, ry1)
#         rx2, ry2 = min(w_frame, rx2), min(h_frame, ry2)

#         roi_frame = frame[ry1:ry2, rx1:rx2]
#         results = self.model(roi_frame, verbose=False, conf=0.5)

#         annotated_frame = frame.copy()
#         cv2.polylines(annotated_frame, [self.workspace_polygon], True, (0, 255, 0), 2)

#         for box in results[0].boxes:
#             bx1, by1, bx2, by2 = [int(v) for v in box.xyxy[0].tolist()]
#             # Re-map ROI coordinates to Full Frame
#             fx1, fy1 = bx1 + rx1, by1 + ry1
#             fx2, fy2 = bx2 + rx1, by2 + ry1
#             cx, cy = (fx1 + fx2) // 2, (fy1 + fy2) // 2

#             # Only process if center is inside manual polygon
#             if cv2.pointPolygonTest(self.workspace_polygon.astype(np.float32), (float(cx), float(cy)), False) >= 0:
#                 cls_name = self.model.names[int(box.cls[0])]
#                 x_m, y_m = self.pixel_to_robot(cx, cy)
#                 x_m, y_m = self.smooth_coords(cls_name, x_m, y_m)

#                 # Visuals
#                 cv2.rectangle(annotated_frame, (fx1, fy1), (fx2, fy2), (0, 255, 255), 2)
#                 cv2.putText(annotated_frame, f"{cls_name}: {x_m:.3f}, {y_m:.3f}", 
#                             (fx1, fy1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

#                 # Publish Coords
#                 point_msg = Point(x=x_m, y=y_m, z=0.0)
#                 self.coord_pub.publish(point_msg)
                
#                 ik_msg = Float32MultiArray(data=[x_m, y_m])
#                 self.ik_pub.publish(ik_msg)

#         cv2.imshow(WINDOW_NAME, annotated_frame)
#         cv2.waitKey(1)
#         self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))

# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloWorkspaceNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()







# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String, Float32MultiArray
# from geometry_msgs.msg import Point
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from ultralytics import YOLO
# import time

# # Workspace Config (30cm x 30cm)
# MARKER_POSITIONS = {
#     1: np.array([-0.15,  0.30]), # Top-Left
#     2: np.array([ 0.15,  0.30]), # Top-Right
#     3: np.array([ 0.15,  0.00]), # Bottom-Right
#     4: np.array([-0.15,  0.00]), # Bottom-Left
# }

# # States
# STATE_CALIBRATING = 0
# STATE_HOMING      = 1
# STATE_DETECTING   = 2
# STATE_WAITING     = 3

# class ScaraTaskNode(Node):
#     def __init__(self):
#         super().__init__('scara_task_node')

#         self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
#         self.image_pub = self.create_publisher(Image, '/yolo/annotated', 10)
#         self.ik_pub = self.create_publisher(Float32MultiArray, '/ik_target', 10)
        
#         # ODrive command topic
#         self.joint_pub = self.create_publisher(Float32MultiArray, '/odrive/angle_cmd', 10) 

#         self.bridge = CvBridge()
#         self.model = YOLO('/home/reach/SCARA/src/yolo_pick_place/yolov8n.pt')
        
#         # Calibration & Logic State
#         self.H = None
#         self.roi_locked = False
#         self.manual_points = []
#         self.state = STATE_CALIBRATING
#         self.wait_start_time = None
#         self.home_sent_time = None
        
#         cv2.namedWindow("Robot Control Center")
#         cv2.setMouseCallback("Robot Control Center", self.mouse_callback)

#         self.get_logger().info('Node Started. Step 1: Click 4 corners (TL->TR->BR->BL).')

#     def mouse_callback(self, event, x, y, flags, param):
#         if event == cv2.EVENT_LBUTTONDOWN and not self.roi_locked:
#             self.manual_points.append([x, y])
#             if len(self.manual_points) == 4:
#                 self.compute_homography()
#                 self.state = STATE_HOMING

#     def compute_homography(self):
#         src = np.array(self.manual_points, dtype=np.float32)
#         dst = np.array([MARKER_POSITIONS[i] for i in range(1, 5)], dtype=np.float32)
#         self.H, _ = cv2.findHomography(src, dst)
#         self.workspace_polygon = np.array(self.manual_points, dtype=np.int32)
#         self.roi_locked = True

#     def send_home_command(self):
#         msg = Float32MultiArray()
#         # Your specific ODrive home sequence
#         msg.data = [1.0, 0.0, 2.0, 0.0] 
#         self.joint_pub.publish(msg)
#         self.get_logger().info('Sent Home command [1, 0, 2, 0]')

#     def listener_callback(self, msg):
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
#         if not self.roi_locked:
#             self.draw_calibration(frame)
#             return

#         # --- STATE MACHINE ---
#         if self.state == STATE_HOMING:
#             if self.home_sent_time is None:
#                 self.send_home_command()
#                 self.home_sent_time = time.time()
            
#             # Non-blocking wait for 2 seconds
#             if (time.time() - self.home_sent_time) > 2.0:
#                 self.state = STATE_DETECTING
#                 self.home_sent_time = None
#                 self.get_logger().info('At Home. Waiting for object...')

#         elif self.state == STATE_DETECTING:
#             self.process_detection(frame)

#         elif self.state == STATE_WAITING:
#             elapsed = time.time() - self.wait_start_time
#             if elapsed >= 5.0:
#                 self.get_logger().info('Wait over. Returning to Home.')
#                 self.state = STATE_HOMING
            
#             cv2.putText(frame, f"Holding: {5.0 - elapsed:.1f}s", (50, 50), 
#                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

#         cv2.imshow("Robot Control Center", frame)
#         cv2.waitKey(1)

#     def process_detection(self, frame):
#         x_roi, y_roi, w_roi, h_roi = cv2.boundingRect(self.workspace_polygon)
#         roi_frame = frame[y_roi:y_roi+h_roi, x_roi:x_roi+w_roi]
#         results = self.model(roi_frame, verbose=False, conf=0.6)

#         for box in results[0].boxes:
#             bx1, by1, bx2, by2 = [int(v) for v in box.xyxy[0].tolist()]
#             cx, cy = (bx1 + bx2) // 2 + x_roi, (by1 + by2) // 2 + y_roi

#             if cv2.pointPolygonTest(self.workspace_polygon.astype(np.float32), (float(cx), float(cy)), False) >= 0:
#                 pt = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
#                 res = cv2.perspectiveTransform(pt, self.H)
#                 rx, ry = float(res[0, 0, 0]), float(res[0, 0, 1])

#                 # Publish target to IK
#                 ik_msg = Float32MultiArray(data=[rx, ry])
#                 self.ik_pub.publish(ik_msg)
#                 self.get_logger().info(f'Detected object at ({rx:.3f}, {ry:.3f})')

#                 # Switch to waiting state
#                 self.wait_start_time = time.time()
#                 self.state = STATE_WAITING
#                 break 

#     def draw_calibration(self, frame):
#         for pt in self.manual_points:
#             cv2.circle(frame, (pt[0], pt[1]), 5, (0, 0, 255), -1)
#         cv2.putText(frame, f"Click corner {len(self.manual_points)+1}/4", (10, 30), 
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
#         cv2.imshow("Robot Control Center", frame)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ScaraTaskNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Float32MultiArray
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import time

# # Workspace Config (30cm x 30cm)
# MARKER_POSITIONS = {
#     1: np.array([-0.15,  0.30]), # Top-Left
#     2: np.array([ 0.15,  0.30]), # Top-Right
#     3: np.array([ 0.15,  0.00]), # Bottom-Right
#     4: np.array([-0.15,  0.00]), # Bottom-Left
# }

# # --- ADJUST THESE TO ALIGN THE CENTER (Parallax Correction) ---
# OFFSET_X = 0.005  
# OFFSET_Y = 0.005   

# # States
# STATE_CALIBRATING = 0
# STATE_HOMING      = 1
# STATE_IDLE        = 2 # Waiting for user click
# STATE_WAITING     = 3 # At target for 10s

# class ScaraManualNode(Node):
#     def __init__(self):
#         super().__init__('scara_manual_node')
#         self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
#         self.ik_pub = self.create_publisher(Float32MultiArray, '/ik_target', 10)
#         self.joint_pub = self.create_publisher(Float32MultiArray, '/odrive/angle_cmd', 10) 

#         self.bridge = CvBridge()
        
#         self.H = None
#         self.roi_locked = False
#         self.manual_points = []
#         self.state = STATE_CALIBRATING
#         self.wait_start_time = None
#         self.home_sent_time = None
#         self.last_target_robot = (0.0, 0.0)
#         self.current_mouse_px = (0, 0)
        
#         cv2.namedWindow("Manual Robot Control")
#         cv2.setMouseCallback("Manual Robot Control", self.mouse_callback)

#         self.get_logger().info('Node started. Step 1: Click 4 corners (TL->TR->BR->BL).')

#     def mouse_callback(self, event, x, y, flags, param):
#         # Always track mouse position for hover display
#         self.current_mouse_px = (x, y)

#         if event == cv2.EVENT_LBUTTONDOWN:
#             if not self.roi_locked:
#                 # Phase 1: Calibration clicks
#                 self.manual_points.append([x, y])
#                 if len(self.manual_points) == 4:
#                     src = np.array(self.manual_points, dtype=np.float32)
#                     dst = np.array([MARKER_POSITIONS[i] for i in range(1, 5)], dtype=np.float32)
#                     self.H, _ = cv2.findHomography(src, dst)
#                     self.workspace_polygon = np.array(self.manual_points, dtype=np.int32)
#                     self.roi_locked = True
#                     self.state = STATE_HOMING
            
#             elif self.state == STATE_IDLE:
#                 # Phase 2: Move-to-Click
#                 pt = np.array([[[float(x), float(y)]]], dtype=np.float32)
#                 res = cv2.perspectiveTransform(pt, self.H)
#                 rx = float(res[0, 0, 0]) + OFFSET_X
#                 ry = float(res[0, 0, 1]) + OFFSET_Y
                
#                 # Send the coordinate to IK
#                 self.ik_pub.publish(Float32MultiArray(data=[rx, ry]))
#                 self.last_target_robot = (rx, ry)
#                 self.get_logger().info(f'Moving to clicked point: ({rx:.3f}, {ry:.3f})')
                
#                 self.wait_start_time = time.time()
#                 self.state = STATE_WAITING

#     def listener_callback(self, msg):
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
#         if not self.roi_locked:
#             self.draw_calibration(frame)
#             return

#         # State Machine logic
#         if self.state == STATE_HOMING:
#             if self.home_sent_time is None:
#                 self.joint_pub.publish(Float32MultiArray(data=[1.0, 0.0, 2.0, 0.0]))
#                 self.home_sent_time = time.time()
#             if (time.time() - self.home_sent_time) > 2.0:
#                 self.state = STATE_IDLE
#                 self.home_sent_time = None
#                 self.get_logger().info('Robot Home. Click a spot to move.')

#         elif self.state == STATE_WAITING:
#             elapsed = time.time() - self.wait_start_time
#             if elapsed >= 10.0:
#                 self.state = STATE_HOMING
#                 self.get_logger().info('10s elapsed. Returning Home.')

#         self.draw_ui(frame)
#         cv2.imshow("Manual Robot Control", frame)
#         cv2.waitKey(1)

#     def draw_ui(self, frame):
#         # 1. Draw Workspace Border
#         cv2.polylines(frame, [self.workspace_polygon], True, (255, 0, 0), 2)
        
#         # 2. Draw Hover Tracking (Transform current mouse pixels to robot coords)
#         mx, my = self.current_mouse_px
#         if self.H is not None:
#             pt_m = np.array([[[float(mx), float(my)]]], dtype=np.float32)
#             res_m = cv2.perspectiveTransform(pt_m, self.H)
#             m_rx = float(res_m[0, 0, 0]) + OFFSET_X
#             m_ry = float(res_m[0, 0, 1]) + OFFSET_Y
#             cv2.putText(frame, f"Cursor: ({m_rx:.3f}, {m_ry:.3f})", (mx + 15, my + 15), 
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

#         # 3. Status Labels
#         if self.state == STATE_IDLE:
#             cv2.putText(frame, "STATUS: READY - CLICK TO MOVE", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
#         elif self.state == STATE_WAITING:
#             rem = 10.0 - (time.time() - self.wait_start_time)
#             cv2.putText(frame, f"STATUS: AT TARGET ({rem:.1f}s)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
#             cv2.putText(frame, f"Target: {self.last_target_robot[0]:.3f}, {self.last_target_robot[1]:.3f}", (20, 70), 
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

#     def draw_calibration(self, frame):
#         for pt in self.manual_points:
#             cv2.circle(frame, (pt[0], pt[1]), 5, (0, 0, 255), -1)
#         cv2.putText(frame, f"CALIBRATION: Click corner {len(self.manual_points)+1}/4", (10, 30), 
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
#         cv2.imshow("Manual Robot Control", frame)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ScaraManualNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()














import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
import os

# Workspace Config (30cm x 30cm)
MARKER_POSITIONS = {
    1: np.array([-0.15,  0.30]),
    2: np.array([ 0.15,  0.30]),
    3: np.array([ 0.15,  0.00]),
    4: np.array([-0.15,  0.00]),
}

OFFSET_X = 0.005
OFFSET_Y = 0.005

WORKSPACE_SAVE_PATH = os.path.expanduser('~/SCARA/src/SCARA_pkg/SCARA_pkg/scara_workspace.json')

# States
STATE_CALIBRATING  = 0
STATE_HOMING       = 1
STATE_IDLE         = 2
STATE_WAITING      = 3
STATE_ASK_RELOAD   = 4


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


class ScaraManualNode(Node):
    def __init__(self):
        super().__init__('scara_manual_node')
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.listener_callback, 10)
        self.ik_pub = self.create_publisher(Float32MultiArray, '/ik_target', 10)
        self.joint_pub = self.create_publisher(Float32MultiArray, '/odrive/angle_cmd', 10)

        self.bridge = CvBridge()

        self.H = None
        self.roi_locked = False
        self.manual_points = []
        self.wait_start_time = None
        self.home_sent_time = None
        self.last_target_robot = (0.0, 0.0)
        self.current_mouse_px = (0, 0)
        self.workspace_polygon = None

        cv2.namedWindow("Manual Robot Control")
        cv2.setMouseCallback("Manual Robot Control", self.mouse_callback)

        saved_points = load_workspace()
        if saved_points is not None:
            self.saved_points = saved_points
            self.state = STATE_ASK_RELOAD
            self.get_logger().info('Saved workspace found. Waiting for user choice...')
        else:
            self.saved_points = None
            self.state = STATE_CALIBRATING
            self.get_logger().info('No saved workspace. Starting calibration...')

    def apply_workspace(self, points):
        src = np.array(points, dtype=np.float32)
        dst = np.array([MARKER_POSITIONS[i] for i in range(1, 5)], dtype=np.float32)
        self.H, _ = cv2.findHomography(src, dst)
        self.workspace_polygon = np.array(points, dtype=np.int32)
        self.manual_points = points
        self.roi_locked = True
        self.state = STATE_HOMING

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

            elif self.state == STATE_IDLE:
                pt = np.array([[[float(x), float(y)]]], dtype=np.float32)
                res = cv2.perspectiveTransform(pt, self.H)
                rx = float(res[0, 0, 0]) + OFFSET_X
                ry = float(res[0, 0, 1]) + OFFSET_Y
                self.ik_pub.publish(Float32MultiArray(data=[rx, ry]))
                self.last_target_robot = (rx, ry)
                self.get_logger().info(f'Moving to: ({rx:.3f}, {ry:.3f})')
                self.wait_start_time = time.time()
                self.state = STATE_WAITING

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ── ASK RELOAD ────────────────────────────────────────────────────────
        if self.state == STATE_ASK_RELOAD:
            self.draw_ask_reload(frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 13:  # Enter — use saved workspace
                self.get_logger().info('Using saved workspace.')
                self.apply_workspace(self.saved_points)
            elif key == ord('r') or key == ord('R'):  # R — recalibrate
                self.get_logger().info('Recalibrating workspace...')
                self.manual_points = []
                self.state = STATE_CALIBRATING
            return

        # ── CALIBRATING ───────────────────────────────────────────────────────
        if not self.roi_locked:
            self.draw_calibration(frame)
            cv2.waitKey(1)
            return

        # ── STATE MACHINE ─────────────────────────────────────────────────────
        if self.state == STATE_HOMING:
            if self.home_sent_time is None:
                self.joint_pub.publish(Float32MultiArray(data=[1.0, 0.0, 2.0, 0.0]))
                self.home_sent_time = time.time()
            if (time.time() - self.home_sent_time) > 2.0:
                self.state = STATE_IDLE
                self.home_sent_time = None
                self.get_logger().info('Robot Home. Click a spot to move.')

        elif self.state == STATE_WAITING:
            elapsed = time.time() - self.wait_start_time
            if elapsed >= 10.0:
                self.state = STATE_HOMING
                self.get_logger().info('10s elapsed. Returning Home.')

        self.draw_ui(frame)
        cv2.imshow("Manual Robot Control", frame)
        cv2.waitKey(1)

    # ── DRAWING HELPERS ───────────────────────────────────────────────────────

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

        cv2.imshow("Manual Robot Control", frame)

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

        cv2.imshow("Manual Robot Control", frame)

    def draw_ui(self, frame):
        cv2.polylines(frame, [self.workspace_polygon], True, (255, 0, 0), 2)

        mx, my = self.current_mouse_px
        if self.H is not None:
            pt_m = np.array([[[float(mx), float(my)]]], dtype=np.float32)
            res_m = cv2.perspectiveTransform(pt_m, self.H)
            m_rx = float(res_m[0, 0, 0]) + OFFSET_X
            m_ry = float(res_m[0, 0, 1]) + OFFSET_Y
            cv2.putText(frame, f"Cursor: ({m_rx:.3f}, {m_ry:.3f})",
                        (mx + 15, my + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        if self.state == STATE_IDLE:
            cv2.putText(frame, "STATUS: READY — CLICK TO MOVE",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        elif self.state == STATE_WAITING:
            rem = 10.0 - (time.time() - self.wait_start_time)
            cv2.putText(frame, f"STATUS: AT TARGET ({rem:.1f}s remaining)",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame,
                        f"Target: ({self.last_target_robot[0]:.3f}, {self.last_target_robot[1]:.3f})",
                        (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        elif self.state == STATE_HOMING:
            cv2.putText(frame, "STATUS: HOMING...",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        cv2.imshow("Manual Robot Control", frame)


def main(args=None):
    rclpy.init(args=args)
    node = ScaraManualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()