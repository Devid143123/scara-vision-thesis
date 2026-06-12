# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String, Float32MultiArray
# from geometry_msgs.msg import Point
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from ultralytics import YOLO

# # Keep your physical robot coordinates for the 4 corners
# # Order: Top-Left, Top-Right, Bottom-Left, Bottom-Right
# MARKER_POSITIONS = np.array([
#     [-0.1185,  0.2685], # TL
#     [ 0.1185,  0.2685], # TR
#     [-0.1185,  0.0315], # BL
#     [ 0.1185,  0.0315], # BR
# ], dtype=np.float32)

# class YoloNode(Node):
#     def __init__(self):
#         super().__init__('yolo_node')

#         self.subscription = self.create_subscription(
#             Image, '/image_raw', self.listener_callback, 10)
        
#         self.image_pub = self.create_publisher(Image, '/yolo/annotated', 10)
#         self.ik_pub = self.create_publisher(Float32MultiArray, '/ik_target', 10)

#         self.bridge = CvBridge()
#         self.model = YOLO('/home/reach/SCARA/src/yolo_pick_place/yolov8n.pt')

#         # --- Manual Mapping Variables ---
#         self.clicked_points = []
#         self.H = None
#         self.roi_locked = False
#         self.workspace_polygon = None
        
#         # Create OpenCV window for mouse interaction
#         cv2.namedWindow("Calibration")
#         cv2.setMouseCallback("Calibration", self.mouse_callback)
        
#         self.get_logger().info('Node started. Click the 4 corners in order: TL, TR, BL, BR')

#     def mouse_callback(self, event, x, y, flags, param):
#         if event == cv2.EVENT_LBUTTONDOWN and not self.roi_locked:
#             self.clicked_points.append([x, y])
#             self.get_logger().info(f'Point {len(self.clicked_points)} recorded: ({x}, {y})')
            
#             if len(self.clicked_points) == 4:
#                 self.compute_manual_homography()

#     def compute_manual_homography(self):
#         image_pts = np.array(self.clicked_points, dtype=np.float32)
#         # Calculate Homography matrix
#         self.H, _ = cv2.findHomography(image_pts, MARKER_POSITIONS)
        
#         # Define the workspace polygon for visualization and cropping
#         self.workspace_polygon = image_pts.astype(np.int32)
        
#         # Calculate ROI for YOLO cropping
#         x, y, w, h = cv2.boundingRect(self.workspace_polygon)
#         self.roi = (x, y, x + w, y + h)
        
#         self.roi_locked = True
#         cv2.destroyWindow("Calibration") # Close calibration window when done
#         self.get_logger().info('Workspace calibrated and locked!')

#     def pixel_to_robot(self, px, py):
#         pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
#         result = cv2.perspectiveTransform(pt, self.H)
#         return float(result[0][0][0]), float(result[0][0][1])

#     def listener_callback(self, msg):
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # PHASE 1: Calibration Mode
#         if not self.roi_locked:
#             temp_frame = frame.copy()
#             for i, pt in enumerate(self.clicked_points):
#                 cv2.circle(temp_frame, (pt[0], pt[1]), 5, (0, 0, 255), -1)
#                 cv2.putText(temp_frame, str(i+1), (pt[0]+10, pt[1]), 
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
#             cv2.putText(temp_frame, f"Click 4 corners (TL, TR, BL, BR). Points: {len(self.clicked_points)}/4", 
#                         (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
#             cv2.imshow("Calibration", temp_frame)
#             cv2.waitKey(1)
#             return

#         # PHASE 2: Detection Mode (Runs after 4 points are clicked)
#         rx1, ry1, rx2, ry2 = self.roi
#         # Ensure ROI is within frame boundaries
#         h_f, w_f = frame.shape[:2]
#         rx1, ry1, rx2, ry2 = max(0, rx1), max(0, ry1), min(w_f, rx2), min(h_f, ry2)
        
#         roi_frame = frame[ry1:ry2, rx1:rx2]
#         results = self.model(roi_frame, verbose=False, conf=0.5)

#         annotated_frame = frame.copy()
#         cv2.polylines(annotated_frame, [self.workspace_polygon], True, (0, 255, 0), 2)

#         for box in results[0].boxes:
#             bx1, by1, bx2, by2 = [int(v) for v in box.xyxy[0].tolist()]
#             # Offset detection back to full frame coordinates
#             cx, cy = (bx1 + bx2) // 2 + rx1, (by1 + by2) // 2 + ry1

#             # Point in Polygon check
#             if cv2.pointPolygonTest(self.workspace_polygon.astype(np.float32), (float(cx), float(cy)), False) >= 0:
#                 x_m, y_m = self.pixel_to_robot(cx, cy)
                
#                 # Publish to IK
#                 ik_msg = Float32MultiArray()
#                 ik_msg.data = [x_m, y_m]
#                 self.ik_pub.publish(ik_msg)

#                 # Draw info
#                 cv2.rectangle(annotated_frame, (bx1+rx1, by1+ry1), (bx2+rx1, by2+ry1), (0, 255, 255), 2)
#                 cv2.putText(annotated_frame, f"{x_m:.3f}, {y_m:.3f}", (cx, cy-10), 
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

#         self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))

# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()




#!/usr/bin/env python3

import threading
import os
import yaml
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

# ── WORKSPACE CONFIGURATION (30cm x 30cm) ──────────────────────────────────
# Dimensions in meters. Origin (0,0) is center-bottom.
MARKER_POSITIONS = {
    1: np.array([-0.15,  0.30]), # Top-Left
    2: np.array([ 0.15,  0.30]), # Top-Right
    3: np.array([ 0.15,  0.00]), # Bottom-Right
    4: np.array([-0.15,  0.00]), # Bottom-Left
}

WORKSPACE_X_BOUNDS = (-0.15, 0.15)
WORKSPACE_Y_BOUNDS = (0.00, 0.30)

WINDOW = "Workspace Calibrator | Click TL->TR->BR->BL | R=reset S=save Q=quit"

class WorkspaceCalibrator(Node):

    def __init__(self):
        super().__init__('workspace_calibrator')

        self.declare_parameter('image_topic',   '/image_raw')
        self.declare_parameter('save_path',     os.path.expanduser('~/workspace_calibration.yaml'))
        self.declare_parameter('preview_scale', 1.0)

        self.topic         = self.get_parameter('image_topic').value
        self.save_path     = self.get_parameter('save_path').value
        self.preview_scale = self.get_parameter('preview_scale').value

        self.bridge      = CvBridge()
        self.frame_lock  = threading.Lock()
        self.latest_frame = None          

        # Calibration state
        self.H                 = None
        self.roi_locked        = False
        self.manual_points     = [] # 4 pixel corners
        self.clicked_points    = [] # History of inspection clicks
        self.current_mouse_px  = (0, 0) # Live mouse tracking

        self.sub = self.create_subscription(
            Image, self.topic, self._image_cb, 10)

        self.get_logger().info(f"Subscribed to [{self.topic}]")

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = frame.copy()
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    def _compute_manual_homography(self):
        if len(self.manual_points) < 4:
            return
        src = np.array(self.manual_points, dtype=np.float32)
        dst = np.array([MARKER_POSITIONS[i] for i in range(1, 5)], dtype=np.float32)
        self.H, _ = cv2.findHomography(src, dst)
        self.roi_locked = True
        self.get_logger().info("Workspace Locked: 30cm x 30cm.")

    def _pixel_to_robot(self, px, py):
        if self.H is None: return 0.0, 0.0
        pt  = np.array([[[float(px), float(py)]]], dtype=np.float32)
        res = cv2.perspectiveTransform(pt, self.H)
        return float(res[0, 0, 0]), float(res[0, 0, 1])

    def _mouse_cb(self, event, x, y, flags, param):
        s  = self.preview_scale
        pu, pv = int(x / s), int(y / s)
        self.current_mouse_px = (pu, pv) # Update live tracking[cite: 1]

        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.roi_locked:
                self.manual_points.append([pu, pv])
                if len(self.manual_points) == 4:
                    self._compute_manual_homography()
            else:
                xm, ym = self._pixel_to_robot(pu, pv)
                self.clicked_points.append({'pixel': (pu, pv), 'robot': (xm, ym)})

    def _draw(self, frame):
        vis = frame.copy()
        s   = self.preview_scale

        if not self.roi_locked:
            for i, pt in enumerate(self.manual_points):
                cv2.circle(vis, (int(pt[0]*s), int(pt[1]*s)), 6, (0, 0, 255), -1)
            cv2.putText(vis, f"CLICK CORNER {len(self.manual_points)+1}/4", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            # 1. Draw Workspace Border
            poly = (np.array(self.manual_points, dtype=np.float32) * s).astype(np.int32)
            cv2.polylines(vis, [poly], True, (0, 255, 0), 2)

            # 2. Display LIVE coordinates at mouse position[cite: 1]
            xm_live, ym_live = self._pixel_to_robot(self.current_mouse_px[0], self.current_mouse_px[1])
            live_text = f"LIVE X: {xm_live:.3f}m, Y: {ym_live:.3f}m"
            cv2.putText(vis, live_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

            # 3. Draw saved click history
            for pt in self.clicked_points:
                px, py = int(pt['pixel'][0]*s), int(pt['pixel'][1]*s)
                cv2.circle(vis, (px, py), 5, (0, 255, 0), -1)
                cv2.putText(vis, f"({pt['robot'][0]:.2f}, {pt['robot'][1]:.2f})", (px+5, py-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        return vis

    def _save(self):
        data = {'homography': self.H.tolist(), 'timestamp': datetime.now().isoformat()}
        with open(self.save_path, 'w') as f:
            yaml.dump(data, f)
        self.get_logger().info(f"Saved to {self.save_path}")

    def _reset(self):
        self.H = None
        self.roi_locked = False
        self.manual_points = []
        self.clicked_points = []

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceCalibrator()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW, node._mouse_cb)

    try:
        while rclpy.ok():
            with node.frame_lock:
                frame = node.latest_frame.copy() if node.latest_frame is not None else None
            if frame is not None:
                cv2.imshow(WINDOW, node._draw(frame))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27): break
            elif key == ord('r'): node._reset()
            elif key == ord('s') and node.H is not None: node._save()
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()