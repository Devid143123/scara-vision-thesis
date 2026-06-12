# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         # Camera driver node (v4l2_camera)
#         Node(
#             package="v4l2_camera",
#             executable="v4l2_camera_node",
#             name="camera_driver",
#             output="screen",
#             parameters=[{
#                 "video_device": "/dev/video2",
#                 "image_size": [640, 480],
#                 "pixel_format": "YUYV",   # or "MJPEG" depending on your camera
#                 "output_encoding": "rgb8"
#             }],
#             remappings=[
#                 ("image_raw", "/image_raw"),
#                 ("camera_info", "/camera_info")
#             ]
#         ),

#         # Your YOLO node (yolotest.py)
#         Node(
#             package="yolo_pick_place",   # your package name
#             executable="yolotest",       # must match entry point in setup.py
#             name="yolo_node",
#             output="screen",
#             remappings=[
#                 ("image_raw", "/image_raw"),
#                 ("detections", "/yolo/detections"),
#                 ("annotated", "/yolo/annotated")
#             ]
#         ),

#         # rqt_image_view for visualization
#         Node(
#             package="rqt_image_view",
#             executable="rqt_image_view",
#             name="image_view",
#             output="screen"
#         )
#     ])




# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([

#         # Camera driver node (v4l2_camera)
#         Node(
#             package="v4l2_camera",
#             executable="v4l2_camera_node",
#             name="camera_driver",
#             output="screen",
#             parameters=[{
#                 "video_device": "/dev/video2",
#                 "image_size": [640, 480],
#                 "pixel_format": "YUYV",
#                 "output_encoding": "rgb8"
#             }],
#             remappings=[
#                 ("image_raw", "/image_raw"),
#                 ("camera_info", "/camera_info")
#             ]
#         ),

#         # Workspace calibrator — live camera feed + click to inspect coords
#         Node(
#             package="yolo_pick_place",
#             executable="camcal",
#             name="camcal",
#             output="screen",
#             parameters=[{
#                 "image_topic":   "/image_raw",
#                 "preview_scale": 1.0,
#             }]
#         ),

#         # YOLO node
#         Node(
#             package="yolo_pick_place",
#             executable="yolotest",
#             name="yolo_node",
#             output="screen",
#             remappings=[
#                 ("image_raw", "/image_raw"),
#                 ("detections", "/yolo/detections"),
#                 ("annotated",  "/yolo/annotated")
#             ]
#         ),

#         # rqt_image_view for visualization
#         Node(
#             package="rqt_image_view",
#             executable="rqt_image_view",
#             name="image_view",
#             output="screen"
#         )
#     ])




# #### This is for External Camera Realsense
# #### This is for External Camera Realsense
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         # RealSense camera driver node
#         Node(
#             package="realsense2_camera",
#             executable="realsense2_camera_node",
#             name="realsense_driver",
#             output="screen",
#             parameters=[{
#                 "enable_color": True,
#                 "color_width": 640,
#                 "color_height": 480,
#                 "color_fps": 30,
#                 "enable_depth": False,    # disabled for stability
#                 "enable_infra1": False,
#                 "enable_infra2": False,
#                 "enable_gyro": False,
#                 "enable_accel": False
#             }],
#             remappings=[
#                 ("color/image_raw", "/camera/realsense_driver/color/image_raw"),
#                 ("color/camera_info", "/camera/realsense_driver/color/camera_info")
#             ]
#         ),

#         # Your YOLO node (yolotest.py)
#         Node(
#             package="yolo_pick_place",   # your package name
#             executable="yolotest",       # must match entry point in setup.py
#             name="yolo_node",
#             output="screen",
#             remappings=[
#                 ("image_raw", "/camera/realsense_driver/color/image_raw"),  # <-- subscribe here
#                 ("annotated", "/yolo/annotated")                           # <-- publish here
#             ]
#         ),

#         # rqt_image_view for visualization
#         Node(
#             package="rqt_image_view",
#             executable="rqt_image_view",
#             name="image_view",
#             output="screen",
#             remappings=[
#                 ("image", "/yolo/annotated")   # view YOLO annotated output
#             ]
#         )
#     ])





# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         # RealSense camera driver node
#         Node(
#             package="realsense2_camera",
#             executable="realsense2_camera_node",
#             name="realsense_driver",
#             output="screen",
#             parameters=[{
#                 "enable_color": True,
#                 "color_width": 640,
#                 "color_height": 480,
#                 "color_fps": 30,
#                 "enable_depth": False,
#                 "enable_infra1": False,
#                 "enable_infra2": False,
#                 "enable_gyro": False,
#                 "enable_accel": False
#             }],
#             remappings=[
#                 ("color/image_raw", "/image_raw"),      # matches YOLO node
#                 ("color/camera_info", "/camera_info")  # consistent with earlier setup
#             ]
#         ),

#         # YOLO node
#         Node(
#             package="yolo_pick_place",
#             executable="yolotest",
#             name="yolo_node",
#             output="screen",
#             remappings=[
#                 ("image_raw", "/image_raw"),          # subscribe to camera feed
#                 ("detections", "/yolo/detections"),   # publish detections
#                 ("annotated", "/yolo/annotated")      # publish annotated image
#             ]
#         ),

#         # rqt_image_view for visualization
#         Node(
#             package="rqt_image_view",
#             executable="rqt_image_view",
#             name="image_view",
#             output="screen",
#             parameters=[{"image_topic": "/yolo/annotated"}]
#         )
#     ])





from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Inverse-kinematics node (receives /ik_target) ─────────────────
        Node(
            package="SCARA_pkg",
            executable="ikpos",
            name="ikpos_node",
            output="screen",
        ),

        # ── New-position / joint command node (angles → CAN) ──────────────
        Node(
            package="SCARA_pkg",
            executable="newposition",
            name="newposition_node",
            output="screen",
        ),

        # ── Physical test node (traces square, logs CSV) ──────────────────
        Node(
            package="yolo_pick_place",
            executable="resultplot",
            name="resultplot_node",
            output="screen",
        ),
    ])