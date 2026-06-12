# # from launch import LaunchDescription
# # from launch_ros.actions import Node

# # def generate_launch_description():
# #     return LaunchDescription([
# #         # Camera driver node (v4l2_camera)
# #         Node(
# #             package="v4l2_camera",
# #             executable="v4l2_camera_node",
# #             name="camera_driver",
# #             output="screen",
# #             parameters=[{
# #                 "video_device": "/dev/video2",
# #                 "image_size": [640, 480],
# #                 "pixel_format": "YUYV",   # or "MJPEG" depending on your camera
# #                 "output_encoding": "rgb8"
# #             }],
# #             remappings=[
# #                 ("image_raw", "/image_raw"),
# #                 ("camera_info", "/camera_info")
# #             ]
# #         ),

# #         # Your YOLO node (yolotest.py)
# #         Node(
# #             package="yolo_pick_place",   # your package name
# #             executable="yolotest",       # must match entry point in setup.py
# #             name="yolo_node",
# #             output="screen",
# #             remappings=[
# #                 ("image_raw", "/image_raw"),
# #                 ("detections", "/yolo/detections"),
# #                 ("annotated", "/yolo/annotated")
# #             ]
# #         ),

# #         # rqt_image_view for visualization
# #         Node(
# #             package="rqt_image_view",
# #             executable="rqt_image_view",
# #             name="image_view",
# #             output="screen"
# #         )
# #     ])


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
#                 "video_device": "/dev/video3",
#                 "image_size": [640, 480],
#                 "pixel_format": "YUYV",
#                 "output_encoding": "rgb8"
#             }],
#             remappings=[
#                 ("image_raw", "/image_raw"),
#                 ("camera_info", "/camera_info")
#             ]
#         ),

#         # SCARA YOLO pick and place node
#         Node(
#             package="yolo_pick_place",
#             executable="yolopickplace",
#             name="scara_yolo_node",
#             output="screen",
#             remappings=[
#                 ("image_raw", "/image_raw"),
#             ]
#         ),
#     ])







from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Camera driver node (v4l2_camera) ──────────────────────────────
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="camera_driver",
            output="screen",
            parameters=[{
                "video_device": "/dev/video2",
                "image_size": [640, 480],
                "pixel_format": "YUYV",
                "output_encoding": "rgb8"
            }],
            remappings=[
                ("image_raw", "/image_raw"),
                ("camera_info", "/camera_info")
            ]
        ),

        # ── SCARA YOLO pick-and-place node ────────────────────────────────
        Node(
            package="yolo_pick_place",
            executable="yolopickplace",
            name="scara_yolo_node",
            output="screen",
            remappings=[
                ("image_raw", "/image_raw"),
            ]
        ),

        # ── Inverse-kinematics node (from SCARA_pkg) ──────────────────────
        Node(
            package="SCARA_pkg",
            executable="ikpos",
            name="ikpos_node",
            output="screen",
        ),

        # ── New-position / joint command node (from SCARA_pkg) ────────────
        Node(
            package="SCARA_pkg",
            executable="newposition",
            name="newposition_node",
            output="screen",
        ),
    ])

