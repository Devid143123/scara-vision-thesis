"""SCARA Simulation v3 - new URDF + mock hardware + RViz."""
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():
    pkg = FindPackageShare("scara_sim")
    urdf = PathJoinSubstitution([pkg, "urdf", "scara.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", "controllers.yaml"])
    rviz_cfg = PathJoinSubstitution([pkg, "config", "scara.rviz"])
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", urdf]), value_type=str)
    }
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[robot_description])
    cm = Node(package="controller_manager", executable="ros2_control_node",
              output="screen", parameters=[robot_description, controllers])
    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    arm = Node(package="controller_manager", executable="spawner",
               arguments=["scara_arm_controller", "--controller-manager", "/controller_manager"])
    rviz = Node(package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", rviz_cfg])
    delay_arm = RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm]))
    return LaunchDescription([rsp, cm, jsb, delay_arm, rviz])
