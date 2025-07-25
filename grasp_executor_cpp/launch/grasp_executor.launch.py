from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    # 경로 설정
    moveit_config_pkg = FindPackageShare("turtlebot3_manipulation_moveit_config")
    description_pkg = FindPackageShare("turtlebot3_manipulation_description")
    grasp_executor_pkg = FindPackageShare("grasp_executor_cpp")

    robot_description_path = PathJoinSubstitution(
        [description_pkg, "urdf", "turtlebot3_manipulation.urdf.xacro"]
    )

    srdf_path = PathJoinSubstitution(
        [moveit_config_pkg, "config", "turtlebot3_manipulation.srdf"]
    )

    kinematics_yaml = PathJoinSubstitution(
        [moveit_config_pkg, "config", "kinematics.yaml"]
    )

    joint_limits_yaml = PathJoinSubstitution(
        [moveit_config_pkg, "config", "joint_limits.yaml"]
    )

    ompl_yaml = PathJoinSubstitution(
        [moveit_config_pkg, "config", "ompl_planning.yaml"]
    )

    controllers_yaml = PathJoinSubstitution(
        [moveit_config_pkg, "config", "moveit_controllers.yaml"]
    )

    robot_description_command = f"xacro {robot_description_path}"

    # Move Group 실행
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_description_command},
            {"robot_description_semantic": srdf_path},
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": ompl_yaml},
            {"robot_description_joint_limits": joint_limits_yaml},
            controllers_yaml
        ]
    )

    # Grasp Executor Node 실행
    grasp_executor_node = Node(
        package="grasp_executor_cpp",
        executable="grasp_executor",
        output="screen"
    )

    return LaunchDescription([
        move_group_node,
        grasp_executor_node
    ])
