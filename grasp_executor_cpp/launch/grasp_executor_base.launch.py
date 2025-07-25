import os
import xacro
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context, *args, **kwargs):
    # 🦾 robot_description 생성 (URDF from xacro)
    desc_pkg_path = FindPackageShare("turtlebot3_manipulation_description").find("turtlebot3_manipulation_description")
    xacro_path = os.path.join(desc_pkg_path, "urdf", "turtlebot3_manipulation.urdf.xacro")
    robot_description = xacro.process_file(xacro_path).toxml()

    # ⚙️ MoveIt config 경로 설정
    moveit_pkg_path = FindPackageShare("turtlebot3_manipulation_moveit_config").find("turtlebot3_manipulation_moveit_config")

    # 🧠 kinematics.yaml 로딩 (MoveIt 용)
    kinematics_path = os.path.join(moveit_pkg_path, "config", "kinematics.yaml")
    with open(kinematics_path, 'r') as f:
        kinematics_data = yaml.safe_load(f)

    # 🤖 MoveGroup 실행 포함
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_manipulation_moveit_config"),
                "launch",
                "move_group.launch.py"
            ])
        ])
    )

    # 🧠 GraspExecutor 노드 실행
    grasp_node = Node(
        package="grasp_executor_cpp",
        executable="grasp_executor",
        name="grasp_executor_cpp_node",
        output="screen"
        # ❌ 파라미터 전달 불필요
    )

    return [moveit_launch, grasp_node]
