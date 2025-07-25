from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 경로
    moveit_config_pkg = get_package_share_directory('turtlebot3_manipulation_moveit_config')
    executor_pkg = get_package_share_directory('grasp_executor_cpp')

    # move_group 실행 포함
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        )
    )

    # grasp_executor 노드 실행
    grasp_executor_node = Node(
        package='grasp_executor_cpp',
        executable='grasp_executor',
        name='grasp_executor_cpp_node',
        output='screen',
    )

    return LaunchDescription([
        move_group_launch,
        grasp_executor_node
    ])