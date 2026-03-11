import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('goal_pub')

    config_path = os.path.join(pkg_share, 'cfg', 'goalpub_param.yaml')

    declare_param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=config_path
    )
    
    goal_manager_node = Node(
        package='goal_pub',
        executable='multigoal_pub_node',
        name='goal_publisher',
        parameters=[config_path]
    )
    print(f"Loading params from: {config_path}")


    return LaunchDescription([
        declare_param_file_cmd,
        goal_manager_node,
    ])