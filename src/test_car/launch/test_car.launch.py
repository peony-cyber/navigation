import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('test_car')

    config_path = os.path.join(pkg_share, 'cfg', 'goalpub_param.yaml')

    declare_param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=config_path
    )
    
    test_car_node = Node(
        package='test_car',
        executable='test_car',
        name='goal_publisher',
        parameters=[config_path]
    )
    print(f"Loading params from: {config_path}")


    return LaunchDescription([
        declare_param_file_cmd,
        test_car_node,
    ])