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

    static_tf_pub2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub2',
        output='screen',
        arguments=['0.5', '0.5', '0', '-1.570796', '0', '0', 'map', 'baselink'],
    )


    return LaunchDescription([
        declare_param_file_cmd,
        test_car_node,
        static_tf_pub2,
    ])