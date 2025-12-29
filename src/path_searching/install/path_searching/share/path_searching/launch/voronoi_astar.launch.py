import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('path_searching')
    config_path = os.path.join(pkg_share, 'cfg', 'plan_param.yaml')

    declare_param_file_cmd=DeclareLaunchArgument(
      'param_file',
      default_value= config_path
      # 'Full path to the ROS2 parameters file'
    )  #声明后可直接在命令行传入参数文件

    plan_manager = Node(
        package='path_searching',
        executable='plan_manager',
        name='plan_manager',
        output='screen',
        respawn=False,
        parameters=[LaunchConfiguration('param_file')],
    )

    # pid_follower = Node(
    #     package='path_following',
    #     executable='pid_follower',
    #     name='pid_follower',
    #     output='screen',
    # )

    return LaunchDescription([
        declare_param_file_cmd,
        plan_manager,
        # pid_follower,
        # If needed add map_server or rviz nodes here:
        # Node(package='nav2_map_server', executable='map_server', name='map_server',
        #      output='screen', parameters=['/path/to/your/map.yaml']),
        # Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
        #      arguments=['-d', '/path/to/config.rviz']),
    ])