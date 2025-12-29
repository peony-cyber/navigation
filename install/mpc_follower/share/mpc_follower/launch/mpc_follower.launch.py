from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mpc_follower_node = Node(
        package='mpc_follower',
        executable='mpc_node',
        name='mpc_follower',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('mpc_follower'),
            'cfg',
            'mpc_params.yaml'
        )]
    )

    return LaunchDescription([
        mpc_follower_node,
    ])