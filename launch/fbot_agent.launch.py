import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory(package_name='fbot_agent'), 'config', 'fbot_agent_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='fbot_agent',
            executable='fbot_agent_node',
            namespace='fbot_agent',
            parameters=[config,],
            output='screen'
        )
    ])