from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
      Node(
            package='utaustin',
            namespace='agents',
            executable='agent_ros',
            name='agent1'
        )
      ])