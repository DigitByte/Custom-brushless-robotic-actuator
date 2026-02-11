from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_sine_publisher',
            executable='joint_publisher_node',
            name='joint_publisher_node',
            parameters=['config/joint_publisher_config.yaml']
        )
    ])
