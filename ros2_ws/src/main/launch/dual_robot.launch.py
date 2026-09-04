from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # PS4 Controller A -> publishes to /joy_a
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_a',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
            }],
            remappings=[
                ('/joy', '/joy_a')
            ]
        ),
        # PS4 Controller B -> publishes to /joy_b
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_b',
            parameters=[{
                'device_id': 1,
                'deadzone': 0.05,
            }],
            remappings=[
                ('/joy', '/joy_b')
            ]
        ),
        # Main Controller Node
        Node(
            package='main',
            executable='main',
            name='dual_robot_controller',
            output='screen'
        )
    ])
