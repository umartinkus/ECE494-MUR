from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                # common knobs:
                # 'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }],
        ),

        Node(
            package='ground_station',
            executable='uart_talker',
            name='uart_talker',
            output='screen'
        )
    ])

