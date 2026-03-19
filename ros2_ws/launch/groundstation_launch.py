from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='game_controller_node',
            name = 'gs',
            arguments=['--ros-args', '-p', 'coalesce_interval_ms:=20']
        ),
        Node(
            package='jetson',
            executable='dualsense',
            name='gs',
        )
    ])