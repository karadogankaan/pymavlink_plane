# launch/launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kaan_deneme',
            executable='kaan1',
            name='mov1_node',
            output='screen'
        ),
        Node(
            package='kaan_deneme',
            executable='kaan2',
            name='mov2_node',
            output='screen'
        ),
        Node(
            package='kaan_deneme',
            executable='kaan3',
            name='mov3_node',
            output='screen'
        )
    ])

