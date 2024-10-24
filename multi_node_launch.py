from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='kaan',
            name='kaan',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='kaan',
            name='kaan2',
            output='screen'
        ),
        Node(
       	    package='robot_control',
       	    executable='kaan3',
       	    name='kaan3',
       	    output='screen'
       	),
    ])

