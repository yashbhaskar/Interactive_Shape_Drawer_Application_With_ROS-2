from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='shapes_pkg',
            executable='square.py',
            name='Square'
        ),
        Node(
            package='simple_pkg',
            executable='star.py',
            name='Star'
        ),
        Node(
            package='simple_pkg',
            executable='triangle.py',
            name='Triangle'
        )
        
    ])
