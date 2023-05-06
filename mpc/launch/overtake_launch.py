from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpc',
            executable='overtaking_node.py',
            name='ego',
            parameters=[{'ego':True}]
        ),
        Node(
            package='mpc',
            executable='overtaking_node.py',
            name='blocker',
            parameters=[{'ego':False}]
        ),
    ])