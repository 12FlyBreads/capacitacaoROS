from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_drone',
            executable='drone_node',
            name='drone_node'
        ),
        Node(
            package='sim_drone',
            executable='lidar_node',
            name='lidar_node'
        ),
        Node(
            package='sim_drone',
            executable='obstaculo',
            name='obstaculo'
        ),
    ])

