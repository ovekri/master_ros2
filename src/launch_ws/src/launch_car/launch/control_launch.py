from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='xbox_elite_pad'),
        Node(
            package='lidar_control',
            executable='lidar_control',
            name='lidar_control',
            output='screen'),
        Node(
            package='xbox',
            executable='xbox',
            name='xbox_manual_control',
            output='screen'),
  ])
