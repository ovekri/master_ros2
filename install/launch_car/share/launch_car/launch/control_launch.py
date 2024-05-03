from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='xbox_elite_pad'),
        Node(
            package='lidar',
            executable='lidar',
            name='lidar',
            output='screen'),
        Node(
            package='plane_fitter',
            executable='plane_fitter',
            name='plane_fitter',
            output='screen'),
        Node(
            package='xbox',
            executable='new_control',
            name='new_control',
            output='screen')
  ])
