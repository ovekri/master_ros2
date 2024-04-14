from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('velodyne'),
                    'launch',
                    'velodyne-all-nodes-VLP16-launch.py'
                ])
            ])
        ), 
        Node(
            package='broadcaster',
            executable='lidar',
            name='lidar_broadcast'),
        Node(
            package='broadcaster',
            executable='lidar',
            name='lidar_broadcast'),
        Node(
            package='listener',
            executable='listener',
            name='listener'),        
  ])
