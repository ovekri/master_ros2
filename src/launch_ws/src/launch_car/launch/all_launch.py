import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Velodyne LiDAR nodes
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('velodyne'),
                'launch',
                'velodyne-all-nodes-VLP16-launch.py'
            ])
        ])
    )

    # Realsense Camera launch and robot state publisher
    robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('realsense2_description') / 'urdf/test_d435_camera.urdf.xacro')]),
        value_type=str
    )
    
    rs_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        launch_arguments={
            "depth_module.profile": "640x480x6", #640x480x30 848x480x15 1280x720x?
            #"rgb_camera.profile": "640x480x6"
        }.items()
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # from realsense raw data to pointcloud2 format
    converter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launch_car'),
                'launch',
                'raw_to_PC2_launch.py'
            ])
        ])
    )
    
    # Broadcaster lidar Node
    lidar_broadcast_node = Node(
        package='broadcaster',
        executable='lidar',
        name='lidar_broadcast'
    )
    
    # Broadcaster realsense node
    realsense_broadcast_node = Node(
        package='broadcaster',
        executable='realsense',
        name='realsense_broadcast'
    )
        
    # Listener Node
    listener_node = Node(
        package='listener',
        executable='listener',
        name='listener'
    )

    # Combine all in LaunchDescription
    return LaunchDescription([
        velodyne_launch,
        rs_cam,
        robot_state_publisher_node,
        converter_launch,
        lidar_broadcast_node,
        realsense_broadcast_node,
        listener_node
    ])
