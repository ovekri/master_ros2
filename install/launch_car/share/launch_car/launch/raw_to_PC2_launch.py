import launch
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    """Generate launch description for converting Realsense data to PointCloud2."""
    container = ComposableNodeContainer(
        name='point_cloud_converter',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rtabmap_util',
                plugin='rtabmap_util::PointCloudXYZ',
                name='points_xyz_rt',
                remappings=[
                            ("depth/image", "/camera/depth/image_rect_raw"),
                            ("depth/camera_info", "/camera/depth/camera_info"),
                            ("cloud", "/camera/depth/color/points")
                            ],
                parameters=[
                            {"decimation": 4},
                            {"voxel_size": 0.05},
                            {"approx_sync": False}
                            ]
                ),
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])

# https://ashbabu.github.io/blog/2023/Obstacle-Detection-from-Pointclouds/