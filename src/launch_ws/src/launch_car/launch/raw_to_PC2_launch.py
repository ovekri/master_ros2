import launch
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    #Generate launch description for converting Realsense data to PointCloud2.
    container = ComposableNodeContainer(
        name='point_cloud_converter',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rtabmap_util',#rtabmap_util
                plugin='rtabmap_util::PointCloudXYZ',#rtabmap_util::PointCloudXYZ
                name='points_xyz_rt', #points_xyz_rt
                remappings=[
                            ("depth/image", "/camera/depth/image_rect_raw"),
                            ("depth/camera_info", "/camera/depth/camera_info"),
                            ("cloud", "/camera/depth/color/points")
                            ],
                parameters=[
                            {"decimation": 2},
                            {"voxel_size": 0.0},
                            {"approx_sync": False},
                            #{"filter_nans": True},
                            #{"min_depth": 0.2},
                            #{"max_depth": 4.0},
                            #{"noise_filter_radius": 0.05},
                            #{"noise_filter_min_neighbors": 5},
                            #{"normal_k": 6},
                            #{"normal_radius": 0}
                            #{"roi_ratios": "0.1 0.1 0.0 0.0"} # [left, right, top, bottom] string format
                            ]
                ),
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])

# https://ashbabu.github.io/blog/2023/Obstacle-Detection-from-Pointclouds/



"""
import launch
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    # Generate launch description for converting Realsense data to colored PointCloud2.
    container = ComposableNodeContainer(
        name='point_cloud_converter',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rtabmap_util',  # Assuming this is the correct package name
                plugin='rtabmap_util::PointCloudXYZRGB',  # Changed to PointCloudXYZRGB for color support
                name='points_xyz_rgb',  # Optional: changed the node name for clarity
                remappings=[
                    ("depth/image", "/camera/depth/image_rect_raw"),
                    ("depth/camera_info", "/camera/depth/camera_info"),
                    ("rgb/image", "/camera/color/image_raw"),  # Ensure this topic is correct
                    ("rgb/camera_info", "/camera/color/camera_info"),  # Ensure this topic is correct
                    ("cloud", "/camera/depth/color/points")  # Output topic
                ],
                parameters=[
                    {"decimation": 4},
                    {"voxel_size": 0.02},
                    {"approx_sync": True}
                ]
            ),
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])





#Launch a obstacle segmentation in a component container.

import launch
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    #Generate launch description with multiple components.
    container = ComposableNodeContainer(
        name='obstacle_detection',
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
            ComposableNode(
                package='rtabmap_util',
                plugin='rtabmap_util::ObstaclesDetection',
                name='obstacle_detection_rt',
                remappings=[
                            ("cloud", "/cropped_pointcloud")
                            ],
                parameters=[
                            {"frame_id": "camera_link"},
                            {"wait_for_transform": 1.0},
                            {"Grid/MaxGroundHeight": "0.04"}
                            ]
                )
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])

"""