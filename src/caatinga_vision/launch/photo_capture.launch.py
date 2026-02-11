#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("caatinga_vision")
    params_file = PathJoinSubstitution([pkg_share, "config", "photo_capture_params.yaml"])

    session_id = LaunchConfiguration("session_id")
    camera_index = LaunchConfiguration("camera_index")
    fps = LaunchConfiguration("fps")
    capture_distance_m = LaunchConfiguration("capture_distance_m")
    speed_min_m_s = LaunchConfiguration("speed_min_m_s")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    jpeg_quality = LaunchConfiguration("jpeg_quality")
    reserve_space_mb = LaunchConfiguration("reserve_space_mb")
    usb_mount_path = LaunchConfiguration("usb_mount_path")
    status_cache_path = LaunchConfiguration("status_cache_path")
    preview_cache_path = LaunchConfiguration("preview_cache_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("session_id", default_value=""),
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("fps", default_value="8.0"),
            DeclareLaunchArgument("capture_distance_m", default_value="0.5"),
            DeclareLaunchArgument("speed_min_m_s", default_value="0.05"),
            DeclareLaunchArgument("image_width", default_value="1280"),
            DeclareLaunchArgument("image_height", default_value="720"),
            DeclareLaunchArgument("jpeg_quality", default_value="90"),
            DeclareLaunchArgument("reserve_space_mb", default_value="512"),
            DeclareLaunchArgument("usb_mount_path", default_value=""),
            DeclareLaunchArgument("status_cache_path", default_value="/tmp/caatinga_capture_status.json"),
            DeclareLaunchArgument("preview_cache_path", default_value="/tmp/caatinga_capture_latest.jpg"),
            Node(
                package="caatinga_vision",
                executable="photo_capture_node",
                name="photo_capture_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "session_id": session_id,
                        "camera_index": camera_index,
                        "fps": fps,
                        "capture_distance_m": capture_distance_m,
                        "speed_min_m_s": speed_min_m_s,
                        "image_width": image_width,
                        "image_height": image_height,
                        "jpeg_quality": jpeg_quality,
                        "reserve_space_mb": reserve_space_mb,
                        "usb_mount_path": usb_mount_path,
                        "status_cache_path": status_cache_path,
                        "preview_cache_path": preview_cache_path,
                    },
                ],
            ),
        ]
    )
