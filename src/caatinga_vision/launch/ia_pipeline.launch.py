#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("caatinga_vision")
    params_file = PathJoinSubstitution([pkg_share, "config", "ia_params.yaml"])

    session_id = LaunchConfiguration("session_id")
    camera_index = LaunchConfiguration("camera_index")
    model_path = LaunchConfiguration("model_path")
    confidence_mode = LaunchConfiguration("confidence_mode")
    confidence_threshold_conservador = LaunchConfiguration("confidence_threshold_conservador")
    confidence_threshold_economico = LaunchConfiguration("confidence_threshold_economico")
    inference_device = LaunchConfiguration("inference_device")
    grid_resolution_m = LaunchConfiguration("grid_resolution_m")
    pump_rate_l_min = LaunchConfiguration("pump_rate_l_min")
    status_cache_path = LaunchConfiguration("status_cache_path")
    overlay_cache_path = LaunchConfiguration("overlay_cache_path")
    logs_base_dir = LaunchConfiguration("logs_base_dir")
    dataset_data_yaml = LaunchConfiguration("dataset_data_yaml")
    model_check_status = LaunchConfiguration("model_check_status")
    model_names_b64 = LaunchConfiguration("model_names_b64")
    model_names_json = LaunchConfiguration("model_names_json")

    return LaunchDescription(
        [
            DeclareLaunchArgument("session_id", default_value=""),
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("model_path", default_value="~/models/best.pt"),
            DeclareLaunchArgument("confidence_mode", default_value="economico"),
            DeclareLaunchArgument("confidence_threshold_conservador", default_value="0.50"),
            DeclareLaunchArgument("confidence_threshold_economico", default_value="0.85"),
            DeclareLaunchArgument("inference_device", default_value="cpu"),
            DeclareLaunchArgument("grid_resolution_m", default_value="1.0"),
            DeclareLaunchArgument("pump_rate_l_min", default_value="3.5"),
            DeclareLaunchArgument("status_cache_path", default_value="/tmp/caatinga_vision_status.json"),
            DeclareLaunchArgument("overlay_cache_path", default_value="/tmp/caatinga_vision_overlay.jpg"),
            DeclareLaunchArgument("logs_base_dir", default_value="~/agro_robot_ws/logs_rastreabilidade"),
            DeclareLaunchArgument(
                "dataset_data_yaml",
                default_value="~/agro_robot_ws/datasets/agro_v1/data.yaml",
            ),
            DeclareLaunchArgument("model_check_status", default_value="unknown"),
            DeclareLaunchArgument("model_names_b64", default_value=""),
            # Compatibilidade com fluxo legado do painel (antes de model_names_b64).
            # Mantém string vazia por padrão para evitar parse como sequência YAML vazia no launch.
            DeclareLaunchArgument("model_names_json", default_value=""),
            Node(
                package="caatinga_vision",
                executable="camera_source_node",
                name="camera_source_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "camera_index": camera_index,
                    },
                ],
            ),
            Node(
                package="caatinga_vision",
                executable="yolo_inference_node",
                name="yolo_inference_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "model_path": model_path,
                        "confidence_mode": confidence_mode,
                        "confidence_threshold_conservador": confidence_threshold_conservador,
                        "confidence_threshold_economico": confidence_threshold_economico,
                        "inference_device": inference_device,
                        "overlay_cache_path": overlay_cache_path,
                    },
                ],
            ),
            Node(
                package="caatinga_vision",
                executable="infestation_analytics_node",
                name="infestation_analytics_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "session_id": session_id,
                        "grid_resolution_m": grid_resolution_m,
                        "pump_rate_l_min": pump_rate_l_min,
                        "confidence_mode": confidence_mode,
                        "status_cache_path": status_cache_path,
                        "logs_base_dir": logs_base_dir,
                        "dataset_data_yaml": dataset_data_yaml,
                        "model_path": model_path,
                        "model_check_status": model_check_status,
                        "model_names_b64": model_names_b64,
                        "model_names_json": model_names_json,
                    },
                ],
            ),
        ]
    )
