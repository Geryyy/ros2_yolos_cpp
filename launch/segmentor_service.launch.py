# Copyright 2024 YOLOs-CPP Team
# SPDX-License-Identifier: AGPL-3.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch YOLO segmentor service node as composable component."""

    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        "model_path", description="Path to ONNX model file"
    )

    labels_path_arg = DeclareLaunchArgument(
        "labels_path", description="Path to class names file"
    )

    use_gpu_arg = DeclareLaunchArgument(
        "use_gpu", default_value="false", description="Enable GPU inference"
    )

    conf_threshold_arg = DeclareLaunchArgument(
        "conf_threshold", default_value="0.4", description="Confidence threshold"
    )

    nms_threshold_arg = DeclareLaunchArgument(
        "nms_threshold", default_value="0.45", description="NMS threshold"
    )

    # Composable node container
    container = ComposableNodeContainer(
        name="yolos_segmentor_service_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="ros2_yolos_cpp",
                plugin="ros2_yolos_cpp::YolosSegmentorServiceNode",
                name="yolos_segmentor_service",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "labels_path": LaunchConfiguration("labels_path"),
                        "use_gpu": LaunchConfiguration("use_gpu"),
                        "conf_threshold": LaunchConfiguration("conf_threshold"),
                        "nms_threshold": LaunchConfiguration("nms_threshold"),
                    }
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            model_path_arg,
            labels_path_arg,
            use_gpu_arg,
            conf_threshold_arg,
            nms_threshold_arg,
            container,
        ]
    )
