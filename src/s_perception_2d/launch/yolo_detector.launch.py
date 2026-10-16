#!/usr/bin/env python3
"""
Launch file for s_perception_2d YOLO detector node.

Usage:
  ros2 launch s_perception_2d yolo_detector.launch.py
  ros2 launch s_perception_2d yolo_detector.launch.py input_image_topic:=/terrascout1/oakd/rgb/image_raw
  ros2 launch s_perception_2d yolo_detector.launch.py confidence_threshold:=0.40
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("s_perception_2d")
    default_config_file = os.path.join(pkg_share, "config", "yolo_detector.yaml")

    # Declare Launch Arguments
    config_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_config_file,
        description="Path to ROS 2 parameters YAML file",
    )

    input_topic_arg = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/terrascout1/oakd/rgb/image_raw",
        description="Camera image topic to subscribe to (e.g. /terrascout1/oakd/rgb/image_raw)",
    )

    output_topic_arg = DeclareLaunchArgument(
        "output_detections_topic",
        default_value="/terrascout1/oakd/detections_2d",
        description="Detections topic to publish vision_msgs/msg/Detection2DArray on",
    )

    conf_thresh_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.35",
        description="Minimum confidence threshold for detections",
    )

    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Path to custom YOLO model weights (.pt file). Leave empty for default package model.",
    )

    publish_annotated_arg = DeclareLaunchArgument(
        "publish_annotated_image",
        default_value="true",
        description="Whether to publish annotated image with bounding boxes for Foxglove/visualization",
    )

    annotated_topic_arg = DeclareLaunchArgument(
        "annotated_image_topic",
        default_value="/terrascout1/oakd/detections_image",
        description="Topic to publish annotated image with bounding boxes on",
    )

    # Detector Node
    detector_node = Node(
        package="s_perception_2d",
        executable="detector_node.py",
        name="yolo_detector_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "input_image_topic": LaunchConfiguration("input_image_topic"),
                "output_detections_topic": LaunchConfiguration("output_detections_topic"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                "model_path": LaunchConfiguration("model_path"),
                "publish_annotated_image": LaunchConfiguration("publish_annotated_image"),
                "annotated_image_topic": LaunchConfiguration("annotated_image_topic"),
            },
        ],
    )

    return LaunchDescription(
        [
            config_file_arg,
            input_topic_arg,
            output_topic_arg,
            conf_thresh_arg,
            model_path_arg,
            publish_annotated_arg,
            annotated_topic_arg,
            detector_node,
        ]
    )
