#!/usr/bin/env python3
"""Launch crawler_mujoco as a sensor/robot simulator without ground-truth TF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("crawler_mujoco"),
            "launch",
            "crawler_mujoco.launch.py",
        ])),
        launch_arguments={
            "publish_ground_truth_tf": LaunchConfiguration(
                "publish_ground_truth_tf"),
            "config": LaunchConfiguration("config"),
            "shape": LaunchConfiguration("shape"),
            "arena_yaml": LaunchConfiguration("arena_yaml"),
            "mujoco_gui": LaunchConfiguration("mujoco_gui"),
            "show_contacts": LaunchConfiguration("show_contacts"),
            "publish_truth_cloud": LaunchConfiguration("publish_truth_cloud"),
            "lidar_output_format": LaunchConfiguration("lidar_output_format"),
            "topic_gui": LaunchConfiguration("topic_gui"),
            "flipper_gui": LaunchConfiguration("flipper_gui"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution([
                FindPackageShare("crawler_mujoco"), "config", "default.yaml"])),
        DeclareLaunchArgument("shape", default_value="semicircle"),
        DeclareLaunchArgument("arena_yaml", default_value="benchmark.yaml"),
        DeclareLaunchArgument("mujoco_gui", default_value="true"),
        DeclareLaunchArgument("show_contacts", default_value="false"),
        DeclareLaunchArgument("publish_truth_cloud", default_value="true"),
        DeclareLaunchArgument("publish_ground_truth_tf", default_value="false"),
        DeclareLaunchArgument("lidar_output_format", default_value="both"),
        DeclareLaunchArgument("topic_gui", default_value="true"),
        DeclareLaunchArgument("flipper_gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        simulator,
    ])


if __name__ == "__main__":
    generate_launch_description()
