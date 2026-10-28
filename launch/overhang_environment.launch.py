#!/usr/bin/env python3
"""Launch the standalone four-overhang MuJoCo environment."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    crawler_share = FindPackageShare("crawler_mujoco")

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([crawler_share, "launch", "crawler_mujoco.launch.py"])
        ),
        launch_arguments={
            "config": LaunchConfiguration("config"),
            "arena_yaml": LaunchConfiguration("arena_yaml"),
            "mujoco_gui": LaunchConfiguration("mujoco_gui"),
            "show_contacts": LaunchConfiguration("show_contacts"),
            "spawn_robot": LaunchConfiguration("spawn_robot"),
            "publish_truth_cloud": LaunchConfiguration("publish_truth_cloud"),
            "cloud_resolution": LaunchConfiguration("cloud_resolution"),
            "pointcloud_mode": LaunchConfiguration("pointcloud_mode"),
            "topic_gui": LaunchConfiguration("topic_gui"),
            "flipper_gui": LaunchConfiguration("flipper_gui"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution([
                crawler_share, "config", "overhang_sensor.yaml"]),
            description="MuJoCo configuration; overhang default enables RGB-D scans"),
        DeclareLaunchArgument(
            "arena_yaml",
            default_value="custom/overhang_environment.yaml",
            description=(
                "Arena YAML: bundled filename, custom/<filename>.yaml, or absolute path")),
        DeclareLaunchArgument("mujoco_gui", default_value="true"),
        DeclareLaunchArgument("show_contacts", default_value="false"),
        DeclareLaunchArgument(
            "spawn_robot", default_value="false",
            description="Spawn the crawler robot in the overhang environment"),
        DeclareLaunchArgument("publish_truth_cloud", default_value="true"),
        DeclareLaunchArgument("cloud_resolution", default_value="0.025"),
        DeclareLaunchArgument(
            "pointcloud_mode", default_value="3d_voxel",
            description="Published cloud representation: 2.5d or 3d_voxel"),
        DeclareLaunchArgument("topic_gui", default_value="true"),
        DeclareLaunchArgument("flipper_gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        simulator,
    ])


if __name__ == "__main__":
    generate_launch_description()
