#!/usr/bin/env python3
"""Launch logix facing a 30 cm step whose leading face is 1 m ahead."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    crawler_share = FindPackageShare("crawler_mujoco")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([crawler_share, "launch", "crawler_mujoco.launch.py"])
        ),
        launch_arguments={
            "config": LaunchConfiguration("config"),
            "python_executable": LaunchConfiguration("python_executable"),
            "output_directory": LaunchConfiguration("output_directory"),
            "shape": LaunchConfiguration("shape"),
            "arena_yaml": "logix_step_30cm.yaml",
            "mujoco_gui": LaunchConfiguration("mujoco_gui"),
            "show_contacts": LaunchConfiguration("show_contacts"),
            "publish_truth_cloud": LaunchConfiguration("publish_truth_cloud"),
            "topic_gui": LaunchConfiguration("topic_gui"),
            "flipper_gui": LaunchConfiguration("flipper_gui"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )
    torque_plot = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="logix_flipper_torque_plot",
        output="screen",
        condition=IfCondition(LaunchConfiguration("torque_plot")),
        arguments=[
            "--empty",
            "/crawler/flipper_torques/data[0]",
            "/crawler/flipper_torques/data[1]",
            "/crawler/flipper_torques/data[2]",
            "/crawler/flipper_torques/data[3]",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution([crawler_share, "config", "default.yaml"])),
        DeclareLaunchArgument("python_executable", default_value=""),
        DeclareLaunchArgument(
            "output_directory",
            default_value=str(Path.home() / ".ros" / "crawler_mujoco" /
                              "logix_step_30cm")),
        DeclareLaunchArgument("shape", default_value="semicircle"),
        DeclareLaunchArgument("mujoco_gui", default_value="true"),
        DeclareLaunchArgument(
            "show_contacts", default_value="true",
            description="Show contact points and force vectors in the MuJoCo viewer"),
        DeclareLaunchArgument("publish_truth_cloud", default_value="true"),
        DeclareLaunchArgument("topic_gui", default_value="true"),
        DeclareLaunchArgument("flipper_gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "torque_plot", default_value="true",
            description="Plot all four flipper joint torques in one rqt_plot window"),
        simulator,
        torque_plot,
    ])


if __name__ == "__main__":
    generate_launch_description()
