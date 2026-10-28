#!/usr/bin/env python3
"""Launch the standalone browser-based MuJoCo environment editor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackagePrefix
from launch.substitutions import PathJoinSubstitution


def launch_setup(context):
    executable = PathJoinSubstitution([
        FindPackagePrefix("crawler_mujoco"), "lib", "crawler_mujoco",
        "crawler_environment_editor",
    ])
    command = [executable, "--host", LaunchConfiguration("host"),
               "--port", LaunchConfiguration("port")]
    if LaunchConfiguration("open_browser").perform(context).lower() not in (
            "1", "true", "yes", "on"):
        command.append("--no-browser")
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8766"),
        DeclareLaunchArgument("open_browser", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
