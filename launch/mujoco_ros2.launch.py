#!/usr/bin/env python3
"""Launch MuJoCo crawler with ROS topic GUIs, TF, and RViz2."""

from pathlib import Path
import sys
import yaml

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LAUNCH_ROOT = Path(__file__).resolve().parent
SOURCE_ROOT = LAUNCH_ROOT.parent
if (SOURCE_ROOT / "mujoco_crawler" / "__init__.py").is_file():
    # Direct source-tree launch: ros2 launch ./launch/mujoco_ros2.launch.py
    PACKAGE_ROOT = SOURCE_ROOT
    EXECUTABLE = SOURCE_ROOT / "scripts" / "mujoco_crawler"
    sys.path.insert(0, str(SOURCE_ROOT))
else:
    # Installed package launch: ros2 launch mujoco_crawler mujoco_ros2.launch.py
    prefix = Path(get_package_prefix("mujoco_crawler"))
    PACKAGE_ROOT = prefix / "share" / "mujoco_crawler"
    EXECUTABLE = prefix / "lib" / "mujoco_crawler" / "mujoco_crawler"
from mujoco_crawler.robot_description import robot_description  # noqa: E402
from mujoco_crawler.arena import resource_roots, resolve_arena_path  # noqa: E402


def as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def launch_setup(context):
    config = Path(LaunchConfiguration("config").perform(context)).resolve()
    shape = LaunchConfiguration("shape").perform(context)
    arena = LaunchConfiguration("arena_yaml").perform(context)
    if not arena:
        with config.open(encoding="utf-8") as stream:
            arena = str((yaml.safe_load(stream) or {}).get("environment", {}).get(
                "arena_yaml", "")).strip()
    arena_path = None
    model_root = None
    if arena:
        arena_root, model_root = resource_roots()
        arena_path = resolve_arena_path(arena, arena_root)
    description = robot_description(config)
    python_executable = LaunchConfiguration("python_executable").perform(context)
    if not python_executable:
        source_venv_python = SOURCE_ROOT / ".venv" / "bin" / "python"
        python_executable = (str(source_venv_python) if source_venv_python.is_file()
                             else sys.executable)
    command = [
        python_executable, str(EXECUTABLE),
        "--config", str(config), "--shape", shape, "--ros2",
        "--output", LaunchConfiguration("output_directory").perform(context),
    ]
    if as_bool(LaunchConfiguration("mujoco_gui").perform(context)):
        command.append("--gui")
    if as_bool(LaunchConfiguration("show_contacts").perform(context)):
        command.append("--show-contacts")
    if arena:
        command.extend(("--arena-yaml", arena))

    actions = [
        ExecuteProcess(cmd=command, output="screen", emulate_tty=True),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="mujoco_robot_state_publisher", output="screen",
            parameters=[{"robot_description": description, "use_sim_time": True}],
            remappings=[("joint_states", "/crawler/joint_states")],
        ),
        Node(
            package="rqt_robot_steering", executable="rqt_robot_steering",
            name="mujoco_robot_steering", output="screen",
            condition=IfCondition(LaunchConfiguration("topic_gui")),
            remappings=[("/cmd_vel", "/target/cmd_vel")],
        ),
        Node(
            package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
            name="mujoco_flipper_joint_gui", output="screen",
            condition=IfCondition(LaunchConfiguration("flipper_gui")),
            parameters=[{"robot_description": description, "use_sim_time": False}],
            remappings=[("joint_states", "/target/joint_states")],
        ),
        Node(
            package="rviz2", executable="rviz2", name="mujoco_rviz2", output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", str(PACKAGE_ROOT / "rviz" / "mujoco.rviz")],
            parameters=[{"use_sim_time": True}],
        ),
    ]
    if as_bool(LaunchConfiguration("publish_truth_cloud").perform(context)) and arena_path:
        resolution = float(LaunchConfiguration("cloud_resolution").perform(context))
        actions.extend([
            Node(
                package="mujoco_crawler", executable="truth_cloud_publisher",
                name="mujoco_truth_cloud_publisher",
                output="screen",
                parameters=[{
                    "arena_yaml": str(arena_path),
                    "model_root": str(model_root),
                    "use_sim_time": True,
                    "resolution": resolution,
                    "plane_step": resolution,
                    "publish_period": 2.0,
                }],
            ),
            Node(
                package="mujoco_crawler", executable="voxel_overhang_removal",
                name="mujoco_truth_cloud_filter",
                output="screen",
                parameters=[{
                    "input_topic": "/octomap_pointcloud",
                    "output_topic": "/octomap_pointcloud/filtering",
                    "voxel_size": max(0.05, resolution),
                    "height_mode": "max",
                    "durability": "transient_local",
                    "use_sim_time": True,
                }],
            ),
        ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value=str(PACKAGE_ROOT / "config" / "default.yaml")),
        DeclareLaunchArgument(
            "python_executable", default_value="",
            description="Python containing mujoco; empty selects source .venv or ROS Python"),
        DeclareLaunchArgument(
            "output_directory",
            default_value=str(Path.home() / ".ros" / "mujoco_crawler" / "results")),
        DeclareLaunchArgument("shape", default_value="semicircle",
                              description="none, rectangle, semicircle, or spike"),
        DeclareLaunchArgument("arena_yaml", default_value=""),
        DeclareLaunchArgument("mujoco_gui", default_value="true"),
        DeclareLaunchArgument("show_contacts", default_value="false"),
        DeclareLaunchArgument("publish_truth_cloud", default_value="true"),
        DeclareLaunchArgument("cloud_resolution", default_value="0.025"),
        DeclareLaunchArgument("topic_gui", default_value="true"),
        DeclareLaunchArgument("flipper_gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == "__main__":
    generate_launch_description()
