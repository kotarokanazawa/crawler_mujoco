"""Generate a compact URDF matching the MuJoCo body and flipper kinematics."""

from __future__ import annotations

from pathlib import Path

from .configuration import load_config


def robot_description(config_path: str | Path) -> str:
    config = load_config(config_path)
    r = config["robot"]
    rgbd = config.get("sensors", {}).get("rgbd", {})
    sx, sy, sz = map(float, r["chassis_size"])
    length, width = float(r["flipper_length"]), float(r["flipper_width"])
    track_radius = float(r["track_radius"])
    track_height = 2 * track_radius
    imu_position = tuple(float(value) for value in r.get("imu_position", (0.0, 0.0, sz)))
    camera_pitch = float(rgbd.get("downward_pitch_deg", 20.0)) * 3.141592653589793 / 180.0
    front_camera_position = tuple(float(value) for value in rgbd.get(
        "front_position", (sx / 2, 0.0, sz * 0.8)))
    rear_camera_position = tuple(float(value) for value in rgbd.get(
        "rear_position", (-sx / 2, 0.0, sz * 0.8)))
    parts = [f'''<?xml version="1.0"?>
<robot name="crawler_mujoco">
  <material name="body"><color rgba="0.78 0.42 0.08 1"/></material>
  <material name="track"><color rgba="0.08 0.08 0.08 1"/></material>
  <material name="sprocket"><color rgba="0.22 0.22 0.22 1"/></material>
  <material name="flipper"><color rgba="0.1 0.35 0.75 1"/></material>
  <link name="base_link"><visual><geometry><box size="{sx} {sy} {sz}"/></geometry><material name="body"/></visual></link>
  <link name="imu_link"/>
  <joint name="base_to_imu" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="{imu_position[0]} {imu_position[1]} {imu_position[2]}"/></joint>
  <link name="front_rgbd_link"/><link name="front_rgbd_optical_frame"/>
  <joint name="base_to_front_rgbd" type="fixed"><parent link="base_link"/><child link="front_rgbd_link"/><origin xyz="{front_camera_position[0]} {front_camera_position[1]} {front_camera_position[2]}" rpy="0 {camera_pitch} 0"/></joint>
  <joint name="front_rgbd_to_optical" type="fixed"><parent link="front_rgbd_link"/><child link="front_rgbd_optical_frame"/><origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/></joint>
  <link name="rear_rgbd_link"/><link name="rear_rgbd_optical_frame"/>
  <joint name="base_to_rear_rgbd" type="fixed"><parent link="base_link"/><child link="rear_rgbd_link"/><origin xyz="{rear_camera_position[0]} {rear_camera_position[1]} {rear_camera_position[2]}" rpy="0 {-camera_pitch} 3.14159265359"/></joint>
  <joint name="rear_rgbd_to_optical" type="fixed"><parent link="rear_rgbd_link"/><child link="rear_rgbd_optical_frame"/><origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/></joint>''']
    for side, sign in (("left", 1), ("right", -1)):
        parts.append(f'''
  <link name="main_track_{side}">
    <visual><geometry><box size="{r['wheelbase']} {r['track_width']} {track_height}"/></geometry><material name="track"/></visual>
    <visual><origin xyz="{float(r['wheelbase'])/2} 0 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="{track_radius}" length="{r['track_width']}"/></geometry><material name="sprocket"/></visual>
    <visual><origin xyz="{-float(r['wheelbase'])/2} 0 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="{track_radius}" length="{r['track_width']}"/></geometry><material name="sprocket"/></visual>
  </link>
  <joint name="base_to_main_track_{side}" type="fixed"><parent link="base_link"/><child link="main_track_{side}"/><origin xyz="0 {sign*float(r['track_separation'])/2} 0"/></joint>''')
        for where, front in (("front", 1), ("rear", -1)):
            # Same convention on all joints: positive raises the flipper tip.
            # A negative command presses it into the terrain and the equal and
            # opposite joint torque raises the chassis.
            axis_y = -front
            center = front * length / 2
            parts.append(f'''
  <link name="flipper_{side}_{where}">
    <visual><origin xyz="{center} 0 0"/><geometry><box size="{length} {width} {track_height}"/></geometry><material name="flipper"/></visual>
    <visual><origin xyz="0 0 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="{track_radius}" length="{width}"/></geometry><material name="sprocket"/></visual>
    <visual><origin xyz="{front*length} 0 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="{track_radius}" length="{width}"/></geometry><material name="sprocket"/></visual>
  </link>
  <joint name="joint_{side}_{where}" type="revolute">
    <parent link="base_link"/><child link="flipper_{side}_{where}"/>
    <origin xyz="{front*float(r['wheelbase'])/2} {sign*float(r['flipper_y_offset'])} 0"/>
    <axis xyz="0 {axis_y} 0"/><limit lower="{r['flipper_lower']}" upper="{r['flipper_upper']}" effort="{r['flipper_effort']}" velocity="10"/>
  </joint>''')
    parts.append("</robot>")
    return "".join(parts)
