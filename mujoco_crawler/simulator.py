#!/usr/bin/env python3
"""Build and run the crawler grouser comparison model in native MuJoCo."""

from __future__ import annotations

import argparse
import csv
import math
import signal
import threading
import time
from pathlib import Path
from xml.sax.saxutils import escape

import mujoco
import numpy as np
from .configuration import load_config
from .resources import package_share_directory


SHARE_DIR = package_share_directory()
SHAPES = ("none", "rectangle", "semicircle", "spike")


def f3(values) -> str:
    return " ".join(f"{float(value):.9g}" for value in values)

def spike_mesh(name: str, width: float, length: float, height: float) -> str:
    """Return an exact triangular prism with its base on local z=0."""
    x = width / 2
    y = length / 2
    vertices = [(-x, -y, 0), (x, -y, 0), (0, -y, height),
                (-x, y, 0), (x, y, 0), (0, y, height)]
    faces = [(0, 1, 2), (3, 5, 4), (0, 3, 4), (0, 4, 1),
             (1, 4, 5), (1, 5, 2), (2, 5, 3), (2, 3, 0)]
    return (f'<mesh name="{name}" vertex="{f3(sum((list(v) for v in vertices), []))}" '
            f'face="{" ".join(str(i) for face in faces for i in face)}"/>')

def semicircle_mesh(name: str, radius: float, length: float,
                    segments: int = 20) -> str:
    """Half-circle cross-section extruded with flat ends across track width."""
    section = [
        (radius * math.cos(math.pi - math.pi * index / segments),
         radius * math.sin(math.pi - math.pi * index / segments))
        for index in range(segments + 1)
    ]
    half_length = length / 2
    vertices = [(x, -half_length, z) for x, z in section]
    vertices += [(x, half_length, z) for x, z in section]
    count = len(section)
    faces = []
    # Flat end caps; reverse the first cap so outward normals oppose each other.
    for index in range(1, count - 1):
        faces.append((0, index + 1, index))
        faces.append((count, count + index, count + index + 1))
    # Extruded side surfaces, including the flat diameter face.
    for index in range(count):
        next_index = (index + 1) % count
        faces.append((index, next_index, count + next_index))
        faces.append((index, count + next_index, count + index))
    return (f'<mesh name="{name}" vertex="{f3(sum((list(v) for v in vertices), []))}" '
            f'face="{" ".join(str(i) for face in faces for i in face)}"/>')

def grouser_assets(shape: str, cfg: dict) -> str:
    g = cfg["grouser"]
    if shape == "spike":
        return (spike_mesh("spike", g["width"], cfg["robot"]["track_width"], g["height"])
                + spike_mesh("spike_flipper", g["width"],
                             cfg["robot"]["flipper_width"], g["height"]))
    if shape == "semicircle":
        return (semicircle_mesh("semicircle", g["height"], cfg["robot"]["track_width"])
                + semicircle_mesh("semicircle_flipper", g["height"],
                                  cfg["robot"]["flipper_width"]))
    return ""

def track_pitch(cfg: dict) -> float:
    """Return the one common grouser pitch used by every track section."""
    robot, g = cfg["robot"], cfg["grouser"]
    perimeter = (2 * float(robot["wheelbase"])
                 + 2 * math.pi * float(robot["track_radius"]))
    return perimeter / int(g["elements_per_round"])

def gazebo_drive_speed(cfg: dict) -> float:
    """Apply the same cmd_vel conversion as crawler_ignition_control_bridge."""
    robot = cfg["robot"]
    return (float(robot["commanded_linear_speed"]) / float(robot["wheel_radius"])
            * float(robot["command_scale"]))

def max_drive_omega(cfg: dict) -> float:
    robot = cfg["robot"]
    max_side_speed = (float(robot["max_commanded_linear_speed"])
                      + float(robot["drive_track_width"])
                      * float(robot["max_commanded_angular_speed"]) / 2)
    return max_side_speed / float(robot["wheel_radius"]) * float(robot["command_scale"])

def straight_force_limit(cfg: dict) -> float:
    """Convert Gazebo's sprocket torque limit to tangential belt force."""
    robot = cfg["robot"]
    return float(robot["drive_force"]) / float(robot["track_contact_radius"])

def velocity_servo_gains(cfg: dict) -> tuple[float, float]:
    """Gains that reach the effort limit when the nominal command is stalled."""
    robot = cfg["robot"]
    strength = float(robot.get("velocity_servo_strength", 1.0))
    nominal_omega = max(abs(gazebo_drive_speed(cfg)), 1e-6)
    nominal_belt_speed = nominal_omega * float(robot["track_contact_radius"])
    wheel_gain = strength * float(robot["drive_force"]) / nominal_omega
    straight_gain = strength * straight_force_limit(cfg) / nominal_belt_speed
    return wheel_gain, straight_gain

def grouser_geoms(shape: str, cfg: dict, prefix: str) -> str:
    if shape == "none":
        return ""
    robot, g = cfg["robot"], cfg["grouser"]
    radius = float(robot["track_radius"])
    width, height = float(g["width"]), float(g["height"])
    half_track = float(robot["track_width"]) / 2
    friction, solref, solimp = f3(g["friction"]), f3(g["solref"]), f3(g["solimp"])
    common = (f'friction="{friction}" solref="{solref}" solimp="{solimp}" '
              f'contype="2" conaffinity="1" rgba="0.12 0.12 0.12 1"')
    result = []
    pitch = track_pitch(cfg)
    count = max(1, round(2 * math.pi * radius / pitch))
    for index in range(count):
        angle = pitch / radius * (index + 0.5)
        radial_x, radial_z = math.cos(angle), math.sin(angle)
        if shape == "rectangle":
            center_r = radius + height / 2
            result.append(
                f'<geom name="{prefix}_grouser_{index}" type="box" '
                f'pos="{center_r*radial_x:.9g} 0 {center_r*radial_z:.9g}" '
                f'euler="0 {math.degrees(math.pi/2-angle):.9g} 0" '
                f'size="{width/2:.9g} {half_track:.9g} {height/2:.9g}" {common}/>'
            )
        elif shape == "semicircle":
            result.append(
                f'<geom name="{prefix}_grouser_{index}" type="mesh" mesh="semicircle" '
                f'pos="{radius*radial_x:.9g} 0 {radius*radial_z:.9g}" '
                f'euler="0 {math.degrees(math.pi/2-angle):.9g} 0" {common}/>'
            )
        else:
            result.append(
                f'<geom name="{prefix}_grouser_{index}" type="mesh" mesh="spike" '
                f'pos="{radius*radial_x:.9g} 0 {radius*radial_z:.9g}" '
                f'euler="0 {math.degrees(math.pi/2-angle):.9g} 0" {common}/>'
            )
    return "\n".join(result)

def straight_grouser_geom(shape: str, cfg: dict, name: str, x: float,
                          top: bool) -> str:
    """One outward-facing grouser on a horizontal moving belt section."""
    robot, g = cfg["robot"], cfg["grouser"]
    radius = float(robot["track_radius"])
    width, height = float(g["width"]), float(g["height"])
    half_track = float(robot["track_width"]) / 2
    sign = 1.0 if top else -1.0
    common = (f'name="{name}" friction="{f3(g["friction"])}" '
              f'solref="{f3(g["solref"])}" solimp="{f3(g["solimp"])}" '
              f'contype="2" conaffinity="1" rgba="0.08 0.08 0.08 1"')
    if shape == "rectangle":
        return (f'<geom {common} type="box" pos="{x:.9g} 0 {sign*(radius+height/2):.9g}" '
                f'size="{width/2:.9g} {half_track:.9g} {height/2:.9g}"/>')
    if shape == "semicircle":
        rotation = "0 0 0" if top else "180 0 0"
        return (f'<geom {common} type="mesh" mesh="semicircle" '
                f'pos="{x:.9g} 0 {sign*radius:.9g}" euler="{rotation}"/>')
    rotation = "0 0 0" if top else "180 0 0"
    return (f'<geom {common} type="mesh" mesh="spike" pos="{x:.9g} 0 {sign*radius:.9g}" '
            f'euler="{rotation}"/>')

def straight_belt_xml(shape: str, cfg: dict, side: int, y: float) -> tuple[str, list[str]]:
    """Create periodic top and bottom arrays using one slider per array."""
    robot, g = cfg["robot"], cfg["grouser"]
    if not bool(robot.get("straight_enabled", True)):
        return "", []
    pitch = track_pitch(cfg)
    length = float(robot["wheelbase"])
    count = max(1, round(length / pitch))
    start = -0.5 * (count - 1) * pitch
    bodies, actuator_names = [], []
    for top in (False, True):
        section = "top" if top else "bottom"
        body_name = f"straight_{side}_{section}"
        sign = 1.0 if top else -1.0
        belt_z = sign * (float(robot["track_radius"])
                         - float(robot["belt_thickness"]) / 2)
        belt = (f'<geom name="{body_name}_belt" type="box" pos="0 0 {belt_z:.9g}" '
                f'size="{length/2:.9g} {float(robot["track_width"])/2:.9g} '
                f'{float(robot["belt_thickness"])/2:.9g}" friction="{f3(g["friction"])}" '
                f'solref="{f3(g["solref"])}" solimp="{f3(g["solimp"])}" '
                f'contype="2" conaffinity="1" rgba="0.06 0.06 0.06 1"/>')
        geoms = "" if shape == "none" else "\n".join(
            straight_grouser_geom(shape, cfg, f"{body_name}_grouser_{index}",
                                  start + index * pitch, top)
            for index in range(count)
        )
        bodies.append(f'''
      <body name="{body_name}" pos="0 {y:.9g} 0">
        <joint name="{body_name}_joint" type="slide" axis="1 0 0" damping="0.02"/>
        <inertial pos="0 0 0" mass="{float(robot['straight_section_mass']):.9g}" diaginertia="0.01 0.01 0.01"/>
        {belt}
        {geoms}
      </body>''')
        actuator_names.append(body_name)
    return "".join(bodies), actuator_names

def flipper_grouser_geom(shape: str, cfg: dict, name: str, x: float,
                         z: float, top: bool) -> str:
    robot, g = cfg["robot"], cfg["grouser"]
    width, height = float(g["width"]), float(g["height"])
    common = (f'name="{name}" friction="{f3(g["friction"])}" '
              f'solref="{f3(g["solref"])}" solimp="{f3(g["solimp"])}" '
              f'contype="2" conaffinity="1" rgba="0.08 0.08 0.08 1"')
    sign = 1 if top else -1
    if shape == "rectangle":
        return (f'<geom {common} type="box" pos="{x:.9g} 0 {z + sign*height/2:.9g}" '
                f'size="{width/2:.9g} {float(robot["flipper_width"])/2:.9g} {height/2:.9g}"/>')
    mesh = "semicircle_flipper" if shape == "semicircle" else "spike_flipper"
    rotation = "0 0 0" if top else "180 0 0"
    return f'<geom {common} type="mesh" mesh="{mesh}" pos="{x:.9g} 0 {z:.9g}" euler="{rotation}"/>'


def flipper_arc_grousers(shape: str, cfg: dict, prefix: str) -> str:
    if shape == "none":
        return ""
    robot, g = cfg["robot"], cfg["grouser"]
    radius, width, height = (float(robot["track_radius"]),
                             float(robot["flipper_width"]), float(g["height"]))
    pitch = track_pitch(cfg)
    count = max(1, round(2 * math.pi * radius / pitch))
    common = (f'friction="{f3(g["friction"])}" solref="{f3(g["solref"])}" '
              f'solimp="{f3(g["solimp"])}" contype="2" conaffinity="1" '
              f'rgba="0.08 0.08 0.08 1"')
    geoms = []
    for index in range(count):
        angle = pitch / radius * (index + 0.5)
        rx, rz = math.cos(angle), math.sin(angle)
        if shape == "rectangle":
            center_r = radius + height / 2
            geoms.append(
                f'<geom name="{prefix}_grouser_{index}" type="box" '
                f'pos="{center_r*rx:.9g} 0 {center_r*rz:.9g}" '
                f'euler="0 {math.degrees(math.pi/2-angle):.9g} 0" '
                f'size="{float(g["width"])/2:.9g} {width/2:.9g} {height/2:.9g}" {common}/>'
            )
        else:
            mesh = "semicircle_flipper" if shape == "semicircle" else "spike_flipper"
            geoms.append(
                f'<geom name="{prefix}_grouser_{index}" type="mesh" mesh="{mesh}" '
                f'pos="{radius*rx:.9g} 0 {radius*rz:.9g}" '
                f'euler="0 {math.degrees(math.pi/2-angle):.9g} 0" {common}/>'
            )
    return "".join(geoms)


def flippers_xml(shape: str, cfg: dict) -> tuple[str, str]:
    robot, g = cfg["robot"], cfg["grouser"]
    if not bool(robot.get("flipper_enabled", True)):
        return "", ""
    length, width = float(robot["flipper_length"]), float(robot["flipper_width"])
    radius, belt = float(robot["track_radius"]), float(robot["belt_thickness"])
    # Use the main-track pitch.  Deriving a second pitch from the shorter
    # flipper perimeter made otherwise identical grousers more densely packed.
    pitch = track_pitch(cfg)
    count = max(1, round(length / pitch))
    common = (f'friction="{f3(g["friction"])}" solref="{f3(g["solref"])}" '
              f'solimp="{f3(g["solimp"])}" contype="2" conaffinity="1" rgba="0.07 0.07 0.07 1"')
    bodies, actuators = [], []
    for side_name, side_sign in (("left", 1), ("right", -1)):
        for where, front_sign in (("front", 1), ("rear", -1)):
            name = f"joint_{side_name}_{where}"
            center = front_sign * length / 2
            end = front_sign * length
            side_index = 1 if side_name == "left" else 0
            moving_bodies = []
            for top in (False, True):
                section = "top" if top else "bottom"
                sign = 1 if top else -1
                slider = f"flipper_straight_{side_index}_{where}_{section}"
                geoms = "" if shape == "none" else "".join(
                    flipper_grouser_geom(
                        shape, cfg, f"{slider}_grouser_{index}",
                        front_sign * pitch * (index + 0.5), sign * radius, top)
                    for index in range(count)
                )
                moving_bodies.append(f'''
        <body name="{slider}">
          <joint name="{slider}_joint" type="slide" axis="1 0 0" damping="0.02"/>
          <inertial pos="{center:.9g} 0 0" mass="0.2" diaginertia="0.001 0.01 0.01"/>
          <geom name="{slider}_belt" type="box" pos="{center:.9g} 0 {sign*(radius-belt/2):.9g}"
                size="{length/2:.9g} {width/2:.9g} {belt/2:.9g}" {common}/>
          {geoms}
        </body>''')
                actuators.append(f'''
    <velocity name="{slider}_motor" joint="{slider}_joint" kv="{velocity_servo_gains(cfg)[1]:.9g}"
              ctrlrange="{-float(robot['track_contact_radius'])*max_drive_omega(cfg):.9g} {float(robot['track_contact_radius'])*max_drive_omega(cfg):.9g}"
              forcerange="{-straight_force_limit(cfg):.9g} {straight_force_limit(cfg):.9g}"/>''')
            wheel_bodies = []
            for axle, x in enumerate((0.0, end)):
                wheel = f"flipper_wheel_{side_index}_{where}_{axle}"
                wheel_bodies.append(f'''
        <body name="{wheel}" pos="{x:.9g} 0 0">
          <joint name="{wheel}_joint" type="hinge" axis="0 1 0" damping="0.05"/>
          <inertial pos="0 0 0" mass="0.2" diaginertia="0.001 0.001 0.001"/>
          <geom name="{wheel}_sprocket" type="cylinder" euler="90 0 0" size="{radius:.9g} {width/2:.9g}" {common}/>
          {flipper_arc_grousers(shape, cfg, wheel)}
        </body>''')
                actuators.append(f'''
    <velocity name="{wheel}_motor" joint="{wheel}_joint" kv="{velocity_servo_gains(cfg)[0]:.9g}"
              ctrlrange="{-max_drive_omega(cfg):.9g} {max_drive_omega(cfg):.9g}"
              forcerange="{-float(robot['drive_force']):.9g} {float(robot['drive_force']):.9g}"/>''')
            bodies.append(f'''
      <body name="flipper_{side_name}_{where}" pos="{front_sign*float(robot['wheelbase'])/2:.9g} {side_sign*float(robot['flipper_y_offset']):.9g} 0">
        <joint name="{name}" type="hinge" axis="0 {-front_sign} 0"
               range="{math.degrees(float(robot['flipper_lower'])):.9g} {math.degrees(float(robot['flipper_upper'])):.9g}"
               damping="10" frictionloss="0.05"/>
        <inertial pos="{center:.9g} 0 0" mass="{float(robot['flipper_mass'])-0.8:.9g}" diaginertia="0.03 0.08 0.08"/>
        <geom name="{name}_center" type="box" pos="{center:.9g} 0 0" size="{length/2:.9g} {width/2:.9g} {radius-belt:.9g}" {common}/>
        {''.join(wheel_bodies)}
        {''.join(moving_bodies)}
      </body>''')
            actuators.append(f'''
    <motor name="{name}_position" joint="{name}"
           ctrlrange="{-float(robot['flipper_effort']):.9g} {float(robot['flipper_effort']):.9g}"
           forcerange="{-float(robot['flipper_effort']):.9g} {float(robot['flipper_effort']):.9g}"/>''')
    return "".join(bodies), "".join(actuators)


def build_xml(cfg: dict, shape: str) -> str:
    if shape not in SHAPES:
        raise ValueError(f"shape must be one of: {', '.join(SHAPES)}")
    sim, robot, env = cfg["simulation"], cfg["robot"], cfg["environment"]
    sx, sy, sz = (float(x) for x in robot["chassis_size"])
    x_positions = (-float(robot["wheelbase"]) / 2, float(robot["wheelbase"]) / 2)
    y_positions = (-float(robot["track_separation"]) / 2, float(robot["track_separation"]) / 2)
    track_radius = float(robot["track_radius"])
    spawn_z = float(robot["spawn_z"])
    imu_position = tuple(float(value) for value in robot.get(
        "imu_position", (0.0, 0.0, sz)))
    rgbd = cfg.get("sensors", {}).get("rgbd", {})
    camera_enabled = bool(rgbd.get("enabled", True))
    camera_pitch = math.radians(float(rgbd.get("downward_pitch_deg", 20.0)))
    camera_sin, camera_cos = math.sin(camera_pitch), math.cos(camera_pitch)
    front_camera_position = tuple(float(value) for value in rgbd.get(
        "front_position", (sx / 2, 0.0, sz * 0.8)))
    rear_camera_position = tuple(float(value) for value in rgbd.get(
        "rear_position", (-sx / 2, 0.0, sz * 0.8)))
    camera_fovy = float(rgbd.get("fovy_deg", 70.0))
    camera_xml = ""
    if camera_enabled:
        camera_xml = f'''
      <camera name="front_rgbd" pos="{f3(front_camera_position)}"
              xyaxes="0 -1 0 {camera_sin:.9g} 0 {camera_cos:.9g}" fovy="{camera_fovy:.9g}"/>
      <camera name="rear_rgbd" pos="{f3(rear_camera_position)}"
              xyaxes="0 1 0 {-camera_sin:.9g} 0 {camera_cos:.9g}" fovy="{camera_fovy:.9g}"/>'''
    max_omega = max_drive_omega(cfg)
    wheel_kv, straight_kv = velocity_servo_gains(cfg)
    max_yaw_torque = straight_force_limit(cfg) * float(robot["drive_track_width"])
    yaw_kv = (float(robot.get("yaw_servo_strength", 0.2)) * max_yaw_torque
              / max(float(robot["max_commanded_angular_speed"]), 1e-6))
    wheels, actuators, straight_sections, straight_names = [], [], [], []
    for side, y in enumerate(y_positions):
        for axle, x in enumerate(x_positions):
            name = f"wheel_{side}_{axle}"
            wheels.append(f'''
      <body name="{name}" pos="{x:.9g} {y:.9g} 0">
        <joint name="{name}_joint" type="hinge" axis="0 1 0" damping="0.05"/>
        <inertial pos="0 0 0" mass="{float(robot['wheel_mass']):.9g}" diaginertia="0.004 0.002 0.004"/>
        <geom type="cylinder" euler="90 0 0" size="{track_radius:.9g} {float(robot['track_width'])/2:.9g}"
              friction="0.8 0.01 0.001" contype="2" conaffinity="1" rgba="0.18 0.18 0.18 1"/>
        {grouser_geoms(shape, cfg, name)}
      </body>''')
            actuators.append(
                f'<velocity name="{name}_motor" joint="{name}_joint" '
                f'kv="{wheel_kv:.9g}" ctrlrange="{-max_omega:.9g} {max_omega:.9g}" '
                f'forcerange="{-float(robot["drive_force"]):.9g} {float(robot["drive_force"]):.9g}"/>'
            )
        section_xml, names = straight_belt_xml(shape, cfg, side, y)
        straight_sections.append(section_xml)
        straight_names.extend(names)
    linear_speed = float(robot["track_contact_radius"]) * max_omega
    for name in straight_names:
        actuators.append(
            f'<velocity name="{name}_motor" joint="{name}_joint" kv="{straight_kv:.9g}" '
            f'ctrlrange="{-linear_speed:.9g} {linear_speed:.9g}" '
            f'forcerange="{-straight_force_limit(cfg):.9g} '
            f'{straight_force_limit(cfg):.9g}"/>'
        )
    flipper_bodies, flipper_actuators = flippers_xml(shape, cfg)
    bridge_center = (float(env["bridge_start_x"]) + float(env["bridge_end_x"])) / 2
    bridge_half_length = (float(env["bridge_end_x"]) - float(env["bridge_start_x"])) / 2
    arena_assets = arena_bodies = ""
    arena_yaml = str(env.get("arena_yaml", "")).strip()
    ground_z = (float(env.get("arena_ground_z", 0.0)) if arena_yaml
                else float(env.get("builtin_ground_z", -0.08)))
    if arena_yaml:
        from .arena import load_arena
        arena_assets, arena_bodies, warnings = load_arena(
            arena_yaml, env.get("friction", (2.5, 0.05, 0.005)))
        for warning in warnings:
            print(f"arena warning: {warning}")
        environment_xml = arena_bodies
    else:
        environment_xml = f'''
    <geom name="bridge" type="box" pos="{bridge_center:.9g} 0 {-float(env['bridge_thickness'])/2:.9g}"
          size="{bridge_half_length:.9g} {float(env['bridge_width'])/2:.9g} {float(env['bridge_thickness'])/2:.9g}"
          material="bridge_mat" friction="{f3(env.get('friction', (2.5, 0.05, 0.005)))}" contype="1" conaffinity="2"/>'''
    return f'''<mujoco model="crawler_{escape(shape)}">
  <compiler angle="degree" autolimits="true"/>
  <option timestep="{float(sim['timestep']):.9g}" integrator="{escape(str(sim['integrator']))}" gravity="0 0 -9.81" cone="elliptic"/>
  <size nconmax="1000" njmax="4000" nuserdata="64"/>
  <visual>
    <global azimuth="130" elevation="-20"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.72 0.72 0.72" specular="0.08 0.08 0.08"/>
    <rgba haze="0.32 0.40 0.52 1" contactpoint="1 0.25 0.1 0.4"
          contactforce="0.1 0.75 1 0.4" contactfriction="1 0.8 0.15 0.4"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.46 0.61 0.79" rgb2="0.10 0.15 0.24" width="512" height="3072"/>
    <material name="bridge_mat" rgba="0.35 0.20 0.08 1" roughness="0.9"/>
    {grouser_assets(shape, cfg)}
    {arena_assets}
  </asset>
  <worldbody>
    <light directional="true" pos="-2 -2 6" dir="0.3 0.3 -1" ambient="0.15 0.15 0.15" diffuse="0.72 0.72 0.72" specular="0.08 0.08 0.08"/>
    <light directional="true" pos="3 2 4" dir="-0.4 -0.2 -1" ambient="0.05 0.05 0.06" diffuse="0.34 0.36 0.42" specular="0.03 0.03 0.03"/>
    <geom name="lower_ground" type="plane" pos="0 0 {ground_z:.9g}" size="20 20 0.1" rgba="0.34 0.39 0.30 1" friction="{f3(env.get('friction', (2.5, 0.05, 0.005)))}" contype="1" conaffinity="2"/>
    {environment_xml}
    <body name="crawler" pos="{float(robot['spawn_x']):.9g} {float(robot['spawn_y']):.9g} {spawn_z:.9g}">
      <freejoint name="root"/>
      <inertial pos="0 0 {sz/2:.9g}" mass="{float(robot['chassis_mass']):.9g}" diaginertia="0.22 0.42 0.45"/>
      <geom name="chassis" type="box" pos="0 0 {sz/2:.9g}" size="{sx/2:.9g} {sy/2:.9g} {sz/2:.9g}"
            rgba="0.78 0.42 0.08 1" contype="2" conaffinity="1"/>
      <site name="imu" pos="{f3(imu_position)}" size="0.01"/>
      {camera_xml}
      {''.join(wheels)}
      {''.join(straight_sections)}
      {flipper_bodies}
    </body>
  </worldbody>
  <actuator>{''.join(actuators)}{flipper_actuators}
    <velocity name="skid_steer_yaw_motor" joint="root" gear="0 0 0 0 0 1"
              kv="{yaw_kv:.9g}" ctrlrange="{-float(robot['max_commanded_angular_speed']):.9g} {float(robot['max_commanded_angular_speed']):.9g}"
              forcerange="{-max_yaw_torque:.9g} {max_yaw_torque:.9g}"/>
  </actuator>
  <sensor>
    <framepos name="body_position" objtype="body" objname="crawler"/>
    <framequat name="body_orientation" objtype="body" objname="crawler"/>
    <gyro name="body_gyro" site="imu"/>
    <accelerometer name="body_acceleration" site="imu"/>
  </sensor>
</mujoco>'''


def pitch_from_quaternion(quat) -> float:
    w, x, y, z = quat
    value = 2 * (w * y - z * x)
    return math.degrees(math.asin(max(-1.0, min(1.0, value))))


def contact_forces(model, data) -> tuple[int, float, float]:
    normal = tangential = 0.0
    force = np.zeros(6)
    for index in range(data.ncon):
        mujoco.mj_contactForce(model, data, index, force)
        normal += abs(float(force[0]))
        tangential += math.hypot(float(force[1]), float(force[2]))
    return data.ncon, normal, tangential


def apply_track_contact_friction(model, data, cfg: dict) -> None:
    """Apply high longitudinal and lower lateral friction to track contacts."""
    g = cfg["grouser"]
    longitudinal = float(g.get("longitudinal_friction", g["friction"][0]))
    lateral = float(g.get("lateral_friction", longitudinal))
    crawler_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "crawler")
    # xmat columns are the crawler body's local axes expressed in world frame.
    forward = data.xmat[crawler_id].reshape(3, 3)[:, 0]
    for index in range(data.ncon):
        contact = data.contact[index]
        type1, type2 = model.geom_contype[contact.geom1], model.geom_contype[contact.geom2]
        if {int(type1), int(type2)} != {1, 2}:
            continue
        frame = np.asarray(contact.frame).reshape(3, 3)
        alignment1 = abs(float(np.dot(frame[1], forward)))
        alignment2 = abs(float(np.dot(frame[2], forward)))
        if alignment1 >= alignment2:
            contact.friction[0], contact.friction[1] = longitudinal, lateral
        else:
            contact.friction[0], contact.friction[1] = lateral, longitudinal


def step_physics(model, data, cfg: dict) -> None:
    """Split the step so contact friction can be set before constraint solving."""
    mujoco.mj_step1(model, data)
    apply_track_contact_friction(model, data, cfg)
    mujoco.mj_step2(model, data)


def wrap_straight_sections(model, data, cfg: dict) -> None:
    """Wrap a periodic array by one pitch without changing its visible pattern."""
    robot, g = cfg["robot"], cfg["grouser"]
    if not bool(robot.get("straight_enabled", True)):
        return
    pitch = track_pitch(cfg)
    for side in range(2):
        for section in ("bottom", "top"):
            joint_name = f"straight_{side}_{section}_joint"
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            qpos_index = model.jnt_qposadr[joint_id]
            if data.qpos[qpos_index] <= -pitch / 2:
                data.qpos[qpos_index] += pitch
            elif data.qpos[qpos_index] >= pitch / 2:
                data.qpos[qpos_index] -= pitch
    if bool(robot.get("flipper_enabled", True)):
        flipper_pitch = track_pitch(cfg)
        for side in range(2):
            for where in ("front", "rear"):
                for section in ("bottom", "top"):
                    joint_name = f"flipper_straight_{side}_{where}_{section}_joint"
                    joint_id = mujoco.mj_name2id(
                        model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                    qpos_index = model.jnt_qposadr[joint_id]
                    if data.qpos[qpos_index] <= -flipper_pitch / 2:
                        data.qpos[qpos_index] += flipper_pitch
                    elif data.qpos[qpos_index] >= flipper_pitch / 2:
                        data.qpos[qpos_index] -= flipper_pitch


def set_velocity_controls(model, data, cfg: dict, linear: float, angular: float) -> None:
    """Set drive targets; the per-step controller applies acceleration limits."""
    robot = cfg["robot"]
    track_width = float(robot["drive_track_width"])
    left_linear = linear - track_width * angular / 2
    right_linear = linear + track_width * angular / 2
    for actuator_id in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        if name == "skid_steer_yaw_motor":
            data.ctrl[actuator_id] = angular
            continue
        if name.endswith("_position"):
            continue
        if name.startswith("flipper_") and not bool(robot.get("flipper_track_drive", True)):
            data.userdata[12 + actuator_id] = 0.0
            continue
        # Main tracks encode side at field 1; flipper track actuators at field 2.
        fields = name.split("_")
        side = int(fields[2] if name.startswith("flipper_") else fields[1])
        side_speed = right_linear if side == 0 else left_linear
        omega = (side_speed / float(robot["wheel_radius"])
                 * float(robot["command_scale"]))
        belt_speed = omega * float(robot["track_contact_radius"])
        if name.startswith(("wheel_", "flipper_wheel_")):
            data.userdata[12 + actuator_id] = omega
        elif "_bottom_motor" in name:
            data.userdata[12 + actuator_id] = -belt_speed
        elif "_top_motor" in name:
            data.userdata[12 + actuator_id] = belt_speed


def update_track_velocity_controls(model, data, cfg: dict) -> None:
    """Slew wheel and straight-section speeds at one common belt acceleration."""
    robot = cfg["robot"]
    acceleration = float(robot.get("track_max_acceleration", 0.5))
    if acceleration <= 0:
        raise ValueError("robot.track_max_acceleration must be positive")
    dt = float(model.opt.timestep)
    radius = float(robot["track_contact_radius"])
    for actuator_id in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
        is_wheel = name.startswith(("wheel_", "flipper_wheel_"))
        is_straight = "_bottom_motor" in name or "_top_motor" in name
        if not (is_wheel or is_straight):
            continue
        target = float(data.userdata[12 + actuator_id])
        maximum_step = acceleration * dt / radius if is_wheel else acceleration * dt
        difference = target - float(data.ctrl[actuator_id])
        data.ctrl[actuator_id] += max(-maximum_step, min(maximum_step, difference))


def set_flipper_controls(model, data, cfg: dict, targets) -> None:
    robot = cfg["robot"]
    lower, upper = float(robot["flipper_lower"]), float(robot["flipper_upper"])
    names = ("joint_left_front", "joint_left_rear", "joint_right_front", "joint_right_rear")
    for index, target in enumerate(targets):
        target = max(lower, min(upper, float(target)))
        if abs(target - float(data.userdata[4 + index])) > 1e-9:
            data.userdata[index] = 0.0
        data.userdata[4 + index] = target


def update_flipper_controls(model, data, cfg: dict) -> None:
    """Run the flipper position PID every physics step.

    MuJoCo's position actuator is proportional-only.  A motor with an explicit
    integral term is used here so a flipper resting on the terrain reaches the
    GUI angle instead of stopping with a persistent load-dependent error.
    """
    robot = cfg["robot"]
    if not bool(robot.get("flipper_enabled", True)):
        return
    kp = float(robot["flipper_position_kp"])
    ki = float(robot.get("flipper_position_ki", 0.0))
    kd = float(robot["flipper_position_kd"])
    effort = float(robot["flipper_effort"])
    max_velocity = float(robot.get("flipper_max_velocity", 1.0))
    integral_limit = float(robot.get("flipper_position_integral_limit", 2.0))
    names = ("joint_left_front", "joint_left_rear",
             "joint_right_front", "joint_right_rear")
    for index, name in enumerate(names):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_position")
        if joint_id < 0 or actuator_id < 0:
            continue
        qpos = float(data.qpos[model.jnt_qposadr[joint_id]])
        qvel = float(data.qvel[model.jnt_dofadr[joint_id]])
        desired = float(data.userdata[4 + index])
        profiled = float(data.userdata[8 + index])
        max_step = max_velocity * float(model.opt.timestep)
        profiled += max(-max_step, min(max_step, desired - profiled))
        data.userdata[8 + index] = profiled
        error = profiled - qpos
        integral = float(data.userdata[index]) + error * float(model.opt.timestep)
        integral = max(-integral_limit, min(integral_limit, integral))
        torque = kp * error + ki * integral - kd * qvel
        # Do not wind up further while the requested torque is saturated.
        clipped = max(-effort, min(effort, torque))
        if clipped != torque and error * torque > 0.0:
            integral = float(data.userdata[index])
            torque = kp * error + ki * integral - kd * qvel
            clipped = max(-effort, min(effort, torque))
        data.userdata[index] = integral
        data.ctrl[actuator_id] = clipped


def run(cfg: dict, shape: str, output: Path, gui: bool = False,
        control_state=None, stop_event: threading.Event | None = None,
        ros_bridge=None, show_contacts: bool = False) -> dict:
    xml = build_xml(cfg, shape)
    output.mkdir(parents=True, exist_ok=True)
    (output / f"crawler_{shape}.xml").write_text(xml, encoding="utf-8")
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    set_velocity_controls(model, data, cfg, float(cfg["robot"]["commanded_linear_speed"]), 0.0)
    set_flipper_controls(model, data, cfg, (0.0, 0.0, 0.0, 0.0))
    duration = (math.inf if control_state is not None or ros_bridge is not None
                else float(cfg["simulation"]["duration"]))
    rows, viewer = [], None
    if gui:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
        if show_contacts or bool(cfg["simulation"].get("show_contacts", False)):
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    if stop_event is None:
        stop_event = threading.Event()
    # Reinstall after launch_passive so native viewer signal handling cannot
    # prevent Python from leaving the simulation loop.
    signal.signal(signal.SIGINT, lambda _signum, _frame: stop_event.set())
    next_sample = 0.0
    sample_period = 1 / float(cfg["simulation"]["render_fps"])
    try:
        while (not stop_event.is_set() and data.time < duration
               and (viewer is None or viewer.is_running())):
            before = time.perf_counter()
            if ros_bridge is not None:
                linear, angular, flippers = ros_bridge.commands()
                set_velocity_controls(model, data, cfg, linear, angular)
                set_flipper_controls(model, data, cfg, flippers)
            if control_state is not None:
                linear, angular, paused, running, reset = control_state.command()
                if not running:
                    break
                if reset:
                    mujoco.mj_resetData(model, data)
                    rows.clear()
                    next_sample = 0.0
                set_velocity_controls(model, data, cfg, linear, angular)
                if paused:
                    if viewer is not None:
                        viewer.sync()
                    time.sleep(0.01)
                    continue
            wrap_straight_sections(model, data, cfg)
            update_track_velocity_controls(model, data, cfg)
            update_flipper_controls(model, data, cfg)
            step_physics(model, data, cfg)
            if control_state is not None:
                control_state.update_pose(data.time, data.sensor("body_position").data[0])
            if data.time >= next_sample:
                count, normal, tangential = contact_forces(model, data)
                position = data.sensor("body_position").data.copy()
                quat = data.sensor("body_orientation").data.copy()
                rows.append((data.time, *position, pitch_from_quaternion(quat),
                             count, normal, tangential))
                if ros_bridge is not None:
                    ros_bridge.publish_state(model, data)
                next_sample += sample_period
            if viewer is not None:
                viewer.sync()
            if viewer is not None or control_state is not None or ros_bridge is not None:
                remaining = float(cfg["simulation"]["timestep"]) - (time.perf_counter() - before)
                if remaining > 0:
                    time.sleep(remaining)
    finally:
        if viewer is not None:
            viewer.close()
    if not rows:
        mujoco.mj_forward(model, data)
        count, normal, tangential = contact_forces(model, data)
        position = data.sensor("body_position").data.copy()
        quat = data.sensor("body_orientation").data.copy()
        rows.append((data.time, *position, pitch_from_quaternion(quat),
                     count, normal, tangential))
    csv_path = output / f"{shape}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", "x_m", "y_m", "z_m", "pitch_deg", "contacts",
                         "normal_force_N", "tangential_force_N"))
        writer.writerows(rows)
    target = float(cfg["environment"]["target_x"])
    bridge_start = float(cfg["environment"]["bridge_start_x"])
    bridge_rows = [row for row in rows if bridge_start <= row[1] <= target]
    reached_rows = [row for row in rows if row[1] >= target]
    return {"shape": shape, "final_x_m": rows[-1][1],
            "max_abs_pitch_deg": max(abs(row[4]) for row in rows),
            "bridge_max_abs_pitch_deg": max((abs(row[4]) for row in bridge_rows), default=float("nan")),
            "max_z_m": max(row[3] for row in rows), "reached_target": rows[-1][1] >= target,
            "target_time_s": reached_rows[0][0] if reached_rows else float("nan"),
            "csv": str(csv_path)}


def plot_results(results: list[dict], output: Path) -> None:
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for result in results:
        values = np.genfromtxt(result["csv"], delimiter=",", names=True)
        axes[0].plot(values["time_s"], values["x_m"], label=result["shape"])
        axes[1].plot(values["time_s"], values["pitch_deg"], label=result["shape"])
        axes[2].plot(values["time_s"], values["normal_force_N"], label=result["shape"])
    axes[0].set_ylabel("x [m]")
    axes[1].set_ylabel("pitch [deg]")
    axes[2].set_ylabel("normal force [N]")
    axes[2].set_xlabel("simulation time [s]")
    axes[0].legend()
    for axis in axes:
        axis.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output / "shape_comparison.png", dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path,
                        default=SHARE_DIR / "config" / "default.yaml")
    parser.add_argument("--shape", choices=(*SHAPES, "all"), default=None)
    parser.add_argument("--gui", action="store_true", help="Open the native MuJoCo viewer")
    parser.add_argument("--show-contacts", action="store_true",
                        help="Show contact points and force vectors in the MuJoCo viewer")
    parser.add_argument("--control-ui", action="store_true", help="Open browser velocity controls")
    parser.add_argument("--ui-port", type=int, default=8765)
    parser.add_argument("--arena-yaml", default=None,
                        help="Bundled arena YAML name/path (for example singlerane/bridge.yaml)")
    parser.add_argument("--ros2", action="store_true",
                        help="Enable /cmd_vel, flipper commands, joint_states, odom, and clock")
    parser.add_argument("--output", type=Path, default=Path.cwd() / "results")
    parser.add_argument("--write-xml-only", action="store_true")
    args = parser.parse_args()
    cfg = load_config(args.config)
    if args.arena_yaml is not None:
        cfg["environment"]["arena_yaml"] = args.arena_yaml
    selected = args.shape or cfg["grouser"]["shape"]
    shapes = SHAPES if selected == "all" else (selected,)
    if (args.gui or args.control_ui or args.ros2) and len(shapes) != 1:
        parser.error("--gui, --control-ui, and --ros2 require one shape")
    if args.write_xml_only:
        args.output.mkdir(parents=True, exist_ok=True)
        for shape in shapes:
            path = args.output / f"crawler_{shape}.xml"
            path.write_text(build_xml(cfg, shape), encoding="utf-8")
            print(path)
        return
    control_state = server = ros_bridge = None
    if args.control_ui:
        from .control_ui import ControlState, start_control_server
        control_state = ControlState(float(cfg["robot"]["commanded_linear_speed"]))
        server, url = start_control_server(control_state, port=args.ui_port)
        print(f"control UI: {url}")
    if args.ros2:
        from .ros_bridge import Ros2Bridge
        ros_bridge = Ros2Bridge(cfg)
        print("ROS 2: /cmd_vel and /crawler/flipper_commands enabled")
    stop_event = threading.Event()
    try:
        results = []
        for shape in shapes:
            results.append(run(cfg, shape, args.output, args.gui, control_state,
                               stop_event, ros_bridge, args.show_contacts))
            if stop_event.is_set():
                break
    finally:
        if server is not None:
            server.shutdown()
        if ros_bridge is not None:
            ros_bridge.close()
    if len(results) > 1:
        plot_results(results, args.output)
    summary = args.output / "summary.csv"
    with summary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("shape", "final_x_m", "max_abs_pitch_deg",
                                                     "bridge_max_abs_pitch_deg", "max_z_m",
                                                     "reached_target", "target_time_s", "csv"))
        writer.writeheader()
        writer.writerows(results)
    for result in results:
        print(result)
    print(f"summary: {summary}")
    if stop_event.is_set():
        print("Ctrl+C: simulation, viewer, control UI, and ROS 2 bridge stopped cleanly.")
        raise SystemExit(130)


if __name__ == "__main__":
    main()
