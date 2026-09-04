#!/usr/bin/env python3
"""Validate every flipper over -pi/2..pi/2 and write tracking metrics."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import mujoco

from mujoco_crawler import simulator as crawler


SOURCE_ROOT = Path(__file__).resolve().parents[1]


JOINTS = ("joint_left_front", "joint_left_rear",
          "joint_right_front", "joint_right_rear")
ANGLES = (-math.pi / 2, -math.pi / 4, 0.0, math.pi / 4, math.pi / 2)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path,
                        default=SOURCE_ROOT / "config" / "default.yaml")
    parser.add_argument(
        "--output", type=Path,
        default=SOURCE_ROOT / "results" / "flipper_control_validation.csv")
    args = parser.parse_args()

    cfg = crawler.load_config(args.config)
    # Isolate actuator tracking from impossible terrain-intersecting poses.
    cfg["environment"]["arena_yaml"] = ""
    rows = []
    for joint_index, joint_name in enumerate(JOINTS):
        for target in ANGLES:
            model = mujoco.MjModel.from_xml_string(
                crawler.build_xml(cfg, cfg["grouser"]["shape"]))
            data = mujoco.MjData(model)
            model.opt.gravity[:] = 0
            model.geom_contype[:] = 0
            model.geom_conaffinity[:] = 0
            targets = [0.0] * 4
            targets[joint_index] = target
            crawler.set_flipper_controls(model, data, cfg, targets)
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            actuator_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name + "_position")
            qpos_index = model.jnt_qposadr[joint_id]
            qvel_index = model.jnt_dofadr[joint_id]
            maximum_velocity = maximum_torque = maximum_overshoot = 0.0
            for _ in range(round(5.0 / model.opt.timestep)):
                crawler.update_flipper_controls(model, data, cfg)
                maximum_torque = max(maximum_torque, abs(float(data.ctrl[actuator_id])))
                crawler.step_physics(model, data, cfg)
                position = float(data.qpos[qpos_index])
                maximum_velocity = max(maximum_velocity, abs(float(data.qvel[qvel_index])))
                if target:
                    maximum_overshoot = max(
                        maximum_overshoot,
                        max(0.0, (position - target) * math.copysign(1.0, target)))
            actual = float(data.qpos[qpos_index])
            rows.append({
                "joint": joint_name, "target_rad": target, "actual_rad": actual,
                "error_deg": math.degrees(target - actual),
                "overshoot_deg": math.degrees(maximum_overshoot),
                "max_velocity_rad_s": maximum_velocity,
                "max_torque_Nm": maximum_torque,
            })
    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(output)
    print("max |error| [deg]:", max(abs(row["error_deg"]) for row in rows))
    print("max overshoot [deg]:", max(row["overshoot_deg"] for row in rows))
    print("max velocity [rad/s]:", max(row["max_velocity_rad_s"] for row in rows))
    print("max torque [N m]:", max(row["max_torque_Nm"] for row in rows))


if __name__ == "__main__":
    main()
