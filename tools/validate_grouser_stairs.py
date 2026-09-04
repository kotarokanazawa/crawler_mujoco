#!/usr/bin/env python3
"""Compare grouser shapes on stair traversal, pivot turning, and CPU load."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import time

import mujoco
import numpy as np

from crawler_mujoco import simulator as crawler


SOURCE_ROOT = Path(__file__).resolve().parents[1]


SHAPES = ("rectangle", "semicircle", "spike")


def yaw(model, data) -> float:
    w, x, y, z = map(float, data.sensor("body_orientation").data)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pivot_test(cfg: dict, shape: str, settle_s: float, test_s: float,
               angular: float) -> dict:
    build_start = time.perf_counter()
    model = mujoco.MjModel.from_xml_string(crawler.build_xml(cfg, shape))
    build_wall = time.perf_counter() - build_start
    data = mujoco.MjData(model)
    crawler.set_velocity_controls(model, data, cfg, 0.0, 0.0)
    crawler.set_flipper_controls(model, data, cfg, (0.0, 0.0, 0.0, 0.0))
    settle_steps = round(settle_s / model.opt.timestep)
    for _ in range(settle_steps):
        crawler.wrap_straight_sections(model, data, cfg)
        crawler.update_track_velocity_controls(model, data, cfg)
        crawler.update_flipper_controls(model, data, cfg)
        crawler.step_physics(model, data, cfg)

    start_position = data.sensor("body_position").data.copy()
    previous_yaw = yaw(model, data)
    accumulated_yaw = 0.0
    crawler.set_velocity_controls(model, data, cfg, 0.0, angular)
    sim_start = time.perf_counter()
    cpu_start = time.process_time()
    for _ in range(round(test_s / model.opt.timestep)):
        crawler.wrap_straight_sections(model, data, cfg)
        crawler.update_track_velocity_controls(model, data, cfg)
        crawler.update_flipper_controls(model, data, cfg)
        crawler.step_physics(model, data, cfg)
        current_yaw = yaw(model, data)
        delta = (current_yaw - previous_yaw + math.pi) % (2 * math.pi) - math.pi
        accumulated_yaw += delta
        previous_yaw = current_yaw
    wall = time.perf_counter() - sim_start
    cpu = time.process_time() - cpu_start
    end_position = data.sensor("body_position").data.copy()
    drift = float(np.linalg.norm(end_position[:2] - start_position[:2]))
    return {
        "pivot_angle_deg": math.degrees(accumulated_yaw),
        "pivot_rate_deg_s": math.degrees(accumulated_yaw) / test_s,
        "pivot_xy_drift_m": drift,
        "pivot_wall_s": wall,
        "pivot_cpu_s": cpu,
        "pivot_realtime_factor": test_s / wall,
        "model_build_wall_s": build_wall,
    }


def traversal_test(cfg: dict, shape: str, output: Path) -> dict:
    start_wall, start_cpu = time.perf_counter(), time.process_time()
    result = crawler.run(cfg, shape, output)
    wall = time.perf_counter() - start_wall
    cpu = time.process_time() - start_cpu
    with Path(result["csv"]).open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    x_values = [float(row["x_m"]) for row in rows]
    result.update({
        "max_x_m": max(x_values),
        "progress_m": max(x_values) - float(cfg["robot"]["spawn_x"]),
        "traversal_wall_s": wall,
        "traversal_cpu_s": cpu,
        "traversal_realtime_factor": float(cfg["simulation"]["duration"]) / wall,
    })
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path,
                        default=SOURCE_ROOT / "config" / "default.yaml")
    parser.add_argument("--output", type=Path,
                        default=SOURCE_ROOT / "results" / "stair_grouser_validation")
    parser.add_argument("--pivot-angular", type=float, default=1.0)
    parser.add_argument("--pivot-duration", type=float, default=10.0)
    args = parser.parse_args()
    cfg = crawler.load_config(args.config)
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for shape in SHAPES:
        print(f"[{shape}] traversal", flush=True)
        result = traversal_test(cfg, shape, args.output)
        print(f"[{shape}] pivot", flush=True)
        result.update(pivot_test(cfg, shape, 2.0, args.pivot_duration,
                                 args.pivot_angular))
        results.append(result)
        print(f"[{shape}] max_x={result['max_x_m']:.3f}, "
              f"pivot={result['pivot_rate_deg_s']:.2f} deg/s, "
              f"RTF={result['traversal_realtime_factor']:.2f}", flush=True)

    fields = (
        "shape", "reached_target", "final_x_m", "max_x_m", "progress_m",
        "max_abs_pitch_deg", "max_z_m", "target_time_s",
        "pivot_angle_deg", "pivot_rate_deg_s", "pivot_xy_drift_m",
        "traversal_wall_s", "traversal_cpu_s", "traversal_realtime_factor",
        "pivot_wall_s", "pivot_cpu_s", "pivot_realtime_factor",
        "model_build_wall_s",
    )
    with (args.output / "comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: result.get(field, "") for field in fields}
                         for result in results)
    print(f"result: {args.output / 'comparison.csv'}")


if __name__ == "__main__":
    main()
