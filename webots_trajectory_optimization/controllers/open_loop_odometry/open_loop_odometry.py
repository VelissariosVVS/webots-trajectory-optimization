"""Open-loop wheel command player with encoder-based odometry logging.

This controller replays optimized wheel-speed commands and estimates the robot pose
from wheel encoders. It is useful for validating the trajectory plan without using
GPS/Compass feedback.

Expected control file columns:
    time_s, left_rad_s, right_rad_s[, x_ref_m, y_ref_m, theta_ref_rad]
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from controller import Robot


@dataclass(frozen=True)
class RobotConfig:
    wheel_radius_m: float = 0.0975
    wheel_base_m: float = 0.331
    max_wheel_speed_rad_s: float = 6.28
    max_duration_s: float = 11.0


class OpenLoopOdometryController:
    def __init__(self, robot: Robot, controls_path: Path) -> None:
        self.robot = robot
        self.timestep_ms = int(robot.getBasicTimeStep())
        self.dt = self.timestep_ms / 1000.0
        self.config = RobotConfig()

        self.left_motor = robot.getDevice("left wheel")
        self.right_motor = robot.getDevice("right wheel")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.stop()

        self.left_encoder = robot.getDevice("left wheel sensor")
        self.right_encoder = robot.getDevice("right wheel sensor")
        self.left_encoder.enable(self.timestep_ms)
        self.right_encoder.enable(self.timestep_ms)

        self.controls = self._load_controls(controls_path)
        self.pose = np.array([0.0, 0.0, 0.0])
        self.last_left_encoder = 0.0
        self.last_right_encoder = 0.0

        log_path = Path(__file__).with_name("odometry_log.csv")
        self.log_file: Optional[object] = open(log_path, "w", newline="", encoding="utf-8")
        self.logger = csv.writer(self.log_file)
        self.logger.writerow(["time_s", "x_m", "y_m", "theta_rad", "left_cmd_rad_s", "right_cmd_rad_s"])

    @staticmethod
    def _load_controls(path: Path) -> np.ndarray:
        if not path.exists():
            raise FileNotFoundError(f"Control file not found: {path}")
        data = np.loadtxt(path, comments="#", delimiter="\t")
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] < 3:
            raise ValueError("Control file must contain at least: time, left, right")
        return data

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def _update_odometry(self) -> None:
        left_encoder = float(self.left_encoder.getValue())
        right_encoder = float(self.right_encoder.getValue())

        left_distance = (left_encoder - self.last_left_encoder) * self.config.wheel_radius_m
        right_distance = (right_encoder - self.last_right_encoder) * self.config.wheel_radius_m
        self.last_left_encoder = left_encoder
        self.last_right_encoder = right_encoder

        distance = 0.5 * (left_distance + right_distance)
        delta_heading = (right_distance - left_distance) / self.config.wheel_base_m

        self.pose[2] = self._normalize_angle(self.pose[2] + delta_heading)
        self.pose[0] += distance * math.cos(self.pose[2])
        self.pose[1] += distance * math.sin(self.pose[2])

    def _command_at(self, step: int) -> tuple[float, float]:
        left = float(self.controls[step, 1])
        right = float(self.controls[step, 2])
        return self._limit(left), self._limit(right)

    def _limit(self, wheel_speed: float) -> float:
        return float(np.clip(wheel_speed, -self.config.max_wheel_speed_rad_s, self.config.max_wheel_speed_rad_s))

    def run(self) -> None:
        elapsed_time = 0.0
        step = 0
        while (
            self.robot.step(self.timestep_ms) != -1
            and elapsed_time < self.config.max_duration_s
            and step < len(self.controls)
        ):
            self._update_odometry()
            left_cmd, right_cmd = self._command_at(step)
            self.left_motor.setVelocity(left_cmd)
            self.right_motor.setVelocity(right_cmd)
            self.logger.writerow([elapsed_time, *self.pose.tolist(), left_cmd, right_cmd])

            elapsed_time += self.dt
            step += 1

        self.stop()
        if self.log_file is not None:
            self.log_file.close()

    def stop(self) -> None:
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)


if __name__ == "__main__":
    webots_robot = Robot()
    project_root = Path(__file__).resolve().parents[2]
    control_file = project_root / "data" / "timed_controls_no_obstacles.txt"
    OpenLoopOdometryController(webots_robot, control_file).run()
