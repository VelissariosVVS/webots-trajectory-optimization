"""Closed-loop trajectory follower for a Webots Pioneer 3-DX robot.

The controller tracks an optimized reference trajectory using GPS/Compass feedback
and converts the resulting linear/angular velocity command into differential-drive
wheel speeds.

Expected trajectory file columns:
    u_left_rad_s, u_right_rad_s, x_m, y_m[, theta_rad]

The optimized wheel speeds are kept available for feedforward/debugging, while the
active controller uses pose feedback for robust tracking in simulation.
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
    simulation_duration_s: float = 10.0


@dataclass(frozen=True)
class ControllerGains:
    linear_kp: float = 3.0
    linear_kd: float = 0.0
    angular_kp: float = 5.0
    angular_kd: float = 0.0


class TrajectoryFollower:
    """Tracks optimized x/y waypoints with a heading-aware PD controller."""

    def __init__(self, robot: Robot, trajectory_path: Path) -> None:
        self.robot = robot
        self.timestep_ms = int(robot.getBasicTimeStep())
        self.dt = self.timestep_ms / 1000.0
        self.config = RobotConfig()
        self.gains = ControllerGains()

        self.left_motor = robot.getDevice("left wheel")
        self.right_motor = robot.getDevice("right wheel")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.stop()

        self.gps = robot.getDevice("gps")
        self.compass = robot.getDevice("compass")
        self.gps.enable(self.timestep_ms)
        self.compass.enable(self.timestep_ms)

        self.reference = self._load_trajectory(trajectory_path)
        self.previous_linear_error = 0.0
        self.previous_heading_error = 0.0

        log_path = Path(__file__).with_name("tracking_log.csv")
        self.log_file: Optional[object] = open(log_path, "w", newline="", encoding="utf-8")
        self.logger = csv.writer(self.log_file)
        self.logger.writerow(
            [
                "time_s",
                "target_x_m",
                "target_y_m",
                "actual_x_m",
                "actual_y_m",
                "heading_rad",
                "left_cmd_rad_s",
                "right_cmd_rad_s",
            ]
        )

    @staticmethod
    def _load_trajectory(path: Path) -> np.ndarray:
        if not path.exists():
            raise FileNotFoundError(f"Trajectory file not found: {path}")

        data = np.loadtxt(path, comments="#", delimiter="\t")
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] < 4:
            raise ValueError(
                "Trajectory file must contain at least 4 columns: "
                "u_left, u_right, x, y"
            )
        return data

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def _current_pose(self) -> tuple[float, float, float]:
        x, y = self.gps.getValues()[:2]
        north = self.compass.getValues()
        heading = math.atan2(north[0], north[1])
        return float(x), float(y), float(heading)

    def _reference_at(self, time_s: float) -> tuple[float, float]:
        index = min(int(time_s / self.dt), len(self.reference) - 1)
        return float(self.reference[index, 2]), float(self.reference[index, 3])

    def _compute_wheel_speeds(
        self, target_x: float, target_y: float, actual_x: float, actual_y: float, heading: float
    ) -> tuple[float, float]:
        error_x = target_x - actual_x
        error_y = target_y - actual_y

        forward_error = error_x * math.cos(heading) + error_y * math.sin(heading)
        desired_heading = math.atan2(error_y, error_x)
        heading_error = self._normalize_angle(desired_heading - heading)

        d_forward_error = (forward_error - self.previous_linear_error) / self.dt
        d_heading_error = (heading_error - self.previous_heading_error) / self.dt
        self.previous_linear_error = forward_error
        self.previous_heading_error = heading_error

        linear_velocity = (
            self.gains.linear_kp * forward_error + self.gains.linear_kd * d_forward_error
        )
        angular_velocity = (
            self.gains.angular_kp * heading_error + self.gains.angular_kd * d_heading_error
        )

        left = (linear_velocity - 0.5 * self.config.wheel_base_m * angular_velocity) / self.config.wheel_radius_m
        right = (linear_velocity + 0.5 * self.config.wheel_base_m * angular_velocity) / self.config.wheel_radius_m
        return self._limit(left), self._limit(right)

    def _limit(self, wheel_speed: float) -> float:
        return float(
            np.clip(
                wheel_speed,
                -self.config.max_wheel_speed_rad_s,
                self.config.max_wheel_speed_rad_s,
            )
        )

    def run(self) -> None:
        time_s = 0.0
        while self.robot.step(self.timestep_ms) != -1 and time_s <= self.config.simulation_duration_s:
            actual_x, actual_y, heading = self._current_pose()
            target_x, target_y = self._reference_at(time_s)
            left_cmd, right_cmd = self._compute_wheel_speeds(target_x, target_y, actual_x, actual_y, heading)

            self.left_motor.setVelocity(left_cmd)
            self.right_motor.setVelocity(right_cmd)
            self.logger.writerow([time_s, target_x, target_y, actual_x, actual_y, heading, left_cmd, right_cmd])
            time_s += self.dt

        self.stop()
        if self.log_file is not None:
            self.log_file.close()

    def stop(self) -> None:
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)


if __name__ == "__main__":
    webots_robot = Robot()
    project_root = Path(__file__).resolve().parents[2]
    trajectory_file = project_root / "data" / "trajectory.txt"
    TrajectoryFollower(webots_robot, trajectory_file).run()
