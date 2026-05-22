# Project Notes

This repository contains a Webots simulation for optimized trajectory tracking with a Pioneer 3-DX differential-drive robot.

## What was cleaned up

- Replaced script-style controllers with class-based, documented Python controllers.
- Added robust file loading, validation, speed limiting, angle normalization, and CSV logging.
- Organized the Webots world, controller code, trajectory/control data, and assets into a repository-ready structure.
- Updated the Webots world to use the production controller name `trajectory_follower`.

## Important technical note

The current Webots world uses `Pioneer3dx`, which is a differential-drive robot with left and right wheels. The uploaded code and data also use left/right wheel commands. If the final project should be described as an omnidirectional robot, the Webots model and controller should be changed to a mecanum/omni-wheel platform with the correct inverse kinematics. For the current files, the accurate description is optimized trajectory tracking for a differential-drive mobile robot.

## Controllers

- `trajectory_follower`: closed-loop GPS/Compass trajectory tracking.
- `open_loop_odometry`: open-loop optimized control playback with encoder odometry logging.
