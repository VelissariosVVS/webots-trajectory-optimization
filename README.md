# Trajectory Optimization and Simulation for Autonomous Mobile Robot in Webots

## Overview

This project presents a trajectory optimization and motion control framework for an autonomous mobile robot simulated in the Webots environment. The system generates optimized wheel control trajectories and evaluates robot performance using simulation-based tracking, odometry, and feedback control techniques.

The project demonstrates:

* Trajectory generation using optimized wheel velocity profiles
* Closed-loop trajectory tracking
* Encoder-based odometry estimation
* Simulation in Webots
* Differential-drive robot motion control
* Obstacle and non-obstacle navigation scenarios

The implementation focuses on robotics control, motion planning, and simulation validation for autonomous navigation research.

---

## Simulation Preview

The robot follows optimized trajectories inside a Webots simulation environment while tracking position references generated offline.

Features demonstrated include:

* Smooth path following
* Wheel velocity optimization
* Real-time feedback correction
* Odometry estimation
* Obstacle-aware trajectory execution

---

## Technologies Used

* **Python**
* **Webots**
* **NumPy**
* **Differential Drive Kinematics**
* **Feedback Control (P / PD Control)**
* **Robot Odometry**
* **Trajectory Optimization**

---

## Project Structure

```bash
.
├── controllers/
│   ├── controller1.py
│   ├── controller_odometry.py
│
├── trajectories/
│   ├── trajectory.txt
│   ├── timed_controls.txt
│   ├── timed_controls_no_Obstacles.txt
│
├── worlds/
│   ├── trying_odometry.wbt
│
├── media/
│   ├── simulation_preview.jpg
│
└── README.md
```

---

## Core Components

### 1. Trajectory Tracking Controller

The primary controller loads optimized trajectory data and performs trajectory tracking using GPS and compass feedback.

Implemented features:

* Position tracking
* Heading correction
* Linear/angular velocity control
* Wheel velocity conversion
* Closed-loop feedback stabilization

The controller computes tracking error in the robot frame and applies proportional control for trajectory correction.

Relevant implementation:

* `controller1.py`

---

### 2. Encoder-Based Odometry

The odometry controller estimates robot pose using wheel encoder measurements.

Features include:

* Incremental encoder integration
* Velocity estimation
* Wheel displacement tracking
* Robot pose estimation

Relevant implementation:

* `controller_odometry.py`

---

### 3. Optimized Trajectory Data

Trajectory files contain:

* Time stamps
* Left/right wheel velocities
* Optimized x-y reference positions
* Heading references

These trajectories are replayed during simulation for validation and performance analysis.

Files:

* `trajectory.txt`
* `timed_controls.txt`
* `timed_controls_no_Obstacles.txt`

---

## Control Methodology

The robot uses differential-drive kinematics:

```math
v = \frac{v_r + v_l}{2}
```

```math
\omega = \frac{v_r - v_l}{L}
```

Where:

* ( v_r ) = right wheel velocity
* ( v_l ) = left wheel velocity
* ( L ) = wheel separation distance

The controller converts desired linear and angular velocities into wheel commands while minimizing trajectory tracking error.

---

## Running the Project

### Requirements

Install:

* Webots
* Python 3.9+
* NumPy

Install Python dependency:

```bash
pip install numpy
```

---

## How to Run

### 1. Open the Webots World

Open:

```bash
trying_odometry.wbt
```

inside Webots.

---

### 2. Assign Controllers

Attach:

* `controller1.py`
  or
* `controller_odometry.py`

to the robot controller field inside Webots.

---

### 3. Run Simulation

Start the simulation from Webots.

The robot will execute optimized wheel trajectories and perform autonomous trajectory tracking.

---

## Experimental Features

This project includes:

* Obstacle and non-obstacle trajectory execution
* Reference trajectory playback
* Feedback trajectory correction
* Real-time odometry estimation
* Wheel encoder validation


---

## Research Relevance

This project demonstrates concepts commonly used in:

* Autonomous robotics
* Mobile robot navigation
* Motion planning
* Robotics simulation
* Trajectory optimization
* Control systems engineering

---

## Author

VelissariosVVS

GitHub: [GitHub Profile]([(https://github.com/VelissariosVVS)])

---

## License

This project is open-source and available under the MIT License.
