# 3-DOF Robotic Arm

## Overview

This project presents the design and simulation of a three-degree-of-freedom (3-DOF) RRR robotic manipulator developed in MATLAB for precise end-effector positioning within a three-dimensional workspace. The robot consists of three revolute joints corresponding to base yaw, shoulder pitch, and elbow pitch motions.

A closed-form geometric inverse kinematics approach was implemented using the elbow-up configuration and validated through forward kinematic round-trip analysis. Cartesian trajectory planning with linear interpolation was employed to generate smooth end-effector motion between multiple target points. Independent PID controllers were designed for each joint to simulate servo motor actuation and trajectory tracking.

The project also incorporates simulated UART communication packets for hardware interfacing, comprehensive error analysis, and interactive 3D visualizations demonstrating manipulator motion, joint trajectories, control signals, and tracking performance.

---

## Features

* 3-DOF RRR robotic manipulator simulation in MATLAB.
* Closed-form geometric inverse kinematics.
* Elbow-up configuration selection.
* Forward kinematics verification.
* Workspace and reachability analysis.
* Cartesian trajectory planning.
* Linear interpolation between waypoints.
* Independent PID control loops for each joint.
* Simulated UART motor command transmission.
* Tracking error and performance analysis.
* Interactive 3D animation and visualization.

---

## Robot Configuration

The manipulator consists of three revolute joints:

* Joint 1: Base Yaw (θ1)
* Joint 2: Shoulder Pitch (θ2)
* Joint 3: Elbow Pitch (θ3)

### Link Lengths

| Link | Length |
| ---- | ------ |
| L1   | 30 mm  |
| L2   | 25 mm  |

---

## Workspace Analysis

A complete reachability study was performed within a 30 × 30 × 30 mm workspace.

### Reachability Limits

* Minimum Reach = 5 mm
* Maximum Reach = 55 mm

All target points were verified to lie within the reachable workspace.

---

## Inverse Kinematics

A direct geometric inverse kinematics approach was implemented without using Denavit-Hartenberg parameters.

### Computed Variables

* Base angle (θ1)
* Shoulder angle (θ2)
* Elbow angle (θ3)

Only the elbow-up configuration was considered to ensure smooth and consistent motion.

---

## Forward Kinematics Verification

Forward kinematics was used to verify the inverse kinematics solution.

Results showed:

* Maximum round-trip error = 0.0000 mm

confirming the correctness of the analytical formulation.

---

## Trajectory Planning

Straight-line Cartesian interpolation was implemented between target points:

* P1 = (10,10,25) mm
* P2 = (25,5,25) mm
* P3 = (20,25,25) mm
* P4 = (5,20,25) mm

The trajectory consists of:

* 3 segments
* 150 interpolated positions
* Smooth end-effector motion

---

## PID Control

Each joint is driven by an independent PID controller operating at 50 Hz.

### Controller Parameters

| Parameter      | Value      |
| -------------- | ---------- |
| Kp             | 2.0        |
| Ki             | 0.05       |
| Kd             | 0.8        |
| Sampling Time  | 0.02 s     |
| Velocity Limit | ±400 deg/s |

The controllers simulate servo motor behavior and trajectory tracking.

---

## Serial Communication

Motor commands are formatted into UART packets:

```text
<J1:angle, J2:angle, J3:angle>
```

This communication framework enables future hardware implementation using:

* UART
* Serial communication
* Arduino
* Embedded controllers

---

## Error Analysis

Major error sources investigated include:

* PID tracking lag
* Numerical precision
* Discretization effects
* Encoder quantization
* Mechanical backlash
* Communication delay

The simulation demonstrated negligible inverse kinematic error and satisfactory tracking performance.

---

## Results

✔ Closed-form inverse kinematics successfully implemented.

✔ Workspace reachability verified.

✔ Zero round-trip position error achieved.

✔ Smooth Cartesian trajectories generated.

✔ PID-based joint tracking simulated.

✔ UART packet communication framework developed.

✔ Interactive 3D visualization completed.

---

## Project Structure

```text
Robotic-Arm-3-DOF
│
├── Code
│   └── roboticarm3dof.m
│
├── images
│   ├── End_Effector_Trajectory.png
│   ├── PID_Tracking_Error.png
│   ├── PID_Joint_Angle_Tracking.png
|   ├── Serial_UART.png
│   └── 3DOF_Robot_arm.png
│
├── video
│   └── roboticarm3dof.mp4
│
├── report
│   └── Report
│
└── README.md
```

---

## Images

### Robot Configuration

![Robot Arm](images/robot_arm.png)

### Workspace Analysis

![Workspace](images/workspace.png)

### Trajectory Planning

![Trajectory](images/trajectory.png)

### PID Response

![Control Signals](images/control_response.png)

---

## Demonstration Video

https://github.com/user-attachments/assets/your-video-link

---

## Future Improvements

* Dynamics modeling using Newton-Euler or Lagrange formulations.
* Gravity compensation.
* Joint limit constraints.
* Elbow-up and elbow-down configuration switching.
* Collision avoidance.
* Real-time serial communication with hardware.
* Extension to a 6-DOF manipulator.
* Integration with ROS and Gazebo.

---

## Author

**Rohit**

M.Tech in Advanced Manufacturing and Design
Mechanical Engineering
