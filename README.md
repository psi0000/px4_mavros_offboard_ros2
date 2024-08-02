# Drone Gazebo Simulation Environment with ROS2, PX4, and MAVROS

Welcome to the **Drone Simulation Environment** project! This repository provides a comprehensive setup for simulating drones using ROS2, Gazebo Sim, PX4, and MAVROS. The ultimate goal is to establish a robust experimental environment for multi-robot systems. (`Task allocation`)

## Prerequisites

Before using this repository, ensure you have the following installed:

- ROS2 ( Humble )
- PX4  (px4_msgs , px4_ros_com )
- MAVROS
- MAVLINK
- QGC

For those who need a complete setup, Docker images are available at the link below. These images provide a fully configured environment to get you started quickly.

[Download Docker Images](#) <!-- Add your Docker image link here -->

## Reference

### `px4`
- **`install`** : 
https://docs.px4.io/main/en/ros2/user_guide.html
https://github.com/PX4/PX4-Autopilot.git
https://github.com/PX4/px4_msgs.git
https://github.com/PX4/px4_ros_com.git

- **`multi_spawn`** : https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation.html

### `mavros`
- **`install`** : 
https://github.com/mavlink/mavros/


etc ...

## Branch Overview

### `base_sim` Branch

This branch contains the fundamental scripts for basic drone operations.

- **`start.py`**: This script handles the takeoff and transitions the drone to OFFBOARD mode, allowing it to hover in the air.
- **`wp_patrol.py`**: This script patrols a predefined set of waypoints and returns the drone to its takeoff location.

These scripts are designed for single robot operations and use MAVROS for local position control. 
For global position commands, consider using different services and message types. Note that the drone must be in OFFBOARD mode to respond to waypoint commands. 
If you prefer manual takeoff and landing without using AUTO modes, refer to the commented sections in `wp_patrol.py`.

### Execution

To run the scripts, use the following commands:

```bash
ros2 run custom start
ros2 run custom wp_patrol
```

### `multi_sim` Branch

Development for multi-robot simulation is ongoing. Stay tuned for updates!

### Contributions
We welcome contributions to enhance this project! If you find this project helpful, please let us know or consider contributing to its development. We appreciate every bit of support and feedback from our community!