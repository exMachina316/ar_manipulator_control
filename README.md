# AR Manipulator Control

This repository contains the ROS 2 workspace for controlling a Universal Robots arm with gesture-based commands and mixed reality.

## Overview

The primary goal of this project is to provide an intuitive way to control a robotic manipulator using hand gestures. A user can define a trajectory for the robot by specifying waypoints in 3D space, which are then executed by the robot arm. The system integrates computer vision for gesture recognition, motion planning with MoveIt for trajectory execution, and a mixed reality interface for user interaction.

## Workspace Packages

This workspace is organized into several ROS 2 packages:

-   `mr_manipulator`: A Python package that provides a gesture-based mixed reality waypoint collector. It uses a computer vision model to recognize hand gestures and publish waypoints for the robot.

-   `follow_traj_moveit`: This C++ package contains an action server responsible for planning and executing trajectories based on the waypoints received from the `mr_manipulator` package. It uses MoveIt 2 for motion planning.

-   `ur_interfaces`: Contains custom ROS message and action definitions used for communication between the different nodes in the system.

## Dependencies

The workspace relies on the following external dependencies, which are included as submodules in the `dependencies` directory:

-   **Universal_Robots_ROS2_Driver**: The official ROS 2 driver for Universal Robots manipulators.
-   **Universal_Robots_ROS2_GZ_Simulation**: Provides Gazebo simulation models for UR robots.
-   **ur_rail_description**: Contains the URDF description for the UR robot mounted on a rail.

## Documentation

Detailed design documents, presentations, and reports can be found in the `docs/` directory. This includes information on the system architecture, computer vision model, and path planning components.

- [Camera Info](./workspace/docs/Camera%20Info.md)
- [Computer Vision Model](./workspace/docs/Computer%20Vision%20Model.md)
- [Path Planning and Execution Action Server](./workspace/docs/Path%20Planning%20and%20Execution%20Action%20Server.md)
- [Reference Trajectory Processor](./workspace/docs/Reference%20Trajectory%20Processor.md)
- [Steps to setup](./workspace/docs/Steps%20to%20setup.md)

## Demos

- [Adding Waypoints](./workspace/docs/demo_videos/adding_wp.mp4)
- [Hardware Setup](./workspace/docs/demo_videos/hardware_setup.mp4)
- [Simulation Setup](./workspace/docs/demo_videos/simulation_setup.mp4)

## License

This project is licensed under the Apache-2.0 License. See the `LICENSE` file in the `mr_manipulator` package for more details.
