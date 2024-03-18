# Robot GUI

## Overview

The `robot_gui` package is a critical component of a larger project aimed at enabling intuitive teleoperation and monitoring of robots through a user-friendly graphical interface. Developed with C++, ROS, and the CVUI library, this package encapsulates the implementation of the graphical user interface (GUI) that interacts with various ROS nodes to provide real-time data visualization and control mechanisms for robots within a simulation environment.

## Installation

### Prerequisites

- ROS (Kinetic, Melodic, Noetic, or other compatible versions)
- catkin workspace
- C++11 or later
- OpenCV (for CVUI)
- CVUI library

### Setup

1. Navigate to your catkin workspace's source directory:
   ```bash
   cd ~/catkin_ws/src
   ```
2. Clone the `robot_gui` package repository:
   ```bash
   git clone https://github.com/MiguelSolisSegura/robot_gui.git
   ```
3. Compile the package using catkin:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
4. Source the environment setup script:
   ```bash
   source devel/setup.bash
   ```

## Usage

To launch the GUI application, ensure the necessary ROS nodes and simulation environment are running. Then, execute the following command:

```bash
rosrun robot_gui robot_gui_node
```

### GUI Features

- **General Info Area:** Displays real-time updates on robot information published to the `robot_info` topic.
- **Teleoperation Controls:** Buttons to increase/decrease linear and angular velocities, affecting the robot's movement in the simulation.
- **Velocity Display:** Shows the current linear and angular velocities.
- **Robot Position Display:** Visualizes the robot's current position based on odometry data.
- **Distance Traveled:** A feature to display the distance traveled by the robot, updated via a service call to `/get_distance`.
- **Reset Distance Service:** An update to the `distance_tracker_service` node to include a service for resetting the distance counter.
