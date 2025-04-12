# TurtleBot3 Navigation Control

## Overview

This project implements a ROS-based navigation control system for the TurtleBot3 robot in a Gazebo simulation environment (`turtlebot3_world`). The application (`turtlebot3_mover.cpp`) enables the TurtleBot3 to navigate randomly within a predefined map by:

- Dynamically setting an initial pose for AMCL (Adaptive Monte Carlo Localization) using the robot’s position from Gazebo’s `/odom` topic.
- Sending random navigation goals using the `move_base` action server.
- Visualizing the robot, map, laser scans, and AMCL particle cloud in RViz.

The project is built for ROS Noetic and uses the TurtleBot3 simulation stack (`turtlebot3_simulations`) and navigation stack (`turtlebot3_navigation`). A single launch file (`turtlebot3_mover_visual.launch`) combines the Gazebo simulation, navigation stack, RViz, and the custom navigation node for a streamlined workflow.

---

## Features

- **Dynamic Initial Pose Setting**: Automatically sets the initial pose for AMCL using the robot’s position from `/odom` after Gazebo boots.
- **Random Navigation**: Sends random goals within the map boundaries using `move_base`.
- **Gazebo Integration**: Simulates the TurtleBot3 in the `turtlebot3_world` environment.
- **RViz Visualization**: Launches RViz with a pre-configured view to display the map, robot model, laser scans, and AMCL particle cloud.
- **Obstacle Avoidance**: Uses `move_base` and costmaps for safe navigation.
- **Single Launch File**: Combines Gazebo, navigation stack, RViz, and the custom node into one launch command.

---

## Project Structure

The project is a ROS package named `turtlebot3_mover`, located in a Catkin workspace (e.g., `~/catkin_ws/src/turtlebot3_mover/`).

---

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 20.04 (Focal Fossa)
- **ROS Distribution**: ROS Noetic Ninjemys
- **Gazebo Version**: Gazebo 11 (compatible with ROS Noetic)

### ROS Packages
The following ROS packages are required:

#### Core ROS Packages
- `roscpp`: Core C++ library for ROS nodes.
- `actionlib`: For interacting with the `move_base` action server.
- `move_base_msgs`: Message definitions for `move_base` actions.
- `geometry_msgs`: Standard geometry messages (e.g., `PoseWithCovarianceStamped`).
- `nav_msgs`: Navigation messages (e.g., `Odometry`).
- `tf`: Transform library for handling quaternions and rotations.
- `sensor_msgs`: Sensor messages (e.g., `LaserScan`).

#### Navigation and Localization
- `move_base`: ROS navigation stack for path planning and obstacle avoidance.
- `amcl`: Adaptive Monte Carlo Localization for robot localization.
- `map_server`: For serving the static map.
- `costmap_2d`: For creating global and local costmaps.
- `nav_core`: Core navigation interfaces.

#### TurtleBot3 Packages
- `turtlebot3`: Core TurtleBot3 package (includes `turtlebot3_description`, etc.).
- `turtlebot3_simulations`: Gazebo simulation for TurtleBot3 (includes `turtlebot3_gazebo`).
- `turtlebot3_navigation`: Navigation configuration for TurtleBot3 (includes AMCL and `move_base` configs).

#### Gazebo Simulation
- `gazebo_ros`: ROS integration for Gazebo.
- `gazebo_plugins`: Plugins for sensors (ex: LiDAR is using for this project) and actuators in Gazebo.

#### Visualization
- `rviz`: Visualization tool for ROS.

#### Build Tools
- `catkin`: ROS build system.
- `cmake`, `g++`: For compiling C++ code.

### System Dependencies
- **ROS Noetic**: `sudo apt install ros-noetic-desktop-full`
- **Gazebo 11**: `sudo apt install gazebo11 libgazebo11-dev`
- **Python ROS Tools**: `sudo apt install python3-rosdep python3-rospkg python3-rospy`
- **Build Tools**: `sudo apt install cmake g++`

---

## Installation

### 1. Install ROS Noetic
If ROS Noetic is not already installed:

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install python3-rosdep python3-rospkg
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Gazebo
Ensure Gazebo 11 is installed:

```bash
sudo apt install gazebo11 libgazebo11-dev
```

### 3. Install TurtleBot3 Packages
Install the TurtleBot3 packages for simulation and navigation:

```bash
sudo apt install ros-noetic-turtlebot3
sudo apt install ros-noetic-turtlebot3-simulations
sudo apt install ros-noetic-turtlebot3-navigation
```

### 4. Install Navigation and Gazebo Dependencies
Install the required navigation and Gazebo packages:

```bash
sudo apt install ros-noetic-move-base ros-noetic-amcl ros-noetic-map-server ros-noetic-costmap-2d ros-noetic-nav-core ros-noetic-gazebo-ros ros-noetic-gazebo-plugins
```

### 5. Install RViz
Install RViz for visualization:
``` bash
sudo apt install ros-noetic-rviz
```

### 6. Set Up TurtleBot3 Model
Set the TURTLEBOT3_MODEL environment variable (e.g., for the Burger model):
``` bash
export TURTLEBOT3_MODEL=burger
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```
### 7. Set Up the Catkin Workspace
Clone or copy this project into your Catkin workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone ...
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## Usage
### Launch the aplication
The turtlebot3_mover_visual.launch file combines Gazebo, the navigation stack, RViz, and the custom navigation node into a single command.
```bash
source ~/.bashrc
roslaunch turtlebot3_navigation turtlebot3_move.launch
```

This launch file will:
Start Gazebo with the turtlebot3_world environment.
Launch the navigation stack (map server, AMCL, move_base).
Run the turtlebot3_mover node to set the initial pose and navigate randomly.
Open RViz with the pre-configured turtlebot3_navigation.rviz view for verification

[Demo video link](https://drive.google.com/file/d/13wv3u3T-fibwXXTHQ265JH7O8wMmSWan/view?usp=drive_link)

