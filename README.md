# Fred - Goal Manager

**note: Package for ROS 2 - C++/Python (CMake) based package**

The ROS 2 Goal Manager package provides nodes for managing goals in a robotic system. This package includes two nodes, each serving a distinct purpose in goal management.

---

## Installation

**1. Clone the repository into your ROS2 workspace:**

```bash
cd ros2_ws/src
git clone https://github.com/AMR-Frederico/fred2_goal_manager.git
```

**2. Build the package:**

```bash
cd ros2_ws
colcon build
```

---

## Usage

### Launch

**Considering `Robot Localization` for odometry and `Robot Descriptor` for publish the TFs:**
```
ros2 launch fred2_goal_manager goal_manager.launch.py 
```

**Considering `Move Base Odometry` to publish odom and TF:**
```
ros2 launch fred2_goal_manager goal_manager.launch.py
```

---


## Goal Provider

The `goal_provider` node is responsible for managing the publication of goals and handling the progression through the goal sequence.

### Parameters

- `frame_id`: The frame ID for the published goals. Default is 'odom'.

- `goals`: Goals array.

### Subscribed Topics

- `goal/reached` (*std_msgs/Bool*): Subscribe to receive updates on goal reached status.

- `odom/reset` (*std_msgs/Bool*): Reset goal index.

-` /machine_states/robot_state` (*std_msgs/Int16*): Robot state 

### Published Topics

- `goals` (geometry_msgs/PoseArray): Publishes an array of goal poses.

- `goal/current` (geometry_msgs/PoseStamped): Publishes the current goal pose.

- `goal/mission_completed` (std_msgs/Bool): Publishes whether the entire mission is completed.


### Run

```bash 
ros2 run fred2_goal_manager goal_provider.py
```

### Run with degub mode

```bash 
ros2 run fred2_goal_manager goal_provider.py --debug
```

---
 
## Goal Reached

The `goal_reached` node determines whether the robot has reached its goal based on odometry and the current goal information. It ensures safety constraints are met during robot movements and integrates with ultrasonic readings and joystick commands for enhanced safety.

### Parameters

- `robot_in_goal_tolerance` (float): Tolerance for considering the robot to be in the goal. Default is 0.1.


### Subscribed Topics

- `/odom` (nav_msgs/Odometry): Subscribe to receive odometry information from `Move Base`.

- `/odometry/filtered` (nav_msgs/Odometry): Subscribe to receive odometry information from `Robot Localization`.

- `goal/current` (geometry_msgs/PoseStamped): Subscribe to get the current goal pose.

### Published Topics

- `goal/reached` (std_msgs/Bool): Publishes whether the robot has reached its goal.

### Run

```bash 
ros2 run fred2_goal_manager goal_reached.py
```

### Run with degub mode

```bash 
ros2 run fred2_goal_manager goal_reached.py --debug
```

---