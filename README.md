# Goal Manager

**note: Package for ROS 2 - C++/Python (CMake) based package**

The ROS 2 Goal Manager package provides nodes for managing goals in a robotic system. This package includes two nodes, each serving a distinct purpose in goal management.

## Nodes

### Goal Provider

The `goal_provider` node is responsible for managing the publication of goals and handling the progression through the goal sequence.

#### Parameters

- `frame_id` (string): The frame ID for the published goals. Default is 'odom'.
- `debug_mode` (bool): Enable debugging mode. Default is False.

#### Subscribed Topics

- `goal/reached` (std_msgs/Bool): Subscribe to receive updates on goal reach status.
- `goal/reset` (std_msgs/Bool): Subscribe to reset the goals.

#### Published Topics

- `goals` (geometry_msgs/PoseArray): Publishes an array of goal poses.
- `goal/current` (geometry_msgs/PoseStamped): Publishes the current goal pose.
- `goal/mission_completed` (std_msgs/Bool): Publishes whether the entire mission is completed.

#### Run

```bash 
ros2 run fred2_goal_manager goal_provider.py
```

#### Run with degub mode

```bash 
ros2 run fred2_goal_manager goal_provider.py --debug
```



---
 
### Goal Reached

The `goal_reached` node determines whether the robot has reached its goal based on odometry and the current goal information. It ensures safety constraints are met during robot movements and integrates with ultrasonic readings and joystick commands for enhanced safety.

#### Parameters

- `robot_in_goal_tolerance` (float): Tolerance for considering the robot to be in the goal. Default is 0.1.
- `debug_mode` (bool): Enable debugging mode. Default is False.

#### Subscribed Topics

- `/odom` (nav_msgs/Odometry): Subscribe to receive odometry information.
- `goal/current` (geometry_msgs/PoseStamped): Subscribe to get the current goal pose.

#### Published Topics

- `goal/reached` (std_msgs/Bool): Publishes whether the robot has reached its goal.

#### Run

```bash 
ros2 run fred2_goal_manager goal_reached.py
```

#### Run with degub mode

```bash 
ros2 run fred2_goal_manager goal_reached.py --debug
```

---


## Installation

**Clone the repository into your ROS 2 workspace:**

   ```bash
   cd ros2_ws/src
   git clone https://github.com/your_username/fred2_goal_manager.git
```

**Build the package:**
```bash
cd ros2_ws
colcon build
```

---

## Usage

**Launch the package:**

```bash
ros2 launch fred2_goal_manager goal_manager_launch.yaml
```


