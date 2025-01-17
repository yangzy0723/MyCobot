# Environment
- Ubuntu 24.04
- ROS Jazzy
- Gazebo Harmonic
- MoveIt 2（ros-jazzy-moveit）

  Maybe you need to install ROS 2 Control packages:
  ```shell
  sudo apt-get install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-gripper-controllers ros-${ROS_DISTRO}-gazebo-ros2-control
  ```

# Introduction
This repository implements joint calls between ROS Jazzy Gazebo Harmonic and MoveIt 2 under the Ubuntu 24.04 operating system, which can control the interactive marker in Rviz2, correctly plan motion trajectories, and execute them in the Gazebo simulation environment.

![](figs/show.png)
# Use
please start in the workspace folder.
```bash
colcon build --packages-select mycobot_description mycobot_gazebo mycobot_moveit
source install/setup.sh
bash src/robot.sh
```
# Todo
- [x] The gripper is still unstable and needs to be improved...

- [x] Use the Open Motion Planning Library to complete obstacle avoidance for robotic arms

- [ ] Try to use the movit2 task constructor
