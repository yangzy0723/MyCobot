#!/bin/bash
# Single script to launch the mycobot with Gazebo, RViz, and MoveIt 2

cleanup() {
    echo "Restarting ROS 2 daemon to cleanup before shutting down all processes..."
    ros2 daemon stop
    sleep 1
    ros2 daemon start
    echo "Terminating all ROS 2-related processes..."
    kill 0
    exit
}

trap 'cleanup' SIGINT;
ros2 launch mycobot_gazebo gz.launch.py&
sleep 10
ros2 launch moveit.launch.py
