# WIP

## Neato Drivers

This repository contains the Neato drivers for ROS2 Foxy.

## Install

Prerequisites:

    sudo apt install python3-serial

You can check this out into your workspace as follows:

    cd <ws>/src
    git clone https://github.com/jnugen/neato_robot.git
    cd <ws>
    colcon build
    source <ws>/install/setup.bash

## Launch

    ros2 launch neato_node neato_node_launch.py

## Try Teleop

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
