## Neato Drivers

This repository contains the Neato drivers for ROS2 Foxy.

## Usage
You can check this out into your workspace as follows:

    cd <ws>/src
    git clone https://github.com/jnugen/neato_robot.git
    cd <ws>
    colcon build
    source <ws>/install/setup.bash

## Launching

    ros2 launch neato_node neato_node_launch.py

## Trying Teleop

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
