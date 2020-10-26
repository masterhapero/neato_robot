from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "neato_node",
            executable = "neato_node",
            name = "neato_node",
            output = "screen",
            emulate_tty = True,
            parameters = [
                {"port": "/dev/ttyACM0"}
            ]
        )
    ])
