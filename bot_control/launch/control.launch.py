from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description(namespace="BallCatcher"):
    ld = LaunchDescription()
    controlNode = Node(
        package="bot_control",
        namespace=namespace,
        executable="brain.py",
        output="screen",
    )

    ld.add_action(controlNode)

    return ld