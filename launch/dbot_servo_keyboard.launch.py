from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments and nodes
    declared_arguments = []
    nodes = []

    # Moveit Servo
    # Get parameters for the Servo node
    servo_keyboard_node = Node(
        package="dbot_servo",
        executable="dbot_servo_keyboard_node",
        parameters=[],
        output="screen",
    )

    # Add nodes
    nodes.append(servo_keyboard_node)

    return LaunchDescription(declared_arguments + nodes)