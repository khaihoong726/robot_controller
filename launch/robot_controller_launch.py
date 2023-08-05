from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_controller_node = Node(
        package="robot_controller",
        executable="robot_controller"
    )

    robot_serial_node = Node(
        package="robot_controller",
        executable="robot_serial"
    )

    robot_pid_node = Node(
        package="robot_controller",
        executable="robot_pid"
    )

    robot_odometry_node = Node(
        package="robot_controller",
        executable="robot_odometry"
    )

    ld.add_action(robot_controller_node)
    ld.add_action(robot_serial_node)
    ld.add_action(robot_pid_node)
    ld.add_action(robot_odometry_node)

    return ld
