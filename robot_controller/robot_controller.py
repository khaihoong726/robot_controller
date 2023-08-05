import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .helper.differential_drive import DifferentialDriveRobot

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        self.get_logger().info('Robot controller node is running')

        self.declare_parameter('wheel_base', 0.168)
        self.declare_parameter('wheel_radius', 0.04)

        wheel_base = self.get_parameter('wheel_base').value
        wheel_radius = self.get_parameter('wheel_radius').value
        self.robot = DifferentialDriveRobot(wheel_base, wheel_radius)

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.get_velocities,
            qos_profile=qos_profile_system_default)

        self.pid_in_pub = self.create_publisher(
            String,
            '/pid_input',
            qos_profile=qos_profile_system_default)

        self.thread_run = True
        self.thread = threading.Thread(target=self.send_velocities, daemon=True)
        self.thread.start()

    def get_velocities(self, msg):
        self.robot.linear_velocity = msg.linear.x
        self.robot.angular_velocity = msg.angular.z

    def send_velocities(self):
        while self.thread_run:
            left_wheel_angular_velocity, right_wheel_angular_velocity = self.robot.compute_wheel_angular_velocities()
            msg = String()
            msg.data = "{} {}\r\n".format(round(left_wheel_angular_velocity,2), round(right_wheel_angular_velocity,2))
            self.pid_in_pub.publish(msg)
            time.sleep(0.025)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.thread_run = False

if __name__ == '__main__':
    main()
