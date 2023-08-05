import threading
import time
from math import sin, cos, pi

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from .helper.differential_drive import DifferentialDriveRobot
from .helper.encoder import Encoder

# For Testing Purposes
from sensor_msgs.msg import LaserScan
# -----------------------------

class RobotOdometry(Node):
    def __init__(self):
        super().__init__('robot_odometry_node')
        self.get_logger().info('Robot odometry node is running')

        self.declare_parameter('wheel_base', 0.168)
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('ticks_per_rev', 35)
        self.declare_parameter('delta_time', 0.1)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.delta_time = self.get_parameter('delta_time').value

        self.robot = DifferentialDriveRobot(self.wheel_base, self.wheel_radius)
        self.left_encoder = Encoder(self.ticks_per_rev)
        self.right_encoder = Encoder(self.ticks_per_rev)
        self.left_encoder_data = 0
        self.right_encoder_data = 0

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.get_velocities,
            qos_profile=qos_profile_system_default)

        self.create_subscription(
            String,
            '/serial_receive',
            self.get_encoder_data,
            qos_profile=qos_profile_system_default)

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            qos_profile=qos_profile_system_default)

        # For Testing Purposes
        self.scan_pub = self.create_publisher(
            LaserScan,
            "/scan",
            qos_profile=qos_profile_system_default
        )
        # -----------------------------------

        self.odom_broadcaster = TransformBroadcaster(self)

        self.thread_run = True
        self.thread = threading.Thread(target=self.calc_odometry, daemon=True)
        self.thread.start()

    def get_velocities(self, msg):
        self.robot.linear_velocity = msg.linear.x
        self.robot.angular_velocity = msg.angular.z

    def get_encoder_data(self, msg):
        data = msg.data.replace("\r\n","").split(',')
        self.left_encoder_data = int(data[0])
        self.right_encoder_data = int(data[1])

    def calc_odometry(self):
        while self.thread_run:
            left_desired_ang_vel, right_desired_ang_vel = self.robot.compute_wheel_angular_velocities()
            self.left_encoder.update(self.left_encoder_data)
            self.right_encoder.update(self.right_encoder_data)

            if left_desired_ang_vel < 0:
                self.left_encoder.direction = -1
            else:
                self.left_encoder.direction = 1

            if right_desired_ang_vel < 0:
                self.right_encoder.direction = -1
            else:
                self.right_encoder.direction = 1

            self.robot.compute_odometry(self.left_encoder, self.right_encoder, self.delta_time)

            curr_time = self.get_clock().now().to_msg()

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.robot.theta / 2)
            quaternion.w = cos(self.robot.theta /2)

            t = TransformStamped()
            t.header.stamp = curr_time
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.robot.x
            t.transform.translation.y = self.robot.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = quaternion.x
            t.transform.rotation.y = quaternion.y
            t.transform.rotation.z = quaternion.z
            t.transform.rotation.w = quaternion.w
            self.odom_broadcaster.sendTransform(t)

            odom = Odometry()
            odom.header.stamp = curr_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.robot.x
            odom.pose.pose.position.y = self.robot.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = self.robot.x_vel
            odom.twist.twist.linear.y = self.robot.y_vel
            odom.twist.twist.angular.z = self.robot.theta_vel
            self.odom_pub.publish(odom)

            # For Testing Purposes
            scan = LaserScan()
            scan.header.stamp = curr_time
            scan.header.frame_id = "lidar_link"
            scan.angle_min = -pi
            scan.angle_max = pi
            scan.angle_increment = pi/180
            scan.time_increment = 0.0
            scan.scan_time = 0.0
            scan.range_min = 0.1
            scan.range_max = 30.0
            scan.ranges = [5.0 for _ in range(360)]
            self.scan_pub.publish(scan)
            # ----------------------

            time.sleep(self.delta_time)

def main(args=None):
    rclpy.init(args=args)
    robot_odometry = RobotOdometry()

    try:
        rclpy.spin(robot_odometry)
    except KeyboardInterrupt:
        robot_odometry.thread_run = False

if __name__ == '__main__':
    main()



