import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String
from .helper.encoder import Encoder

class RobotPID(Node):
    def __init__(self):
        super().__init__('robot_pid_node')
        self.get_logger().info('Robot PID controller node is running')

        self.declare_parameter('delta_time', 0.1)
        self.declare_parameter('Kp', 2.5)
        self.declare_parameter('Kd', 1.25)
        self.declare_parameter('Ki', 0.001)
        self.declare_parameter('enc_ticks_per_rev', 35)
        self.declare_parameter('left_base_speed', 0)
        self.declare_parameter('right_base_speed', 0)

        self.create_subscription(
            String,
            '/pid_input',
            self.get_setpoints,
            qos_profile=qos_profile_system_default)

        self.create_subscription(
            String,
            '/serial_receive',
            self.get_encoder_data,
            qos_profile=qos_profile_system_default)

        self.motor_pub = self.create_publisher(
            String,
            '/serial_send',
            qos_profile=qos_profile_system_default)

        self.desired_left_ang_vel = 0
        self.desired_right_ang_vel = 0
        self.left_encoder_data = 0
        self.right_encoder_data = 0

        self.thread_run = True
        self.thread = threading.Thread(target=self.run_pid, daemon=True)
        self.thread.start()

    def get_setpoints(self, msg):
        data = msg.data.replace("\r\n","").split(' ')
        self.desired_left_ang_vel = float(data[0])
        self.desired_right_ang_vel = float(data[1])

    def get_encoder_data(self, msg):
        data = msg.data.split(',')
        self.left_encoder_data = int(data[0])
        self.right_encoder_data = int(data[1])

    def run_pid(self):
        delta_time = self.get_parameter('delta_time').value
        Kp = self.get_parameter('Kp').value
        Kd = self.get_parameter('Kd').value
        Ki = self.get_parameter('Ki').value
        enc_ticks_per_rev = self.get_parameter('enc_ticks_per_rev').value
        left_base_speed = self.get_parameter('left_base_speed').value
        right_base_speed = self.get_parameter('right_base_speed').value

        pwm_limit = 255
        prev_left_error = 0
        prev_right_error = 0
        left_direction = 0
        right_direction = 0
        I_left = 0
        I_right = 0

        left_encoder = Encoder(enc_ticks_per_rev)
        right_encoder = Encoder(enc_ticks_per_rev)

        while self.thread_run:
            left_encoder.update(self.left_encoder_data)
            right_encoder.update(self.right_encoder_data)

            if self.desired_left_ang_vel < 0:
                left_encoder.direction = -1
            else:
                left_encoder.direction = 1

            if self.desired_right_ang_vel < 0:
                right_encoder.direction = -1
            else:
                right_encoder.direction = 1

            measured_left_ang_vel = ((left_encoder.increment / enc_ticks_per_rev) * 2 * np.pi) / delta_time
            measured_right_ang_vel = ((right_encoder.increment / enc_ticks_per_rev) * 2 * np.pi) / delta_time

            left_error = abs(self.desired_left_ang_vel) - measured_left_ang_vel
            right_error = abs(self.desired_right_ang_vel) - measured_right_ang_vel

            P_left = Kp * left_error
            D_left = Kd * (left_error - prev_left_error) / delta_time
            I_left = I_left + Ki * left_error * delta_time

            P_right = Kp * right_error
            D_right = Kd * (right_error - prev_right_error) / delta_time
            I_right = I_right + Ki * right_error * delta_time

            left_base_speed += P_left + I_left + D_left
            right_base_speed += P_right + I_right + D_right

            if left_base_speed > pwm_limit:
                left_base_speed = pwm_limit
            if left_base_speed < -pwm_limit:
                left_base_speed = -pwm_limit

            if right_base_speed > pwm_limit:
                right_base_speed = pwm_limit
            if right_base_speed < -pwm_limit:
                right_base_speed = -pwm_limit

            if self.desired_left_ang_vel == 0:
            	left_base_speed = 0
            if self.desired_right_ang_vel == 0:
            	right_base_speed = 0

            prev_left_error = left_error
            prev_right_error = right_error

            msg = String()
            msg.data = "{} {}\r\n".format(int(left_encoder.direction*left_base_speed), int(right_encoder.direction*right_base_speed))
            self.motor_pub.publish(msg)

            time.sleep(delta_time)

def main(args=None):
    rclpy.init(args=args)
    robot_pid = RobotPID()

    try:
        rclpy.spin(robot_pid)
    except KeyboardInterrupt:
        robot_pid.thread_run = False

if __name__ == '__main__':
    main()
