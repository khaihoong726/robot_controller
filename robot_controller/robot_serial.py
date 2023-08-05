import serial
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String

class RobotSerial(Node):
    def __init__(self):
        super().__init__('robot_serial_node')
        self.get_logger().info('Robot serial node is running')

        self.connected = False

        self.declare_parameter('stm32_port', '/dev/ttyACM0')
        self.declare_parameter('stm32_baud_rate', 115200)

        stm32_port = self.get_parameter('stm32_port').value
        stm32_baud_rate = self.get_parameter('stm32_baud_rate').value
        try:
            self.stm32 = serial.Serial(stm32_port, stm32_baud_rate)
            self.connected = True
            self.get_logger().info('Connected to STM32')
        except:
            self.get_logger().error('Could not connect to STM32')
            self.connected = False

        self.create_subscription(
            String,
            '/serial_send',
            self.send_serial,
            qos_profile=qos_profile_system_default)

        self.serial_pub = self.create_publisher(
            String,
            '/serial_receive',
            qos_profile=qos_profile_system_default)

        self.thread_run = True
        self.thread = threading.Thread(target=self.receive_serial, daemon=True)
        self.thread.start()

    def send_serial(self, msg):
    	if self.connected:
       	    data = msg.data
            self.stm32.write(data.encode('utf-8'))

    def receive_serial(self):
        if self.connected:
            while self.thread_run:
                data = self.stm32.readline()
                data = data.decode('utf-8').replace('\x00','').replace('\r\n','')
                msg = String()
                msg.data = data
                self.serial_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    robot_serial = RobotSerial()

    try:
        rclpy.spin(robot_serial)
    except KeyboardInterrupt:
        robot_serial.thread_run = False

if __name__ == '__main__':
    main()

