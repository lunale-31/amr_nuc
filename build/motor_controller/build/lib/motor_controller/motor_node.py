#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import minimalmodbus
import math


class MinimalModbusNode(Node):
    def __init__(self):
        super().__init__('minimal_modbus_node')

        # Subscriber for velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )

        # Publishers for encoder ticks
        self.right_ticks_pub = self.create_publisher(Int32, 'right_ticks', 10)
        self.left_ticks_pub = self.create_publisher(Int32, 'left_ticks', 10)

        # Initialize motor controllers
        self.rmcs2303_right = minimalmodbus.Instrument('/dev/ttyUSB0', 1, minimalmodbus.MODE_ASCII)
        self.rmcs2303_left = minimalmodbus.Instrument('/dev/ttyUSB0', 2, minimalmodbus.MODE_ASCII)

        self.rmcs2303_right.serial.baudrate = 9600
        self.rmcs2303_left.serial.baudrate = 9600

        # Publish ticks at a regular interval
        self.timer = self.create_timer(0.1, self.publish_ticks_loop)  # 10 Hz

        self.get_logger().info("Minimal Modbus Node initialized")

    def listener_callback(self, msg):
        """Callback for processing velocity commands."""
        gear_ratio = 90
        wheel_radius = 0.05  # meters
        wheel_base = 0.54

        lin_velocity = msg.linear.x  # m/s
        ang_velocity = msg.angular.z

        # Calculate velocities for left and right wheels
        velocity_left = lin_velocity - (wheel_base / 2) * ang_velocity
        velocity_right = lin_velocity + (wheel_base / 2) * ang_velocity

        # Convert to RPM
        rpm_right = int(abs((velocity_right / (2 * math.pi * wheel_radius)) * 60 * gear_ratio))
        rpm_left = int(abs((velocity_left / (2 * math.pi * wheel_radius)) * 60 * gear_ratio))

        try:
            # Set encoder counts to 0
            self.rmcs2303_right.write_register(2, 2048, number_of_decimals=0, functioncode=6, signed=False)
            self.rmcs2303_left.write_register(2, 2048, number_of_decimals=0, functioncode=6, signed=False)

            # Set speed commands
            self.rmcs2303_right.write_register(14, rpm_right, number_of_decimals=0, functioncode=6, signed=False)
            self.rmcs2303_left.write_register(14, rpm_left, number_of_decimals=0, functioncode=6, signed=False)

            # Set direction based on velocity
            if velocity_left >= 0:
                self.rmcs2303_left.write_register(2, 257, number_of_decimals=0, functioncode=6, signed=False)  # CW
            else:
                self.rmcs2303_left.write_register(2, 265, number_of_decimals=0, functioncode=6, signed=False)  # CCW

            if velocity_right >= 0:
                self.rmcs2303_right.write_register(2, 265, number_of_decimals=0, functioncode=6, signed=False)  # CW
            else:
                self.rmcs2303_right.write_register(2, 257, number_of_decimals=0, functioncode=6, signed=False)  # CCW

            self.get_logger().info(f"Set RPM: Left={rpm_left}, Right={rpm_right}")

        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")

    """def publish_ticks_loop(self):
        Publish encoder ticks.
        try:
            # Read and publish right encoder ticks
            right_position_feedback1 = self.rmcs2303_right.read_register(20)
            right_position_feedback2 = self.rmcs2303_right.read_register(22)
            right_feedback = right_position_feedback2 * (2 ** 16) + right_position_feedback1
            self.right_ticks_pub.publish(Int32(data=int(right_feedback)))

            # Read and publish left encoder ticks
            left_position_feedback1 = self.rmcs2303_left.read_register(20)
            left_position_feedback2 = self.rmcs2303_left.read_register(22)
            left_feedback = left_position_feedback2 * (2 ** 16) + left_position_feedback1
            self.left_ticks_pub.publish(Int32(data=int(left_feedback)))

            self.get_logger().info(f"Ticks: Left={left_feedback}, Right={right_feedback}")

        except Exception as e:
            self.get_logger().error(f"Error in publish_ticks_loop: {e}")"""

    def publish_ticks_loop(self):
        try:
            # Right motor feedback
            right_position_feedback1 = self.rmcs2303_right.read_register(20)
            right_position_feedback2 = self.rmcs2303_right.read_register(22)
            right_feedback = right_position_feedback2 * (2 ** 16) + right_position_feedback1
            # right_feedback = max(min(right_feedback, 2147483647), -2147483648)
            self.right_ticks_pub.publish(Int32(data=int(right_feedback)))

            # Left motor feedback
            left_position_feedback1 = self.rmcs2303_left.read_register(20)
            left_position_feedback2 = self.rmcs2303_left.read_register(22)
            left_feedback = left_position_feedback2 * (2 ** 16) + left_position_feedback1
            # left_feedback = max(min(left_feedback, 2147483647), -2147483648)
            self.left_ticks_pub.publish(Int32(data=int(left_feedback)))

            self.get_logger().info(f"Ticks: Left={left_feedback}, Right={right_feedback}")

        except Exception as e:
            self.get_logger().error(f"Error in publish_ticks_loop: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = MinimalModbusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Minimal Modbus Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
