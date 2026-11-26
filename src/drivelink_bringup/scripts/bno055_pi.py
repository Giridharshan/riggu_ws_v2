#!/usr/bin/env python3
import fcntl
import math
import os
import struct
import time

from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class Bno055PiNode(Node):

    def __init__(self):
        super().__init__('bno055_pi_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x28)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('topic_name', '/bno055/imu')

        self.bus_num = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = self.get_parameter('publish_rate_hz').value
        topic = self.get_parameter('topic_name').value

        # I2C setup
        dev_path = f'/dev/i2c-{self.bus_num}'
        try:
            self.fd = os.open(dev_path, os.O_RDWR)
            fcntl.ioctl(self.fd, 0x0703, self.address)
        except Exception as e:
            self.get_logger().error(f'Failed to open {dev_path} at 0x{self.address:02X}: {e}')
            raise e

        # Initialize BNO055 into NDOF mode
        self.init_sensor()

        # Publisher
        self.imu_pub = self.create_publisher(Imu, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self.get_logger().info(f'BNO055 Pi Node started. Publishing to {topic} at {rate_hz} Hz')

    def write_reg(self, reg, val):
        os.write(self.fd, bytes([reg, val]))

    def read_reg(self, reg):
        os.write(self.fd, bytes([reg]))
        return os.read(self.fd, 1)[0]

    def init_sensor(self):
        self.get_logger().info('Initializing BNO055 into NDOF mode...')
        # 1. Enter CONFIG mode
        self.write_reg(0x3D, 0x00)
        time.sleep(0.05)
        # 2. Normal power mode & Page 0
        self.write_reg(0x3E, 0x00)
        time.sleep(0.02)
        self.write_reg(0x07, 0x00)
        time.sleep(0.02)
        # 3. Switch to NDOF mode (0x0C)
        self.write_reg(0x3D, 0x0C)
        time.sleep(0.1)

        mode = self.read_reg(0x3D) & 0x0F
        if mode == 0x0C:
            self.get_logger().info('BNO055 confirmed in NDOF mode (0x0C)')
        else:
            self.get_logger().warn(f'BNO055 mode is 0x{mode:02X} (expected 0x0C)')

    def timer_callback(self):
        try:
            # 1. Read Quaternion (0x20, 8 bytes: w, x, y, z)
            os.write(self.fd, bytes([0x20]))
            raw_q = os.read(self.fd, 8)
            w, x, y, z = [v / 16384.0 for v in struct.unpack('<hhhh', raw_q)]

            # Normalize quaternion
            norm = math.sqrt(w * w + x * x + y * y + z * z)
            if norm > 0.001:
                w /= norm
                x /= norm
                y /= norm
                z /= norm

            # 2. Read Linear Acceleration (0x28, 6 bytes: x, y, z; 1 m/s^2 = 100 LSB)
            os.write(self.fd, bytes([0x28]))
            raw_a = os.read(self.fd, 6)
            ax, ay, az = [v / 100.0 for v in struct.unpack('<hhh', raw_a)]

            # 3. Read Angular Velocity (0x14, 6 bytes: x, y, z; 1 dps = 16 LSB)
            os.write(self.fd, bytes([0x14]))
            raw_g = os.read(self.fd, 6)
            scale_gyro = (1.0 / 16.0) * (math.pi / 180.0)
            gx, gy, gz = [v * scale_gyro for v in struct.unpack('<hhh', raw_g)]

            # Build ROS2 Imu message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            msg.orientation.x = x
            msg.orientation.y = y
            msg.orientation.z = z
            msg.orientation.w = w
            msg.orientation_covariance = [
                0.005, 0.0, 0.0,
                0.0, 0.005, 0.0,
                0.0, 0.0, 0.005
            ]

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.linear_acceleration_covariance = [
                0.05, 0.0, 0.0,
                0.0, 0.05, 0.0,
                0.0, 0.0, 0.05
            ]

            self.imu_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'I2C read error: {e}', throttle_duration_sec=2.0)

    def destroy_node(self):
        try:
            os.close(self.fd)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Bno055PiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
