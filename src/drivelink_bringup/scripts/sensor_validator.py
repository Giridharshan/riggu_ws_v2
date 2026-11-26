#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Thresholds
        self.max_linear_speed = 1.5   # m/s
        self.max_angular_speed = 4.0  # rad/s

        # Subscribers
        self.create_subscription(Imu, '/bno055/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/velocity_controller/odom', self.odom_callback, 10)

        # Publishers
        self.clean_imu_pub = self.create_publisher(Imu, '/imu/clean', 10)
        self.clean_odom_pub = self.create_publisher(Odometry, '/odom/clean', 10)

        self.get_logger().info('Sensor Validator & Protection Node Started')

    def imu_callback(self, msg: Imu):
        # 1. Check for NaN or Inf
        q = msg.orientation
        if any(math.isnan(v) or math.isinf(v) for v in [q.x, q.y, q.z, q.w]):
            return

        # 2. Check Quaternion Normalization (Detects MOCK data!)
        q_norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if abs(q_norm - 1.0) > 0.08:
            self.get_logger().warn(f'Rejected invalid/mock IMU quaternion (norm={q_norm:.2f})', throttle_duration_sec=2.0)
            return

        # 3. Check Angular Velocity limits
        w = msg.angular_velocity
        if abs(w.x) > self.max_angular_speed or abs(w.y) > self.max_angular_speed or abs(w.z) > self.max_angular_speed:
            return

        # 4. Inject realistic Covariances if they are all 0.0
        if msg.orientation_covariance[0] == 0.0:
            msg.orientation_covariance = [
                0.005, 0.0,   0.0,
                0.0,   0.005, 0.0,
                0.0,   0.0,   0.005
            ]
        if msg.angular_velocity_covariance[0] == 0.0:
            msg.angular_velocity_covariance = [
                0.01, 0.0,  0.0,
                0.0,  0.01, 0.0,
                0.0,  0.0,  0.01
            ]

        self.clean_imu_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        # 1. Check Linear & Angular Velocity limits
        v_linear = msg.twist.twist.linear.x
        v_angular = msg.twist.twist.angular.z

        if abs(v_linear) > self.max_linear_speed or abs(v_angular) > self.max_angular_speed:
            self.get_logger().warn(f'Rejected wheel speed spike (v={v_linear:.2f} m/s)', throttle_duration_sec=2.0)
            return

        # 2. Inject realistic Covariances if 0.0
        if msg.pose.covariance[0] == 0.0:
            msg.pose.covariance = [
                0.01, 0.0,  0.0,  0.0, 0.0, 0.0,
                0.0,  0.01, 0.0,  0.0, 0.0, 0.0,
                0.0,  0.0,  1e6,  0.0, 0.0, 0.0,
                0.0,  0.0,  0.0,  1e6, 0.0, 0.0,
                0.0,  0.0,  0.0,  0.0, 1e6, 0.0,
                0.0,  0.0,  0.0,  0.0, 0.0, 0.05
            ]
        if msg.twist.covariance[0] == 0.0:
            msg.twist.covariance = [
                0.02, 0.0,  0.0,  0.0, 0.0, 0.0,
                0.0,  0.02, 0.0,  0.0, 0.0, 0.0,
                0.0,  0.0,  1e6,  0.0, 0.0, 0.0,
                0.0,  0.0,  0.0,  1e6, 0.0, 0.0,
                0.0,  0.0,  0.0,  0.0, 1e6, 0.0,
                0.0,  0.0,  0.0,  0.0, 0.0, 0.05
            ]

        self.clean_odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
