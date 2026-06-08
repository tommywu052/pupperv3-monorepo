"""Lightweight Madgwick-style IMU orientation filter for EKF input."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ImuMadgwickNode(Node):
    """Fuse gyro integration with measured yaw (2D) for smoother heading."""

    def __init__(self) -> None:
        super().__init__('imu_madgwick_node')
        self.declare_parameter('input_topic', '/imu_sensor_broadcaster/imu')
        self.declare_parameter('output_topic', '/imu/data_filtered')
        self.declare_parameter('gain', 0.033)
        self.declare_parameter('max_rate', 100.0)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.gain = self.get_parameter('gain').get_parameter_value().double_value
        self.max_rate = self.get_parameter('max_rate').get_parameter_value().double_value

        self.yaw = 0.0
        self.initialized = False
        self.last_process_time = None
        self._latest_msg: Imu | None = None

        self.pub = self.create_publisher(Imu, output_topic, 10)
        self.create_subscription(Imu, input_topic, self._store_imu, 1)
        rate = self.max_rate if self.max_rate > 0.0 else 100.0
        self.create_timer(1.0 / rate, self._process)
        self.get_logger().info(
            f'Filtering {input_topic} -> {output_topic} '
            f'(gain={self.gain}, max_rate={rate} Hz)'
        )

    def _store_imu(self, msg: Imu) -> None:
        self._latest_msg = msg

    def _process(self) -> None:
        if self._latest_msg is None:
            return
        msg = self._latest_msg
        now = self.get_clock().now()
        if self.last_process_time is not None:
            dt = (now - self.last_process_time).nanoseconds * 1e-9
        else:
            dt = 0.0
        self.last_process_time = now

        meas_yaw = yaw_from_quaternion(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )
        if not self.initialized:
            self.yaw = meas_yaw
            self.initialized = True

        if dt > 0.0:
            self.yaw += msg.angular_velocity.z * dt
            err = normalize_angle(meas_yaw - self.yaw)
            self.yaw += self.gain * err

        out = Imu()
        out.header.stamp = now.to_msg()
        out.header.frame_id = msg.header.frame_id
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuMadgwickNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
