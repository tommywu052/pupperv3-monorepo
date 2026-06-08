"""Integrate /cmd_vel + IMU into /odom and odom->base_link TF."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class DeadReckoningNode(Node):
    def __init__(self) -> None:
        super().__init__('dead_reckoning_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('imu_topic', '/imu_sensor_broadcaster/imu')
        self.declare_parameter('odom_raw_topic', '/odom/raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('use_imu_yaw', False)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        odom_raw_topic = self.get_parameter('odom_raw_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.publish_odom = self.get_parameter('publish_odom').get_parameter_value().bool_value
        self.use_imu_yaw = self.get_parameter('use_imu_yaw').get_parameter_value().bool_value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.active_twist = Twist()
        self.last_cmd_vel_time = None
        self.last_update_time = self.get_clock().now()
        self.imu_orientation = None
        self.imu_yaw = 0.0
        self.have_imu = False
        self.theta_seeded = False

        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 10)
        if self.use_imu_yaw:
            self.create_subscription(Imu, imu_topic, self._imu_cb, 10)
        self.odom_raw_pub = self.create_publisher(Odometry, odom_raw_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10) if self.publish_odom else None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / publish_rate, self._publish_odom)

        imu_src = imu_topic if self.use_imu_yaw else '(no IMU — EKF mode)'
        self.get_logger().info(
            f'Publishing {odom_raw_topic}'
            + (f' + {odom_topic}' if self.publish_odom else '')
            + f' from {cmd_vel_topic}, imu={imu_src}'
        )

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self.active_twist = msg
        self.last_cmd_vel_time = self.get_clock().now()

    def _imu_cb(self, msg: Imu) -> None:
        self.imu_orientation = msg.orientation
        q = msg.orientation
        self.imu_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if not self.have_imu:
            self.have_imu = True
            if not self.use_imu_yaw and not self.theta_seeded:
                self.theta = self.imu_yaw
                self.theta_seeded = True

    def _current_twist(self) -> Twist:
        if self.last_cmd_vel_time is None:
            return Twist()

        age = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds * 1e-9
        if age > self.cmd_vel_timeout:
            return Twist()
        return self.active_twist

    def _heading(self) -> float:
        if self.use_imu_yaw and self.have_imu:
            return self.imu_yaw
        return self.theta

    def _integrate(self, twist: Twist, dt: float) -> None:
        if dt <= 0.0:
            return

        vth = twist.angular.z
        if not (self.use_imu_yaw and self.have_imu):
            self.theta = math.atan2(
                math.sin(self.theta + vth * dt),
                math.cos(self.theta + vth * dt),
            )

        heading = self._heading()
        vx = twist.linear.x
        vy = twist.linear.y
        vth = twist.angular.z

        self.x += (vx * math.cos(heading) - vy * math.sin(heading)) * dt
        self.y += (vx * math.sin(heading) + vy * math.cos(heading)) * dt

    def _set_orientation(self, odom: Odometry) -> None:
        """Use yaw-only (2D odom). Full IMU quaternion tilts the robot mesh in RViz/Foxglove."""
        theta = self._heading()
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(theta * 0.5)
        odom.pose.pose.orientation.w = math.cos(theta * 0.5)

    def _fill_odom(self, odom: Odometry, twist: Twist, now) -> None:
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.twist.twist = twist
        self._set_orientation(odom)

        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.05 if (self.use_imu_yaw and self.have_imu) else 0.4
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.05

    def _publish_odom(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        twist = self._current_twist()
        self._integrate(twist, dt)

        raw_odom = Odometry()
        self._fill_odom(raw_odom, twist, now)
        self.odom_raw_pub.publish(raw_odom)

        fused_odom = Odometry()
        self._fill_odom(fused_odom, twist, now)
        if self.odom_pub is not None:
            self.odom_pub.publish(fused_odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.rotation = fused_odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeadReckoningNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
