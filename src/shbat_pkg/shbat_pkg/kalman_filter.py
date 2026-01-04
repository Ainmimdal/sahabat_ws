import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class OdomFusion(Node):
    def __init__(self):
        super().__init__('kalman_filter')

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Dead-reckoning theta
        self.yaw = 0.0    # IMU yaw
        self.last_time = self.get_clock().now()

        # Latest RPMs
        self.left_rpm = 0
        self.right_rpm = 0

        # IMU yaw tracking
        self.imu_yaw = 0.0
        self.imu_received = False

        # Robot parameters - MUST match base_controller.py
        self.wheel_radius = 0.1524  # meters (6 inch wheels)
        self.wheel_base = 0.33      # meters (distance between wheels)
        self.rpm_to_rps = 2 * math.pi / 60  # RPM to rad/s

        # ROS interfaces
        # Publishing to /wheel_odom as backup (ZED publishes primary /odom)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.rpm_left_sub = self.create_subscription(
            Int32,
            'rpm_left',
            self.rpm_left_callback,
            10
        )
        self.rpm_right_sub = self.create_subscription(
            Int32,
            'rpm_right',
            self.rpm_right_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Timer to compute and publish odometry at 50Hz
        self.create_timer(0.02, self.publish_odom)

    def rpm_left_callback(self, msg):
        # RPM comes from CAN feedback - use directly without scaling
        self.left_rpm = msg.data

    def rpm_right_callback(self, msg):
        # RPM comes from CAN feedback - use directly without scaling
        self.right_rpm = msg.data

    def imu_callback(self, msg):
        # Convert quaternion to yaw
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_received = True

    def publish_odom(self):
        if not self.imu_received:
            return  # Wait for first IMU data

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        left_rps = self.left_rpm * self.rpm_to_rps
        right_rps = self.right_rpm * self.rpm_to_rps

        v_left = left_rps * self.wheel_radius
        v_right = right_rps * self.wheel_radius

        linear_vel = (v_right + v_left) / 2.0
        angular_vel = (v_right - v_left) / self.wheel_base

        # Use IMU yaw for orientation
        self.yaw = self.imu_yaw

        # Dead-reckoning update using linear vel and IMU yaw
        delta_x = linear_vel * math.cos(self.yaw) * dt
        delta_y = linear_vel * math.sin(self.yaw) * dt

        self.x += delta_x
        self.y += delta_y

        # NOTE: TF publishing disabled - robot_localization EKF will publish odom->base_link
        # The EKF fuses /wheel_odom (this node) with /zed2i/zed_node/odom (ZED visual odometry)
        # 
        # # Publish TF (odom->base_link from wheel odometry, as backup)
        # # Note: ZED also publishes this TF - in production, only one should be active
        # t = TransformStamped()
        # t.header.stamp = current_time.to_msg()
        # t.header.frame_id = 'wheel_odom'  # Different frame to avoid conflict with ZED
        # t.child_frame_id = 'base_link'
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # t.transform.rotation.z = math.sin(self.yaw / 2.0)
        # t.transform.rotation.w = math.cos(self.yaw / 2.0)
        # self.tf_broadcaster.sendTransform(t)

        # Publish Odometry (message only, no TF - EKF will consume this)
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'  # Changed to 'odom' for EKF compatibility
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # Add covariance for pose (6x6 matrix: x, y, z, roll, pitch, yaw)
        # Values tuned for wheel odometry + IMU fusion
        odom.pose.covariance[0] = 0.01   # x variance (m^2)
        odom.pose.covariance[7] = 0.01   # y variance (m^2)
        odom.pose.covariance[14] = 1e6   # z variance (not used, set high)
        odom.pose.covariance[21] = 1e6   # roll variance (not used)
        odom.pose.covariance[28] = 1e6   # pitch variance (not used)
        odom.pose.covariance[35] = 0.05  # yaw variance (rad^2) - IMU based, fairly accurate

        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel

        # Add covariance for twist (6x6 matrix: vx, vy, vz, vroll, vpitch, vyaw)
        odom.twist.covariance[0] = 0.02   # linear x velocity variance
        odom.twist.covariance[7] = 1e6    # linear y variance (not used)
        odom.twist.covariance[14] = 1e6   # linear z variance (not used)
        odom.twist.covariance[21] = 1e6   # angular x variance (not used)
        odom.twist.covariance[28] = 1e6   # angular y variance (not used)
        odom.twist.covariance[35] = 0.1   # angular z velocity variance

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
