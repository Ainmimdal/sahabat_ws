"""
Base Controller Node for ZLAC8015D Differential Drive Robot

This node provides:
- cmd_vel subscription for velocity commands
- Wheel odometry publication
- TF broadcast (odom -> base_link)
- Motor fault monitoring

Configured for 6.5 inch wheel motors with RS485 Modbus communication.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
import math
import numpy as np
from .zlac8015d.ZLAC8015D import Controller


class BaseController(Node):
    """
    ROS2 Node for controlling ZLAC8015D dual motor driver.
    
    Implements differential drive kinematics and wheel odometry.
    """
    
    def __init__(self):
        super().__init__('base_controller')
        
        # Declare parameters with defaults for 173mm wheel robot
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.0875)  # 175mm diameter / 2 = 87.5mm
        self.declare_parameter('wheel_base', 0.33)  # Distance between wheels in meters
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', 'wheel_odom')  # Changed to avoid conflict with EKF
        self.declare_parameter('accel_time_ms', 200)
        self.declare_parameter('decel_time_ms', 200)
        self.declare_parameter('max_linear_vel', 1.0)  # m/s
        self.declare_parameter('max_angular_vel', 2.0)  # rad/s
        self.declare_parameter('cmd_vel_timeout', 0.5)  # seconds
        self.declare_parameter('odom_rate', 20.0)  # Hz
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.accel_time = self.get_parameter('accel_time_ms').value
        self.decel_time = self.get_parameter('decel_time_ms').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        odom_rate = self.get_parameter('odom_rate').value
        
        # Initialize motor driver
        self.driver = None
        self._init_driver()
        
        # QoS for cmd_vel (best effort for real-time performance)
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            cmd_vel_qos
        )
        
        # Emergency stop subscriber - directly stops motors
        self.estop_sub = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.motor_enabled_pub = self.create_publisher(Bool, 'motor_enabled', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Emergency stop state
        self.emergency_stopped = False
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Command velocity (protected by timeout)
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        
        # Covariance matrices for odometry
        # Diagonal: [x, y, z, roll, pitch, yaw]
        self.pose_covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,  # z is unknown
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,  # roll is unknown
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,  # pitch is unknown
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03  # yaw
        ]
        
        # Twist covariance: [vx, vy, vz, wx, wy, wz]
        self.twist_covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e6, 0.0, 0.0, 0.0, 0.0,  # vy is unknown (diff drive)
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,  # vz is unknown
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,  # wx is unknown
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,  # wy is unknown
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01  # wz
        ]
        
        # Timers
        odom_period = 1.0 / odom_rate
        self.odom_timer = self.create_timer(odom_period, self.update_odom)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        self.fault_timer = self.create_timer(1.0, self.check_faults)
        
        self.get_logger().info(
            f'BaseController initialized:\n'
            f'  Port: {self.port}\n'
            f'  Wheel radius: {self.wheel_radius:.4f} m\n'
            f'  Wheel base: {self.wheel_base:.3f} m\n'
            f'  Odom topic: {self.odom_topic}\n'
            f'  Odom rate: {odom_rate} Hz'
        )

    def _init_driver(self):
        """Initialize the ZLAC8015D motor driver."""
        try:
            self.driver = Controller(port=self.port, baudrate=self.baudrate)
            
            # Update driver with wheel parameters
            self.driver.R_Wheel = self.wheel_radius
            self.driver.travel_in_one_rev = 2 * np.pi * self.wheel_radius
            
            # Configure driver
            self.driver.disable_motor()
            self.driver.clear_alarm()
            self.driver.set_mode(3)  # Velocity control mode
            self.driver.set_accel_time(self.accel_time, self.accel_time)
            self.driver.set_decel_time(self.decel_time, self.decel_time)
            self.driver.enable_motor()
            self.driver.reset_encoder()  # Reset odometry tracking
            
            self.get_logger().info(f'ZLAC8015D driver initialized on {self.port}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ZLAC8015D driver: {e}')
            self.driver = None

    def emergency_stop_callback(self, msg: Bool):
        """
        Handle emergency stop requests.
        
        When activated, immediately stops motors and ignores cmd_vel until cleared.
        """
        if msg.data and not self.emergency_stopped:
            self.emergency_stopped = True
            self.get_logger().warn('EMERGENCY STOP ACTIVATED - motors stopped!')
            self.stop_motors()
        elif not msg.data and self.emergency_stopped:
            self.emergency_stopped = False
            self.get_logger().info('Emergency stop cleared - motors enabled')
    
    def stop_motors(self):
        """Immediately stop all motors."""
        if self.driver:
            try:
                self.driver.set_rpm(0, 0)
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0
            except Exception as e:
                self.get_logger().error(f'Failed to stop motors: {e}')

    def cmd_vel_callback(self, msg: Twist):
        """
        Handle incoming velocity commands.
        
        Converts twist message to differential drive wheel velocities.
        """
        if not self.driver:
            return
        
        # If emergency stopped, ignore velocity commands
        if self.emergency_stopped:
            return
        
        # Update last command time
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Clamp velocities to limits
        v = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        w = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))
        
        self.target_linear_vel = v
        self.target_angular_vel = w
        
        # Differential drive kinematics: convert (v, w) to wheel velocities
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)
        
        try:
            # Use set_velocity method which handles motor direction internally
            self.driver.set_velocity(v_left, v_right)
        except Exception as e:
            self.get_logger().warn(f'Failed to set velocity: {e}')

    def watchdog_callback(self):
        """
        Safety watchdog - stops motors if no cmd_vel received.
        """
        if not self.driver:
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9
        
        if dt > self.cmd_vel_timeout:
            if self.target_linear_vel != 0.0 or self.target_angular_vel != 0.0:
                self.get_logger().debug('cmd_vel timeout - stopping motors')
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0
                try:
                    self.driver.set_rpm(0, 0)
                except:
                    pass

    def check_faults(self):
        """
        Periodically check for motor faults.
        """
        if not self.driver:
            return
        
        try:
            (l_fault, l_code), (r_fault, r_code) = self.driver.get_fault_code()
            
            if l_fault:
                self.get_logger().warn(
                    f'Left motor fault: {self.driver.get_fault_string(l_code)}'
                )
            if r_fault:
                self.get_logger().warn(
                    f'Right motor fault: {self.driver.get_fault_string(r_code)}'
                )
                
            # Publish motor enabled status
            enabled_msg = Bool()
            enabled_msg.data = not (l_fault or r_fault)
            self.motor_enabled_pub.publish(enabled_msg)
            
        except Exception as e:
            self.get_logger().debug(f'Fault check failed: {e}')

    def update_odom(self):
        """
        Update and publish odometry from wheel encoders.
        
        Uses velocity feedback from motors to compute position.
        """
        if not self.driver:
            return

        try:
            # Get wheel velocities from motor driver
            v_left, v_right = self.driver.get_linear_velocities()
        except Exception as e:
            self.get_logger().debug(f'Failed to read velocities: {e}')
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Prevent division issues with very small dt
        if dt < 0.001:
            return

        # Differential drive kinematics: compute robot velocities
        self.vx = (v_right + v_left) / 2.0
        self.vth = (v_right - v_left) / self.wheel_base

        # Integrate position using midpoint method for better accuracy
        delta_th = self.vth * dt
        delta_x = self.vx * math.cos(self.th + delta_th / 2.0) * dt
        delta_y = self.vx * math.sin(self.th + delta_th / 2.0) * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # Normalize theta to [-pi, pi]
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # Convert orientation to quaternion
        q = self.quaternion_from_euler(0.0, 0.0, self.th)

        # Publish TF transform
        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.pose.covariance = self.pose_covariance
        
        # Twist (in child_frame_id = base_link)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0  # Differential drive cannot move sideways
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vth
        odom.twist.covariance = self.twist_covariance
        
        self.odom_pub.publish(odom)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion [x, y, z, w]."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        
        return [qx, qy, qz, qw]

    def reset_odometry(self):
        """Reset odometry to origin."""
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth = 0.0
        if self.driver:
            self.driver.reset_encoder()
        self.get_logger().info('Odometry reset to origin')

    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down BaseController...')
        if self.driver:
            try:
                self.driver.set_rpm(0, 0)
                self.driver.disable_motor()
                self.driver.close()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = BaseController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()