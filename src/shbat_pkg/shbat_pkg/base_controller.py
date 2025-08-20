import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import requests

CAN_EFF_FLAG = 0x80000000
RPM_FLAG = 0x300
LEFT_ID = 33
RIGHT_ID = 15

def status_flag(sa):
    return 0x900 | (sa & 0xFF) | CAN_EFF_FLAG

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.rpm_left_pub = self.create_publisher(Int32, 'rpm_left', 10)
        self.rpm_right_pub = self.create_publisher(Int32, 'rpm_right', 10)
        self.wheel_base = 0.33  # meters
        self.wheel_radius = 0.1524  # meters

        # Timer to poll CAN server for RPM feedback
        self.create_timer(0.01, self.poll_rpm_status)

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematics
        v_left = v - (w * self.wheel_base / 2)
        v_right = v + (w * self.wheel_base / 2)

        # Convert to RPM
        rpm_left = int((v_left / (2 * 3.1416 * self.wheel_radius)) * 60)
        rpm_right = int((v_right / (2 * 3.1416 * self.wheel_radius)) * 60)

        # Send left RPM
        can_id_left = LEFT_ID | RPM_FLAG | CAN_EFF_FLAG
        data_left = list(rpm_left.to_bytes(4, byteorder='big', signed=True))
        payload_left = {"can_id": can_id_left, "data": data_left}
        requests.post("http://localhost:8000", json=payload_left)

        # Send right RPM
        can_id_right = RIGHT_ID | RPM_FLAG | CAN_EFF_FLAG
        data_right = list(rpm_right.to_bytes(4, byteorder='big', signed=True))
        payload_right = {"can_id": can_id_right, "data": data_right}
        requests.post("http://localhost:8000", json=payload_right)

    def poll_rpm_status(self):
        try:
            received_left = None
            received_right = None
            while received_left is None or received_right is None:
                response = requests.get("http://localhost:8000")
                data = response.json()
                can_id = data.get("can_id")
                frame_data = data.get("data", [])
                # Check for left status
                if can_id == status_flag(LEFT_ID) and len(frame_data) >= 4:
                    rpm = int.from_bytes(frame_data[:4], byteorder='big', signed=True)
                    received_left = rpm
                    self.get_logger().info(f"Received left RPM: {rpm}")
                # Check for right status
                if can_id == status_flag(RIGHT_ID) and len(frame_data) >= 4:
                    rpm = int.from_bytes(frame_data[:4], byteorder='big', signed=True)
                    received_right = rpm
                    self.get_logger().info(f"Received right RPM: {rpm}")
            # Publish both as Int32
            msg_left = Int32()
            msg_left.data = received_left
            self.rpm_left_pub.publish(msg_left)
            msg_right = Int32()
            msg_right.data = received_right
            self.rpm_right_pub.publish(msg_right)
        except Exception as e:
            self.get_logger().warn(f"Failed to poll RPM status: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()