#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import GimbalManagerSetAttitude
from geometry_msgs.msg import Quaternion
import tf_transformations  # pip install tf-transformations

class GimbalControlNode(Node):
    def __init__(self):
        super().__init__('gimbal_control_node')
        self.pub = self.create_publisher(GimbalManagerSetAttitude,
                                         '/mavros/gimbal_control/manager/set_attitude',
                                         10)
        self.timer = self.create_timer(1.0, self.send_command)
        self.pitch = 0

    def send_command(self):
        msg = GimbalManagerSetAttitude()
        msg.flags = 16  # Lock all axes (e.g., YAW_LOCK | PITCH_LOCK | ROLL_LOCK)
        msg.gimbal_device_id = 1

        # Example: look straight down (pitch -90 in FLU = +90 in quaternion)
        self.pitch = 1.0
        quat = tf_transformations.quaternion_from_euler(0.0, self.pitch, 0.0)  # Roll, Pitch, Yaw
        msg.q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        msg.angular_velocity_x = 0.0
        msg.angular_velocity_y = 0.0
        msg.angular_velocity_z = 0.0

        self.pub.publish(msg)
        self.get_logger().info('Published gimbal attitude command.')

rclpy.init()
node = GimbalControlNode()
rclpy.spin(node)
