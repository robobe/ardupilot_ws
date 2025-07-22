import rclpy
from rclpy.node import Node
# from rosidl_runtime_py.utilities import get_message_names, get_message_type
from importlib import import_module
from . import Event
# from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R
from mavros_msgs.msg import GimbalManagerSetAttitude, GimbalDeviceAttitudeStatus
from geometry_msgs.msg import Quaternion
from pymavlink.dialects.v20 import common as mavlink_common
import tf_transformations
from math import nan, radians, degrees
from PyQt5.QtCore import QThread

TOPIC_SET_ATTITUDE = 'mavros/gimbal_control/manager/set_attitude'
TOPIC_ATTITUDE_STATUS = "mavros/gimbal_control/device/attitude_status"

class RangeTranslate():
    def __init__(self, min_in, max_in, min_out, max_out):
        self.min_in = min_in
        self.max_in = max_in
        self.min_out = min_out
        self.max_out = max_out

    def translate(self, value):
        return self.min_out + (self.max_out - self.min_out) * ((value - self.min_in) / (self.max_in - self.min_in))

    def translate_reverse(self,mapped_value):
        return ((mapped_value - self.min_out) / (self.max_out - self.min_out)) * (self.max_in - self.min_in) + self.min_in

class ROS2SpinThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.quit()
        self.wait()

class BackendNode(Node):
    def __init__(self):
        super().__init__('my_plugin_backend')
        self.gimbal_command_sent = Event()

        self.gimbal_attitude_pub = self.create_publisher(
            GimbalManagerSetAttitude, 
            TOPIC_SET_ATTITUDE, 
            10
        )

        self.status_sub = self.create_subscription(
            GimbalDeviceAttitudeStatus,
            TOPIC_ATTITUDE_STATUS,
            self.gimbal_status_handler,
            10
        )

        self.on_gimbal_status = Event()

        self.spin_thread = ROS2SpinThread(self)
        self.spin_thread.start()

    def close(self):
        self.spin_thread.stop()


    def gimbal_status_handler(self, msg: GimbalDeviceAttitudeStatus):
        r, p, y = tf_transformations.euler_from_quaternion([
            msg.q.x,
            msg.q.y,
            msg.q.z,
            msg.q.w]
        )

        self.on_gimbal_status.fire(
            degrees(r),
            degrees(p),
            degrees(y)
        )
        

    def send_gimbal_command(self, roll, pitch, yaw, flags=0):
        msg = GimbalManagerSetAttitude()
        msg.flags = flags
        msg.gimbal_device_id = 1

        r_roll = radians(roll)
        r_pitch = radians(pitch)
        r_yaw = radians(yaw)
        quat = tf_transformations.quaternion_from_euler(r_roll, r_pitch, r_yaw)  # Roll, Pitch, Yaw

        msg.q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        msg.angular_velocity_x = nan
        msg.angular_velocity_y = nan
        msg.angular_velocity_z = nan

        self.gimbal_attitude_pub.publish(msg)
        self.get_logger().info(f"Sending gimbal command: p={pitch}, y={yaw}, r={roll}, flags={flags}")