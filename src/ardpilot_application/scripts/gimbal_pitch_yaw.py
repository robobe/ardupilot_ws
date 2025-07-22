#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import GimbalDeviceAttitudeStatus, GimbalManagerSetPitchyaw
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scipy.spatial.transform import Rotation as R
from math import radians, cos, nan
# from pymavlink.dialects.v20 import ardupilotmega

TOPIC_PITCH_YAW_COMMAND = "/mavros/gimbal_control/manager/set_pitchyaw"

class MyNode(Node):
    def __init__(self):
        node_name="gimbal_pitch_yaw"
        super().__init__(node_name)
        self.gimbal_pub = self.create_publisher(
            GimbalManagerSetPitchyaw,
            TOPIC_PITCH_YAW_COMMAND,
            qos_profile=qos_profile_system_default,
        )
        self.t = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Hello ROS2")

    def timer_callback(self):
        
        GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME = 32
        flags = GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME | 1
        tilt = radians(0.0)
        pan = radians(30.0)
        tilt_rate = radians(0)
        pan_rate = radians(0)
        # pan_rate *= abs(cos(radians(self.gimbal_status.tilt_deg)))
        msg = GimbalManagerSetPitchyaw()
        msg.target_system = 0
        msg.target_component = 0
        msg.flags = flags
        msg.gimbal_device_id = 1
        msg.pitch = -tilt
        msg.yaw = -pan
        msg.pitch_rate = nan
        msg.yaw_rate = nan
        self.gimbal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()