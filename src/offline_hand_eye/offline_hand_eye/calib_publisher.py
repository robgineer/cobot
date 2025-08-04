#!/usr/bin/env python

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import ParameterType, ParameterDescriptor
import tf2_ros
from geometry_msgs.msg import Transform, Vector3, Quaternion, TransformStamped
from .calib_io_utils import load_calibration

class CalibrationPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('calib_publisher')

        self.declare_parameter('calibration_file', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        filename = self.get_parameter('calibration_file').get_parameter_value().string_value

        self.get_logger().info(f'Loading the calibration from file {filename}')

        self.calibration = load_calibration(filename)

        if self.calibration['calibration_type'] == 'eye_in_hand':
            orig = self.calibration['robot_effector_frame']
        else:
            orig = self.calibration['robot_base_frame']
        dest = self.calibration['tracking_base_frame']

        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.static_transformStamped = TransformStamped()

        self.static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        self.static_transformStamped.header.frame_id = orig
        self.static_transformStamped.child_frame_id = dest

        (hcqw, hcqx, hcqy, hcqz) = self.calibration['rotation_q_wxyz']
        (hctx, hcty, hctz) = self.calibration['translation']

        self.static_transformStamped.transform = Transform(translation=Vector3(x=hctx, y=hcty, z=hctz),
                           rotation=Quaternion(x=hcqx, y=hcqy, z=hcqz, w=hcqw))        

        self.broadcaster.sendTransform(self.static_transformStamped)


def main(args=None):
    rclpy.init(args=args)

    calibration_publisher = CalibrationPublisher()

    try:
        rclpy.spin(calibration_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        calibration_publisher.destroy_node()


if __name__ == '__main__':
    main()