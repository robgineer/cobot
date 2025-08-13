#!/usr/bin/env python

"""
Copyright (c) 2025, Thao Dang, Esslingen University.
This file is part of the offline_hand_eye package (see https://github.com/robgineer/cobot).
License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import ParameterType, ParameterDescriptor
from rclpy.time import Duration, Time
import tf2_ros
from geometry_msgs.msg import Transform, Vector3, Quaternion, TransformStamped
from .calib_io_utils import load_calibration
from time import sleep

class CalibrationPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('calib_publisher')

        self.declare_parameter('calibration_file', value='handeye_calibration.json', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        filename = self.get_parameter('calibration_file').get_parameter_value().string_value

        self.get_logger().info(f'Loading the calibration from file {filename}')

        self.calibration = load_calibration(filename)

        if self.calibration['calibration_type'] == 'eye_in_hand':
            orig = self.calibration['robot_effector_frame']
        else:
            orig = self.calibration['robot_base_frame']
        dest = self.calibration['tracking_base_frame']

        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # get transform from tracking_base_frame (i.e. the camera image) to camera_bottom_screw_frame
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer, self)
        if not rclpy.ok():
            assert False, "rclpy is not ok, cannot proceed with calibration publishing"

        wait_time_sec = int(5)
        self.get_logger().info(f'Waiting {wait_time_sec}secs to obtain tf2 transforms...')
        for _ in range(wait_time_sec*10):
            rclpy.spin_once(self)
            sleep(0.1)
        marker_image_to_cam_screw = tfBuffer.lookup_transform(dest, 'camera_bottom_screw_frame', Time(), Duration(seconds=1))
        self.get_logger().info(f'Obtained transform from {dest} to camera_bottom_screw_frame:')
        self.get_logger().info(f'{marker_image_to_cam_screw.transform}')

        # broadcast camera calibration transform
        self.static_transformStamped = TransformStamped()
        self.static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        self.static_transformStamped.header.frame_id = orig
        self.static_transformStamped.child_frame_id = dest

        (hcqw, hcqx, hcqy, hcqz) = self.calibration['rotation_q_wxyz']
        if type(hcqw) is list:
            assert len(hcqw) == 1, "Expected a single value for rotation quaternion w"
            hcqw = hcqw[0]
            hcqx = hcqx[0]
            hcqy = hcqy[0]
            hcqz = hcqz[0]
            
        (hctx, hcty, hctz) = self.calibration['translation']
        if type(hctx) is list:
            assert len(hctx) == 1, "Expected a single value for translation x"
            hctx = hctx[0]
            hcty = hcty[0]
            hctz = hctz[0]
        
        self.static_transformStamped.transform = Transform(translation=Vector3(x=hctx, y=hcty, z=hctz),
                           rotation=Quaternion(x=hcqx, y=hcqy, z=hcqz, w=hcqw))        

        self.broadcaster.sendTransform(self.static_transformStamped)   
                
        # broadcast camera_color_optical_frame to camera_bottom_screw_frame so that all transforms are updated
        self.static_transformStamped = TransformStamped()
        self.static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        self.static_transformStamped.header.frame_id = dest
        self.static_transformStamped.child_frame_id = "/camera_bottom_screw_frame"
        self.static_transformStamped.transform = marker_image_to_cam_screw.transform        
        self.broadcaster.sendTransform(self.static_transformStamped)          
     
        self.get_logger().info(f'Done - published static transform from {orig} to {dest}.')
        self.get_logger().info(f'To obtain the transformation to be used in cobot_model/urdf/festo_cobot_model.urdf.xacro, run:\n' + 
                               f'       ros2 run tf2_ros tf2_echo {orig} camera_bottom_screw_frame')


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