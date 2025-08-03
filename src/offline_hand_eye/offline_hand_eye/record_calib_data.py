import sys
from typing import List, Optional
import logging
from datetime import datetime
import json
import os
import threading
import pickle

import rclpy
from rclpy.node import Node
from rclpy.time import Duration, Time
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from rclpy_message_converter import message_converter

from .record_calib_data_gui import CalibrationGUI, NO_TRACKER_MARKER_STR
from .image_to_numpy import image_to_numpy


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('record_calib_data_node')
        
        self.image_subscription = self.create_subscription(
            Image,
            'invalid_image', 
            self.image_callback,
            1)
        
        self.config = None
        
        self.camera_info = None        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            'invalid_caminfo',
            self.camera_info_callback,
            1)
                
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.mutex = threading.Lock()
        self.frame_counter_ = 0
        self.frame_file_path_ = None
        self.recording_active_ = False
            
    def get_available_topics(self):
        """Get available image and camera info topics"""                
        topic_list = self.get_topic_names_and_types()
        image_topics_list = [t[0] for t in topic_list if 'sensor_msgs/msg/Image' in t[1]]
        camera_info_topics_list = [t[0] for t in topic_list if 'sensor_msgs/msg/CameraInfo' in t[1]]
        return image_topics_list, camera_info_topics_list
    
    def get_tf_frames(self) -> List[str]:
        """Get available TF frames from the TF tree"""
        try:
            # Get all frame IDs 
            tf_frame_ids = self.tf_buffer._getFrameStrings()
                
        except Exception as e:
            self.get_logger().warning(f"Could not get TF frames: {e}")
            # Fallback to common frame names
            tf_frame_ids = ["invalid"]
            
        return sorted(tf_frame_ids)

    def set_new_subscriptions(self, config: dict):
        """Set new subscriptions for the image and camera info topics"""
        with self.mutex:
            self.config = config
            
        if self.image_subscription:
            self.destroy_subscription(self.image_subscription)
            self.image_subscription = self.create_subscription(
                Image,
                config['camera_image_topic'],
                self.image_callback,
                1
            )

        if self.camera_info_subscription:
            self.destroy_subscription(self.camera_info_subscription)
            self.camera_info_subscription = self.create_subscription(
                CameraInfo,
                config['camera_info_topic'],
                self.camera_info_callback,
                1
            )            
        self.get_logger().info(f"Subscribed to new topics: {config['camera_image_topic']}, {config['camera_info_topic']}")

    def start_recording(self, file_path: str):
        """Start recording the calibration data"""
        self.get_logger().info("Starting calibration data recording")
        self._frame_counter = 0
        self._frame_file_path = file_path
        self.recording_active_ = True

    def stop_recording(self):
        """Stop recording the calibration data"""
        self.get_logger().info("Stopping calibration data recording")
        self.recording_active_ = False
        
    def is_recording_active(self) -> bool:
        """Check if recording is active"""
        return self.recording_active_

    def image_callback(self, msg):
        if not self.recording_active_:
            self.get_logger().debug('Recording is not active, ignoring image message')
            return
        
        if not self.config:
            self.get_logger().error('Configuration is not set, cannot process image')
            return
        
        if not self.camera_info and self.config['camera_info_topic']:
            self.get_logger().warning('Camera info is not available, cannot process image')
            return
        
        with self.mutex:
            # why is this needed?
            curr_time = Time().from_msg(msg.header.stamp) - Duration(seconds=2)

            # here we trick the library (it is actually made for eye_in_hand only). Trust me, I'm an engineer
            try:
                if self.config['calibration_type'] == 'eye_in_hand':
                    robot = self.tf_buffer.lookup_transform(self.config['robot_base_frame'],
                                                            self.config['robot_effector_frame'], curr_time,
                                                            Duration(seconds=1))
                else:
                    robot = self.tf_buffer.lookup_transform(self.config['robot_effector_frame'],
                                                            self.config['robot_base_frame'], curr_time,
                                                            Duration(seconds=1))                    
                
                if self.config['tracking_marker_frame'] == NO_TRACKER_MARKER_STR:
                    tracking = None
                else:
                    # If tracking marker is set, get the transform
                    tracking = self.tf_buffer.lookup_transform(self.config['tracking_base_frame'],
                                                               self.config['tracking_marker_frame'], curr_time,
                                                               Duration(seconds=1))

            except tf2_ros.ExtrapolationException as e:
                self.get_logger().error(f'Failed to get the tracking transform: {e}')
                return
            
        self.get_logger().info('Received new sample data')
        frame = {
            'image_timestamp': Time().from_msg(msg.header.stamp).seconds_nanoseconds(),
            'robot_transform': message_converter.convert_ros_message_to_dictionary(robot.transform),
            'tracking_transform': message_converter.convert_ros_message_to_dictionary(tracking.transform) if tracking else None,
            'camera_info': message_converter.convert_ros_message_to_dictionary(self.camera_info) if self.camera_info else None,
            'image': image_to_numpy(msg)
        }
        
        # Log the received data
        self.get_logger().debug(f"Image timestamp: {frame['image_timestamp']}")
        self.get_logger().debug(f"Robot transform: {frame['robot_transform']}")
        self.get_logger().debug(f"Robot transform: {frame['tracking_transform']}")
        self.get_logger().debug(f"Camera info: {frame['camera_info']}")
        self.get_logger().debug(f"Image shape: {frame['image'].shape}, dtype: {frame['image'].dtype}")
        
        # Save the frame to file
        if self._frame_file_path:
            frame_file = os.path.join(self._frame_file_path, f'frame_{self._frame_counter:04d}.pkl')
            with open(frame_file, 'wb') as f:
                pickle.dump(frame, f)
            self.get_logger().info(f'Saved frame {self._frame_counter} to {frame_file}')
            self._frame_counter += 1
                
    def camera_info_callback(self, msg):
        with self.mutex:
            self.camera_info = msg
        self.get_logger().debug('Received camera info')


def main():
    """Main function to run the GUI"""
    # Initialize ROS 2 (optional)
    rclpy.init()
    
    try:
        # Create a simple node for TF access
        node = ImageSubscriber()
        
        # Create and run GUI
        gui = CalibrationGUI(node)
        
        gui.run()
        
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()