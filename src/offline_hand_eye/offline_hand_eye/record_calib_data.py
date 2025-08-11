#!/usr/bin/env python

from typing import List
import os
import threading
import pickle
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Duration, Time
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from rclpy_message_converter import message_converter

from .record_calib_data_gui import CalibrationGUI, NO_TRACKER_MARKER_STR
from .image_to_numpy import image_to_numpy

RECORDING_OFF = 0
RECORDING_ON = 1
RECORDING_SINGLE = 2

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
        self.recording_mode_ = RECORDING_OFF # RECORDING_OFF, RECORDING_ON, RECORDING_SINGLE
            
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
        self._frame_file_path = file_path
        self.recording_mode_ = RECORDING_ON

    def stop_recording(self):
        """Stop recording the calibration data"""
        self.get_logger().info("Stopping calibration data recording")
        self.recording_mode_ = RECORDING_OFF
        
    def single_frame_recording(self, file_path: str):
        """Record a single frame of calibration data"""
        self.get_logger().info("Stopping calibration data recording")
        self._frame_file_path = file_path
        self.recording_mode_ = RECORDING_SINGLE        
        
    def is_recording_active(self) -> bool:
        """Check if recording is active"""
        return self.recording_mode_ != RECORDING_OFF

    def is_valid_transform(self, t) -> bool:
        """Check if the transform is valid (not None and contains necessary fields)"""
        if t is None:
            self.get_logger().error('Robot transform is None, cannot process image')
            return False
        
        translation = [ t.transform.translation.x, \
                        t.transform.translation.y, \
                        t.transform.translation.z]
        rotation = [t.transform.rotation.w, \
                    t.transform.rotation.x, \
                    t.transform.rotation.y, \
                    t.transform.rotation.z]
        if any(np.isnan(translation)) or any(np.isnan(rotation)):
            self.get_logger().error("Invalid frame: Robot transform contains NaN values.")
            return False
        if any(np.isinf(translation)) or any(np.isinf(rotation)):
            self.get_logger().error("Invalid frame: Robot transform contains infinite values.")
            return False    
        if np.allclose(translation, 0) and np.allclose(rotation, [1,0,0,0]):
            self.get_logger().error("Invalid frame: Robot transform is all zeros.")
            return False
        
        return True

    def image_callback(self, msg):
        if not self.is_recording_active():
            self.get_logger().debug('Recording is not active, ignoring image message')
            return
        
        if not self.config:
            self.get_logger().error('Configuration is not set, cannot process image')
            return
        
        if not self.camera_info and self.config['camera_info_topic']:
            self.get_logger().warning('Camera info is not available, cannot process image')
            return
        
        with self.mutex:
            get_newest_transform = True
            if get_newest_transform:
                curr_time = Time() # i.e. zero
                curr_time_corrected = Time() # i.e. zero
            else:
                # todo: why is time correction needed?
                curr_time = Time().from_msg(msg.header.stamp) 
                curr_time_corrected = curr_time - Duration(seconds=2)
            timeout = Duration(seconds=1)

            # cf https://github.com/marcoesposito1988/easy_handeye2
            try:
                if self.config['calibration_type'] == 'eye_in_hand':
                    robot = self.tf_buffer.lookup_transform(self.config['robot_base_frame'],
                                                            self.config['robot_effector_frame'], curr_time_corrected,
                                                            timeout)
                else:
                    robot = self.tf_buffer.lookup_transform(self.config['robot_effector_frame'],
                                                            self.config['robot_base_frame'], curr_time_corrected,
                                                            timeout)
                if not self.is_valid_transform(robot):
                    self.get_logger().error('Robot transform is invalid, cannot process image')
                    return
                
                if self.config['tracking_marker_frame'] == NO_TRACKER_MARKER_STR:
                    tracking = None
                else:
                    # If tracking marker is set, get the transform
                    tracking = self.tf_buffer.lookup_transform(self.config['tracking_base_frame'],
                                                               self.config['tracking_marker_frame'], curr_time,
                                                               timeout)
                    if not self.is_valid_transform(tracking):
                        self.get_logger().error('Tracking transform is invalid, cannot process image')
                        return

            except tf2_ros.ExtrapolationException as e:
                self.get_logger().error(f'Failed to get the tracking transform: {e}')
                return
            
        self.get_logger().info('Received new sample data')
        frame = {
            'image_timestamp': Time().from_msg(msg.header.stamp).seconds_nanoseconds(),
            'robot_transform': message_converter.convert_ros_message_to_dictionary(robot.transform),
            'tracking_transform': message_converter.convert_ros_message_to_dictionary(tracking.transform) if tracking else None,
            'camera_info': message_converter.convert_ros_message_to_dictionary(self.camera_info) if self.camera_info else None,
            'image': image_to_numpy(msg),
            'image_encoding': msg.encoding,
            'image_width': msg.width,
            'image_height': msg.height
        }
        
        # Log the received data
        self.get_logger().debug(f"Image timestamp: {frame['image_timestamp']}")
        self.get_logger().debug(f"Robot transform: {frame['robot_transform']}")
        self.get_logger().debug(f"Robot transform: {frame['tracking_transform']}")
        self.get_logger().debug(f"Camera info: {frame['camera_info']}")
        self.get_logger().debug(f"Image shape: {frame['image'].shape}, dtype: {frame['image'].dtype}")
        
        # Save the frame to file
        if self._frame_file_path:
            frame_file = os.path.join(self._frame_file_path, f'frame_{self.frame_counter_:04d}.pkl')
            with open(frame_file, 'wb') as f:
                pickle.dump(frame, f)
            self.get_logger().info(f'Saved frame {self.frame_counter_} to {frame_file}')
            self.frame_counter_ += 1

        if self.recording_mode_ == RECORDING_SINGLE:
            self.recording_mode_ = RECORDING_OFF
                
    def camera_info_callback(self, msg):
        with self.mutex:
            self.camera_info = msg
        self.get_logger().debug('Received camera info')


def main():
    """Main function to run the GUI"""
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