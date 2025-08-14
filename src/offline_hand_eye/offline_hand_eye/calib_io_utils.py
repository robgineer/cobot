"""
Copyright (c) 2025, Thao Dang, Esslingen University.
This file is part of the offline_hand_eye package (see https://github.com/robgineer/cobot).
License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
"""
import json

def save_calibration(filename, calibration_config, hand_camera_qwxyz, hand_camera_tr, 
                     selected_samples=None, data_file_path=None):
    config = {
        'calibration_type': calibration_config['calibration_type'],
        'robot_base_frame': calibration_config['robot_base_frame'],
        'robot_effector_frame': calibration_config['robot_effector_frame'],
        'tracking_base_frame': calibration_config['tracking_base_frame'],
        'translation': hand_camera_tr,
        'rotation_q_wxyz': hand_camera_qwxyz,
        'selected_samples': selected_samples,
        'data_file_path': data_file_path
    }
    with open(filename, 'w') as f:
        json.dump(config, f, indent=2)
    

def load_calibration(filename):
    with open(filename, 'r') as f:
        config = json.load(f)
    return config

