import json

def save_calibration(filename, calibration_config, hand_camera_qwxyz, hand_camera_tr):
    config = {
        'calibration_type': calibration_config['calibration_type'],
        'robot_base_frame': calibration_config['robot_base_frame'],
        'robot_effector_frame': calibration_config['robot_effector_frame'],
        'tracking_base_frame': calibration_config['tracking_base_frame'],
        'translation': hand_camera_tr,
        'rotation_q_wxyz': hand_camera_qwxyz
    }
    with open(filename, 'w') as f:
        json.dump(config, f, indent=2)
    

def load_calibration(filename):
    with open(filename, 'r') as f:
        config = json.load(f)
    return config

