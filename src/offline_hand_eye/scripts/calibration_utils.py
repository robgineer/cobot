"""
Copyright (c) 2025, Thao Dang, Esslingen University.
This file is part of the offline_hand_eye package (see https://github.com/robgineer/cobot).
License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
"""

import pickle
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os

def load_and_detect(frame_count, data_root, detector, tagsize):
    with open(os.path.join(data_root, "frame_%04d.pkl" % frame_count), "rb") as input_file:
        frame = pickle.load(input_file)

    im = frame['image']
    gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
    detections = detector.detect(gray)

    cameraMatrix = np.array(frame['camera_info']['k']).reshape((3, 3))
    camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )
    detections = detector.detect(gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=tagsize)

    return frame, gray, detections

def frame_is_valid(frame, detections, use_tracking_marker=False):
    if not use_tracking_marker:
        if detections is None:
            print("Invalid frame: no detection.")
            return False
        if len(detections) == 0:
            print("Invalid frame: no detection.")
            return False
        elif len(detections) > 1:
            print("Warning: Multiple detections found, using the first one.")
        if detections[0].tag_id < 0:
            print("Invalid frame: invalid detection id.")
            return False
    else:
        if 'tracking_transform' not in frame:
            print("Invalid frame: No tracking transform data found.")
            return False
    
    if frame.get('robot_transform') is None:
        print("Invalid frame: No robot transform data found.")
        return False
    if 'translation' not in frame['robot_transform'] or 'rotation' not in frame['robot_transform']:
        print("Invalid frame: Robot transform data is incomplete.")
        return False
    translation = [frame['robot_transform']['translation']['x'], \
                 frame['robot_transform']['translation']['y'], \
                 frame['robot_transform']['translation']['z']]
    rotation = [frame['robot_transform']['rotation']['w'], \
                frame['robot_transform']['rotation']['x'], \
                frame['robot_transform']['rotation']['y'], \
                frame['robot_transform']['rotation']['z']]
    if any(np.isnan(translation)) or any(np.isnan(rotation)):
        print("Invalid frame: Robot transform contains NaN values.")
        return False
    if any(np.isinf(translation)) or any(np.isinf(rotation)):
        print("Invalid frame: Robot transform contains infinite values.")
        return False
    if np.allclose(translation, 0) and np.allclose(rotation, [1,0,0,0]):
        print("Invalid frame: Robot transform is all zeros.")
        return False
    
    return True
    

def show_detections(gray, detections, apriltag_family, show_legend=True, show_family=True):
    plt.imshow(gray, cmap='gray')
    if show_family:
        plt.text(5, 24, 'family: %s' % (apriltag_family), 
                color='red', fontsize=12, ha='left', va='bottom')

    if detections:
        colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
        for i, detection in enumerate(detections):
            color = colors[i % len(colors)]
            plt.plot(detection.center[0], detection.center[1], color+'x')
            contour = detection.corners
            contour = np.vstack((contour, contour[0]))  # Close the contour
            plt.plot(contour[:, 0], contour[:, 1], color+'o-', label='id: %d' % detection.tag_id)
            for j in range(4):
                plt.text(contour[j, 0] - 5, contour[j, 1] - 5, '%d' % j, color=color, fontsize=12, ha='center', va='bottom')

    plt.axis('off')
    if show_legend:
        plt.legend(loc='upper right')
        
def quat2mat(q_wxyz):
    """ Calculate rotation matrix corresponding to quaternion.

    For reference, see https://github.com/matthew-brett/transforms3d/blob/main/transforms3d/quaternions.py
    
    Args:
        q_wxyz: 4 element quaternion, in order w, x, y, z.

    Returns:
        (3,3)-rotation matrix corresponding to input quaternion q_wxyz.

    Examples:
    
    >>> import numpy as np
    >>> M = quat2mat([1, 0, 0, 0]) # Identity quaternion
    >>> np.allclose(M, np.eye(3))
    True
    >>> M = quat2mat([0, 1, 0, 0]) # 180 degree rotn around axis 0
    >>> np.allclose(M, np.diag([1, -1, -1]))
    True
    """
    EPS = np.finfo(np.float64).eps
    w, x, y, z = q_wxyz
    Nq = w*w + x*x + y*y + z*z
    if Nq < EPS:
        return np.eye(3)
    s = 2.0/Nq
    X = x*s
    Y = y*s
    Z = z*s
    wX = w*X; wY = w*Y; wZ = w*Z
    xX = x*X; xY = x*Y; xZ = x*Z
    yY = y*Y; yZ = y*Z; zZ = z*Z
    return np.array(
           [[ 1.0-(yY+zZ), xY-wZ, xZ+wY ],
            [ xY+wZ, 1.0-(xX+zZ), yZ-wX ],
            [ xZ-wY, yZ+wX, 1.0-(xX+yY) ]])

def mat2quat(M):
    """ Calculate quaternion corresponding to given rotation matrix.
    
    For reference, see https://github.com/matthew-brett/transforms3d/blob/main/transforms3d/quaternions.py

    Args:
      M : 3x3 rotation matrix.

    Returns:
      Closest quaternion (w,x,y,z) to input matrix, having positive w.

    Examples:
    
    >>> import numpy as np
    >>> q = mat2quat(np.eye(3)) # Identity rotation
    >>> np.allclose(q, [1, 0, 0, 0])
    True
    >>> q = mat2quat(np.diag([1, -1, -1]))
    >>> np.allclose(q, [0, 1, 0, 0]) # 180 degree rotn around axis 0
    True
    """
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = M.flat
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0              ],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
        ) / 3.0
    vals, vecs = np.linalg.eigh(K)
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    if q[0] < 0:
        q *= -1
    return q

def extract_pose_from_detection(detections):
    """
    Extracts the pose (translation and rotation matrix) from the first detection.
    """
    assert len(detections) > 0, "No detections to extract pose from."
    tvec_marker = np.array(detections[0].pose_t)
    rmat_marker = np.array(detections[0].pose_R)
    
    return tvec_marker, rmat_marker

def get_marker_pose(frame, frame_count, detections, use_tracking_marker):
    """
    Get marker pose, either from tracking transform or from image detections.
    """
    if use_tracking_marker:
        assert 'tracking_transform' in frame, \
            f"Frame {frame_count} does not contain 'tracking_transform' requested for hand-eye calibration."
        tvec_marker = np.array([frame['tracking_transform']['translation']['x'], 
                            frame['tracking_transform']['translation']['y'], 
                            frame['tracking_transform']['translation']['z']]).reshape((3, 1))
        quat_wxyz_marker = np.array([frame['tracking_transform']['rotation']['w'], 
                                    frame['tracking_transform']['rotation']['x'], 
                                    frame['tracking_transform']['rotation']['y'], 
                                    frame['tracking_transform']['rotation']['z']])
        rmat_marker = quat2mat(quat_wxyz_marker)

    else:
        if not frame_is_valid(frame, detections, use_tracking_marker):
            print(f"Warning: Skipping invalid frame {frame_count}")
            return None, None
            
        tvec_marker, rmat_marker = extract_pose_from_detection(detections)
    
    return tvec_marker, rmat_marker

def compute_hand_eye_calibration(data_root, frame_samples, detector, tagsize, 
                                 method_str='PARK', use_tracking_marker=False):
    """
    Computes the hand-eye calibration using the specified frames and detector.
    """
    marker_camera_rot, marker_camera_tr = [], []
    hand_world_rot, hand_world_tr = [], []

    for frame_count in frame_samples:
        frame, _, detections = load_and_detect(frame_count, data_root, detector, tagsize)

        tvec_marker, rmat_marker = get_marker_pose(frame, frame_count, detections, use_tracking_marker)
        if tvec_marker is None or rmat_marker is None:
            continue

        marker_camera_rot.append(rmat_marker)
        marker_camera_tr.append(tvec_marker)

        tvec_robot = np.array([frame['robot_transform']['translation']['x'], 
                            frame['robot_transform']['translation']['y'], 
                            frame['robot_transform']['translation']['z']]).reshape((3, 1))
        quat_wxyz_robot = np.array([frame['robot_transform']['rotation']['w'], 
                                    frame['robot_transform']['rotation']['x'], 
                                    frame['robot_transform']['rotation']['y'], 
                                    frame['robot_transform']['rotation']['z']])
        rmat_robot = quat2mat(quat_wxyz_robot)
        hand_world_rot.append(rmat_robot)
        hand_world_tr.append(tvec_robot)
        
    calibration_variants = {
        'TSAI': cv2.CALIB_HAND_EYE_TSAI,
        'PARK': cv2.CALIB_HAND_EYE_PARK,
        'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
        'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
        'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS
    }
    assert method_str.upper() in calibration_variants, f"Unknown calibration method: {method_str}"
    method = calibration_variants.get(method_str.upper(), cv2.CALIB_HAND_EYE_TSAI)
    try:
        hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(
            hand_world_rot, hand_world_tr,
            marker_camera_rot, marker_camera_tr, method=method
        )
    except cv2.error as e:
        print(f"cv2.calibrateHandEye error: {e}")
        raise
    except Exception as e:
        print(f"Unexpected error in calibrateHandEye: {e}")
        raise
    hand_camera_qwxyz = mat2quat(hand_camera_rot)

    return hand_camera_rot, hand_camera_tr, hand_camera_qwxyz

def compute_TCP_image_position(frame, hand_camera_rot, hand_camera_tr):
    """
    Compute the image position of the TCP in the camera frame.
    """
    
    base2gripper_trans = [frame['robot_transform']['translation']['x'], frame['robot_transform']['translation']['y'], frame['robot_transform']['translation']['z']]

    base2gripper_quat_wxyz = [frame['robot_transform']['rotation']['w'], frame['robot_transform']['rotation']['x'], frame['robot_transform']['rotation']['y'], frame['robot_transform']['rotation']['z']]
    base2gripper_rot = quat2mat(base2gripper_quat_wxyz)

    gripper2base_rot = np.transpose(base2gripper_rot)
    gripper2base_trans = -gripper2base_rot @ base2gripper_trans

    cam2base_rot = hand_camera_rot
    cam2base_trans = hand_camera_tr

    base2cam_rot = np.transpose(cam2base_rot)
    base2cam_trans = -base2cam_rot @ cam2base_trans

    TCP_world = gripper2base_rot @ np.array([0,0,0]).reshape(3, 1) +  np.array(gripper2base_trans).reshape(3, 1)

    d = np.array(frame['camera_info']['d'])
    K = np.array(frame['camera_info']['k']).reshape(3, 3)

    base2cam_rvec, _ = cv2.Rodrigues(base2cam_rot)
    img_points, _ = cv2.projectPoints(objectPoints=TCP_world, rvec=base2cam_rvec, tvec=base2cam_trans, cameraMatrix=K, distCoeffs=d)
    return img_points.reshape(2,), TCP_world

def compute_target_to_gripper_transform(hand_camera_rot, hand_camera_tr, data_root, frame_samples, detector, tagsize, use_tracking_marker=False):
    """
    Compute the 3D transform from the calibration target to the gripper coordinate frame.
    Estimate is obtained by computing this transform for each frame and averaging the results (in 3d translation and 3d rotation vector space).
    
    Returns:
        (rvec_target_to_gripper, tvec_target_to_gripper): rotation and translation vectors
    """    
    b_T_c = np.eye(4)
    b_T_c[:3, :3] = hand_camera_rot
    b_T_c[:3, 3] = hand_camera_tr.flatten()

    tvecs_target_to_gripper = []
    rvecs_target_to_gripper = []

    # estimate target to gripper transform in each frame
    for frame_count in frame_samples:
        frame, _, detections = load_and_detect(frame_count, data_root, detector, tagsize)
        tvec_marker, rmat_marker = get_marker_pose(frame, frame_count, detections, use_tracking_marker)
        if tvec_marker is None or rmat_marker is None:
            continue
      
        c_T_t = np.eye(4)
        c_T_t[:3, :3] = rmat_marker
        c_T_t[:3, 3] = tvec_marker.flatten()

        tvec_robot = np.array([frame['robot_transform']['translation']['x'], 
                            frame['robot_transform']['translation']['y'], 
                            frame['robot_transform']['translation']['z']]).reshape((3, 1))
        quat_wxyz_robot = np.array([frame['robot_transform']['rotation']['w'], 
                                    frame['robot_transform']['rotation']['x'], 
                                    frame['robot_transform']['rotation']['y'], 
                                    frame['robot_transform']['rotation']['z']])
        rmat_robot = quat2mat(quat_wxyz_robot)
        
        g_T_b = np.eye(4)
        g_T_b[:3, :3] = rmat_robot 
        g_T_b[:3, 3] = tvec_robot.flatten()     

        g_T_t = g_T_b @ b_T_c @ c_T_t
        rmat_target_to_gripper = g_T_t[:3, :3]
        rvec_target_to_gripper, _ = cv2.Rodrigues(rmat_target_to_gripper)
        tvec_target_to_gripper = g_T_t[:3, 3].reshape((3, 1))

        tvecs_target_to_gripper.append(tvec_target_to_gripper)
        rvecs_target_to_gripper.append(rvec_target_to_gripper)

    # average in 3D translation and rotation vector space
    rvec_target_to_gripper = np.zeros((3,1))
    tvec_target_to_gripper = np.zeros((3,1))
    for i in range(len(tvecs_target_to_gripper)):
        rvec_target_to_gripper += rvecs_target_to_gripper[i]
        tvec_target_to_gripper += tvecs_target_to_gripper[i]

    rvec_target_to_gripper /= len(tvecs_target_to_gripper)
    tvec_target_to_gripper /= len(tvecs_target_to_gripper)
    
    return rvec_target_to_gripper, tvec_target_to_gripper

def compute_target_image_position(frame, hand_camera_rot, hand_camera_tr, target2gripper_rvec, target2gripper_trans, tagsize):
    """
    Compute the image positions of the calibration target.
    Returns: 
        Target image corners in ccw order.
    """
    objectPoints = np.array([
        [-tagsize / 2, +tagsize / 2, 0], 
        [+tagsize / 2, +tagsize / 2, 0], 
        [+tagsize / 2, -tagsize / 2, 0], 
        [-tagsize / 2, -tagsize / 2, 0], 
    ], dtype=np.float64)
    
    base2gripper_trans = [frame['robot_transform']['translation']['x'], frame['robot_transform']['translation']['y'], frame['robot_transform']['translation']['z']]
    base2gripper_quat_wxyz = [frame['robot_transform']['rotation']['w'], frame['robot_transform']['rotation']['x'], frame['robot_transform']['rotation']['y'], frame['robot_transform']['rotation']['z']]
    base2gripper_rot = quat2mat(base2gripper_quat_wxyz)

    gripper2base_rot = np.transpose(base2gripper_rot)
    gripper2base_trans = -gripper2base_rot @ base2gripper_trans

    cam2base_rot = hand_camera_rot
    cam2base_trans = hand_camera_tr

    base2cam_rot = np.transpose(cam2base_rot)
    base2cam_trans = -base2cam_rot @ cam2base_trans
    base2cam_rvec, _ = cv2.Rodrigues(base2cam_rot)
    
    target2gripper_rot, _ = cv2.Rodrigues(target2gripper_rvec)
    XC_gripper = target2gripper_rot @ np.transpose(objectPoints) +  np.array(target2gripper_trans).reshape(3, 1)
    XC_world = gripper2base_rot @ XC_gripper +  np.array(gripper2base_trans).reshape(3, 1)

    d = np.array(frame['camera_info']['d'])
    K = np.array(frame['camera_info']['k']).reshape(3, 3)

    img_points, _ = cv2.projectPoints(objectPoints=XC_world, rvec=base2cam_rvec, tvec=base2cam_trans, cameraMatrix=K, distCoeffs=d)
    img_points = np.squeeze(img_points)
    assert img_points.shape == (4, 2), f"Expected 4 points, got {img_points.shape}"

    return img_points

def compute_reprojection_error_mean_max(hand_camera_rot, hand_camera_tr, data_root, frame_samples, detector, target2gripper_rvec, target2gripper_trans, tagsize):
    """
    Compute the mean and max reprojection error for the calibration frames.

    Returns:
        (mean_error, max_error)
    """
    dist_norms = []
    for frame_count in frame_samples:
        frame, _, detections = load_and_detect(frame_count, data_root, detector, tagsize)
        if not frame_is_valid(frame, detections):
            print(f"Warning: Skipping invalid frame {frame_count}")
            continue

        xc_proj = compute_target_image_position(frame, hand_camera_rot, hand_camera_tr, target2gripper_rvec, target2gripper_trans, tagsize)
        
        # reorder to match corners - not necessary
        #dist_1st_point = xc_proj[0,:] - detections[0].corners
        #dist_1st_point = np.sum(dist_1st_point * dist_1st_point, axis=1).tolist()
        #new_indices = (-np.arange(len(dist_1st_point)) + np.argmin(dist_1st_point)) % len(dist_1st_point)
        #xc_proj = xc_proj[new_indices, :]

        dist = xc_proj - detections[0].corners
        dist_norm = np.sqrt(np.sum(dist * dist, axis=1)).tolist()
        dist_norms.extend(dist_norm)
        #print(f'. difference projected to detected: {dist}')
        #print(f'. distance: {dist_norm}')
        
    if dist_norms:
        return np.mean(dist_norms), np.max(dist_norms)
    return None, None