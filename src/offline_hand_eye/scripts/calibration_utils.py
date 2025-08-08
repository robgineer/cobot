import pickle
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os

from apriltag import apriltag


def load_and_detect(frame_count, data_root, detector):
    with open(os.path.join(data_root, "frame_%04d.pkl" % frame_count), "rb") as input_file:
        frame = pickle.load(input_file)

    im = frame['image']
    gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
    detections = detector.detect(gray)
    
    return frame, gray, detections

def frame_is_valid(frame, detections):
    if detections is None:
        print("Invalid frame: no detection.")
        return False
    if len(detections) == 0:
        print("Invalid frame: no detection.")
        return False
    elif len(detections) > 1:
        print("Warning: Multiple detections found, using the first one.")
    if 'center' not in detections[0] or 'id' not in detections[0]:
        print("Invalid frame: no center or id in detection.")
        return False
    if detections[0]['id'] < 0:
        print("Invalid frame: invalid detection id.")
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

    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
    for i, detection in enumerate(detections):
        color = colors[i % len(colors)]
        plt.plot(detection['center'][0], detection['center'][1], color+'x')
        contour = detection['lb-rb-rt-lt']
        contour = np.vstack((contour, contour[0]))  # Close the contour
        plt.plot(contour[:, 0], contour[:, 1], color+'o-', label='id: %d' % detection['id'])
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

def extract_pose_from_detection(frame, detection, tagsize=0.074):
    objectPoints = np.array([
        [-tagsize / 2, -tagsize / 2, 0], # left bottom
        [+tagsize / 2, -tagsize / 2, 0], # right bottom
        [+tagsize / 2, +tagsize / 2, 0], # right top
        [-tagsize / 2, +tagsize / 2, 0], # left top
    ], dtype=np.float64)

    imagePoints = detection['lb-rb-rt-lt']

    cameraMatrix = np.array(frame['camera_info']['k']).reshape((3, 3))
    distortion_coeffs = np.array(frame['camera_info']['d']).reshape((1, 5))

    retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distortion_coeffs)
    assert retval, "Error: solvePnP failed"
    rmat, _ = cv2.Rodrigues(rvec)

    return rvec, tvec, rmat

def compute_hand_eye_calibration(data_root, frame_samples, detector, tagsize):
    """
    Computes the hand-eye calibration using the specified frames and detector.
    """
    marker_camera_rot, marker_camera_tr = [], []
    hand_world_rot, hand_world_tr = [], []

    for frame_count in frame_samples:
        frame, gray, detections = load_and_detect(frame_count, data_root, detector)
        if not frame_is_valid(frame, detections):
            print(f"Warning: Skipping invalid frame {frame_count}")
            continue
            
        _, tvec_marker, rmat_marker = extract_pose_from_detection(frame, detections[0], tagsize)
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
        
    method = cv2.CALIB_HAND_EYE_TSAI
    hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(hand_world_rot, hand_world_tr, 
                                                        marker_camera_rot, marker_camera_tr, method=method)
    hand_camera_qwxyz = mat2quat(hand_camera_rot)

    return hand_camera_rot, hand_camera_tr, hand_camera_qwxyz

def compute_TCP_image_position(frame, hand_camera_rot, hand_camera_tr):
    """
    Compute the image position of the TCP in the camera frame.
    """
    
    base2gripper_trans = [frame['robot_transform']['translation']['x'], frame['robot_transform']['translation']['y'], frame['robot_transform']['translation']['z']]

    base2gripper_quat_wxyz = [frame['robot_transform']['rotation']['w'], frame['robot_transform']['rotation']['x'], frame['robot_transform']['rotation']['y'], frame['robot_transform']['rotation']['z']]
    base2gripper_rot = quat2mat(base2gripper_quat_wxyz)

    gripper2base_rot = np.linalg.inv(base2gripper_rot)
    gripper2base_trans = -gripper2base_rot @ base2gripper_trans

    cam2base_rot = hand_camera_rot
    cam2base_trans = hand_camera_tr

    base2cam_rot = np.linalg.inv(cam2base_rot)
    base2cam_trans = -base2cam_rot @ cam2base_trans

    TCP_world = gripper2base_rot @ np.array([0,0,0]).reshape(3, 1) +  np.array(gripper2base_trans).reshape(3, 1)

    d = np.array(frame['camera_info']['d'])
    K = np.array(frame['camera_info']['k']).reshape(3, 3)

    img_points, _ = cv2.projectPoints(objectPoints=TCP_world, rvec=base2cam_rot, tvec=base2cam_trans, cameraMatrix=K, distCoeffs=d)
    return img_points.reshape(2,), TCP_world