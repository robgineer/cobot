# Offline Hand Eye Calibration

[doc/tag36h11_0_140mm.pdf](doc/tag36h11_0_140mm.pdf)
<https://shiqiliu-67.github.io/apriltag-generator/>

## ROS2 data extraction

To record a bag file:

```bash
ros2 launch demo rviz_demo_launch.py controller_type:=real enable_realsense_camera:=true
ros2 bag record --all
```

Play recording (if not running with live realsense):

```bash
ros2 bag play data/cobot/rosbag2_2025_07_31-10_56_58 --clock 100
```

Run data extractor (if running live):

```bash
ros2 run offline_hand_eye record_calib_data
```

or if from recording:

```bash
ros2 run offline_hand_eye record_calib_data --ros-args --remap use_sim_time:=true
```

Publish calibration result (after calibration has been computed):

```bash
ros2 run offline_hand_eye calib_publisher --ros-args -p calibration_file:=handeye_calibration.json
```

## Install Python Calibration

```bash
conda create --name offline-hand-eye python=3.12 ipython jupyter conda-forge::matplotlib 
conda activate offline-hand-eye
pip3 install opencv-python
conda install conda-forge::apriltag
python -m ipykernel install --user --name=offline-hand-eye
```

or in venv

```bash
cd offline_hand_eye/offline_hand_eye
python -m venv venv
source venv/bin/activate
pip install ipython jupyter matplotlib opencv-python apriltag
python -m ipykernel install --user --name=offline-hand-eye
```

## Test this with april tag detector

```bash
sudo apt install ros-jazzy-apriltag-ros
```

```bash
ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/camera/camera/color/image_raw \
    -r camera_info:=/camera/camera/color/camera_info \
    --params-file /workspace/src/offline_hand_eye/doc/tags_36h11.yaml
```

## Test with aruco marker

```bash
ros2 run aruco_ros single --ros-args -p image_is_rectified:=true -p marker_size:=0.1 -p marker_id:=1 -p reference_frame:=camera_link -p camera_frame:=/camera/camera/color/image_raw -p marker_frame:=camera_marker -p corner_refinement:=LINES

ros2 bag record --topics /aruco_single/debug /aruco_single/debug/compressed /aruco_single/debug/compressedDepth /aruco_single/debug/theora /aruco_single/debug/zstd /aruco_single/marker /aruco_single/marker_array /aruco_single/pixel /aruco_single/pose /aruco_single/position /aruco_single/result /aruco_single/result/compressed /aruco_single/result/compressedDepth /aruco_single/result/theora /aruco_single/result/zstd /aruco_single/transform /camera/camera/color/camera_info /camera/camera/color/image_raw /camera/camera/color/image_raw/compressed /camera/camera/color/image_raw/compressedDepth /camera/camera/color/image_raw/theora /camera/camera/color/image_raw/zstd /camera/camera/color/metadata /cobot_arm_group_controller/transition_event /goal_pose /initialpose /joint_state_broadcaster/transition_event /joint_states /tf /tf_static
```

## How to check the transformations with tf2_ros

Create a pdf of all frames:
```bash
ros2 run tf2_tools view_frames
```

Check trafo between two frames:
```bash
#Usage: tf2_echo source_frame target_frame
ros2 run tf2_ros tf2_echo base_link camera_color_frame
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

**Note**: trafo from camera_color_optical_frame to camera_color_frame for our realsense camera:
$$
  {}^{colorFrame}\mathbf{T}_{colorOpticalFrame} = \begin{pmatrix} 
  0 & -1 &  0 & 0 \\
  0 &  0 & -1 & 0 \\
  1 &  0 &  0 & 0 \\
  0 &  0 &  0 & 1
  \end{pmatrix} 
$$

- Translation: [0.000, 0.000, 0.000]
- Rotation: in Quaternion [0.500, -0.500, 0.500, 0.500]
- Rotation: in RPY (radian) [0.785, -1.571, 0.000]
- Rotation: in RPY (degree) [45.000, -90.000, 0.000]

ros2 run tf2_ros tf2_echo camera_color_optical_frame camera_bottom_screw_frame -> 
- Translation: [0.032, 0.012, -0.010]
- Rotation: in Quaternion [-0.500, 0.501, -0.499, -0.500]
- Rotation: in RPY (radian) [3.013, -1.568, -1.444]
- Rotation: in RPY (degree) [172.638, -89.853, -82.761]
- Matrix:
  0.000 -1.000 -0.002  0.032
 -0.003  0.002 -1.000  0.012
  1.000  0.000 -0.003 -0.010
  0.000  0.000  0.000  1.000


**Before:**

ros2 run tf2_ros tf2_echo base_link camera_color_frame -> 
- Translation: [-0.432, 0.010, 1.213]
- Rotation: in Quaternion [-0.000, 0.002, 0.707, 0.707]
- Rotation: in RPY (radian) [0.002, 0.003, 1.570]
- Rotation: in RPY (degree) [0.124, 0.146, 89.973]
- Matrix:
  0.000 -1.000  0.002 -0.432
  1.000  0.000  0.003  0.010
 -0.003  0.002  1.000  1.213
  0.000  0.000  0.000  1.000

ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame ->
- Translation: [-0.432, 0.010, 1.213]
- Rotation: in Quaternion [-0.708, 0.001, 0.001, 0.706]
- Rotation: in RPY (radian) [-1.573, 0.002, -0.000]
- Rotation: in RPY (degree) [-90.146, 0.124, -0.027]
- Matrix:
  1.000 -0.002  0.000 -0.432
 -0.000 -0.003  1.000  0.010
 -0.002 -1.000 -0.003  1.213
  0.000  0.000  0.000  1.000

**After:**

ros2 run tf2_ros tf2_echo base_link camera_color_frame ->
- Translation: [-0.432, 0.010, 1.213]
- Rotation: in Quaternion [-0.000, 0.002, 0.707, 0.707]
- Rotation: in RPY (radian) [0.002, 0.003, 1.570]
- Rotation: in RPY (degree) [0.124, 0.146, 89.973]
- Matrix:
  0.000 -1.000  0.002 -0.432
  1.000  0.000  0.003  0.010
 -0.003  0.002  1.000  1.213
  0.000  0.000  0.000  1.000

ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame ->
- Translation: [-0.626, -0.464, 1.538]
- Rotation: in Quaternion [0.026, 0.924, 0.376, 0.065]
- Rotation: in RPY (radian) [2.363, 0.101, 3.043]
- Rotation: in RPY (degree) [135.378, 5.761, 174.363]
- Matrix:
 -0.990 -0.000  0.140 -0.626
  0.098  0.715  0.692 -0.464
 -0.100  0.699 -0.708  1.538
  0.000  0.000  0.000  1.000


ros2 run tf2_ros tf2_echo base_link camera_bottom_screw_frame ->
- Translation: [-0.659, -0.459, 1.550]
- Rotation: in Quaternion [0.696, 0.604, -0.254, 0.293]
- Rotation: in RPY (radian) [2.999, 0.787, 1.371]
- Rotation: in RPY (degree) [171.827, 45.086, 78.555]