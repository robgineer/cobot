# Offline Hand Eye Calibration

[doc/tag36h11_0_140mm.pdf](doc/tag36h11_0_140mm.pdf)
https://shiqiliu-67.github.io/apriltag-generator/


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
