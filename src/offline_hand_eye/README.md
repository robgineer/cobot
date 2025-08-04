# Offline Hand Eye Calibration

## ROS2 data extraction 

Play recording (if not running with live realsense):
```bash
ros2 bag play data/cobot/rosbag2_2025_07_31-10_56_58 --clock
```

Run data extractor:
```bash
ros2 run offline_hand_eye record_calib_data
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
    --params-file /workspace/src/offline_hand_eye/doc/tags_41h12.yaml
```
