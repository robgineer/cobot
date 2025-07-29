# Cobot calibration

This contains launchfiles for running the the realsense to cobot calibration. 
It is based on two repos:
- [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2) for running the actual calibration procedure
- [ros_aruco_opencv](https://github.com/fictionlab/ros_aruco_opencv.git) for tracking aruco markers.

## Installation

```bash
git clone --recurse-submodules https://github.com/robgineer/cobot.git cobot
cd cobot
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
source install/setup.bash
```

## Check camera

Check if realsense is working **outside** your docker environment on your native system:
1. Unplug and connect the realsense, each time checking for new usb devices with ``lsusb``. The realsense should be shown as a Intel Corp. device.
2. Check for live images using ``rs-multicam``.
3. Make sure that your docker is started in priviledged mode, e.g. by passing ``"privileged": true`` in your ``devcontainer.json``
4. Check for Realsense within your docker with ``lsusb``. Should be listed as ``Intel Corp. RealSense D435``.
5. Run the demo:
```bash
ros2 launch demo rviz_demo_launch.py enable_realsense_camera:=true
```

## Check Aruco markers

1. Print an Aruco marker, e.g. [img/Aruco_4x4_0_100mm.pdf](img/Aruco_4x4_0_100mm.pdf) <br>
<img src="img/4x4_1000-0.svg" width="48">
2. Track with 
   ```bash
   ros2 run aruco_opencv aruco_tracker_autostart --ros-args -p cam_base_topic:=/camera/camera/color/image_raw -p marker_size:=0.1
   ```
   and check output in rviz in topic ``/aruco_tracker/debug``.
