Test this with april tag detector:

```bash
sudo apt install ros-jazzy-apriltag-ros
```

```bash
ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/camera/camera/color/image_raw \
    -r camera_info:=/camera/camera/color/camera_info \
    --params-file /workspace/src/offline_hand_eye/doc/tags_41h12.yaml
```
