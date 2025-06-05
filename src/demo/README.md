# Demos for Cobot with MoveIt2 and Gazebo

We have currently one demo implemented: a simple inverse kinematic using MoveIt2 and their C++ interface.

## Running the demo

Launch the gazebo demo
```
ros2 launch cobot_moveit_config gz_demo_launch.py
```
Run the demo (in a separate terminal)
```
ros2 run demo simple_ik
```

To run the demo with fake controls, use the following launch-file:
```
ros2 launch demo simple_ik_launch.py
```

![](vid/champion_simple_ik_fake_control.gif)