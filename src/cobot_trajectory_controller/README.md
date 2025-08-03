# Cobot Trajectory Controller

Custom ROS2 controller for the Cobot.

## Getting started

This controller works only in combination with the `cobot_hardware` package and aims to control the real Cobot.

In order to manipulate the Cobot in ROS2, connect to HS-Esslingen VPN and run
```
git clone https://github.com/robgineer/cobot.git .
git submodule init src/cobot_hardware
git submodule update src/cobot_hardware
```
Then build this project and run
```
ros2 launch demo rviz_demo_launch.py controller_type:=real
```

## Implementation Overview

We have two options to pass commands to the Cobot: single point (the final point of a trajectory, via ```SetRobotWaypoint```) or a list of points (via ```ExecuteInstructionList```, representing a path). We cannot send actual trajectories or fine grained position commands as the SPS of the Cobot takes care of the trajectory (and we cannot by-pass the SPS).

This, unfortunately, does not align with the ROS2 control principles. A basic overview of the trajectory generation and control is provided in the following:
```
MoveIt Planning (OMPL) → Generates a collision free path
                ↓
Time Parameterization Plugin (Iterative / TOTG) → Adds time (making it an actual trajectory)
                ↓
Final RobotTrajectory message
                ↓
MoveIt Simple Controller Manager
                ↓
ros2_control controller
                ↓
HardwareInterface (this package)
```
The ROS2 controllers are designed to send position commands to the hardware in real time; which is not suitable for the Cobot. 

In previous implementations, we simply sent the final trajectory point to the Cobot (using a custom MoveItControllerManager, *the CobotControllerManager*). Although this resulted in smooth movements of the Cobot arm, the resulting trajectory was calculated by the SPS. We therefore had no influence on the trajectory and collisions that were considered within trajectory planning were obsolete. Hence, even with a camera attached, the Cobot would have been blind.

In order to overcome this limitation we use the instruction list mechanism, where we pass a list of points to the Cobot and since the standard implementation of the ROS2 trajectory controller does not allow to send full trajectories (or lists of points), we are using this custom ROS2 controller, that send out either the last point of a trajectory or the entire trajectory to this interface.
Note: ROS2 control implies using single joint commands (declared as double) and since a full trajectory was therefore difficult to be passed, we introduced another communication feature: a singleton realtime buffer.

```
MoveIt Planning (OMPL) → Generates a collision free path
        ↓
Time Parameterization Plugin (Iterative / TOTG) → Adds time (making it an actual trajectory)
        ↓
Final RobotTrajectory message
        ↓
MoveIt Simple Controller Manager
        ↓
cobot_trajectory_controller (this package)
        ↓ trajectory (via shared buffer)
cobot_hardware
```


