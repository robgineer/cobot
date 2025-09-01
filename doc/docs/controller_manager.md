# Cobot Controller Manager

## Discontinued 

This solution is currently not active in the Cobot project. It has been replaced by a custom trajectory controller. To reactivate this solution, revert commit: 9a403d50ed45b9cc583237b94f2c068a97f4bd74 and use the following branch for the `cobot_hardware`: `adapt-hw-interface-to-custom-controller-manager`.

### Reason 
Although this solution results in smooth movements of the Cobot arm, the resulting trajectory is calculated by the SPS of the Cobot. We therefore have no influence on the trajectory and collisions that were considered within trajectory planning are obsolete. Hence, even with a camera attached, the Cobot will be blind with this solution.

## Implementation Overview

Our Cobot is not able to follow a full trajectory. It instead accepts the final trajectory point or a list of points (in form of a path). MoveIt2 and ROS2 control, however are designed to generate a trajectory and send out single position / velocity / effort commands to the hardware in realtime. Since we cannot use these single points, we use the last trajectory point and forward it to the Cobot hardware interface. 


### Communication Flow
```text
cobot_controller_manager (this package)
        ↓ (send last trajectory point)
ros2_control controller
        ↓ (forward one single point to hardware interface)
cobot_hardware (real hardware interface)
```

## Getting started

This controller manager works only in combination with the `cobot_hardware` package (branch: `adapt-hw-interface-to-custom-controller-manager`) and aims to control the real Cobot.

In order to manipulate the Cobot in ROS2, connect to HS-Esslingen VPN and run
```bash
git clone https://github.com/robgineer/cobot.git .
git submodule init src/cobot_hardware
git submodule update src/cobot_hardware
```
Then build this project and run
```bash
ros2 launch demo rviz_demo_launch.py controller_type:=real
```


