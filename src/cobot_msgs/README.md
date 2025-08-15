# Cobot custom ROS2 communication

For the moment this package only includes a service definition for the communication between the rviz control panel and the cobot_trajectory_controller.

The service contains the following attributes:

```
request_full_trajectory_mode: send either the last point of a trajectory (=false) or the entire trajectory (=true) to the hardware interface
resampling_delta: if > 0.0 the "full_trajectory" is up-sampled to contain only points that have a minimum delta in between
acknowledge_error: reset Cobot error
```