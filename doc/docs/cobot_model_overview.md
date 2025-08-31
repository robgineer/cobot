# Cobot Model Overview

The following paragraphs serve as an overview to the basic Cobot Model characteristics. The actual specifics of the model including our approach on the modelling of the physical attributes is provide in the [Cobot Modelling Jupyter Notebook](cobot_model/cobot_modelling.ipynb).

## Robot Overview

The cobot model defines

* an axis element and a corresponding mount,
* 7 segments (the links of the robot),
* 2 fingers (representing the gripper),
* two vacuum grippers,
* four TCP elements for each gripper


![cobot_full](cobot_model/cobot_modelling/img/overview/cobot_full.png)


## Grippers

We have two types of grippers: a finger gripper and a vacuum gripper system. The vacuum grippers can be controlled independently.

Each gripper defines its own axis.


## URDF

The URDF is implemented as a `xacro` to enable variation handling. We define three variations:

1. `enable_realsense_camera`: to activate the real-sense camera model (deactivated by default)
2. `ros2_controls_plugin`, to select the controller (fake by default) and 
3. `use_collision_meshes`, to select between simple and mesh based collisions (mesh by default).


To generate the URDF run
```
xacro src/cobot_model/urdf/festo_cobot_model.urdf.xacro -o cobot.urdf
```

## TCPs

Since we define all rotations around the $Z$ axis, our TCPs do not have a neutral orientation. This implies the following "obstacle": for a pick and place task for example, we would position an object into the world that by default would have a neutral orientation. Since our TCP is does not have a neutral orientation, grasping the object would always imply a rotation of the TCP. Hence, for each task, we would need to transform the Cobot's TCP orientation into the object's (neutral) orientation (in a corresponding software module). Since this could be tedious, we avoid this overhead by adding different TCP frames: one TCP aligned to the world frame, and two slightly tilted TCP frames (w.r.t. the world frame) that compensate for the Cobot's natural tilt at the tool base.

Example for the gripper_tcp:


| gripper_tcp | gripper_tcp_world | gripper_tcp_world_tilted_up |  gripper_tcp_world_tilted_down |
|-----------------------|-----------------------|-----------------------|-----------------------|
| The TCP with all rotations | TCP in world frame. <br/> All rotations are inverted. | TCP in world frame tilted by -45 deg. <br/> Compensates the natural tilt <br/> in the physical model, Z+ is up | TCP in world frame tilted by +135 deg. <br/> Compensates the natural tilt <br/> in the physical model, Z+ is down |
| ![gripper_tcp](cobot_model/cobot_modelling/img/rotations/gripper_tcp.png) | ![gripper_tcp_world](cobot_model/cobot_modelling/img/rotations/gripper_tcp_world.png) | ![gripper_tcp_world_tilted_up](cobot_model/cobot_modelling/img/rotations/gripper_tcp_world_tilted_up.png) | ![gripper_tcp_world_tilted_down](cobot_model/cobot_modelling/img/rotations/gripper_tcp_world_tilted_down.png) |
