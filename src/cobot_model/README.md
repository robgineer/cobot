# Cobot Model

## Copyright Notice
The model was kindly provided by the company [**Festo SE & Co. KG**](https://www.festo.com/) and remains their intellectual property. Refer to their [GitHub profile](https://github.com/Festo-se) and their [website](https://www.festo.com/de/en/e/about-festo/blog/robotics-id_9229-1153/) for more information on their robotics related activities.
<br/>
<br/>
The model will be adjusted based on the needs of this project.
<br/>
<br/>
License: the model (folder: *meshes* / *urdf*) is licensed under Apache-2.0. Kindly refer to the LICENSE.txt for more information.

## Model Overview

The model used represents a pneumatic arm with seven joints, indexed as: 0 - 6, while joint_0 represents the base axis. It contains a (dummy) gripper that requires remodelling. <br/>

In its original form, the model does not contain any physics related information as *mass* or *inertial* values necessary for the simulation. These values, along with a functional gripper, will be added within the scope of this project.

![Original Cobot Model](img/festo_cobot_original.png)

