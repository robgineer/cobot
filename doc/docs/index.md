# Cobot Documentation

This page serves as a reference for the integration of a pneumatic Cobot (provided by [**Festo SE & Co. KG**](https://www.festo.com/)) into MoveIt2 and ROS2 Control.

![cobot_logo_small](img/cobot_logo_small.png)

In oder to get a deep insight into this project and to use the Cobot Model in simulation or physically for your custom tasks, follow the steps below.

1. Check out the [Quickstart Guide](quickstart.md) to get the Cobot running in Simulation or on its real hardware.
2. Run some demos using the C++ / Python API or the MoveIt Task Constructor from the [demo package](https://github.com/robgineer/cobot/tree/main/src/demo).
3. Get an overview of the [MoveIt2 / ROS2 Control Configuration](cobot_configuration.md) and the [launch files](launch_files.md).
4. Get familiar with the [Cobot Model](cobot_model_overview.md) and delve deeper into model creation in the [Cobot Modelling Jupyter Notebook](https://github.com/robgineer/cobot/blob/main/src/cobot_model/doc/cobot_modelling.ipynb).
5. Understand the custom Cobot control setup: [Cobot Trajectory Controller](cobot_trajectory_controller.md) / [Cobot Hardware Interface](cobot_hardware.md).
6. Learn how to evaluate your own planner config based on our [planner evaluation](planner_evaluation.md).
7. Calibrate your custom camera using the [hand eye calibration](offline_hand_eye.md) implementation.
