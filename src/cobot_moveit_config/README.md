# Gazebo MoveIt2 Robot Manipulation Example

This package contains a manipulation example for the Cobot using MoveIt2 and Gazebo.

![](vid/zebra_moveit_gz_run.gif)

## Run example from docker container

Clone the repo
```
git clone https://github.com/robgineer/cobot.git .
cd cobot
```

Build the repo
```
source /opt/ros/jazzy/.{bash/zsh} 
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
source install/setup.{bash/zsh}
```

Run the manipulation example
```
ros2 launch cobot_moveit_config gz_demo_launch.py
```
This will open a gazebo / rviz GUI that allows the manipulation of the cobot.

![](../cobot_moveit_config/vid/zebra_moveit_gz_run.gif)

To control the joints in gazebo and rviz manually run the ```rqt_joint_trajectory_controller``` in a separate terminal (don't forget to start an xpra session with a different display
and connect to it from your client machine if you are using xpra).
```
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

## Limitations

Since we are using **position** controllers, we do not have any information on force and hence no option for the interaction with the real world. Changing to **effort** controllers should fix this.
   => Using effort controllers implies the presence of proper effort values in the robot model (URDF). These are yet to be identified.



## References

The example has been created using the[ MoveIt2 setup assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html), the [MoveIt2 tutorials](https://github.com/moveit/moveit2_tutorials) and have been inspired from examples for a [Panda robot](https://github.com/AndrejOrsula/panda_gz_moveit2/tree/jazzy) from [Andrej Orsula](https://github.com/AndrejOrsula) (who seems to be providing one of the first working examples for ros jazzy and gazebo harmonic).
