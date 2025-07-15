# Cobot

Development of simulation and control for a pneumatic cobot (provided by [**Festo SE & Co. KG**](https://www.festo.com/)).


## Getting Started

### 1. Clone this Repo

```
mkdir ~/cobot
git clone --recurse-submodules https://github.com/robgineer/cobot.git .
cd ~/cobot
```

### 2. Install docker
The entire dev. environment is based on a docker container. If you running Ubuntu and you don't have docker installed, run the following script.
```
./scripts/docker_installation.sh
```

### 3. Choose your GUI
We offer two different options for displaying the graphical user interface: xpra or VNC. Both are different and have their specific use cases.

* The VNC option enables viewing an entire Ubuntu Desktop within your browser. Useful in case you are running this project on a local machine. Refer [doc/howToVCN.md](doc/howToVNC.md) for details and configuration.
* The xpra option enables forwarding single X11 windows from your terminal. Its a bit more lightweight and useful if you are running this project on a remote machine. It requires your user to be ported into the docker container, however. This might not work on all configurations. Refer [doc/howToXpra.md](doc/howToXpra.md) for details and configuration.


### 4. Build the project

Once you have configured your preferred GUI type, build the project within the docker container using:
```
source /opt/ros/jazzy/setup.{bash/zsh}
cd ~/cobot
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
source install/setup.{bash/zsh}
```
**Note** if you are not connected to the HS-Esslingen VPN, the ```cobot_hardware``` submodule will not be cloned and you will not be able to control the cobot via ROS2. The the hardware submodule, however, is not relevant for the demos.

### 5. Run MoveIt2 Gazebo examples
Check out the [demo](src/demo/README.md) directory and run some demos.

## License
SPDX-License-Identifier: BSD 3-Clause AND Apache-2.0
<br/>
<br/>
While this project is licensed under BSD 3-Clause, the robot model provided is licensed under Apache-2.0 (refer *src/cobot_model* for more details).
