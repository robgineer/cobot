# syntax=docker/dockerfile:1
FROM tiryoh/ros2-desktop-vnc:jazzy

# update apt for further downloads
RUN apt-get update

# install required utilities
RUN apt-get install -y apt-utils software-properties-common x11-apps wget vim curl
# install dev. packages
RUN apt-get install -y git build-essential python3.10 python-is-python3 python3-pip
# get zsh and zimfw
RUN apt-get install -y zsh && curl -fsSL https://raw.githubusercontent.com/zimfw/install/master/install.zsh | zsh

## set locales for ROS2 Jazzy ##
RUN apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# install ROS2 control
RUN apt-get update
RUN apt-get install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers
# install Gazebo Hamonic ROS2 control
RUN apt-get install -y ros-jazzy-gz-ros2-control
# install Moveit2
RUN apt-get install -y ros-jazzy-moveit ros-jazzy-moveit-py ros-jazzy-moveit-ros-control-interface ros-jazzy-control-msgs
RUN apt-get install -y ros-jazzy-moveit-visual-tools
# install panda config (for reference purposes)
RUN apt-get install -y ros-jazzy-moveit-resources-panda-moveit-config
# install joint manipulator
RUN apt-get install -y ros-jazzy-rqt-joint-trajectory-controller
#ros-jazzy-moveit-ros-move-group-dbgsym (a nice to have for debugging)
# required for MoveIt2 Task Constructor
RUN apt-get install -y ros-jazzy-py-binding-tools
# colcon utils
RUN apt-get install -y python3-colcon-clean python3-colcon-mixin
# realsense drivers
RUN apt-get install -y ros-jazzy-librealsense2*
# pyaudio
RUN apt-get install -y python3-pyaudio

## install xpra (with latest version) ##
RUN wget -O "/usr/share/keyrings/xpra.asc" https://xpra.org/xpra.asc \
    && wget -P /etc/apt/sources.list.d/ https://raw.githubusercontent.com/Xpra-org/xpra/master/packaging/repos/noble/xpra.sources \
    && apt-get update \
    && apt-get install -y xpra

#################################################
## port current user to container if required ##
#################################################
ARG USERNAME=*IAmNotAUser*
ARG USER_ID=1000
ARG GROUP_ID=1000

RUN if [ "$USERNAME" == "runner" ] ; then \
    echo "Running in GitHub Actions, skipping user creation." ; \
elif [ "$USERNAME" != "*IAmNotAUser*" ] ; then \
    # create the user
    groupadd --gid $GROUP_ID $USERNAME && \
    useradd --uid $USER_ID --gid $GROUP_ID -m $USERNAME && \
    apt-get update && apt-get install -y sudo && \
    # enable sudo with no password (there is no need for that but its a nice to have)
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    # create directories required for xpra logging
    mkdir -p /run/user/$USER_ID && chown $USERNAME /run/user/$USER_ID ; \
fi

RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
