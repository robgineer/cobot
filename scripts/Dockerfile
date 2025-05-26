# syntax=docker/dockerfile:1
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# update apt for further downloads
RUN apt-get update

# install required utilities
RUN apt-get install -y apt-utils software-properties-common x11-apps wget vim curl
# install dev. packages
RUN apt-get install -y git build-essential python3.10 python-is-python3 python3-pip
# install firefox (without snap)
RUN add-apt-repository ppa:mozillateam/ppa
RUN apt-get install -y firefox-esr
# get zsh and zimfw
RUN apt-get install -y zsh && curl -fsSL https://raw.githubusercontent.com/zimfw/install/master/install.zsh | zsh

## install ROS2 Humble ##
# set locales
RUN apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
# set universe repo
RUN add-apt-repository universe
# get ROS2 package sources
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y
# fix broken packages
#RUN apt-get install libbrotli1=1.0.9-2build6 -y --allow-downgrades && apt-get install libfreetype6-dev -y
# install ROS2
RUN apt-get install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
# install ROS2 control
RUN apt-get update
RUN apt-get install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers
# install Gazebo Hamonic
RUN apt-get install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
# install Moveit2
RUN apt-get install -y ros-jazzy-moveit
# install joint manipulator
RUN apt-get install -y ros-jazzy-rqt-joint-trajectory-controller

## install xpra (with latest version) ##
# ssh server is necessary for xpra
RUN apt-get install -y openssh-server
# get latest version of xpra
RUN wget -O "/usr/share/keyrings/xpra.asc" https://xpra.org/xpra.asc \
    && wget -P /etc/apt/sources.list.d/ https://raw.githubusercontent.com/Xpra-org/xpra/master/packaging/repos/noble/xpra.sources \
    && apt-get update \
    && apt-get install -y xpra

####################################### 
## add a first user to the container ##
#######################################
ARG USERNAME=developer
ARG USER_ID=1000
ARG GROUP_ID=$USER_ID
# create the user
RUN groupadd --gid $GROUP_ID $USERNAME \
    && useradd --uid $USER_ID --gid $GROUP_ID -m $USERNAME
# enable sudo with no password (there is no need for that but its a nice to have)
RUN apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers \
    && chmod 0440 /etc/sudoers
# create dicrectories required for xpra logging
RUN mkdir /run/user/$USER_ID && chown $USERNAME /run/user/$USER_ID

#USER $USERNAME
