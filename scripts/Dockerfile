# syntax=docker/dockerfile:1
FROM ubuntu:22.04

# update apt for further downloads
RUN apt-get update

# get zsh and zimfw
RUN apt-get install -y zsh && apt install -y curl && curl -fsSL https://raw.githubusercontent.com/zimfw/install/master/install.zsh | zsh
# install required utilities
RUN apt-get install -y vim

# ssh server is necessary for xpra
RUN apt-get install -y openssh-server

# TODO @rhabach: fix this. xpra requires post install config for the keyboard
# ==> need auto config for US keyboard (options 32 followed by 1)
# could be: DEBIAN_FRONTEND=noninteractive  
#RUN apt-get install -y xpra

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

