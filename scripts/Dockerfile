# syntax=docker/dockerfile:1
FROM ubuntu:22.04

# update apt for further downloads
RUN apt-get update

# get zsh and zimfw
# TODO @rharbach: fix "compinit being called at /etc/zsh/zshrc:112"
RUN apt-get install -y zsh && apt install -y curl && curl -fsSL https://raw.githubusercontent.com/zimfw/install/master/install.zsh | zsh

# install required utilities
RUN apt-get install -y vim

# ssh server is necessary for xpra
RUN apt-get install -y openssh-server

# install xpra if required
#ENV DEBIAN_FRONTEND=noninteractive
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
# create dicrectories required for xpra logging
RUN mkdir /run/user/1010 && chown $USERNAME /run/user/$USER_ID \
    && mkdir /run/xpra && chmod 777 /run/xpra

#USER $USERNAME