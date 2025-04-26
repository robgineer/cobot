# Environment Setup

Docker boilerplate installation. Enables running ISAAC Sim out of a docker container with xpra.

## ```docker_installation.sh```
Installs docker and enables non-sudo usage of docker. Has to be run with sudo right.

## Dockerfile
The Dockerfile contains the necessary configuration / package installations for the development environment. Summary:<br/>
1. uses the ISAAC Sim docker image (based on Ubuntu 22.04)
2. contains useful utilities as vim, zimfw, openssh, xpra, etc. <br/>
3. creates initial user (user executing script) within the docker container <br/>
4. enables activation of sudo user without a password

## ```docker_configuration.sh```
Builds the docker image, creates the docker cointainer with required X11 configs / GPU usage and a local docker repo (registry).


## Requirements
In order to run ISAAC Sim on the container, we need to have access to the host GPU => install Nvidia Container tools

```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
    && \
    sudo apt-get update

sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```
Source: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html