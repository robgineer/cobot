# Artbot

Development of imitation learning for articulated robots


## Getting Started

The entire development environment is based on an Ubuntu 22.04 docker container.
This allows to migrate fast between host systems. In order to build the development environment, run the following scripts:

1. ```scripts/pre-installation.sh```: to run in case you are using a remote cloud server (such as AWS EC2) that requires the creation of a new user
2. ```scripts/docker_installation.sh```: to install docker on host system and to add current user to docker group (this script is obsolete on most cloud severs)
3. ```scripts/docker_configuration.sh```: to build and start the docker image that contains ISAAC Sim

## Simulation using ISAAC Sim

Start docker container as defined in ```scripts/docker_configuration.sh``` (note that all build and run options are required for the execution of ISAAC Sim and xpra).

In the docker container run:
```
export DISPLAY=:100 # or any display number you prefer that is not used
xpra start :100
cd /isaac-sim/
./isaac-sim.sh
```

On client run:
```
xpra attach ssh://<user>@<server>:22/100
```
Where \<server\> represents the address of your remote host that runs the docker container. X11 will be forwarded from the remote host to the docker container.

ISAAC Sim will start in non-headless mode.