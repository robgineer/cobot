# Environment Setup

Docker boilerplate installation.

```docker_installation.sh```: installs docker and enables non-sudo usage of docker. Has to be run with sudo right.
<br/>

```docker_configuration.sh```: installs docker and enables non-sudo usage of docker. Has to be run with sudo right.

Use ```sudo chmod u+w docker_installation.sh``` / ```docker_configuration.sh``` to make the scripts executable.

## Dockerfile
The Dockerfile contains the necessary configuration / package installations for the cobot environment. Summary:<br/>
1. installation of useful utilities as vim, zimfw, openssh, etc. <br/>
2. creation of initial user (user executing script) within the docker container. <br/>
3. activation of sudo user without a password.

## ```docker_configuration.sh```

Builds the docker image, creates the docker cointainer with required X11 configs and a local docker repo (registry).