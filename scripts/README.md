# Environment Setup

The entire development environment is based on an Ubuntu 24.04 docker container.
This allows to migrate fast between host systems. In order to build the development environment, run the following scripts:

1. ```scripts/pre-installation.sh```: to run in case you are using a remote cloud server (such as AWS EC2) that requires the creation of a new user
2. ```scripts/docker_installation.sh```: to install docker on host system and to add current user to docker group (this script is obsolete on most cloud severs)
3. ```scripts/docker_configuration.sh```: to build and start the docker image that contains the development environment


## ```docker_installation.sh```
Installs docker and enables non-sudo usage of docker (requires sudo rights for execution).

## ```pre-installation.sh```
This script eases up migration between cloud servers (as AWS EC2 instances) by installing and configuring the initial environment prior to running the docker build (requires sudo rights for execution).
1. Creates new user + adds sudo rights with no password
2. Configures ssh access
3. Installs / configures zsh + zimfw
4. Configures GitHub access (optional)
5. Adds user to docker group (optional)

## ```docker_configuration.sh```

1. Builds the docker container and creates a user with the same username, user id and the group id as the user executing the script. This means your current user will be present in the docker container.
2. Mounts your home directory, exposes `/etc/shadows` (for authentication), `/run/dbus/system_bus_socket` (for xpra communication) and the usb ports.

The container within the same network as the host and in `--priviledged` mode (required for USB access).<br/>

Porting the current user to the docker container is unfortunately essential to enable a seamless xpra integration. Using this setup, you will be able to authenticate in your xpra session with the same user and password as on your host.

## ```build_doc.sh```

Creates a `venv`, downloads the required python packages and creates an `mkdocs` documentation page (on http://127.0.0.1:8000/).