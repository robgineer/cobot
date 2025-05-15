# Environment Setup

Docker boilerplate installation.

## ```docker_installation.sh```
Installs docker and enables non-sudo usage of docker (requires sudo rights for execution).

## ```pre-installation.sh```
This script eases up migration between cloud servers (as AWS EC2 instances) by installing and configuring the initial environment prior to running the docker build (requires sudo rights for execution).
1. Creates new user + adds sudo rights with no password
2. Configures ssh access
3. Installs / configures zsh + zimfw
4. Configures GitHub access (optional)
5. Adds user to docker group (optional)

## Dockerfile
The Dockerfile contains the necessary configuration / package installations for the development environment. Summary:
1. uses Ubuntu 24.04 (noble)
2. installs useful utilities as vim, zimfw, openssh, xpra, etc.
3. creates initial user (user executing script) within the docker container
4. enables activation of sudo user without a password

## ```docker_configuration.sh```
Builds the docker image, creates the docker cointainer with required X11 configs / GPU usage and a local docker repo (registry).
