#! /bin/bash

echo "  ▗▄▄▖ ▗▄▖ ▗▖  ▗▖▗▄▄▄▖▗▄▖ ▗▄▄▄▖▗▖  ▗▖▗▄▄▄▖▗▄▄▖     ▗▄▄▖ ▗▖ ▗▖▗▄▄▄▖▗▖   ▗▄▄▄  "
echo " ▐▌   ▐▌ ▐▌▐▛▚▖▐▌  █ ▐▌ ▐▌  █  ▐▛▚▖▐▌▐▌   ▐▌ ▐▌    ▐▌ ▐▌▐▌ ▐▌  █  ▐▌   ▐▌  █ "
echo " ▐▌   ▐▌ ▐▌▐▌ ▝▜▌  █ ▐▛▀▜▌  █  ▐▌ ▝▜▌▐▛▀▀▘▐▛▀▚▖    ▐▛▀▚▖▐▌ ▐▌  █  ▐▌   ▐▌  █ "
echo " ▝▚▄▄▖▝▚▄▞▘▐▌  ▐▌  █ ▐▌ ▐▌▗▄█▄▖▐▌  ▐▌▐▙▄▄▖▐▌ ▐▌    ▐▙▄▞▘▝▚▄▞▘▗▄█▄▖▐▙▄▄▖▐▙▄▄▀ "


if [ ! -e "Dockerfile" ]; then
    echo -e "\nDockerfile not found. Execute this script from a directory with a Dockerfile!"
    exit
fi

# create the Ubuntu 22.04 image with required configuration (refer Dockerfile for more information)
# and add current user to docker container
echo "============== Build container =============="
docker build --build-arg USERNAME=${USER} --build-arg USER_ID=${UID} --build-arg GROUP_ID=$(id -r -g $UID) -t artbot_image .
echo ""

echo "============== Start container =============="
# Create instance of built image 
# bind local home directory to docker container (careful: you have write access to your host home directory),
# run as current user (with correspinding user id and group id) 
# takeover X11 and password configs => this way we take over the username / passwords from the host machine
# use GPU drivers of host
docker run -itd \
            --privileged \
            --runtime=nvidia --gpus all \
            --mount type=bind,src=/home/${USER},dst=/home/${USER} \
            --user=${UID}:${GID} -w /home/${USER} \
            --volume /tmp/.X11-unix:/tmp/.X11-unix \
            --volume /etc/shadow:/etc/shadow \
            --volume /run/dbus/system_bus_socket:/run/dbus/system_bus_socket \
            --network host \
            --rm \
            --entrypoint /bin/bash \
            --name artbot_container artbot_image
echo ""

echo "============== Start local docker repo =============="
docker run -d -p 5000:5000 --name local_docker_repo registry:2.7
echo ""

echo "============== List current containers =============="
docker ps -a # we should see only 2 now (docker repo and ubuntu container)
echo ""

# TODO @rharbach: tag image to local registry
# TODO @rharbach: commit image in local registry

echo "============== Connect to container =============="
docker exec -it artbot_container /bin/bash
