#! /bin/bash

echo " ▗▄▄▖ ▗▄▖ ▗▄▄▖  ▗▄▖▗▄▄▄"
echo "▐▌   ▐▌ ▐▌▐▌ ▐▌▐▌ ▐▌ █"
echo "▐▌   ▐▌ ▐▌▐▛▀▚▖▐▌ ▐▌ █"
echo "▝▚▄▄▖▝▚▄▞▘▐▙▄▞▘▝▚▄▞▘ █"
echo ""

echo "#################################################################"
echo "############### Docker Ubuntu 24.04 configuration ###############"
echo "#################################################################"
echo ""

if [ ! -e "Dockerfile" ]; then
    echo -e "\nDockerfile not found. Execute this script from a directory with a Dockerfile!"
    exit
fi

# create the Ubuntu 24.04 image with required configuration (refer Dockerfile for more information)
# and add current user to docker container
echo "============== Build cobot container =============="
docker build --build-arg USERNAME=${USER} --build-arg USER_ID=${UID} --build-arg GROUP_ID=$(id -r -g $UID) -t cobot_noble_image .
echo ""

echo "============== Start cobot container =============="
# Create instance of built image 
# bind local home directory to docker container (careful: you have write access to your host home directory),
# run as current user (with correspinding user id and group id) 
# takeover X11 and password configs => this way we take over the username / passwords from the host machine
# also do some port forwarding
docker run -itd \
            --privileged \
            --mount type=bind,src=/home/${USER},dst=/home/${USER} \
            --user=${UID}:${GID} -w /home/${USER} \
            --volume /tmp/.X11-unix:/tmp/.X11-unix \
            --volume /etc/shadow:/etc/shadow \
            --volume="$HOME/.Xauthority:/home/root/.Xauthority:rw" \
            --volume /run/dbus/system_bus_socket:/run/dbus/system_bus_socket \
            --network host \
            --rm \
            --name cobot_noble_container cobot_noble_image
echo ""

echo "============== Start local docker repo =============="
docker run -d -p 5000:5000 --name cobot_docker_repo registry:2.7
echo ""

echo "============== List current containers =============="
docker ps -a # we should see only 2 now (docker repo and ubuntu container)
echo ""

# TODO @rharbach: tag image to local registry
# TODO @rharbach: commit image in local registry

echo "============== Connect to container =============="
docker exec -it cobot_noble_container /bin/bash
