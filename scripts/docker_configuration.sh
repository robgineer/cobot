#! /bin/bash

echo "  ▗▄▄▖ ▗▄▖ ▗▖  ▗▖▗▄▄▄▖▗▄▖ ▗▄▄▄▖▗▖  ▗▖▗▄▄▄▖▗▄▄▖     ▗▄▄▖ ▗▖ ▗▖▗▄▄▄▖▗▖   ▗▄▄▄  "
echo " ▐▌   ▐▌ ▐▌▐▛▚▖▐▌  █ ▐▌ ▐▌  █  ▐▛▚▖▐▌▐▌   ▐▌ ▐▌    ▐▌ ▐▌▐▌ ▐▌  █  ▐▌   ▐▌  █ "
echo " ▐▌   ▐▌ ▐▌▐▌ ▝▜▌  █ ▐▛▀▜▌  █  ▐▌ ▝▜▌▐▛▀▀▘▐▛▀▚▖    ▐▛▀▚▖▐▌ ▐▌  █  ▐▌   ▐▌  █ "
echo " ▝▚▄▄▖▝▚▄▞▘▐▌  ▐▌  █ ▐▌ ▐▌▗▄█▄▖▐▌  ▐▌▐▙▄▄▖▐▌ ▐▌    ▐▙▄▞▘▝▚▄▞▘▗▄█▄▖▐▙▄▄▖▐▙▄▄▀ "

# Check for additional args (currently only "gpu" is supported)
ARGS="${1:-}"
# set the name for the image and the container
IMAGE_NAME="cobot_noble_image"
CONTAINER_NAME="cobot_container"

# This should be run from script directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
if [ "$PWD" != "$SCRIPT_DIR" ]; then 
    echo -e "\nPlease run $(basename "$0") from 'cobot/scripts' directory!"
    exit
fi

if [ ! -e "../.devcontainer/Dockerfile" ]; then
    echo -e "\nDockerfile not found. Execute this script from a directory with a Dockerfile!"
    exit
fi

# Create the Ubuntu 24.04 image with required configuration (refer Dockerfile for more information)
# and add current user to docker container
echo "============== Build container =============="
docker build --build-arg USERNAME=${USER} --build-arg USER_ID=${UID} --build-arg GROUP_ID=$(id -r -g $UID) -t "$IMAGE_NAME" ../.devcontainer
echo ""

echo "============== Start container =============="
# Create instance of built image 
# bind local home directory to docker container (careful: you have write access to your host home directory),
# run as current user (with corresponding user id and group id)
# takeover X11 and password configs => this way we take over the username / passwords from the host machine
DOCKER_ARGS=(
    -itd \
    --privileged \
    --mount type=bind,src=/home/${USER},dst=/home/${USER} \
    --user=${UID}:${GID} -w /home/${USER} \
    --volume /etc/shadow:/etc/shadow \
    --volume /run/dbus/system_bus_socket:/run/dbus/system_bus_socket \
    --network host \
    --device=/dev/bus/usb:/dev/bus/usb \
    --rm \
    --entrypoint /bin/bash \
    --name "$CONTAINER_NAME" \
)

# Add GPU / CUDA support if requested
if [[ "$ARGS" == "gpu" ]]; then
  DOCKER_ARGS+=(
    --runtime=nvidia \
    --gpus all \
    --volume /usr/local/cuda-12.8:/usr/local/cuda-12.8 \
  )
fi

# Start the container
docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME"
echo ""

# rharbach: deactivated (not required for now)
#echo "============== Start local docker repo =============="
#docker run -d -p 5000:5000 --name local_docker_repo registry:2.7
#echo ""

echo "============== List current containers =============="
docker ps -a # we should see only 1 now (ubuntu container)
echo ""

# TODO @rharbach: tag image to local registry
# TODO @rharbach: commit image in local registry

echo "============== Connect to container =============="
docker exec -it "$CONTAINER_NAME" /bin/bash
