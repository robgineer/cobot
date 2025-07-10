# Running xpra sessions from from docker container

The docker container will enable X11 forwarding to client machines using xpra. This is particularly useful if the container runs on a remote server.<br/>
Note: although SSH access and X11 forwarding within docker container is (sometimes) referred to as an "anti-pattern" (apparently this implies treating the docker container like a VM), we believe that this solution is the way-to-go for containers hosted on remote servers that contains all required dependencies for the development of this project.


## 1. Install xpra on host and client system

In order for xpra to forward X11 sessions from the container, you need to have xpra installed on your host system (where your docker container will be running). If your host system is Ubuntu, simply use the following code snippet to download and install xpra. If not, download it from [https://github.com/Xpra-org/xpra/](https://github.com/Xpra-org/xpra/).

```
sudo wget -O "/usr/share/keyrings/xpra.asc" https://xpra.org/xpra.asc && \
    sudo wget -P /etc/apt/sources.list.d/ https://raw.githubusercontent.com/Xpra-org/xpra/master/packaging/repos/$(lsb_release -sc)/xpra.sources && \
    sudo apt-get update && \
    sudo apt-get install -y xpra
```

Dont forget to install xpra on your local client machine as well.

## 2. Build the docker image

After successfully having cloned this repo, run the `docker_configuration` script.
```
cd ~/cobot/scripts
./docker_configuration.sh
```

The configuration script contains the following steps:
1. Builds the docker container and creates a user with the same username, user id and the group id as the user executing the script. This means your current user will be present in the docker container.
2. Mounts your home directory, exposes `/etc/shadows` (for authentication), `/run/dbus/system_bus_socket` (for xpra communication) and the usb ports.

The container within the same network as the host and in `--priviledged` mode (required for USB access).<br/>

Porting the current user to the docker container is unfortunately essential to enable a seamless xpra integration. Using this setup, you will be able to authenticate in your xpra session with the same user and password as on your host.

### Troubleshooting
If building the docker container fails due to the following error:

```
0.276 groupadd: GID '1000' already exists
```

then you are probably running on host that has only one user (your user) and the user id is the same as the `ubuntu` user (uid=1000) created within the docker build. Unfortunately we could not find elegant solutions for this issue. Some options are creating a different user (`uid!=1000`) and re-running the `docker_configuration` script or deleting the `ubuntu` user within the dockerfile. As mentioned above, it is absolutely necessary to port your current user into the docker container (otherwise xpra authentications will fail). If you cannot create a new user on your machine, kindly check out the [VCP option](howToVCN.md).

## 3. Run X11 sessions from your terminal

In the docker container run:
```
export DISPLAY=:100 # or any display number you prefer that is not used
xpra start :100
```

On client run:
```
xpra attach ssh://<user>@<server>:22/100
```
Where \<server\> represents the address of your remote host that runs the docker container. X11 will be forwarded from the docker container via the host to your client.

All graphical user interfaces started from the terminal of the docker container will be forwarded to the client machine. <br/>

You can run multiple xpra sessions from the same container but different terminals. Simply change the DISPLAY number for different sessions.




