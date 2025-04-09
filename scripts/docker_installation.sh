#! /bin/bash

echo "##################################################################"
echo "################ Docker Ubuntu 24.04 installation ################"
echo "##################################################################"

if [ "$EUID" -ne 0 ]
  then echo -e "\nPlease run as root"
  exit
fi

current_user=$(logname)

echo -e  "\nInstalling docker"
apt install docker.io -y
echo -e "\ndocker installation complete"

echo -e "\nActivating docker on system startup"
systemctl enable --now docker

#echo -e "\nChecking docker service"
#systemctl status docker

echo -e "\nAdding user to docker group"
usermod -aG docker $current_user
# activate user on group (this spawns a new shell)
sudo -u $current_user -H sh -c "newgrp docker"


