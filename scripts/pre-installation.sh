#! /bin/bash

# This script handles the following pre-installation steps (required for usage cloud servers): 
# user creation (https://repost.aws/knowledge-center/new-user-accounts-linux-instance)
# ssh access
# zsh + zimfw installation
# GitHub access (optional)
# Adding user to docker group (optional)

if [ "$EUID" -ne 0 ]
  then echo -e "\nPlease run as root\n"
  exit
fi

echo -e  "\n*********** Creating new user ***********"
read -p 'Enter user name: ' username

# make new user sudo without a password
sudo adduser --gecos GECOS --home /home/$username --disabled-password $username
sudo echo "$username ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

echo -e  "\n*********** Activating SSH access ***********"
sudo -u $username mkdir /home/$username/.ssh
sudo -u $username chmod 700 /home/$username/.ssh
sudo -u $username touch /home/$username/.ssh/authorized_keys
sudo -u $username chmod 600 /home/$username/.ssh/authorized_keys
echo -e  "Adding public key. Run 'ssh-keygen -y -f /path_to_key_pair/my-key-pair.pem' and paste result below"
read -p 'Public key: ' public_key
sudo -u $username echo "$public_key" >> /home/$username/.ssh/authorized_keys 
# activate line numbers in vim
sudo -u $username touch /home/$username/.vimrc
sudo -u $username echo "set number" >> /home/$username/.vimrc

echo -e  "\n*********** Installing ZSH and ZimFW ***********"
sudo apt-get update
sudo apt-get install zsh
sudo -u $username wget -nv -O - https://raw.githubusercontent.com/zimfw/install/master/install.zsh | sudo -u $username zsh
# activate preferred theme
sudo -u $username sed -i -e 's/zmodule asciiship/zmodule prompt-pwd \n zmodule eriner/g' /home/$username/.zimrc
# fix issues resulting from multiple compinit calls
sudo -u $username touch /home/$username/.zshenv
sudo -u $username echo "autoload -Uz +X compinit
functions[compinit]=\$'print -u2 \'compinit being called at \'\${funcfiletrace[1]}
'\${functions[compinit]}
skip_global_compinit=1" >> /home/$username/.zshenv

echo -e  "\n*********** GitHub configuration ***********"
read -p "Do you want to configure your GitHub account? [Y/n] " reply_git
 if [[ $reply_git =~ ^[Y|y]$ ]] || [[ -z $reply_git ]]; then
    read -p "GitHub user: " github_user
    read -p "GitHub email: " github_email
    read -p "GitHub key: " github_key
    sudo -u $username touch /home/$username/.gitconfig
    sudo -u $username touch /home/$username/.git-credentials
    sudo -u $username echo -e "[user]\n       name = $github_user\n       email = $github_email" >> /home/$username/.gitconfig
    sudo -u $username echo "https://$github_user:$github_key@github.com" >> /home/$username/.git-credentials
 fi

echo -e  "\n*********** Docker user ***********"
read -p "Do you want to add the user to the docker group? [Y/n] " reply_docker
 if [[ $reply_docker =~ ^[Y|y]$ ]] || [[ -z $reply_docker ]]; then
    sudo usermod -aG docker $username
    echo -e "\Done"
 fi

echo -e  "\n*********** Pre-installation complete ***********"
