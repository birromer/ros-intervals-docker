#! /bin/bash

# Functions
# TOOD: Check if we can use: getent passwd $USER to extract all variables
# TODO: Check for valid inputs, cause now it will go through even with bad inputs
check_envs () {
    DOCKER_CUSTOM_USER_OK=true;
    if [ -z ${DOCKER_USER_NAME+x} ]; then
        DOCKER_CUSTOM_USER_OK=false;
        return;
    fi

    if [ -z ${DOCKER_USER_ID+x} ]; then
        DOCKER_CUSTOM_USER_OK=false;
        return;
    else
        if ! [ -z "${DOCKER_USER_ID##[0-9]*}" ]; then
            echo -e "\033[1;33mWarning: User-ID should be a number. Falling back to defaults.\033[0m"
            DOCKER_CUSTOM_USER_OK=false;
            return;
        fi
    fi

    if [ -z ${DOCKER_USER_GROUP_NAME+x} ]; then
        DOCKER_CUSTOM_USER_OK=false;
        return;
    fi

    if [ -z ${DOCKER_USER_GROUP_ID+x} ]; then
        DOCKER_CUSTOM_USER_OK=false;
        return;
    else
        if ! [ -z "${DOCKER_USER_GROUP_ID##[0-9]*}" ]; then
            echo -e "\033[1;33mWarning: Group-ID should be a number. Falling back to defaults.\033[0m"
            DOCKER_CUSTOM_USER_OK=false;
            return;
        fi
    fi
}

setup_env_user () {
    USER=$1
    USER_ID=$2
    GROUP=$3
    GROUP_ID=$4

    ## Create user
    useradd -m $USER

    ## add custom commands to zsh
    echo "
     alias t="tmux"
     alias ta="t a -t"    # attach session
     alias tls="t ls"     # list sessions
     alias tn="t new -t"  # new sessiosn
     " >> /root/.zshrc

    ## Copy configs
    cp /root/.profile /home/$USER/
    cp /root/.bashrc /home/$USER/
    cp /root/.Xdefaults /home/$USER/
    cp /root/.zshrc /home/$USER/
    cp /root/.vimrc /home/$USER/
    cp /root/.tmux.conf /home/$USER/

    cp -rf /root/.oh-my-zsh /home/$USER/
    rm -rf /home/$USER/.oh-my-zsh/custom/pure.zsh-theme /home/$USER/.oh-my-zsh/custom/async.zsh
    ln -s /home/$USER/.oh-my-zsh/custom/pure/pure.zsh-theme /home/$USER/.oh-my-zsh/custom/
    ln -s /home/$USER/.oh-my-zsh/custom/pure/async.zsh /home/$USER/.oh-my-zsh/custom/

    # fix eigen folder structure
    ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
    ln -sf /usr/include/eigen3/unsupported /usr/include/unsupported

    sed -i -e 's@ZSH=\"/root@ZSH=\"/home/$USER@g' /home/$USER/.zshrc
    # Copy SSH keys & fix owner
    if [ -d "/root/.ssh" ]; then
        cp -rf /root/.ssh /home/$USER/
        chown -R $USER:$GROUP /home/$USER/.ssh
    fi

    ## Fix owner
    chown $USER:$GROUP /home/$USER
#    chown -R $USER:$GROUP /home/$USER/.config
    chown $USER:$GROUP /home/$USER/.profile
    chown $USER:$GROUP /home/$USER/.bashrc
    chown $USER:$GROUP /home/$USER/.zshrc
    chown $USER:$GROUP /home/$USER/.Xdefaults
    chown -R $USER:$GROUP /home/$USER/.oh-my-zsh

    ## This a trick to keep the evnironmental variables of root which is important!
    echo "if ! [ \"$DOCKER_USER_NAME\" = \"$(id -un)\" ]; then" >> /root/.bashrc
    echo "    cd /home/$DOCKER_USER_NAME" >> /root/.bashrc
    echo "    su $DOCKER_USER_NAME" >> /root/.bashrc
    echo "fi" >> /root/.bashrc

    echo "if ! [ \"$DOCKER_USER_NAME\" = \"$(id -un)\" ]; then" >> /root/.zshrc
    echo "    cd /home/$DOCKER_USER_NAME" >> /root/.zshrc
    echo "    su $DOCKER_USER_NAME" >> /root/.zshrc
    echo "fi" >> /root/.zshrc



    ## Setup Password-file
    PASSWDCONTENTS=$(grep -v "^${USER}:" /etc/passwd)
    GROUPCONTENTS=$(grep -v -e "^${GROUP}:" -e "^docker:" /etc/group)

    (echo "${PASSWDCONTENTS}" && echo "${USER}:x:$USER_ID:$GROUP_ID::/home/$USER:/bin/bash") > /etc/passwd
    (echo "${GROUPCONTENTS}" && echo "${GROUP}:x:${GROUP_ID}:") > /etc/group
    (if test -f /etc/sudoers ; then echo "${USER}  ALL=(ALL)   NOPASSWD: ALL" >> /etc/sudoers ; fi)
}


# ---Main---

# Create new user
## Check Inputs
check_envs

## Determine user & Setup Environment
if [ $DOCKER_CUSTOM_USER_OK == true ]; then
    echo "  -->DOCKER_USER Input is set to '$DOCKER_USER_NAME:$DOCKER_USER_ID:$DOCKER_USER_GROUP_NAME:$DOCKER_USER_GROUP_ID'";
    echo -e "\033[0;32mSetting up environment for user=$DOCKER_USER_NAME\033[0m"
    setup_env_user $DOCKER_USER_NAME $DOCKER_USER_ID $DOCKER_USER_GROUP_NAME $DOCKER_USER_GROUP_ID
else
    echo "  -->DOCKER_USER* variables not set. Using 'root'.";
    echo -e "\033[0;32mSetting up environment for user=root\033[0m"
    DOCKER_USER_NAME="root"
fi

# Change shell to zsh
chsh -s /usr/bin/zsh $DOCKER_USER_NAME

# Run CMD from Docker
tmux
#"$@"
