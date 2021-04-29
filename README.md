# ROS Noetic Docker Intervals Edition

Because of the many problems I have face while working with ROS, with the pain that it is to find solutions for them and inspired by [turlucode's repository](https://github.com/turlucode/ros-docker-gui) I ended up making this image with all the software that I use for my research and development.

It includes, among other things, ROS Noetic as it is the core of this thing, the latest opencv build for image processing, ibex and codac for constraint programming and some customization for my own pleasure.

You'll be able to find every extra customization in the *assets/* folder, including vim, tmux, zsh and xdefaults for urxvt terminal.

Most of the details can be understood from the original repo, but I must point that it is a simplified version that does not access a dedicated gaphics card nor has out-of-the-box customizability - gotta edit the files as you wish.

In order to build it you need only to be inside the folder with the Dockerfile and run 
````
docker build -t birromer/ros-noetic:cpu base
````

For ease of use, edited a bit of the previously shown functions and added the following to my .zshrc file
````
ros-env(){
    source /opt/ros/noetic/setup.bash
    source /ros/catkin_ws/devel/setup.bash
    export ROS_PACKAGE_PATH=/ros/catkin_ws/:/opt/ros/noetic/share/
}
ros-env

ros-start(){
docker run --rm -it --privileged --net=host --ipc=host --env="DISPLAY" \
    --device=/dev/dri:/dev/dri \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v $HOME/.Xauthority:/home/$(id -un)/.Xauthority -e XAUTHORITY=/home/$(id -un)/.Xauthority \
    -v /home/$(id -un)/ros:/home/$(id -un)/ros \
    -e DISPLAY=$DISPLAY \
    -e DOCKER_USER_NAME=$(id -un) \
    -e DOCKER_USER_ID=$(id -u) \
    -e DOCKER_USER_GROUP_NAME=$(id -gn) \
    -e DOCKER_USER_GROUP_ID=$(id -g) \
    -e ROS_IP=127.0.0.1 \
    birromer/ros-noetic:cpu #-c "cp /ros/.bashrc /root/.bashrc && bash"
}
  
ros-connect(){
docker exec -ti $(docker ps -aq --filter ancestor=birromer/ros-noetic:cpu --filter status=running)     bash
}
 
ros-clean(){
docker rm $(docker ps -aq --filter ancestor=birromer/ros-noetic:cpu --filter status=exited)
}
````
