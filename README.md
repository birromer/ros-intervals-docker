# ROS Noetic Docker Intervals Edition

So, installing ROS is quite tiresome and doing it many times is borderline torture, so with this pain in mind and inspired by [turlucode's repository](https://github.com/turlucode/ros-docker-gui) I ended up making this image with all the software that I use for my research and development.

It includes, among other things:
  - Ubuntu 20.04, as base image
  - ROS Noetic, as it is the core of this thing
  - OpenCV, the latest build for image processing
  - Ibex and Codac, for constraint programming
  - Tmux configuration and layout ready for development, for getting into action in less than 10 seconds
  - and some customization for my own pleasure

You'll be able to find everything extra in the *assets/* folder, including vim, tmux, zsh and xdefaults for urxvt terminal.

Most of the details can be understood from the original repo, but I must point that it is a simplified version that does not access a dedicated gaphics card nor has out-of-the-box customizability - gotta edit the files as you wish.

In order to build it you need only to be inside the folder with the Dockerfile and run
````
docker build -t birromer/ros-noetic:cpu base
````

For ease of use, edited a bit of the previously existing functions and added the following to my .zshrc file
````
ros-start(){
docker run --rm -it --privileged --net=host --ipc=host --env="DISPLAY" \
    --device=/dev/dri:/dev/dri \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v $HOME/.Xauthority:/home/$(id -un)/.Xauthority -e XAUTHORITY=/home/$(id -un)/.Xauthority \
    -v /home/$(id -un)/ros:/home/$(id -un)/ros \
    -v /home/$(id -un)/Documents/CoppeliaSim:/home/$(id -un)/ros/CoppeliaSim \
    -e DISPLAY=$DISPLAY \
    -e DOCKER_USER_NAME=$(id -un) \
    -e DOCKER_USER_ID=$(id -u) \
    -e DOCKER_USER_GROUP_NAME=$(id -gn) \
    -e DOCKER_USER_GROUP_ID=$(id -g) \
    -e ROS_IP=127.0.0.1 \
    birromer/ros-noetic:cpu 
}
  
ros-connect(){
docker exec -ti $(docker ps -aq --filter ancestor=birromer/ros-noetic:cpu --filter status=running) bash
}
 
ros-clean(){
docker rm $(docker ps -aq --filter ancestor=birromer/ros-noetic:cpu --filter status=exited)
}
````

After sourcing the your new shell configuration all you gotta do is run `ros-start` and a tmux session will start. 
In case of doubt the command key has been remapped for C-a and the ? still works for help.
