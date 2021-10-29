FROM ubuntu:20.04

MAINTAINER Bernardo Hummes <hummes@ieee.org>
LABEL Description="ROS Noetic Desktop Intervals Edition (Ubuntu 20.04)" Version="1.2"
LABEL com.turlucode.ros.version="noetic"

# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND noninteractive

# Install packages
RUN apt update && apt install -y \
locales \
net-tools \
mlocate \
lsb-release \
mesa-utils \
git \
gnuplot \
rxvt-unicode \
subversion \
vim \
xterm \
wget \
curl \
htop \
libssl-dev \
build-essential \
fonts-powerline \
dbus-x11 \
tmuxp \
software-properties-common \
g++ \
gcc \
xsltproc \
liblua5.3-0 \
fuse \
flex \
bison \
libeigen3-dev \
build-essential \
pkg-config \
libgtk-3-dev \
libavcodec-dev \
libavformat-dev \
libswscale-dev \
libv4l-dev \
libxvidcore-dev \
libx264-dev  \
libjpeg-dev \
libpng-dev \
libtiff-dev \
gfortran \
openexr \
libatlas-base-dev \
python3-dev \
python3-numpy \
libtbb2 \
libtbb-dev \
libdc1394-22-dev \
fonts-terminus \
fonts-inconsolata \
cmake-gui \
qt5-default \
qtpositioning5-dev \
qml-module-qtquick-controls2 \
qml-module-qtpositioning \
qml-module-qtlocation \
qtcreator \
sqlite3 \
libsqlite3-dev \
libtiff-dev \
libcurl4-openssl-dev \
gdb valgrind \
libpython2.7 && \
apt-get clean && rm -rf /var/lib/apt/lists/*

# Install cmake 3.17.3
RUN git clone https://github.com/Kitware/CMake.git && \
cd CMake && git checkout tags/v3.17.3 && ./bootstrap --parallel=8 && make -j8 && make install && \
cd .. && rm -rf CMake

# Install tmux 3.1
RUN apt-get update && apt-get install -y automake autoconf pkg-config libevent-dev libncurses5-dev bison && \
apt-get clean && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/tmux/tmux.git && \
cd tmux && git checkout tags/3.1 && ls -la && sh autogen.sh && ./configure && make -j8 && make install

# Install new paramiko (solves ssh issues)
RUN apt-add-repository universe
RUN apt-get update && apt-get install -y python3-pip python build-essential && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN /usr/bin/yes | pip3 install --upgrade pip
RUN /usr/bin/yes | pip3 install --upgrade virtualenv
RUN /usr/bin/yes | pip3 install --upgrade paramiko
RUN /usr/bin/yes | pip3 install --ignore-installed --upgrade numpy protobuf
RUN /usr/bin/yes | pip3 install --upgrade setuptools
RUN /usr/bin/yes | pip3 install --no-cache-dir pyibex
RUN /usr/bin/yes | pip3 install --no-cache-dir tubex-lib
RUN /usr/bin/yes | pip3 install --no-cache-dir xmlschema
RUN /usr/bin/yes | pip3 install --no-cache-dir numpy

# Install Tubex and Ibex (for interval computations)
RUN git clone -b develop https://github.com/ibex-team/ibex-lib.git
RUN cd ibex-lib && mkdir build && cd build && \
cmake -DCMAKE_BUILD_TYPE=Debug ..
RUN cd ibex-lib/build && \
make && make install && cd ..

RUN git clone https://github.com/codac-team/codac
RUN cd codac && git submodule init && git submodule update && \
mkdir build && cd build && cmake .. && make && make install && cd ..

# Fix Eigen3 locations for linking
RUN cd /usr/include && \
ln -sf eigen3/Eigen Eigen && \
ln -sf eigen3/unsupported unsupported

# Install OpenCV (for computer vision stuff)
RUN mkdir opencv_build && cd opencv_build && git clone https://github.com/opencv/opencv.git
RUN cd opencv_build && git clone https://github.com/opencv/opencv_contrib.git
RUN cd opencv_build/opencv && mkdir build && cd build && \
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=/opencv_build/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON .. && \
make -j8 && make install

# Install ROOT (for graph plotting)
RUN wget https://root.cern/download/root_v6.24.02.Linux-ubuntu20-x86_64-gcc9.3.tar.gz
RUN tar -xzvf root_v6.24.02.Linux-ubuntu20-x86_64-gcc9.3.tar.gz

# Locale
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Install OhMyZSH
RUN apt update && apt install -y zsh
Run apt clean #&& rm -rf /var/lib/apt/lists/*
RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true
RUN chsh -s /usr/bin/zsh root
RUN git clone https://github.com/sindresorhus/pure /root/.oh-my-zsh/custom/pure
RUN ln -s /root/.oh-my-zsh/custom/pure/pure.zsh-theme /root/.oh-my-zsh/custom/
RUN ln -s /root/.oh-my-zsh/custom/pure/async.zsh /root/.oh-my-zsh/custom/
RUN sed -i -e 's/robbyrussell/refined/g' /root/.zshrc
RUN sed -i '/plugins=(/c\plugins=(git git-flow adb pyenv tmux)' /root/.zshrc

# Install Proj
RUN wget https://download.osgeo.org/proj/proj-8.1.1.tar.gz && \
    tar xfz proj-8.1.1.tar.gz && \
    cd proj-8.1.1 && \
    mkdir build && \
    cd build && \
    cmake .. && cmake --build . && \
    cmake --build . --target install && \
    projsync --system-directory --all

# Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y --allow-downgrades --allow-remove-essential --allow-change-held-packages \
libpcap-dev \
gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev \
ros-noetic-desktop-full python3-rosdep python3-rosinstall-generator python3-vcstool build-essential \
ros-noetic-socketcan-bridge \
ros-noetic-geodesy && \
apt-get clean && rm -rf /var/lib/apt/lists/*

# configuration done with a "robot" user for correct permissions
ENV user "robot"
ENV PATH ${PATH}:/home/${user}/.local/bin

RUN useradd -m -d /home/${user} ${user} && \
    chown -R ${user} /home/${user} && \
    adduser ${user} sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER ${user}
WORKDIR /home/${user}

# Install pip2 for some old stuff
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
RUN python2 get-pip.py
RUN rm get-pip.py

# Configure python2 environment
RUN mkdir envs && \
    cd envs && \
    virtualenv -p /usr/bin/python2.7 py2 && \
    cd py2 && \
    /bin/zsh -c "source bin/activate" && \
    /home/${user}/.local/bin/pip2 install --user numpy && \
    /home/${user}/.local/bin/pip2 install --user opencv-contrib-python && \
    /home/${user}/.local/bin/pip2 install --user Pillow

USER "root"
WORKDIR "/"

# Configure ROS
RUN rosdep init && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.zsh" >> /root/.zshrc
RUN echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.zshrc

# Configure ROOT
RUN echo "source /root/bin/thisroot.sh" >> /root/.bashrc
RUN echo "source /root/bin/thisroot.sh" >> /root/.zshrc

# Copy assets
COPY assets/lambda.zsh-theme /root/.oh-my-zsh/themes/
COPY assets/.tmux.conf /root/.tmux.conf
COPY assets/.vimrc /root/.vimrc

COPY assets/.Xdefaults /root/.Xdefaults
COPY assets/fix_eigen.sh /root/fix_eigen.sh

# Entry script - This will also run rxvt-unicode
COPY assets/entrypoint_setup.sh /
ENTRYPOINT ["/entrypoint_setup.sh"]

# ---
#CMD ["rxvt-unicode"]
