# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:melodic-ros-core-bionic

# install bootstrap tools
RUN apt update
RUN apt install -y build-essential
RUN apt install -y python-rosdep
RUN apt install -y python-rosinstall
RUN apt install -y python-vcstools


# bootstrap rosdep
RUN rosdep init
RUN rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt install -y ros-melodic-ros-base=1.4.1-0*
RUN rm -rf /var/lib/apt/lists/*

# install catkin
RUN apt update
RUN apt install -y wget
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt install -y python3-catkin-tools

# install lamide dependencies
RUN apt install -y ros-melodic-pcl-ros
RUN apt install -y ros-melodic-diagnostic-updater
RUN apt install -y ros-melodic-tf-conversions
RUN apt install -y ros-melodic-eigen-conversions
RUN apt install -y ros-melodic-angles
RUN apt install -y ros-melodic-roslint
RUN apt install -y ros-melodic-laser-geometry
RUN apt install -y ros-melodic-cv-bridge
RUN apt install -y ros-melodic-rviz
RUN apt install -y ros-melodic-octomap
RUN apt install -y ros-melodic-octomap-server

RUN apt install -y libyaml-cpp-dev
RUN apt install -y libpcap-dev
RUN apt install -y freeglut3-dev
RUN apt install -y libgsl-dev
RUN apt install -y git

# if want to run mem limit builds, psutil is needed
RUN apt install -y python3.7
RUN rm /usr/bin/python3
RUN ln -s /usr/bin/python3.7 /usr/bin/python3

RUN apt install -y python3-pip
RUN pip3 install --upgrade pip
RUN pip3 install psutil

# install lidar-bonnetal requirements
RUN pip3 install tqdm
RUN pip3 install numpy
RUN pip3 install rospkg
RUN pip3 install opencv-python
RUN pip3 install vispy
RUN pip3 install matplotlib

# install running and basic usage dependencies
RUN apt install -y screen


ENV TERM=xterm-256color
RUN echo "PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;93m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /root/.bashrc
ENV cuda=$CUDA_HOME
WORKDIR $wd