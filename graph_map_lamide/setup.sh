#!/bin/bash

if [ $# -eq 0 ]
then
    # No arguments supplied
    ubuntu_version_name="xenial"
    ros_version="kinetic"

elif [ $# -eq 2 ]
then
    ubuntu_version_name=${1}
    ros_version=${2}

else
    echo "ERROR: Wrong usage"
    echo "Usage: $0 ubuntu_code_name ros_code_name eg. $0 xenial kinetic"
    exit 1

fi

echo "Installing Dependencies (Ubuntu ${ubuntu_version_name} using ROS ${ros_version})..."
echo "If you want to change this you can pass in the codename of the Ubuntu release. Eg. $0 xenial kinetic for 16.04 with ROS kinetic"

if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]; then
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu '${ubuntu_version_name}' main" > /etc/apt/sources.list.d/ros-latest.list'
fi

wget http://packages.ros.org/ros.key -O - | apt-key add -
apt-get update -qq > /dev/null
apt-get install -qq -y python-rosdep build-essential cmake > /dev/null
rosdep init > /dev/null

rosdep update 2> /dev/null
rosdep install -q --from-paths . -i -y -r --rosdistro ${ros_version} 2> /dev/null
