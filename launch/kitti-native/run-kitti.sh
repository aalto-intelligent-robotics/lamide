#!/bin/bash

if [ -z $1 ]
then
    seq="00"
else
    seq=$1
fi

if [ -z $2 ]
then
    run1=1
else
    run1=$2
fi

if [ -z $3 ]
then
    run2=1
else
    run2=$3
fi

if [ -z $4 ]
then
    run3=1
else
    run3=$4
fi

echo "run kitti mapping experiments"

echo "clearing screen sessions..."
killall screen
echo "killed"

hn=$(hostname)
source /opt/ros/melodic/setup.bash
if [ $hn == <local> ]
then
    source <path to code>/install/setup.bash
else
    source <path to code>/devel/setup.bash
fi

cd $seq

if [ $(($run1)) -gt 0 ]
then
    roslaunch kitti-fuse-gt-gt.launch
    sleep 2
fi

if [ $(($run2)) -gt 0 ]
then
    roslaunch kitti-fuse-gt-om.launch
    sleep 2
fi

if [ $(($run3)) -gt 0 ]
then
    roslaunch kitti-fuse-gt-cluster.launch
fi
echo "all done!"