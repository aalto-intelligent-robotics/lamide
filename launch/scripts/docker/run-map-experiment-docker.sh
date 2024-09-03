#!/bin/bash

echo "run experiment $1 $2"

publish_success=1
startup_success=1

starttime=$(date +"%T")

echo "clearing screen sessions..."
pkill screen
echo "killed"

hn=$(hostname)
source /opt/ros/melodic/setup.bash
if [ $hn == <local> ]
then
    source <path to code>/install/setup.bash
else
    source <path to code>/devel/setup.bash
fi

cd ../../mapping
screen -dmS base roslaunch docker-base-launch.launch
echo "launched base"
sleep 2
screen -dmS map roslaunch oxfuse-$1-$2.launch
echo "launched map"
sleep 2

# eval "$(conda shell.zsh hook)"
# source ~/miniconda3/etc/profile.d/conda.sh
# conda activate bonnetal

cd <path to semantic segmentation>/lidar-bonnetal/train/tasks/semantic
python ros_monitor_main.py -t m
startup_success=$?

if [ $startup_success -eq 0 ]
then
    ./publish-oxford-and-save-wait.sh def def $1
    publish_success=$?
    sleep 5
fi

# conda deactivate

source /opt/ros/melodic/setup.bash
if [ $hn == <local> ]
then
    source <path to code>/install/setup.bash
else
    source <path to code>/devel/setup.bash
fi

screen -XS map quit
echo "killed map"
sleep 2

screen -XS base quit
echo "killed base"
sleep 2

echo "extra kill all screens"
pkill screen


now=$(date +"%T")
hn=$(hostname)
if [ $startup_success -eq 0 ] && [ $publish_success -eq 0 ]
then
    echo "succesfully done!"
    exit 0
else
    echo "script error. exit."
    exit 1
fi

