#!/bin/bash

echo "run experiment $1 $2"

publish_success=1
startup_success=1

starttime=$(date +"%T")

echo "clearing screen sessions..."
killall screen
echo "killed"

source /opt/ros/melodic/setup.bash
source <path to code>/install/setup.bash

cd ../../mapping
screen -dmS base roslaunch base-launch.launch
echo "launched base"
sleep 2
screen -dmS map roslaunch oxfuse-$1-$2.launch
echo "launched map"
sleep 2

eval "$(conda shell.zsh hook)"
source ~/miniconda3/etc/profile.d/conda.sh
conda activate bonnetal

cd <path to semantic segmentation>/lidar-bonnetal/train/tasks/semantic
python ros_monitor_main.py -t m
startup_success=$?

if [ $startup_success -eq 0 ]
then
    ./publish-oxford-and-save-wait.sh def def $1
    publish_success=$?
    sleep 5
fi

conda deactivate

source /opt/ros/melodic/setup.bash
source <path to code>/install/setup.bash


screen -XS map quit
echo "killed map"
sleep 2

screen -XS base quit
echo "killed base"
sleep 2

echo "extra kill all screens"
killall screen

