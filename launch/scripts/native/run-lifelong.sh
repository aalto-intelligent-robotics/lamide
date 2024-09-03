#!/bin/bash

if [ $3 == "cl" ]
then
    cl=1
else
    cl=0
fi

echo "run experiment run data: $1 map: $2. cluster: $cl"

HOSTNAME=$(hostname)
export HOSTNAME

publish_success=1
startup_success=1

starttime=$(date +"%T")

echo "clearing screen sessions..."
killall screen
echo "killed"

source /opt/ros/melodic/setup.bash
source <path to code>/install/setup.bash
source <path to code>/install/setup.bash

cd ../../localization
screen -dmS base roslaunch base-launch.launch
echo "launched base"
screen -list
sleep 2

pwd
if [ $((cl)) -eq 1 ]
then
    screen -dmS loc roslaunch lifelong-cluster.launch
else
    screen -dmS loc roslaunch lifelong-om.launch
fi
echo "launched loc"
screen -list
sleep 5
screen -list

eval "$(conda shell.zsh hook)"
source ~/miniconda3/etc/profile.d/conda.sh
conda activate bonnetal

cd <path to semantic segmentation>/lidar-bonnetal/train/tasks/semantic

python ros_monitor_main.py -t l
startup_success=$?

if [ $startup_success -eq 0 ]
then
    ./publish-oxford-noisy.sh def def $1 $2

    publish_success=$?
    sleep 5
fi

conda deactivate

source /opt/ros/melodic/setup.bash
source <path to code>/install/setup.bash

screen -XS loc quit
echo "killed loc"
screen -list
sleep 2

screen -XS base quit
echo "killed base"
screen -list
sleep 2

echo "extra kill all screens"
killall screen
