#!/bin/bash

echo "run experiment $1 $2 $3 $4"

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
cd normal
cd $1
cd $3
screen -dmS loc roslaunch oxloc-$2-$4.launch
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
    ./publish-oxford-noisy.sh def def $3 $1

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

hn=$(hostname)
now=$(date +"%T")

fn=$(find <path to maps>/experiment2/log -name "$1*$2*$3*$4*log*summary")
sep=$'\n'

echo "found $fn"

arrIN=(${fn//$sep/ })
IFS=' ' read -r -a ADDR <<< "${arrIN}"

textbody=''
NEWLINE=$'\n'

for i in "${ADDR[@]}"; do
  # process "$i"
  body=$(cat $i)
  textbody=$textbody$body${NEWLINE}${NEWLINE}
done

if [ $startup_success -eq 0 ] && [ $publish_success -eq 0 ]
then
    #Notify with email
    echo "succesfully done!"
    exit 0
else
    echo "script error. exit."
    exit 1
fi
