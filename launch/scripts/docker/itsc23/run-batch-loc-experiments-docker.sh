#!/bin/bash


if [ -z $1 ]
then
    echo "no map given"
    map=d1
else
    echo "using map $1"
    map=$1
fi

if [ -z $2 ]
then
    echo "no dataset given"
    dataset=d4
else
    echo "using dataset $2"
    dataset=$2
fi

echo "Running all localizations for map $map and dataset $dataset"

ok1=$3 #static static
ok2=$4 #static semi
ok3=$5 #static bl
ok4=$6 #semi static
ok5=$7 #semi semi
ok6=$8 #semi bl
ok7=$9 #om static
ok8=${10} #om semi
ok9=${11} #om bl
ok10=${12} #om combo
ok11=${13} #static combo

while [ $ok1 -eq 1 ] || [ $ok2 -eq 1 ] || [ $ok3 -eq 1 ] || [ $ok4 -eq 1 ] || [ $ok5 -eq 1 ] || [ $ok6 -eq 1 ] || [ $ok7 -eq 1 ] || [ $ok8 -eq 1 ] || [ $ok9 -eq 1 ] || [ $ok10 -eq 1 ] || [ $ok11 -eq 1 ]
do
    echo "ok1 $ok1"
    echo "ok2 $ok2"
    echo "ok3 $ok3"
    echo "ok4 $ok4"
    echo "ok5 $ok5"
    echo "ok6 $ok6"
    echo "ok7 $ok7"
    echo "ok8 $ok8"
    echo "ok9 $ok9"
    echo "ok10 $ok10"
    echo "ok11 $ok11"

    if [ $ok1 -eq 1 ]
    then
        ./run-loc-experiment-docker.sh $map static $dataset static
        ok1=$?
    fi
    if [ $ok2 -eq 1 ]
    then
        ./run-loc-experiment-docker.sh $map static $dataset semi
        ok2=$?
    fi
    if [ $ok3 -eq 1 ]
    then
        ./run-loc-experiment-docker.sh $map static $dataset baseline
        ok3=$?
    fi
    if [ $ok4 -eq 1 ]
    then
        ./run-loc-experiment-docker.sh $map semi $dataset static
        ok4=$?
    fi
    if [ $ok5 -eq 1 ]
    then
        ./run-loc-experiment-docker.sh $map semi $dataset semi
        ok5=$?
    fi
    if [ $ok6 -eq 1 ]
    then
        ./run-loc-experiment-docker.sh $map semi $dataset baseline
        ok6=$?
    fi
    if [ $ok7 -eq 1 ]
    then
        ./run-loc-experiment-docker.sh $map om $dataset static
        ok7=$?
    fi
    if [ $ok8 -eq 1 ]
    then
        ./run-loc-experiment-docker.sh $map om $dataset semi
        ok8=$?
    fi
    if [ $ok9 -eq 1 ]
    then
        ./run-loc-experiment-docker.sh $map om $dataset baseline
        ok9=$?
    fi
    if [ $ok10 -eq 1 ]
    then
        ./run-loc-experiment-docker.sh $map om $dataset combo
        ok10=$?
    fi
    if [ $ok11 -eq 1 ]
    then
        ./run-loc-experiment-docker.sh $map static $dataset combo
        ok11=$?
    fi
done


echo "ok1 $ok1"
echo "ok2 $ok2"
echo "ok3 $ok3"
echo "ok4 $ok4"
echo "ok5 $ok5"
echo "ok6 $ok6"
echo "ok7 $ok7"
echo "ok8 $ok8"
echo "ok9 $ok9"
echo "ok10 $ok10"
echo "ok11 $ok11"
echo "all experiments run succesfully!"




