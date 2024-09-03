#!/bin/bash

if [ -z $2 ]
then
    for d2 in d2 d3 d4 d5 d6 d7 d9
    do
        ./run-all-static-loc-experiments-docker.sh $1 $d2
        ./run-all-semi-loc-experiments-docker.sh $1 $d2
        ./run-all-om-loc-experiments-docker.sh $1 $d2
    done
else
    ./run-all-static-loc-experiments-docker.sh $1 $2
    ./run-all-semi-loc-experiments-docker.sh $1 $2
    ./run-all-om-loc-experiments-docker.sh $1 $2
fi