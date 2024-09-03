#!/bin/bash

if [ -z $2 ]
then
    d2="-"
else
    d2=$2
fi
./run-all-batch-loc-experiments-docker.sh $1 $d2 0 0 0 1 1 1 0 0 0