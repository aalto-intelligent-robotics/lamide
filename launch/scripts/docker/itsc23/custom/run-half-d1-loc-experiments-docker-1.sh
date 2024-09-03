#!/bin/bash

starttime=$(date +"%T")

cd ..

./run-all-loc-experiments-docker.sh d1 d2
./run-all-loc-experiments-docker.sh d1 d3
./run-all-loc-experiments-docker.sh d1 d4
