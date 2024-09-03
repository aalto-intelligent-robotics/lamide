#!/bin/bash

starttime=$(date +"%T")

cd ..

# ./run-all-loc-experiments-docker.sh d8 d2
# ./run-all-loc-experiments-docker.sh d8 d3
# ./run-all-loc-experiments-docker.sh d8 d4
./run-all-loc-experiments-docker.sh d8 d5
./run-all-loc-experiments-docker.sh d8 d6
./run-all-loc-experiments-docker.sh d8 d7
./run-all-loc-experiments-docker.sh d8 d9
