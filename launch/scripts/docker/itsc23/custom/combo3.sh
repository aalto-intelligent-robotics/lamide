#!/bin/bash

cd ..

./run-loc-experiment-docker.sh d8 om d2 combo
./run-loc-experiment-docker.sh d8 om d3 combo
./run-loc-experiment-docker.sh d8 om d4 combo
./run-loc-experiment-docker.sh d8 om d5 combo
./run-loc-experiment-docker.sh d8 om d6 combo
./run-loc-experiment-docker.sh d8 om d7 combo
./run-loc-experiment-docker.sh d8 om d9 combo

./run-loc-experiment-docker.sh d8 static d6 combo
./run-loc-experiment-docker.sh d8 static d7 combo
./run-loc-experiment-docker.sh d8 static d9 combo