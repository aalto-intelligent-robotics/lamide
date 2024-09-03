#!/bin/bash

cd ..

./run-loc-experiment-docker.sh d1 static d2 combo
./run-loc-experiment-docker.sh d1 static d3 combo
./run-loc-experiment-docker.sh d1 static d4 combo
./run-loc-experiment-docker.sh d1 static d5 combo
./run-loc-experiment-docker.sh d1 static d6 combo
./run-loc-experiment-docker.sh d1 static d7 combo
./run-loc-experiment-docker.sh d1 static d9 combo

./run-loc-experiment-docker.sh d8 static d4 combo
./run-loc-experiment-docker.sh d8 static d5 combo
