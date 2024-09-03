#!/bin/bash

cd ..

./run-loc-experiment-docker.sh d8 om d3 semi
./run-loc-experiment-docker.sh d8 om d5 baseline
./run-loc-experiment-docker.sh d8 om d5 static





