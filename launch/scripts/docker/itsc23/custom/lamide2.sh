#!/bin/bash

cd ..

./run-loc-experiment-docker.sh d1 om d4 lamide
./run-loc-experiment-docker.sh d1 om d5 lamide

./run-loc-experiment-docker.sh d8 om d4 lamide
./run-loc-experiment-docker.sh d8 om d5 lamide
./run-loc-experiment-docker.sh d8 om d6 lamide