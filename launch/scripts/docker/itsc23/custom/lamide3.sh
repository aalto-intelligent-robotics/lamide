#!/bin/bash

cd ..

./run-loc-experiment-docker.sh d1 om d6 lamide
./run-loc-experiment-docker.sh d1 om d7 lamide
./run-loc-experiment-docker.sh d1 om d9 lamide

./run-loc-experiment-docker.sh d8 om d7 lamide
./run-loc-experiment-docker.sh d8 om d9 lamide