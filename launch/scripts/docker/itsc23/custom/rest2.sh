#!/bin/bash

cd ..

./run-loc-experiment-docker.sh d8 om d6 static
./run-loc-experiment-docker.sh d8 semi d4 baseline
