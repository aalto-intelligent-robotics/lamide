#!/bin/bash

if [[ "$2" == "-" ]]
then
    for d2 in d2 d3 d4 d5 d6 d7 d9
    do
        ./run-batch-loc-experiments-docker.sh $1 $d2 $3 $4 $5 $6 $7 $8 $9 ${10} ${11}
    done
else
    ./run-batch-loc-experiments-docker.sh $1 $2 $3 $4 $5 $6 $7 $8 $9 ${10} ${11}
fi