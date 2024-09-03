#!/bin/bash

for i in {1..20}
do
  ./run-batch-loc-experiments-docker.sh $1 $2 $3 $4 $5 $6 $7 $8 $9 ${10} ${11}
done