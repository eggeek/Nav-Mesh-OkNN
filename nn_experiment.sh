#!/bin/bash
echo "running nn experiment"

map="brc202d-20"
for i in `seq 1 1 9`
do 
  echo "./bin/experiment nn $i < ./inputs/${map}.in > ./outputs/nn/${map}-$i.log"
  ./bin/experiment nn $i < ./inputs/${map}.in > ./outputs/nn/${map}-$i.log
done

for i in `seq 10 10 100`
do 
  echo "./bin/experiment nn $i < ./inputs/${map}.in > ./outputs/nn/${map}-$i.log"
  ./bin/experiment nn $i < ./inputs/${map}.in > ./outputs/nn/${map}-$i.log
done