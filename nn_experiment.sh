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

map="CatwalkAlley"
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

map="AR0602SR"
for i in `seq 1 1 9`
do 
  echo "./bin/experiment nn $i < ./inputs/bgmaps/${map}.in > ./outputs/nn/${map}-$i.log"
  ./bin/experiment nn $i < ./inputs/bgmaps/${map}.in > ./outputs/nn/${map}-$i.log
done

for i in `seq 10 10 100`
do 
  echo "./bin/experiment nn $i < ./inputs/bgmaps/${map}.in > ./outputs/nn/${map}-$i.log"
  ./bin/experiment nn $i < ./inputs/bgmaps/${map}.in > ./outputs/nn/${map}-$i.log
done

map="9000-10"
for i in `seq 1 1 9`
do 
  echo "./bin/experiment nn $i < ./inputs/s2/${map}.in > ./outputs/nn/${map}-$i.log"
  ./bin/experiment nn $i < ./inputs/s2/${map}.in > ./outputs/nn/${map}-$i.log
done

for i in `seq 10 10 100`
do 
  echo "./bin/experiment nn $i < ./inputs/s2/${map}.in > ./outputs/nn/${map}-$i.log"
  ./bin/experiment nn $i < ./inputs/s2/${map}.in > ./outputs/nn/${map}-$i.log
done
