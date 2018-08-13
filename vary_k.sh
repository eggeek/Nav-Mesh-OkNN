#!/bin/bash
map="9000"
cmd="./bin/experiment k _ < ./inputs/${map}.in > ./outputs/k/${map}.log"
echo $cmd
eval $cmd

cmd="./bin/experiment dense _ < inputs/9000-cluster.in > outputs/cluster/9000-cluster.log"
echo $cmd
eval $cmd
