#!/bin/bash
map="9000"
cmd="bin/experiment k < ./inputs/${map}.in > ./outputs/k/${map}.log"
echo $cmd
eval $cmd
