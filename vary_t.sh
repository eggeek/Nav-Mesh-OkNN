#!/bin/bash
map="9000"
for ((i=1; i<=1000; i*=10)) {
  r=`python -c "print $i/10000.0"`
  cmd="./bin/experiment t $r < ./inputs/${map}.in > ./outputs/t/${map}-$i.log"
  echo $cmd
  eval $cmd
}
