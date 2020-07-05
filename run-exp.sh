#!/bin/bash
execute=0
exc="./bin/experiment"
path="./input-data"
outto="./outputs"
density=(0.1 0.01 0.001 0.0001)
domains=(
  #"bg512-map"
  "da2-map"
  #"dao-map"
  #"random-park-tiled"
  #"sc1-map"
  #"street-map"
  #"wc3maps512-map"
)

function gen_cmd() {
  exp=$1
  domain=$2
  map=$3
  desc=$4 # *-<density>.pts, *-cluster-<density>.pts
  k=$5

  mkdir -p $outto/$exp
  mkdir -p $path/inputs/$exp

  polypath="$path/polys/$domain/$map.poly"
  meshpath="$path/meshes/$domain/$map.mesh"
  pointpath="$path/points/$domain/$map-$desc.pts"
  querypath="$path/points/$domain/$map-query.pts"
  file="$path/inputs/$exp/$map-$desc.in"

  echo "$meshpath" > $file
  echo "$polypath" >> $file
  echo "$pointpath" >> $file
  echo "$querypath" >> $file

  cmd="$exc $exp $k < $file > $outto/$exp/$map-$desc.log"
  echo $cmd
}

function small() {
  cmd=$(gen_cmd "small" "random-park-tiled" "300" "0.1" 5)
  echo "eval \"$cmd\""
  if [ $execute -eq 1 ]; then eval $cmd ; fi

  cmd=$(gen_cmd "small" "dao-map" "brc502d" "0.1" 5)
  echo "eval \"$cmd\""
  if [ $execute -eq 1 ]; then eval $cmd ; fi

  cmd=$(gen_cmd "small" "street-map" "Berlin_0_256" "0.1" 5)
  echo "eval \"$cmd\""
  if [ $execute -eq 1 ]; then eval $cmd ; fi
}

function random() {
  exp="random"
  for d in "${density[@]}"; do

    for domain in "${domains[@]}"; do
      for mesh in $path/meshes/$domain/*.mesh; do
        map=$(basename -- ${mesh%.*})
        cmd=$(gen_cmd "$exp" "$domain" "$map" "$d")

        echo "eval \"$cmd\""
        if [ $execute -eq 1 ]; then eval $cmd ; fi

      done
    done

    cmd=$(gen_cmd "$exp" "random-park-tiled" "9000" "$d")
    echo "eval \"$cmd\""
    if [ $execute -eq 1 ]; then eval $cmd ; fi
  done
}

function cluster() {
  exp="cluster"
  d=0.01

  for domain in "${domains[@]}"; do
    for mesh in $path/meshes/$domain/*.mesh; do
      map=$(basename -- ${mesh%.*})
      cmd=$(gen_cmd "$exp" "$domain" "$map" "cluster-$d")
      echo "eval \"$cmd\""
      if [ $execute -eq 1 ]; then eval $cmd ; fi
    done
  done

  cmd=$(gen_cmd "$exp" "random-park-tiled" "9000" "cluster-$d")
  echo "eval \"$cmd\""
  if [ $execute -eq 1 ]; then eval $cmd ; fi
}

function nn() {
  exp="nn"
  for d in "${density[@]}"; do
    for domain in "${domains[@]}"; do
      for mesh in $path/meshes/$domain/*.mesh; do
        map=$(basename -- ${mesh%.*})
        cmd=$(gen_cmd "$exp" "$domain" "$map" "$d")
        echo "eval \"$cmd\""
        if [ $execute -eq 1 ]; then eval $cmd ; fi
      done
    done

    cmd=$(gen_cmd "$exp" "random-park-tiled" "9000" "$d")
    echo "eval \"$cmd\""
    if [ $execute -eq 1 ]; then eval $cmd ; fi
  done
}

case "$1" in 
  small) small;;
  random) random;;
  cluster) cluster;;
  nn) nn;;
  *)
    small
    random
    cluster
    nn;;
esac
