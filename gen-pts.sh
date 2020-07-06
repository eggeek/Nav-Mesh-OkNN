#!/bin/bash
execute=0
exc="./bin/gen"
path="./input-data"
querynum=1000
density=(0.0001 0.001 0.01 0.1)
domains=(
  "bg512-map"
  "da2-map"
  "dao-map"
  "random-park-tiled"
  "sc1-map"
  "street-map"
  "wc3maps512-map"
)

entity() {
  for dir in "${domains[@]}"; do
    mkdir -p $path/points/$dir
    for poly in $path/polys/$dir/*.poly; do
      map=$(basename -- ${poly%.*})
      cmd="$exc query-pts $poly $path/meshes/$dir/$map.mesh $querynum > $path/points/$dir/$map-query.pts"
      echo "eval \"$cmd\""
      if [ $execute -eq 1 ]; then eval $cmd ; fi
    done
  done
}

cluster() {
  for dir in "${domains[@]}"; do
    mkdir -p $path/points/$dir
    for poly in $path/polys/$dir/*.poly; do
      map=$(basename -- ${poly%.*})
      d=0.01
      cmd="$exc cluster $poly $path/meshes/$dir/$map.mesh $d > $path/points/$dir/$map-cluster-$d.pts"
      echo "eval \"$cmd\""
      if [ $execute -eq 1 ]; then eval $cmd ; fi
    done
  done
}

rand() {
  for dir in "${domains[@]}"; do
    mkdir -p $path/points/$dir
    for poly in $path/polys/$dir/*.poly; do
      map=$(basename -- ${poly%.*})
      for d in "${density[@]}"; do
        cmd="$exc pts $poly $path/meshes/$dir/$map.mesh $d > $path/points/$dir/$map-$d.pts"
        echo "eval \"$cmd\""
        if [ $execute -eq 1 ]; then eval $cmd ; fi
      done
    done
  done
}

case "$1" in
  rand) rand;;
  cluster) cluster;;
  entity) entity;;
  *)
    entity
    rand
    cluster;;
esac
