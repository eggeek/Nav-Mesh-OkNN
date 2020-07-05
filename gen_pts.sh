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

cluster() {
  for dir in "${domains[@]}"; do
    mkdir -p $path/points/$dir
    for poly in $path/polys/$dir/*.poly; do
      map=$(basename -- ${poly%.*})
      d=0.01
      cmd="$exc pts $poly $path/meshes/$dir/$map.mesh $d > $path/points/$dir/$map-cluster-$d.pts"
      echo $cmd
      eval $cmd
    done
  done
}

rand() {
  for dir in "${domains[@]}"; do
    mkdir -p $path/points/$dir
    for poly in $path/polys/$dir/*.poly; do
      map=$(basename -- ${poly%.*})
      cmd="$exc query-pts $poly $path/meshes/$dir/$map.mesh $querynum > $path/points/$dir/$map-query.pts"
      echo $cmd
      eval $cmd
      for d in "${density[@]}"; do
        cmd="$exc pts $poly $path/meshes/$dir/$map.mesh $d > $path/points/$dir/$map-$d.pts"
        echo $cmd
        eval $cmd
      done
    done
  done
}

if [ "$1" == "" ]
then
  rand
elif [ "$1" == "rand" ]
then
  rand
elif [ "$1" == "cluster" ]
then 
  cluster
fi
