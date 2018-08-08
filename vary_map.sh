map="AR0602SR"
cmd="./bin/experiment knn 0.01 5 < ./inputs/${map}.in > ./outputs/maps/${map}.log"
echo $cmd
eval $cmd
#---------------------------
map="brc202d"
cmd="./bin/experiment knn 0.01 5 < ./inputs/${map}.in > ./outputs/maps/${map}.log"
echo $cmd
eval $cmd
#---------------------------
map="CatwalkAlley"
cmd="./bin/experiment knn 0.01 5 < ./inputs/${map}.in > ./outputs/maps/${map}.log"
echo $cmd
eval $cmd
#---------------------------
map="9000"
cmd="./bin/experiment knn 0.01 5 < ./inputs/${map}.in > ./outputs/maps/${map}.log"
echo $cmd
eval $cmd
