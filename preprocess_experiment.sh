map="brc202d"
for ((i=1; i<10; i+=1)) {
  r=`python -c "print $i/1000.0"`
  cmd="./bin/experiment fence $r cluster < ./inputs/${map}.in > ./outputs/fence/${map}-$i-cluster.log"
  echo $cmd
  eval $cmd
}

for ((i=10; i<=100; i+=10)) {
  r=`python -c "print $i/1000.0"`
  cmd="./bin/experiment fence $r random < ./inputs/${map}.in > ./outputs/fence/${map}-$i.log"
  echo $cmd
  eval $cmd
}
#----------------------------
map="CatwalkAlley"
for ((i=1; i<10; i+=1)) {
  r=`python -c "print $i/1000.0"`
  cmd="./bin/experiment fence $r cluster < ./inputs/${map}.in > ./outputs/fence/${map}-$i-cluster.log"
  echo $cmd
  eval $cmd
}

for ((i=10; i<=100; i+=10)) {
  r=`python -c "print $i/1000.0"`
  cmd="./bin/experiment fence $r random < ./inputs/${map}.in > ./outputs/fence/${map}-$i.log"
  echo $cmd
  eval $cmd
}
#----------------------------
map="AR0602SR"
for ((i=1; i<10; i+=1)) {
  r=`python -c "print $i/1000.0"`
  cmd="./bin/experiment fence $r cluster < ./inputs/${map}.in > ./outputs/fence/${map}-$i-cluster.log"
  echo $cmd
  eval $cmd
}

for ((i=10; i<=100; i+=10)) {
  r=`python -c "print $i/1000.0"`
  cmd="./bin/experiment fence $r random < ./inputs/${map}.in > ./outputs/fence/${map}-$i.log"
  echo $cmd
  eval $cmd
}
#----------------------------
map="9000"
for ((i=1; i<10; i+=1)) {
  r=`python -c "print $i/1000.0"`
  cmd="./bin/experiment fence $r cluster < ./inputs/${map}.in > ./outputs/fence/${map}-$i-cluster.log"
  echo $cmd
  eval $cmd
}

for ((i=10; i<=100; i+=10)) {
  r=`python -c "print $i/1000.0"`
  cmd="./bin/experiment fence $r random < ./inputs/${map}.in > ./outputs/fence/${map}-$i.log"
  echo $cmd
  eval $cmd
}
