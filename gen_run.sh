echo "gen run script for s1."
fname="s1run.sh"
touch $fname
echo "set -v" > $fname
for i in `seq 3000 300 9000`; do
  for j in `seq 4 2 10`; do
    cmd="./bin/experiment s1 $j < ./inputs/s1/$i.in > ./outputs/s1/$i-$j.log"
    echo "$cmd" >> $fname
  done
done

echo "gen run script for s2."
fname="s2run.sh"
touch $fname
echo "set -v" > $fname
for i in `seq 3000 300 9000`; do
  for j in `seq 4 2 10`; do
    for k in `seq 1 1 $j`; do
      cmd="./bin/experiment s2 $k < ./inputs/s2/$i-$j.in > ./outputs/s2/$i-$j-$k.log"
      echo $cmd >> $fname
    done
  done
done
