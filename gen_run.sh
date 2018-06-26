echo "gen run script for s1."
fname="s1run.sh"
touch $fname
echo "set -v" > $fname

# experiment 1 dense: |O|=9000, |T|=|O|, k=1
o=9000
k=1
cmd="./bin/experiment s1 $k < ./inputs/s1/$o.in > ./outputs/s1/$o-$k.log"
echo "$cmd" >> $fname

# experiment 2 dense: |O|=9000, |T|=|O|, k=1...10
o=9000
for k in `seq 2 1 10`; do
	cmd="./bin/experiment s1 $k < ./inputs/s1/$o.in > ./outputs/s1/$o-$k.log"
	echo "$cmd" >> $fname
done

echo "gen run script for s2."
fname="s2run.sh"
touch $fname
echo "set -v" > $fname
# experiment 1 sparse: |O|=9000, |T|=5, k=1
o=9000
k=1
t=5
cmd="./bin/experiment s2 $k < ./inputs/s2/$o-$t.in > ./outputs/s2/$o-$t-$k.log"
echo $cmd >> $fname

# experiment 2 sparse: |O|=9000, |T|=10, k=1...10
o=9000
t=10
for k in `seq 1 1 10`; do
	cmd="./bin/experiment s2 $k < ./inputs/s2/$o-$t.in > ./outputs/s2/$o-$t-$k.log"
	echo $cmd >> $fname
done

# experiment 3 sparse: |O|=9000, k=1, t=1...15
o=9000
k=1
for t in `seq 1 1 15`; do
	cmd="./bin/experiment s2 $k < ./inputs/s2/$o-$t.in > ./outputs/s2/$o-$t-$k.log"
	echo $cmd >> $fname
done
