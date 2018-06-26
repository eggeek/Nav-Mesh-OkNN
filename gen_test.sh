# gen dense
o=9000
echo "gen inputs/s1/$o.in"
mesh="./meshes/$o.mesh"
poly="./polygons/$o.poly"
obs="./polygons/$o.poly2"
s1pts="./points/s1-poly$o-pts$o.points"
fname=inputs/s1/$o.in
touch $fname
echo $mesh > $fname
echo $poly >> $fname
echo $obs >> $fname
echo $s1pts >> $fname

# gen sparse
o=9000
for t in `seq 1 1 15`; do
	echo "gen inputs/s2/$o-$t.in"
	fname=inputs/s2/$o-$t.in
	s2pts="./points/s2-poly$o-pts$t.points"
	touch $fname
	echo $mesh > $fname
	echo $poly >> $fname
	echo $obs >> $fname
	echo $s2pts >> $fname
done
