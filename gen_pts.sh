i=9000
./bin/gen pts polygons/$i.poly2 meshes/$i.mesh $i > points/s1-poly$i-pts$i.points
for j in `seq 1 1 15`; do
	./bin/gen pts polygons/$i.poly2 meshes/$i.mesh $j > points/s2-poly$i-pts$j.points
done
