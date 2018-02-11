for i in `seq 300 300 9000`; do
  ./bin/gen pts polygons/$i.poly2 meshes/$i.mesh $i > points/s1-poly$i-pts$i.points
  for j in `seq 3 1 10`; do
    ./bin/gen pts polygons/$i.poly2 meshes/$i.mesh $j > points/s2-poly$i-pts$j.points
  done
done
