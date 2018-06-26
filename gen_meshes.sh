for i in `seq 300 300 9000`; do
  ./bin/gen pl polygons/raw_au_park.poly 1e5 $i > polygons/$i.poly
  cd utils
  ./bin/poly2mesh < ../polygons/$i.poly > tmp
  sed '1,11d' tmp > ../meshes/$i.mesh
  cd ..
done
