for i in `seq 300 300 9000`; do
  ./bin/gen vg polygons/$i.poly meshes/$i.mesh > polygons/$i.poly2
done
