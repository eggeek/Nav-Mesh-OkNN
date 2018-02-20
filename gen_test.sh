for i in `seq 3000 300 9000`; do
  echo "gen inputs/s1/$i.in"
  mesh="./meshes/$i.mesh"
  poly="./polygons/$i.poly"
  obs="./polygons/$i.poly2"
  s1pts="./points/s1-poly$i-pts$i.points"
  fname=inputs/s1/$i.in
  touch $fname
  echo $mesh > $fname
  echo $poly >> $fname
  echo $obs >> $fname
  echo $s1pts >> $fname

  for j in `seq 4 2 10`; do
    echo "gen inputs/s2/$i-$j.in"
    fname=inputs/s2/$i-$j.in
    s2pts="./points/s2-poly$i-pts$j.points"
    touch $fname
    echo $mesh > $fname
    echo $poly >> $fname
    echo $obs >> $fname
    echo $s2pts >> $fname
  done

done
