for i in `find ./testcases -mindepth 1 -type d -not -path "./testcases/test-data*"`; do
  testname=$(basename $i)
  for j in `find $i -type f`; do
    cmd="./bin/testing $testname --input $j"
    echo $cmd
    eval $cmd
  done
done
