arg="dev"
if [ $# -gt 0 ]; then
	arg=$1
fi

build_dir="cmake-build-debug"
if [ $arg == "clean" ]; then
	cmd="cd $build_dir && make clean"
	echo $cmd
	eval $cmd
elif [ $arg == "fast" ]; then
	flag="Release"
	cmd="cmake -B$build_dir -H. -DCMAKE_BUILD_TYPE=$flag"
	echo $cmd
	eval $cmd
	cmd2="cd $build_dir && make -j8"
	echo $cmd2
	eval $cmd2
else
	flag="Debug"	
	cmd="cmake -B$build_dir -H. -DCMAKE_BUILD_TYPE=$flag"
	echo $cmd
	eval $cmd
	cmd2="cd $build_dir && make -j8"
	echo $cmd2
	eval $cmd2
fi
