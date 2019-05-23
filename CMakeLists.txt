cmake_minimum_required(VERSION 3.13)
project(nav_mesh_oknn)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-pedantic -Wall -Wno-strict-aliasing -Wno-long-long -Wno-deprecated -Wno-deprecated-declarations")
#set(DEV_CXX_FLAGS "-g -ggdb -O0 -fno-omit-frame-pointer -fsanitize=address")
set(DEV_CXX_FLAGS "-g -ggdb -O0 -fno-omit-frame-pointer")
set(RELEASE_CXX_FLAGS "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${DEV_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${RELEASE_CXX_FLAGS}")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/../bin)


include_directories(EDBT)
include_directories(generator)
include_directories(helpers)
include_directories(index)
include_directories(rstar)
include_directories(search)
include_directories(structs)
include_directories(testcases)

set(SRC
        EDBT/EDBTknn.cpp
        EDBT/EDBTknn.h
        EDBT/EDBTObstacles.h
        generator/genPoints.h
        generator/park2poly.h
        helpers/cfg.cpp
        helpers/cfg.h
        helpers/cpool.h
        helpers/geometry.cpp
        helpers/geometry.h
        helpers/rtree.h
        helpers/scenario.cpp
        helpers/scenario.h
        helpers/timer.cpp
        helpers/timer.h
        index/knnMeshFence.cpp
        index/knnMeshFence.h
        rstar/Data.cpp
        rstar/Data.h
        rstar/RStarTree.cpp
        rstar/RStarTree.h
        rstar/RStarTreeUtil.cpp
        rstar/RStarTreeUtil.h
        rstar/Util2D.h
        search/expansion.cpp
        search/expansion.h
        search/fenceHeuristic.cpp
        search/fenceHeuristic.h
        search/IERPolyanya.cpp
        search/IERPolyanya.h
        search/intervaHeuristic.cpp
        search/intervaHeuristic.h
        search/searchinstance.cpp
        search/searchinstance.h
        search/targetHeuristic.cpp
        search/targetHeuristic.h
        structs/consts.h
        structs/mesh.cpp
        structs/mesh.h
        structs/point.h
        structs/polygon.h
        structs/polygraph.h
        structs/searchnode.h
        structs/successor.h
        structs/vertex.h
        testcases/catch.hpp)

add_executable(gen ${SRC} gen.cpp)
add_executable(experiment ${SRC} experiment.cpp)
add_executable(testing ${SRC} testing.cpp)

find_package(Boost)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(gen ${Boost_LIBRARIES})
    target_link_libraries(experiment ${Boost_LIBRARIES})
    target_link_libraries(testing ${Boost_LIBRARIES})
endif()
