#pragma once
#include <vector>
#include "point.h"

namespace polyanya
{

// A point in the polygon mesh.
struct Vertex
{
    Point p;
    // "int" here means an array index.
    std::vector<int> polygons;

    bool is_corner;
};

}
