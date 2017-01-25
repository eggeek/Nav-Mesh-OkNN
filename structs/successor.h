#pragma once
#include "point.h"

namespace polyanya
{

// A search node.
// Only makes sense given a mesh and an endpoint, which the node does not store.
// This means that the f value needs to be set manually.
enum struct SuccessorType
{
    RIGHT_COLLINEAR,
    RIGHT_NON_OBSERVABLE,
    OBSERVABLE,
    LEFT_NON_OBSERVABLE,
    LEFT_COLLINEAR,
};

struct Successor
{
    SuccessorType type;

    Point left, right;

    // Used to get next_polygon (Polygon.polygons[left_ind])
    // as well as right_vertex (Polygon.vertices[left_ind - 1])
    int poly_left_ind;
};

}
