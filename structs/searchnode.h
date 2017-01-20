#pragma once
#include "point.h"

namespace polyanya
{

// A search node.
// Only makes sense given a mesh and an endpoint, which the node does not store.
// This means that the f value needs to be set manually.
struct SearchNode
{
    SearchNode* parent;
    // Note that all Points here will be in terms of a Cartesian plane.
    Point root;

    // If possible, set the orientation of left / root / right to be
    // "if I'm standing at 'root' and look at 'left', 'right' is on my right"
    Point left, right;

    // Index of the polygon we're going to "push" into.
    int next_polygon;

    double f, g;
};

}
