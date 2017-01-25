#pragma once
#include "point.h"

namespace polyanya
{

// A search node.
// Only makes sense given a mesh and an endpoint, which the node does not store.
// This means that the f value needs to be set manually.
struct SearchNode
{
    const SearchNode* parent;
    // Note that all Points here will be in terms of a Cartesian plane.
    const Point root;

    // If possible, set the orientation of left / root / right to be
    // "if I'm standing at 'root' and look at 'left', 'right' is on my right"
    const Point left, right;

    // The right vertex of the edge the interval is lying on.
    // When generating the successors of this node, start there.
    const int right_vertex;

    // Index of the polygon we're going to "push" into.
    // Every successor must lie within this polygon.
    const int next_polygon;

    const double f, g;

    // Comparison.
    // Always take the "smallest" search node in a priority queue.
    bool operator<(const SearchNode& other) const
    {
        if (this->f == other.f)
        {
            // If two nodes have the same f, the one with the bigger g
            // is "smaller" to us.
            return this->g > other.g;
        }
        return this->g < other.g;
    }

    bool operator>(const SearchNode& other) const
    {
        if (this->f == other.f)
        {
            return this->g < other.g;
        }
        return this->g > other.g;
    }
};

}
