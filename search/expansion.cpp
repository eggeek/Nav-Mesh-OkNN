#include "expansion.h"
#include "geometry.h"
#include "point.h"
#include "consts.h"
#include <cassert>

namespace polyanya
{

// Gets the h value of a search node with interval l-r and root "root",
// given a goal.
double get_h_value(const Point& root, Point goal,
                   const Point& l, const Point& r)
{
    // First, check whether goal and root are on the same side of the interval.
    // If either are collinear with r/l, reflecting does nothing.
    const Point lr = r - l;
    if (((root - l) * lr > 0) == ((goal - l) * lr > 0))
    {
        // Need to reflect.
        goal = reflect_point(goal, l, r);
    }
    // Now we do the actual line intersection test.
    double rg_num, lr_num, denom;
    line_intersect_time(root, goal, l, r, rg_num, lr_num, denom);

    // Check the edge case first.
    if (denom == 0.0)
    {
        // This is surprisingly tricky!
        // We need to order root, goal, l and r.
        // However, we should be guaranteed that the root is either one of l/r,
        // so we just take the straight line distance.
        assert(root == l || root == r); // May assert false when testing...
        return root.distance(goal);
    }

    // Assuming that root, goal, l and r are not on the same line...

    #ifndef NDEBUG
    // Double check that the goal and root are on different sides.
    const ZeroOnePos rg_pos = line_intersect_bound_check(rg_num, denom);
    assert(rg_pos != ZeroOnePos::LT_ZERO && rg_pos != ZeroOnePos::GT_ONE);
    #endif

    // Check what place on the interval the line intersects.
    const ZeroOnePos lr_pos = line_intersect_bound_check(lr_num, denom);
    switch (lr_pos)
    {
        case ZeroOnePos::LT_ZERO:
            // Too far left.
            // Use left end point.
            return root.distance(l) + l.distance(goal);

        case ZeroOnePos::EQ_ZERO:
        case ZeroOnePos::IN_RANGE:
        case ZeroOnePos::EQ_ONE:
            // Line goes through interval, so just use the direct distance.
            return root.distance(goal);

        case ZeroOnePos::GT_ONE:
            // Too far right.
            // Use right end point.
            return root.distance(r) + r.distance(goal);

        default:
            // Unreachable.
            assert(false);
            return -1;
    }
}

}
