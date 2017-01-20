#include "expansion.h"
#include "searchnode.h"
#include "mesh.h"
#include "geometry.h"
#include "vertex.h"
#include "point.h"
#include "consts.h"
#include <cassert>
#include <vector>

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

// TODO: Wrap this in a class so we don't have to keep passing the same params
// over and over again
// Generates the successors of the search node and appends them to the successor
// vector.
void get_successors(const SearchNode& node, const Point& goal, const Mesh& mesh,
                    std::vector<SearchNode>& successors)
{
    // If the next polygon is -1, we did a bad job at pruning...
    assert(node.next_polygon != -1);

    const Polygon& polygon = mesh.mesh_polygons[node.next_polygon];
    const std::vector<Vertex>& mesh_vertices = mesh.mesh_vertices;
    const int num_vertices = (int) polygon.vertices.size();

    // Is this node collinear?
    // If so, generate all other intervals in the polygon.
    // Assume that the interval is maximal across the edge.
    if (node.left == node.root || node.right == node.root)
    {
        // We can be lazy and start iterating from any point.
        // We still need to exclude the current interval as a successor, though.
        int last_vertex = polygon.vertices.back();

        for (int i = 0; i < num_vertices; i++)
        {
            const int this_vertex = polygon.vertices[i];
            if (this_vertex == node.right_vertex)
            {
                // The interval we're going to generate is the same as our
                // current one, so skip it.
                last_vertex = this_vertex;
                continue;
            }
            const Point& left = mesh_vertices[this_vertex].p,
                         right = mesh_vertices[last_vertex].p;
            const int next_polygon = polygon.polygons[i];
            const double g = node.g,
                         h = get_h_value(node.root, goal, left, right);
            successors.push_back({&node, node.root, left, right, last_vertex,
                                  next_polygon, g + h, g});
            last_vertex = this_vertex;
        }
        return;
    }

    // It is not collinear.
    // Find the starting vertex (the "right" vertex).
    // TODO: Compare this to std::find + const int. Which is faster?
    int cur_vertex_index = 0; // position of vertex in polygon.vertices
    while (polygon.vertices[cur_vertex_index] != node.right_vertex)
    {
        cur_vertex_index++;
        assert(cur_vertex_index < num_vertices);
    }


    // Find whether we can turn at either endpoint.
    const Vertex& right_vertex_obj = mesh_vertices[node.right_vertex],
                  left_vertex_obj  = mesh_vertices[
                                        cur_vertex_index == 0 ?
                                        polygon.vertices.back() :
                                        polygon.vertices[cur_vertex_index - 1]
                                     ];
    const bool can_turn_right = right_vertex_obj.p == node.right &&
                                right_vertex_obj.is_corner,
               can_turn_left  = left_vertex_obj.p == node.left &&
                                left_vertex_obj.is_corner;


    // Set that as our last vertex.
    int last_vertex = node.right_vertex;

    // Increment to get the first segment we're interested in.
    cur_vertex_index++;
    if (cur_vertex_index == num_vertices)
    {
        cur_vertex_index = 0;
    }

    int cur_vertex = polygon.vertices[cur_vertex_index];

    // Set the right endpoint of our next successor to be the last vertex.
    Point successor_right = mesh_vertices[last_vertex].p;

    // Initialise our "line seen" bools.
    bool right_seen = false, left_seen = false;

    #define push_successor(root, left, g) successors.push_back( \
            {&node, root, left, successor_right, last_vertex, \
             polygon.polygons[cur_vertex_index], \
             g + get_h_value(root, goal, left, successor_right), g})

    auto gen_noncollinear_successor = [&](const Point& left)
    {
        // ensure we don't generate a node with a 0-width interval
        if (left == successor_right)
        {
            return;
        }

        Point root;
        double g = node.g;

        // non-observable from left
        if (left_seen)
        {
            if (!can_turn_left)
            {
                // terminate early
                successor_right = left;
                return;
            }
            root = left_vertex_obj.p;
            g += root.distance(node.root);
        }
        // non-observable from right
        else if (!right_seen)
        {
            if (can_turn_right)
            {
                successor_right = left;
                return;
            }
            root = right_vertex_obj.p;
            g += root.distance(node.root);
        }
        // observable
        else
        {
            root = node.root;
        }


        push_successor(root, left, g);
        successor_right = left;
    };

    auto gen_collinear_successor = [&](const int turn_vertex)
    {
        const Vertex& turn_vertex_obj = mesh_vertices[turn_vertex];
        // turn_vertex_obj is ONLY used for root.
        // Assume the endpoints are last_vertex and cur_vertex.
        if (!turn_vertex_obj.is_corner)
        {
            return;
        }
        // Collinear successors sometimes have the same root as before.
        const double g = node.g + (turn_vertex_obj.p == node.root ?
                                   0 :
                                   turn_vertex_obj.p.distance(node.root));
        const Point& cur_p = mesh_vertices[cur_vertex].p;
        push_successor(turn_vertex_obj.p, cur_p, g);
        successor_right = cur_p;
    };


    // Check whether the right vertex is on a vertex.
    if (right_vertex_obj.p == node.right)
    {
        // If so, special case the first line intersection test.
        // Use an orientation check instead.
        const Point& cur_p = mesh_vertices[cur_vertex].p;
        switch (get_orientation(node.root, node.right,
                                cur_p))
        {
            case Orientation::CCW:
                // Counterclockwise, therefore observable.
                // Falls down below to generate successors.
                right_seen = true;
            case Orientation::CW:
                // Clockwise, therefore non-observable.
                // Don't set right_seen.
                gen_noncollinear_successor(cur_p);
                break;
            case Orientation::COLLINEAR:
                // Collinear. We must turn around the left vertex of our current
                // edge, which is also the right vertex of the node.
                // Note that we DON'T set right_seen here as we can catch it
                // later on.
                gen_collinear_successor(last_vertex);
                break;
        }
    }
    while (cur_vertex != node.right_vertex)
    {
        const Point& last_p = mesh_vertices[last_vertex].p,
                     cur_p  = mesh_vertices[cur_vertex].p,
                     root   = node.root;
        // Note that it is possible for two of the if statements below to be
        // executed in the same iteration.

        if (!right_seen)
        {
            // Check whether root-right intersects with the current segment.
            // If so, generate non-observable from successor_right to intersect
            // point, set successor_right to intersect point and set right_seen.
            //
            // If parallel, check whether whether
            // root-right-last_vertex-cur_vertex are collinear. If so, generate
            // the collinear successor, turning at last_vertex.
            //
            // If none of the above, do nothing.
            // Also assert things about the t value when debugging.
            const Point& right = node.right;
            double root_right_num, segment_num, denom;
            line_intersect_time(root, right, last_p, cur_p,
                                root_right_num, segment_num, denom);
            if (denom == 0.0)
            {
                // Check whether total collinear.
                // As we know that they're parallel, only one check is needed.
                if (is_collinear(root, right, last_p))
                {
                    // Collinear successor, turn at last_vertex.
                    gen_collinear_successor(last_vertex);
                }
            }
            else
            {
                // If segment_num / denom is in [0, 1], generate non-observable.
                switch (line_intersect_bound_check(segment_num, denom))
                {
                    case ZeroOnePos::LT_ZERO:
                    case ZeroOnePos::GT_ONE:
                        break;
                    case ZeroOnePos::EQ_ZERO:
                        // Exactly at last_p.
                        // Wait, what? This shouldn't be possible.
                        assert(false); // Intersected last_p in non-observable.
                        break;
                    case ZeroOnePos::IN_RANGE:
                        // In range.
                        // Generate non-observables and set right_seen.
                        gen_noncollinear_successor(get_point_on_line(
                            last_p, cur_p, segment_num / denom));
                        right_seen = true;
                        break;
                    case ZeroOnePos::EQ_ONE:
                        // Exactly at cur_p.
                        // Generate non-observables and set right_seen.
                        gen_noncollinear_successor(cur_p);
                        right_seen = true;
                }
            }
        }
        if (right_seen && !left_seen)
        {
            // Check whether root-left intersects with the current segment.
            // Assert that it's not colinear.
            //
            // If so, generate observable from successor_right to intersect
            // point, set successor_right to intersect point and set left_seen.
            //
            // Else, do nothing.
            // Also assert things about the t value.
            const Point& left = node.left;
            double root_left_num, segment_num, denom;
            line_intersect_time(root, left, last_p, cur_p,
                                root_left_num, segment_num, denom);
            // If segment_num / denom is in [0, 1], generate observable.
            switch (line_intersect_bound_check(segment_num, denom))
            {
                case ZeroOnePos::LT_ZERO:
                case ZeroOnePos::GT_ONE:
                    break;
                case ZeroOnePos::EQ_ZERO:
                    // Exactly at last_p.
                    // Wait, what? This shouldn't be possible.
                    assert(false); // Intersected last_p in observable.
                    break;
                case ZeroOnePos::IN_RANGE:
                    // In range.
                    // Generate observables and set left_seen.
                    gen_noncollinear_successor(get_point_on_line(
                        last_p, cur_p, segment_num / denom));
                    left_seen = true;
                    break;
                case ZeroOnePos::EQ_ONE:
                    // Exactly at cur_p.
                    // Generate observables and set left_seen.
                    gen_noncollinear_successor(cur_p);
                    left_seen = true;
            }
        }
        if (left_seen) // implies right_seen as well
        {
            // Check whether root-left-cur_vertex-last_vertex are collinear.
            // (Check left == cur_vertex to speed things up.)
            // If so, check whether we can turn at cur_vertex. If so,
            // generate the collinear successor and set successor_right to
            // cur_vertex's point.
            const Point& left = node.left;
            if (left == cur_p || is_collinear(root, left, cur_p))
            {
                if (is_collinear(root, cur_p, last_p))
                {
                    // Turn point is at cur_vertex.
                    gen_collinear_successor(cur_vertex);
                }
            }
        }

        // Finally generate successor_right to cur_vertex's point.
        gen_noncollinear_successor(cur_p);

        last_vertex = cur_vertex;
        cur_vertex_index++;
        if (cur_vertex_index == num_vertices)
        {
            cur_vertex_index = 0;
        }
        cur_vertex = polygon.vertices[cur_vertex_index];
    }

}

}
