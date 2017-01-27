#include "expansion.h"
#include "searchnode.h"
#include "successor.h"
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

// Internal binary search helper.
// All indices must be within the range [0, 2 * N - 1] to make binary search
// easier. You can normalise an index with this macro:
#define normalise(index) (index) - ((index) >= N ? N : 0)
// Assume that there exists at least one element within the range which
// satisifies the predicate.
template<typename Type, typename Pred>
inline int binary_search(const std::vector<int>& arr, const int N,
                         const std::vector<Type>& objects, int lower, int upper,
                         const Pred pred, const bool is_upper_bound)
{
    if (lower == upper) return lower;
    int best_so_far = -1;
    while (lower <= upper)
    {
        const int mid = lower + (upper - lower) / 2;
        const bool matches_pred = pred(objects[arr[normalise(mid)]]);
        if (matches_pred)
        {
            best_so_far = mid;
        }
        // If we're looking for an upper bound:
            // If we match the predicate, go higher.
            // If not, go lower.
        // If we're looking for a lower bound:
            // If we match the predicate, go lower.
            // If not, go higher.
        if (matches_pred == is_upper_bound)
        {
            // Either "upper bound AND matches pred"
            // or "lower bound AND doesn't match pred"
            // We should go higher, so increase the lower bound.
            lower = mid + 1;
        }
        else
        {
            // The opposite.
            // Decrease the upper bound.
            upper = mid - 1;
        }
    }
    return best_so_far;
}

// TODO: Wrap this in a class so we don't have to keep passing the same params
// over and over again
// Generates the successors of the search node and appends them to the successor
// vector.
void get_successors(SearchNode& node, const Mesh& mesh,
                    std::vector<Successor>& successors)
{
    // If the next polygon is -1, we did a bad job at pruning...
    assert(node.next_polygon != -1);

    const Polygon& polygon = mesh.mesh_polygons[node.next_polygon];
    const std::vector<Vertex>& mesh_vertices = mesh.mesh_vertices;
    // V, P and N are solely used for conciseness
    const std::vector<int>& V = polygon.vertices, P = polygon.polygons;
    const int N = (int) V.size();

    // Is this node collinear?
    // If so, generate all other intervals in the polygon.
    // Assume that the interval is maximal across the edge.
    if (node.left == node.root || node.right == node.root)
    {
        // We can be lazy and start iterating from any point.
        // We still need to exclude the current interval as a successor, though.
        int last_vertex = V.back();

        for (int i = 0; i < N; i++)
        {
            const int this_vertex = V[i];
            if (this_vertex == node.right_vertex)
            {
                // The interval we're going to generate is the same as our
                // current one, so skip it.
                last_vertex = this_vertex;
                continue;
            }
            const Point& left = mesh_vertices[this_vertex].p,
                         right = mesh_vertices[last_vertex].p;
            successors.push_back({Successor::OBSERVABLE, left, right, i});
            last_vertex = this_vertex;
        }
        return;
    }

    assert(get_orientation(node.root, node.left, node.right) ==
           Orientation::CW);

    // It is not collinear.
    // Find the starting vertex (the "right" vertex).

    // Note that "_ind" means "index in V/P",
    // "_vertex" means "index of mesh_vertices".
    // "_vertex_obj" means "object of the vertex" and
    // "_p" means "point".
    const int right_ind = [&]() -> int
    {
        // TODO: Compare to std::find.
        int temp = 0; // position of vertex in V
        while (V[temp] != node.right_vertex)
        {
            temp++;
            assert(temp < N);
        }
        return temp;
    }();
    // Note that left_ind MUST be greater than right_ind.
    // This will make binary searching easier.
    const int left_ind = N + right_ind - 1;



    // Find whether we can turn at either endpoint.
    const Vertex& right_vertex_obj = mesh_vertices[node.right_vertex],
                  left_vertex_obj  = mesh_vertices[V[normalise(left_ind)]];

    const Point& right_p = right_vertex_obj.p,
                 left_p  = left_vertex_obj.p;
    const bool can_turn_right = right_p == node.right &&
                                right_vertex_obj.is_corner,
               can_turn_left  = left_p == node.left &&
                                left_vertex_obj.is_corner;

    // find the transition between non-observable-right and observable.
    // we will call this A, defined by:
    // "first P such that root-right-p is strictly CCW".
    // lower bound is right+1, as root-right-right is not CCW (it is collinear).
    // upper bound is left.
    // the "transition" will lie in the range [A-1, A)

    const Point root_right = node.right - node.root;
    const int A = binary_search(V, N, mesh_vertices, right_ind + 1, left_ind,
        [&root_right, &node](const Vertex& v)
        {
            return root_right * (v.p - node.right) > EPSILON; // STRICTLY CCW.
        }, false
    );
    assert(A != -1);
    const int normalised_A = normalise(A),
              normalised_Am1 = normalise(A-1);

    // Macro for getting a point from a polygon point index.
    #define index2point(index) mesh_vertices[V[index]].p


    const Point& A_p = index2point(normalised_A),
                 Am1_p = index2point(normalised_Am1);
    const Point right_intersect = [&]() -> Point
    {
        double root_right_num, segment_num, denom;
        line_intersect_time(node.root, node.right, Am1_p, A_p,
                            root_right_num, segment_num, denom);
        assert(denom != 0.0);
        assert(root_right_num / denom >= 1 - EPSILON);
        // possibility that t = 0 for segment. if so, use A-1
        if (std::abs(segment_num) < EPSILON)
        {
            return Am1_p;
        }
        // we WILL need to do the division now
        const double t = segment_num / denom;
        assert(t < 1 + EPSILON|| t > -EPSILON);
        return get_point_on_line(Am1_p, A_p, t);
    }();

    // find the transition between observable and non-observable-left.
    // we will call this B, defined by:
    // "first P such that root-left-p is strictly CW".
    // lower-bound is A - 1 (in the same segment as A).
    // upper bound is left.
    // the "transition" will lie in the range (B, B+1]
    const Point root_left = node.left - node.root;
    const int B = binary_search(V, N, mesh_vertices, A - 1, left_ind,
        [&root_left, &node](const Vertex& v)
        {
            return root_left * (v.p - node.left) < -EPSILON; // STRICTLY CW.
        }, true
    );
    assert(B != -1);
    const int normalised_B = normalise(B),
              normalised_Bp1 = normalise(B+1);
    const Point& B_p = index2point(normalised_B),
                 Bp1_p = index2point(normalised_Bp1);
    const Point left_intersect = [&]() -> Point
    {
        double root_left_num, segment_num, denom;
        line_intersect_time(node.root, node.left, Bp1_p, B_p,
                            root_left_num, segment_num, denom);
        assert(denom != 0.0);
        assert(root_left_num / denom >= 1 - EPSILON);
        // possibility that t = 0 for segment. if so, use B+1
        if (std::abs(segment_num) < EPSILON)
        {
            return Bp1_p;
        }
        // we WILL need to do the division now
        const double t = segment_num / denom;
        assert(t < 1 + EPSILON|| t > -EPSILON);
        return get_point_on_line(Bp1_p, B_p, t);
    }();

    // TODO: Check right+2 / left-2 for C/D when the search size exceeds 2.

    // Macro to update this_inde/last_ind.
    #define update_ind() last_ind = cur_ind++; if (cur_ind == N) cur_ind = 0
    if (can_turn_right)
    {
        // Find the last point which makes collinear successors on the right.
        // we will call this C, defined by:
        // "last P such that right-(right+1)-p is collinear"
        // lower-bound is right + 1 (definitely collinear)
        // upper-bound is A-1

        // Right plus one.
        const Point& rightp1_p = mesh_vertices[V[normalise(right_ind + 1)]].p;
        const Point right_rightp1 = rightp1_p - right_p;
        const int C = binary_search(V, N, mesh_vertices, right_ind + 1, A - 1,
            [&right_rightp1, &rightp1_p](const Vertex& v)
            {
                return std::abs(right_rightp1 * (v.p - rightp1_p)) < EPSILON;
            }, true
        );
        // C may be -1.
        // Note that if there exists any number of non-observable successors,
        // there must exist at least one collinear successor (right-rightp1).
        // If C is -1, there does not exist any collinear successors.
        if (C != -1) {
            // Generate right collinear + non-observable.

            // Generate collinear to C.
            // Generate non-observable to Am1.
            // If right_intersect != Am1_p,
            // generate non-observable from Am1 to intersect.
            const int normalised_C = normalise(C);

            // We always generate successors from last_ind to cur_ind.
            // right_ind should always be normalised.
            assert(normalise(right_ind) == right_ind);
            int last_ind = right_ind;
            int cur_ind = normalise(right_ind + 1);

            // Generate collinear.
            // Terminate once we pass C.
            while (last_ind != normalised_C)
            {
                // Generate last-cur, turning at LAST.
                successors.push_back({
                    Successor::RIGHT_COLLINEAR,
                    index2point(cur_ind), index2point(last_ind),
                    cur_ind
                });

                // Update the indices.
                update_ind();
            }

            // Generate non-observable to Am1.
            while (last_ind != normalised_Am1)
            {
                // Generate last-cur, turning at right.
                successors.push_back({
                    Successor::RIGHT_NON_OBSERVABLE,
                    index2point(cur_ind), index2point(last_ind),
                    cur_ind
                });

                update_ind();
            }
            assert(cur_ind == normalised_A);

            // Generate non-observable in Am1-A segment if needed.
            if (right_intersect != Am1_p)
            {
                // Generate Am1-right_intersect, turning at right.
                successors.push_back({
                    Successor::RIGHT_NON_OBSERVABLE,
                    right_intersect, Am1_p,
                    normalised_A
                });
            }
        }
    }

    // Start at Am1.
    // last_node = right_intersect
    // If index is normalised_Bp1, go from last_node to left_intersect.
    // (And terminate too!)
    // Else, go to the end and set that as last_node

    // Special case when there only exists one observable successor.
    // Note that we used the non-normalised indices for this.
    if (A == B + 1)
    {
        assert(normalised_A == normalised_Bp1);
        successors.push_back({
            Successor::OBSERVABLE,
            left_intersect, right_intersect,
            normalised_A // (the same as normalised_Bp1)
        });
    }
    else
    {
        // Generate first (probably non-maximal) successor
        // (right_intersect-A)
        successors.push_back({
            Successor::OBSERVABLE,
            A_p, right_intersect,
            normalised_A
        });

        // Generate all guaranteed-maximal successors.
        // Should generate B-A of them.
        int last_ind = normalised_A;
        int cur_ind = normalise(A+1);

        #ifndef NDEBUG
        int counter = 0;
        #endif

        while (last_ind != normalised_B)
        {
            #ifndef NDEBUG
            counter++;
            #endif

            // Generate last-cur.
            successors.push_back({
                Successor::OBSERVABLE,
                index2point(cur_ind), index2point(last_ind),
                cur_ind
            });

            update_ind();
        }

        #ifndef DEBUG
        assert(counter == B - A);
        #endif

        // Generate last (probably non-maximal) successor
        // (B-left_intersect)
        successors.push_back({
            Successor::OBSERVABLE,
            left_intersect, B_p,
            normalised_Bp1
        });
    }

    if (can_turn_left)
    {
        // Find the last point which makes collinear successors on the left.
        // we will call this D, defined by:
        // "first P such that left-(left-1)-p is collinear"
        // lower-bound is B+1
        // upper-bound is left - 1 (definitely collinear)

        // Right plus one.
        const Point& leftm1_p = mesh_vertices[V[normalise(left_ind - 1)]].p;
        const Point left_leftm1 = leftm1_p - left_p;
        const int D = binary_search(V, N, mesh_vertices, B + 1, left_ind - 1,
            [&left_leftm1, &leftm1_p](const Vertex& v)
            {
                return std::abs(left_leftm1 * (v.p - leftm1_p)) < EPSILON;
            }, false
        );

        // D may be -1.
        // Note that if there exists any number of non-observable successors,
        // there must exist at least one collinear successor (leftm1-left).
        // If D is -1, there does not exist any collinear successors.
        if (D != -1)
        {
            const int normalised_D = normalise(D);
            // If left_intersect != Bp1_p,
            // generate non-observable from left_intersect to Bp1_p.
            // Generate non-observable up to D.
            // Generate collinear up to end.
            if (left_intersect != Bp1_p)
            {
                // Generate left_intersect-Bp1, turning at left.
                successors.push_back({
                    Successor::LEFT_NON_OBSERVABLE,
                    Bp1_p, left_intersect,
                    normalised_Bp1
                });
            }

            int last_ind = normalised_Bp1;
            int cur_ind = normalise(B + 2);

            while (last_ind != normalised_D)
            {
                // Generate last_ind-cur_ind, turning at left.
                successors.push_back({
                    Successor::LEFT_NON_OBSERVABLE,
                    index2point(cur_ind), index2point(last_ind),
                    cur_ind
                });

                update_ind();
            }

            const int normalised_left_ind = normalise(left_ind);
            while (last_ind != normalised_left_ind)
            {
                // Generate last-cur, turning at CUR.
                successors.push_back({
                    Successor::LEFT_COLLINEAR,
                    index2point(cur_ind), index2point(last_ind),
                    cur_ind
                });
                update_ind();
            }
        }

    }

    #undef update_ind

    #undef index_to_point

}

#undef normalise

}
