#include "searchinstance.h"
#include "expansion.h"
#include "searchnode.h"
#include "successor.h"
#include "vertex.h"
#include "mesh.h"
#include "point.h"
#include "consts.h"
#include <queue>
#include <vector>
#include <cassert>
#include <iostream>

namespace polyanya
{

// "Bonus" contains an additional polygon if p lies on an edge.
PointLocation SearchInstance::get_point_location(Point p)
{
    assert(mesh != nullptr);
    PointLocation out = mesh->get_point_location(p);
    if (out.type == PointLocation::ON_CORNER_VERTEX)
    {
        // Add a few EPSILONS to the point and try again.
        static const Point CORRECTOR = {EPSILON * 10, EPSILON * 10};
        Point corrected = p + CORRECTOR;
        PointLocation corrected_loc = mesh->get_point_location(corrected);

        switch (corrected_loc.type)
        {
            case PointLocation::ON_CORNER_VERTEX:
            case PointLocation::ON_NON_CORNER_VERTEX:
                std::cerr << "Warning: corrected " << p << "lies on a vertex"
                          << std::endl;
            case PointLocation::NOT_ON_MESH:
            {
                // Iterate over the polys of the vertex to see whether there's
                // only one non-traversable on the point.
                bool has_seen = false;
                int set_poly1 = -1;
                for (int& poly : mesh->mesh_vertices[out.vertex1].polygons)
                {
                    if (poly != -1)
                    {
                        if (set_poly1 != -1)
                        {
                            set_poly1 = poly;
                        }
                        continue;
                    }
                    if (has_seen)
                    {
                        set_poly1 = -1;
                        break;
                    }
                    has_seen = true;
                }
                assert(has_seen);
                if (set_poly1 == -1)
                {
                    std::cerr << "Warning: completely ambiguous point at " << p
                              << std::endl;
                }
                else
                {
                    out.poly1 = set_poly1;
                }
                break;
            }

            case PointLocation::IN_POLYGON:
            case PointLocation::ON_MESH_BORDER:
            // Note that ON_EDGE should be fine: any polygon works and there's
            // no need to special case successor generation.
            case PointLocation::ON_EDGE:
                out.poly1 = corrected_loc.poly1;
                break;

            default:
                // Should be impossible to reach.
                assert(false);
                break;
        }
    }
    return out;
}

void SearchInstance::push_successors(
    SearchNodePtr parent, std::vector<Successor>& successors
)
{
    assert(mesh != nullptr);
    assert(parent != nullptr);
    const Polygon& polygon = mesh->mesh_polygons[parent->next_polygon];
    const std::vector<int>& V = polygon.vertices, P = polygon.polygons;

    double right_g = -1, left_g = -1;

    for (Successor& succ : successors)
    {
        const int next_polygon = P[succ.poly_left_ind];
        if (next_polygon == -1)
        {
            continue;
        }
        const int right_vertex = succ.poly_left_ind ?
                                 V[succ.poly_left_ind - 1] :
                                 V.back();


        // Note that g is evaluated twice here.
        // Always try to precompute before using this macro.
        // h is fine, though.
        #define p(root, g, h) open_list.push(SearchNodePtr(new SearchNode \
            {parent, root, succ.left, succ.right, right_vertex, next_polygon, \
             g+h, g}))

        #define get_g(new_root) parent->g + parent->root.distance(new_root)
        #define get_h(new_root) get_h_value(new_root, goal, \
                                            succ.left, succ.right)

        switch (succ.type)
        {
            case Successor::RIGHT_COLLINEAR:
                if (succ.right == parent->right)
                {
                    if (right_g < -0.5)
                    {
                        right_g = get_g(parent->right);
                    }
                    // use right_g
                    p(succ.right, right_g, succ.right.distance(goal));
                }
                else
                {
                    const double g = get_g(succ.right);
                    p(succ.right, g, succ.right.distance(goal));
                }
                break;

            case Successor::RIGHT_NON_OBSERVABLE:
                // equivalent to right_g == -1
                if (right_g < -0.5)
                {
                    right_g = get_g(parent->right);
                }
                p(parent->right, right_g, get_h(parent->right));
                break;

            case Successor::OBSERVABLE:
                p(parent->root, parent->g, get_h(parent->root));
                break;

            case Successor::LEFT_NON_OBSERVABLE:
                if (left_g < -0.5)
                {
                    left_g = get_g(parent->left);
                }
                p(parent->left, left_g, get_h(parent->left));
                break;

            case Successor::LEFT_COLLINEAR:
                if (succ.left == parent->left)
                {
                    if (left_g < -0.5)
                    {
                        left_g = get_g(parent->left);
                    }
                    // use left_g
                    p(succ.left, left_g, succ.left.distance(goal));
                }
                else
                {
                    const double g = get_g(succ.left);
                    p(succ.left, g, succ.left.distance(goal));
                }
                break;

            default:
                assert(false);
                break;
        }
        #undef get_h
        #undef get_g
        #undef p
    }
}

void SearchInstance::set_end_polygon()
{
    // Any polygon is fine.
    end_polygon = get_point_location(goal).poly1;
}

void SearchInstance::gen_initial_nodes()
{
    // {parent, root, left, right, next_polygon, right_vertex, f, g}
    // be VERY lazy and abuse how our function expands collinear search nodes
    // if right_vertex is not valid, it will generate EVERYTHING
    // and we can set right_vertex if we want to omit generating an interval.
    const PointLocation pl = get_point_location(start);
    const double h = start.distance(goal);
    #define get_lazy(next, right) SearchNodePtr( \
        new SearchNode{nullptr, start, start, start, right, next, h, 0})
    switch (pl.type)
    {
        // Don't bother.
        case PointLocation::NOT_ON_MESH:
            break;

        // Generate all in the polygon.
        case PointLocation::IN_POLYGON:
        case PointLocation::ON_MESH_BORDER:
        // Generate all in an arbirary polygon.
        case PointLocation::ON_CORNER_VERTEX:
            open_list.push(get_lazy(pl.poly1, -1));
            break;

        case PointLocation::ON_EDGE:
            // Generate all in both polygons except for the shared side.
            open_list.push(get_lazy(pl.poly1, pl.vertex2));
            open_list.push(get_lazy(pl.poly2, pl.vertex1));
            break;


        case PointLocation::ON_NON_CORNER_VERTEX:
        {
            // The hardest case.
            // Need to MANUALLY generate all the polygons around the point.
            // Will be lazy and generate Successors, not SearchNodes.
            assert(mesh != nullptr);
            // gets the corresponding vertex object from a vertex index
            #define v(vertex) mesh->mesh_vertices[vertex]
            for (int& poly : v(pl.vertex1).polygons)
            {
                SearchNodePtr dummy_init = get_lazy(poly, -1);
                if (poly == -1)
                {
                    continue;
                }
                std::vector<Successor> successors;
                // iterate over poly, throwing away vertices if one of them is
                // pl.vertex1
                const std::vector<int>& vertices =
                    mesh->mesh_polygons[poly].vertices;
                int last_vertex = vertices.back();
                for (int i = 0; i < (int) vertices.size(); i++)
                {
                    const int vertex = vertices[i];
                    if (vertex == pl.vertex1 || last_vertex == pl.vertex1)
                    {
                        last_vertex = vertex;
                        continue;
                    }
                    successors.push_back({Successor::OBSERVABLE, v(vertex).p,
                                          v(last_vertex).p, i});
                    last_vertex = vertex;
                }
                push_successors(dummy_init, successors);
            }
            #undef v
        }
            break;


        default:
            assert(false);
            break;
    }

    #undef get_lazy
}

bool SearchInstance::search()
{
    init_search();
    if (mesh == nullptr || end_polygon == -1)
    {
        return false;
    }

    while (!open_list.empty())
    {
        SearchNodePtr node = open_list.top(); open_list.pop();
        if (node->next_polygon == end_polygon)
        {
            final_node = node;
            return true;
        }
        std::vector<Successor> successors;
        get_successors(*node, *mesh, successors);
        push_successors(node, successors);
    }

    return false;
}

}
