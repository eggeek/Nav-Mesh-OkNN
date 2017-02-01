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
#include <algorithm>
#include <ctime>

namespace polyanya
{

// "Bonus" contains an additional polygon if p lies on an edge.
PointLocation SearchInstance::get_point_location(Point p)
{
    assert(mesh != nullptr);
    PointLocation out = mesh->get_point_location(p);
    if (out.type == PointLocation::ON_CORNER_VERTEX_AMBIG)
    {
        // Add a few EPSILONS to the point and try again.
        static const Point CORRECTOR = {EPSILON * 10, EPSILON * 10};
        Point corrected = p + CORRECTOR;
        PointLocation corrected_loc = mesh->get_point_location(corrected);

        std::cerr << p << " " << corrected_loc << std::endl;
        return out;

        switch (corrected_loc.type)
        {
            case PointLocation::ON_CORNER_VERTEX_AMBIG:
            case PointLocation::ON_CORNER_VERTEX_UNAMBIG:
            case PointLocation::ON_NON_CORNER_VERTEX:
                std::cerr << "Warning: corrected " << p << " lies on vertex"
                          << std::endl;
            case PointLocation::NOT_ON_MESH:
                std::cerr << "Warning: completely ambiguous point at " << p
                          << std::endl;
                break;

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
    SearchNodePtr parent, std::vector<Successor>& successors, int num_succ
)
{
    assert(mesh != nullptr);
    assert(parent != nullptr);
    const Polygon& polygon = mesh->mesh_polygons[parent->next_polygon];
    const std::vector<int>& V = polygon.vertices, P = polygon.polygons;

    double right_g = -1, left_g = -1;

    for (int i = 0; i < num_succ; i++)
    {
        const Successor& succ = successors[i];
        const int next_polygon = P[succ.poly_left_ind];
        if (next_polygon == -1)
        {
            continue;
        }

        // If the successor we're about to push pushes into a one-way polygon,
        // and the polygon isn't the end polygon, just continue.
        if (mesh->mesh_polygons[next_polygon].is_one_way &&
            next_polygon != end_polygon)
        {
            continue;
        }
        const int left_vertex  = V[succ.poly_left_ind];
        const int right_vertex = succ.poly_left_ind ?
                                 V[succ.poly_left_ind - 1] :
                                 V.back();


        // Note that g is evaluated twice here.
        // Always try to precompute before using this macro.
        // h is fine, though.
        const auto p = [&](const int root, const double g,
                                  const double h)
        {
            if (root != -1)
            {
                assert(root >= 0 && root < (int) root_g_values.size());
                // Can POSSIBLY prune?
                if (root_search_ids[root] != search_id)
                {
                    // First time reaching root
                    root_search_ids[root] = search_id;
                    root_g_values[root] = g;
                }
                else
                {
                    // We've been here before!
                    // Check whether we've done better.
                    if (root_g_values[root] + EPSILON < g)
                    {
                        // We've done better!
                        return;
                    }
                    else
                    {
                        // This is better.
                        root_g_values[root] = g;
                    }
                }
            }
            nodes_generated++;
            open_list.push(new (node_pool->allocate()) SearchNode
                {parent, root, succ.left, succ.right, left_vertex, right_vertex,
                 next_polygon, g+h, g});
        };

        const Point& parent_root = (parent->root == -1 ?
                                    start :
                                    mesh->mesh_vertices[parent->root].p);
        #define get_g(new_root) parent->g + parent_root.distance(new_root)
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
                    p(right_vertex, right_g, succ.right.distance(goal));
                }
                else
                {
                    const double g = get_g(succ.right);
                    p(right_vertex, g, succ.right.distance(goal));
                }
                break;

            case Successor::RIGHT_NON_OBSERVABLE:
                // equivalent to right_g == -1
                if (right_g < -0.5)
                {
                    right_g = get_g(parent->right);
                }
                p(parent->right_vertex, right_g, get_h(parent->right));
                break;

            case Successor::OBSERVABLE:
                p(parent->root, parent->g, get_h(parent_root));
                break;

            case Successor::LEFT_NON_OBSERVABLE:
                if (left_g < -0.5)
                {
                    left_g = get_g(parent->left);
                }
                p(parent->left_vertex, left_g, get_h(parent->left));
                break;

            case Successor::LEFT_COLLINEAR:
                if (succ.left == parent->left)
                {
                    if (left_g < -0.5)
                    {
                        left_g = get_g(parent->left);
                    }
                    // use left_g
                    p(left_vertex, left_g, succ.left.distance(goal));
                }
                else
                {
                    const double g = get_g(succ.left);
                    p(left_vertex, g, succ.left.distance(goal));
                }
                break;

            default:
                assert(false);
                break;
        }
        #undef get_h
        #undef get_g
    }
    nodes_expanded++;
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
    #define get_lazy(next, left, right) new (node_pool->allocate()) SearchNode \
        {nullptr, -1, start, start, left, right, next, h, 0}
    switch (pl.type)
    {
        // Don't bother.
        case PointLocation::NOT_ON_MESH:
            break;

        // Generate all in the polygon.
        case PointLocation::IN_POLYGON:
        case PointLocation::ON_MESH_BORDER:
        // Generate all in an arbirary polygon.
        case PointLocation::ON_CORNER_VERTEX_AMBIG:
        case PointLocation::ON_CORNER_VERTEX_UNAMBIG:
            open_list.push(get_lazy(pl.poly1, -1, -1));
            break;

        case PointLocation::ON_EDGE:
            // Generate all in both polygons except for the shared side.
            open_list.push(get_lazy(pl.poly2, pl.vertex1, pl.vertex2));
            open_list.push(get_lazy(pl.poly1, pl.vertex2, pl.vertex1));
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
                SearchNodePtr dummy_init = get_lazy(poly, -1, -1);
                if (poly == -1)
                {
                    continue;
                }
                if (poly == end_polygon)
                {
                    // Trivial case - we can see the goal from start!
                    final_node = dummy_init;
                    return;
                }
                std::vector<Successor> successors;
                // iterate over poly, throwing away vertices if one of them is
                // pl.vertex1
                const std::vector<int>& vertices =
                    mesh->mesh_polygons[poly].vertices;
                int last_vertex = vertices.back();
                int num_succ = 0;
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
                    num_succ++;
                    last_vertex = vertex;
                }
                push_successors(dummy_init, successors, num_succ);
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
    const clock_t start_time = clock();
    init_search();
    if (mesh == nullptr || end_polygon == -1)
    {
        return false;
    }

    if (final_node != nullptr)
    {
        return true;
    }

    std::vector<Successor> successors;
    successors.resize(mesh->max_poly_sides + 2);
    while (!open_list.empty())
    {
        SearchNodePtr node = open_list.top(); open_list.pop();
        const int next_poly = node->next_polygon;
        if (next_poly == end_polygon)
        {
            search_time = clock() - start_time;
            final_node = node;
            return true;
        }
        // We will never update our root list here.
        const int root = node->root;
        if (root != -1)
        {
            assert(root >= 0 && root < (int) root_g_values.size());
            if (root_search_ids[root] == search_id)
            {
                // We've been here before!
                // Check whether we've done better.
                if (root_g_values[root] + EPSILON < node->g)
                {
                    // We've done better!
                    continue;
                }
            }
        }
        const int num_succ = get_successors(*node, start, *mesh, successors);
        push_successors(node, successors, num_succ);
    }

    return false;
}

void SearchInstance::get_path_points(std::vector<Point>& out)
{
    if (final_node == nullptr)
    {
        return;
    }
    out.clear();
    out.push_back(goal);
    SearchNodePtr cur_node = final_node;

    #define root_to_point(root) mesh->mesh_vertices[root].p

    while (cur_node != nullptr)
    {
        if (root_to_point(cur_node->root) != out.back())
        {
            out.push_back(root_to_point(cur_node->root));
        }
        cur_node = cur_node->parent;
    }
    std::reverse(out.begin(), out.end());
}

void SearchInstance::print_search_nodes(std::ostream& outfile)
{
    if (final_node == nullptr)
    {
        return;
    }
    SearchNodePtr cur_node = final_node;
    while (cur_node != nullptr)
    {
        outfile << *cur_node << std::endl;
        mesh->print_polygon(outfile, cur_node->next_polygon);
        outfile << std::endl;
        cur_node = cur_node->parent;
    }
}

}
