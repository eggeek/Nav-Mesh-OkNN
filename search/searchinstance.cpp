#include "searchinstance.h"
#include "expansion.h"
#include "geometry.h"
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

int SearchInstance::succ_to_node(
    SearchNodePtr parent, Successor* successors, int num_succ,
    SearchNodePtr* nodes
)
{
    assert(mesh != nullptr);
    assert(parent != nullptr);
    const Polygon& polygon = mesh->mesh_polygons[parent->next_polygon];
    const std::vector<int>& V = polygon.vertices;
    const std::vector<int>& P = polygon.polygons;

    double right_g = -1, left_g = -1;

    int out = 0;
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


        // Note that g is evaluated twice here. (But this is a lambda!)
        // Always try to precompute before using this macro.
        // We implicitly set h to be zero and let search() update it.
        const auto p = [&](const int root, const double g)
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
            nodes[out++] = new (node_pool->allocate()) SearchNode(
                {parent, root, succ.left, succ.right, left_vertex, right_vertex,
                 next_polygon, g, g});
        };

        const Point& parent_root = (parent->root == -1 ?
                                    start :
                                    mesh->mesh_vertices[parent->root].p);
        #define get_g(new_root) parent->g + parent_root.distance(new_root)
        switch (succ.type)
        {
            case Successor::RIGHT_COLLINEAR:
                if (succ.right == parent->right)
                {
                    if (right_g == -1)
                    {
                        right_g = get_g(parent->right);
                    }
                    // use right_g
                    p(right_vertex, right_g);
                }
                else
                {
                    const double g = get_g(succ.right);
                    p(right_vertex, g);
                }
                break;

            case Successor::RIGHT_NON_OBSERVABLE:
                // equivalent to right_g == -1
                if (right_g == -1)
                {
                    right_g = get_g(parent->right);
                }
                p(parent->right_vertex, right_g);
                break;

            case Successor::OBSERVABLE:
                p(parent->root, parent->g);
                break;

            case Successor::LEFT_NON_OBSERVABLE:
                if (left_g == -1)
                {
                    left_g = get_g(parent->left);
                }
                p(parent->left_vertex, left_g);
                break;

            case Successor::LEFT_COLLINEAR:
                if (succ.left == parent->left)
                {
                    if (left_g == -1)
                    {
                        left_g = get_g(parent->left);
                    }
                    // use left_g
                    p(left_vertex, left_g);
                }
                else
                {
                    const double g = get_g(succ.left);
                    p(left_vertex, g);
                }
                break;

            default:
                assert(false);
                break;
        }
        #undef get_h
        #undef get_g
    }

    return out;
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
        {
            SearchNodePtr lazy = get_lazy(pl.poly1, -1, -1);
            if (verbose)
            {
                std::cerr << "generating init node: ";
                print_node(lazy, std::cerr);
                std::cerr << std::endl;
            }
            open_list.push(lazy);
        }
            nodes_generated++;
            nodes_pushed++;
            break;

        case PointLocation::ON_EDGE:
            // Generate all in both polygons except for the shared side.
        {
            SearchNodePtr lazy1 = get_lazy(pl.poly2, pl.vertex1, pl.vertex2);
            SearchNodePtr lazy2 = get_lazy(pl.poly1, pl.vertex2, pl.vertex1);
            if (verbose)
            {
                std::cerr << "generating init node: ";
                print_node(lazy1, std::cerr);
                std::cerr << std::endl;
                std::cerr << "generating init node: ";
                print_node(lazy2, std::cerr);
                std::cerr << std::endl;
            }
            open_list.push(lazy1);
            open_list.push(lazy2);
        }
            nodes_generated += 2;
            nodes_pushed += 2;
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
                    if (verbose)
                    {
                        std::cerr << "got a trivial case!" << std::endl;
                    }
                    return;
                }
                // iterate over poly, throwing away vertices if one of them is
                // pl.vertex1
                const std::vector<int>& vertices =
                    mesh->mesh_polygons[poly].vertices;
                Successor* successors = new Successor [vertices.size()];
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
                    successors[num_succ++] =
                        {Successor::OBSERVABLE, v(vertex).p,
                         v(last_vertex).p, i};
                    last_vertex = vertex;
                }
                SearchNodePtr* nodes = new SearchNodePtr [num_succ];
                const int num_nodes = succ_to_node(dummy_init, successors,
                                                   num_succ, nodes);
                delete[] successors;
                for (int i = 0; i < num_nodes; i++)
                {
                    if (verbose)
                    {
                        std::cerr << "generating init node: ";
                        print_node(nodes[i], std::cerr);
                        std::cerr << std::endl;
                    }
                    open_list.push(nodes[i]);
                }
                delete[] nodes;
                nodes_generated += num_nodes;
                nodes_pushed += num_nodes;
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
    timer.start();
    init_search();
    if (mesh == nullptr || end_polygon == -1)
    {
        timer.stop();
        return false;
    }

    if (final_node != nullptr)
    {
        timer.stop();
        return true;
    }

    Successor* successors = new Successor [mesh->max_poly_sides + 2];
    SearchNodePtr* nodes_to_push = new SearchNodePtr [mesh->max_poly_sides + 2];
    while (!open_list.empty())
    {
        SearchNodePtr node = open_list.top(); open_list.pop();

        if (verbose)
        {
            std::cerr << "popped off: ";
            print_node(node, std::cerr);
            std::cerr << std::endl;
        }

        nodes_popped++;
        const int next_poly = node->next_polygon;
        if (next_poly == end_polygon)
        {
            timer.stop();

            if (verbose)
            {
                std::cerr << "found end - terminating!" << std::endl;
            }

            final_node = node;
            delete[] successors;
            delete[] nodes_to_push;
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
                    nodes_pruned_post_pop++;

                    if (verbose)
                    {
                        std::cerr << "node is dominated!" << std::endl;
                    }

                    // We've done better!
                    continue;
                }
            }
        }
        int num_nodes = 1;
        nodes_to_push[0] = node;

        // We use a do while here because the first iteration is guaranteed
        // to work.
        do
        {
            if (verbose)
            {
                std::cerr << "\tintermediate: ";
                print_node(nodes_to_push[0], std::cerr);
                std::cerr << std::endl;
            }

            SearchNodePtr cur_node = nodes_to_push[0];
            // don't forget this!!!
            if (cur_node->next_polygon == end_polygon)
            {
                break;
            }
            int num_succ = get_successors(*cur_node, start, *mesh,
                                          successors);
            num_nodes = succ_to_node(cur_node, successors,
                                     num_succ, nodes_to_push);
            nodes_generated += num_nodes;
        }
        while (num_nodes == 1); // if num_nodes == 0, we still want to break

        for (int i = 0; i < num_nodes; i++)
        {
            // We need to update the h value before we push!
            const SearchNodePtr n = nodes_to_push[i];
            const Point& n_root = (n->root == -1 ? start :
                                   mesh->mesh_vertices[n->root].p);
            n->f += get_h_value(n_root, goal, n->left, n->right);

            if (verbose)
            {
                std::cerr << "\tpushing: ";
                print_node(n, std::cerr);
                std::cerr << std::endl;
            }

            open_list.push(n);
        }
        nodes_pushed += num_nodes;
    }

    timer.stop();
    delete[] successors;
    delete[] nodes_to_push;
    return false;
}

#define root_to_point(root) ((root) == -1 ? start : mesh->mesh_vertices[root].p)

void SearchInstance::print_node(SearchNodePtr node, std::ostream& outfile)
{
    outfile << "root=" << root_to_point(node->root) << "; left=" << node->left
            << "; right=" << node->right << ", f=" << node->f << ", g="
            << node->g;
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

    {
        // Recreate the h value heuristic to see whether we need to add the
        // left or right endpoint of the final node's interval.
        const Point& root = root_to_point(cur_node->root),
                        l = cur_node->left,
                        r = cur_node->right;
        const Point lr = r - l;
        Point fixed_goal;
        if (((root - l) * lr > 0) == ((goal - l) * lr > 0))
        {
            fixed_goal = reflect_point(goal, l, r);
        }
        else
        {
            fixed_goal = goal;
        }
        double rg_num, lr_num, denom;
        line_intersect_time(root, fixed_goal, l, r, rg_num, lr_num, denom);

        if (denom != 0.0)
        {
            switch (line_intersect_bound_check(lr_num, denom))
            {
                case ZeroOnePos::LT_ZERO:
                    // Use left end point.
                    out.push_back(l);
                    break;

                case ZeroOnePos::EQ_ZERO:
                case ZeroOnePos::IN_RANGE:
                case ZeroOnePos::EQ_ONE:
                    // h value heuristic uses straight line.
                    break;

                case ZeroOnePos::GT_ONE:
                    // Use right end point.
                    out.push_back(r);
                    break;

                default:
                    assert(false);
            }
        }

    }

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
        print_node(cur_node, outfile);
        outfile << std::endl;
        mesh->print_polygon(outfile, cur_node->next_polygon);
        outfile << std::endl;
        cur_node = cur_node->parent;
    }
}

#undef root_to_point

}
