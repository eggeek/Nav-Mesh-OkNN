#pragma once
#include "searchnode.h"
#include "successor.h"
#include "mesh.h"
#include "point.h"
#include <queue>
#include <vector>
#include <memory>

namespace polyanya
{

template<typename T, typename Compare = std::greater<T> >
struct PointerComp
{
    bool operator()(const std::shared_ptr<T> x,
                    const std::shared_ptr<T> y) const
    {
        return Compare()(*x, *y);
    }
};

typedef std::shared_ptr<Mesh> MeshPtr;

class SearchInstance
{
    typedef std::priority_queue<SearchNodePtr, std::vector<SearchNodePtr>,
                                PointerComp<SearchNode> > pq;
    private:
        MeshPtr mesh;
        Point start, goal;

        SearchNodePtr final_node;
        int end_polygon; // set by init_search
        pq open_list;
        void init_search()
        {
            open_list = pq();
            final_node = nullptr;
            set_end_polygon();
            gen_initial_nodes();
        }
        PointLocation get_point_location(Point p);
        void set_end_polygon();
        void gen_initial_nodes();
        void push_successors(
            SearchNodePtr parent, std::vector<Successor>& successors
        );

    public:
        SearchInstance() { }
        SearchInstance(MeshPtr m) : mesh(m) { }
        SearchInstance(MeshPtr m, Point s, Point g) :
            mesh(m), start(s), goal(g) {}

        void set_start_goal(Point s, Point g)
        {
            start = s;
            goal = g;
            final_node = nullptr;
        }

        bool search();
        double get_cost()
        {
            if (final_node == nullptr)
            {
                return -1;
            }

            return final_node->f;
        }
        void get_path_points(std::vector<Point>& out);
        void print_search_nodes(std::ostream& outfile);

};

}
