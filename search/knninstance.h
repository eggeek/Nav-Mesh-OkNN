#pragma once
#include "searchnode.h"
#include "searchinstance.h"
#include "successor.h"
#include "mesh.h"
#include "point.h"
#include "cpool.h"
#include "timer.h"
#include <queue>
#include <vector>
#include <ctime>

namespace polyanya {

class KnnInstance {
    typedef std::priority_queue<SearchNodePtr, std::vector<SearchNodePtr>,
                                PointerComp<SearchNode> > pq;
    private:
        int K = 1;
        warthog::mem::cpool* node_pool;
        MeshPtr mesh;
        Point start;
        std::vector<Point> goals;

        // kNN has k final node
        std::vector<SearchNodePtr> final_nodes;
        // poly_id: goal1, goal2, ...
        std::vector<std::vector<int>> end_polygons;
        // <i, v>: reached ith goal with cost v
        std::map<int, double> reached;
        pq open_list;

        // Best g value for a specific vertex.
        std::vector<double> root_g_values;
        // Contains the current search id if the root has been reached by
        // the search.
        std::vector<int> root_search_ids;  // also used for root-level pruning
        int search_id;

        warthog::timer timer;

        // Pre-initialised variables to use in search().
        Successor* search_successors;
        SearchNode* search_nodes_to_push;

        void init() {
            verbose = false;
            search_successors = new Successor [mesh->max_poly_sides + 2];
            search_nodes_to_push = new SearchNode [mesh->max_poly_sides + 2];
            node_pool = new warthog::mem::cpool(sizeof(SearchNode));
            init_root_pruning();
        }

        void init_root_pruning() {
            assert(mesh != nullptr);
            search_id = 0;
            size_t num_vertices = mesh->mesh_vertices.size();
            root_g_values.resize(num_vertices);
            root_search_ids.resize(num_vertices);
            fill(root_search_ids.begin(), root_search_ids.end(), 0);
        }

        void init_search() {
            assert(node_pool);
            node_pool->reclaim();
            search_id++;
            open_list = pq();
            final_nodes = std::vector<SearchNodePtr>();
            reached.clear();
            nodes_generated = 0;
            nodes_pushed = 0;
            nodes_popped = 0;
            nodes_pruned_post_pop = 0;
            successor_calls = 0;
            set_end_polygon();
            gen_initial_nodes();
        }
        PointLocation get_point_location(Point p);
        void set_end_polygon();
        void gen_initial_nodes();
        int succ_to_node(
            SearchNodePtr parent, Successor* successors,
            int num_succ, SearchNodePtr nodes
        );

        void print_node(SearchNodePtr node, std::ostream& outfile);

    public:
        int nodes_generated;        // Nodes stored in memory
        int nodes_pushed;           // Nodes pushed onto open
        int nodes_popped;           // Nodes popped off open
        int nodes_pruned_post_pop;  // Nodes we prune right after popping off
        int successor_calls;        // Times we call get_successors
        bool verbose;

        KnnInstance() { }
        KnnInstance(MeshPtr m) : mesh(m) { init(); }
        KnnInstance(int k, MeshPtr m, Point s, std::vector<Point> gs) :
            K(k), mesh(m), start(s), goals(gs) { init(); }
        KnnInstance(KnnInstance const &) = delete;
        void operator=(KnnInstance const &x) = delete;
        ~KnnInstance() {
            if (node_pool) {
                delete node_pool;
            }
            delete[] search_successors;
            delete[] search_nodes_to_push;
        }

        void set_K(int k) { this->K = k; }

        void set_start_goal(Point s, std::vector<Point> gs) {
            start = s;
            goals.clear();
            for (const auto it: gs)
              goals.push_back(it);
            final_nodes.clear();
        }

        int search();

        double get_cost(int k) {
          if (k > (int)final_nodes.size()) {
            return -1;
          }
          return final_nodes[k]->f;
        }

        double get_search_micro()
        {
            return timer.elapsed_time_micro();
        }

        void get_path_points(std::vector<Point>& out, int k);
        void print_search_nodes(std::ostream& outfile, int k);
        void deal_final_node(const SearchNodePtr node);
        void gen_final_nodes(const SearchNodePtr node, const Point& rootPoint);

        double get_gid(int k) {
          if (k > (int)final_nodes.size()) return -1;
          else return final_nodes[k]->goal_id;
        }

        int get_goal_ord(int gid) {
          for (int i=0; i<K; i++) if (final_nodes[i]->goal_id == gid) return i;
          return -1;
        }
};

}
