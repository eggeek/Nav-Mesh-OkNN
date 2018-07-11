#pragma once
#include "searchnode.h"
#include "geometry.h"
#include "searchinstance.h"
#include "expansion.h"
#include "successor.h"
#include "mesh.h"
#include "point.h"
#include "cpool.h"
#include "timer.h"
#include "RStarTree.h"
#include "RStarTreeUtil.h"
#include "knnMeshFence.h"
#include <chrono>
#include <queue>
#include <vector>
#include <ctime>

namespace polyanya {

namespace rs = rstar;

class FenceHeuristic {
    typedef std::priority_queue<SearchNodePtr, std::vector<SearchNodePtr>,
                                PointerComp<SearchNode> > pq;
    private:
        int K = 1;
        warthog::mem::cpool* node_pool;
        MeshPtr mesh;
        Point start;
        std::vector<Point> goals;
        KnnMeshEdgeFence* meshFence;

        // kNN has k final node
        std::vector<SearchNodePtr> final_nodes;
        // poly_id: goal1, goal2, ...
        std::vector<std::vector<int>> end_polygons;
        // <i, v>: reached ith goal with cost v
        //std::map<int, double> reached;
        std::vector<double> reached;
        pq open_list;

        // Best g value for a specific vertex.
        std::vector<double> root_g_values;
        // Contains the current search id if the root has been reached by
        // the search.
        std::vector<int> root_search_ids;  // also used for root-level pruning
        int search_id;

        warthog::timer timer;
        double heuristic_using;
        double angle_using;

        // Pre-initialised variables to use in search().
        Successor* search_successors;
        SearchNode* search_nodes_to_push;

        void init() {
            meshFence= nullptr;
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
            reached.resize(goals.size());
            fill(reached.begin(), reached.end(), INF);
            nodes_generated = 0;
            nodes_pushed = 0;
            nodes_popped = 0;
            nodes_pruned_post_pop = 0;
            successor_calls = 0;
            nodes_reevaluate = 0;
            set_end_polygon();
            gen_initial_nodes();
            heuristic_using = 0;
            heuristic_call = 0;
            angle_using = 0;
        }
        void set_end_polygon();
        void gen_initial_nodes();
        int succ_to_node(
            SearchNodePtr parent, Successor* successors,
            int num_succ, SearchNodePtr nodes
        );
        void push_lazy(SearchNodePtr lazy);
        void print_node(SearchNodePtr node, std::ostream& outfile);
        inline Point root_to_point(int root_id) {
          return root_id == -1? start: mesh->mesh_vertices[root_id].p;
        }

    public:
        int nodes_generated;        // Nodes stored in memory
        int nodes_pushed;           // Nodes pushed onto open
        int nodes_popped;           // Nodes popped off open
        int nodes_pruned_post_pop;  // Nodes we prune right after popping off
        int successor_calls;        // Times we call get_successors
        int heuristic_call;
        int nodes_reevaluate;
        bool verbose;
        std::vector<int> gids;

        FenceHeuristic() { }
        FenceHeuristic(MeshPtr m) : mesh(m) { init(); }
        FenceHeuristic(int k, MeshPtr m, Point s, std::vector<Point> gs) :
            K(k), mesh(m), start(s), goals(gs) { init(); }
        FenceHeuristic(FenceHeuristic const &) = delete;
        void operator=(FenceHeuristic const &x) = delete;
        ~FenceHeuristic() {
            if (node_pool) {
                delete node_pool;
            }
            delete[] search_successors;
            delete[] search_nodes_to_push;
        }

        void set_K(int k) { this->K = k; }

        void set_goals(std::vector<Point> gs) {
          goals = std::vector<Point>(gs);
        }

        void set_start(Point s) { start = s; }

        void set_meshFence(KnnMeshEdgeFence* meshFence) { this->meshFence= meshFence; }

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

        double get_heuristic_micro() {
          return heuristic_using;
        }

        double get_angle_micro() {
          return angle_using;
        }
        pair<int, double> get_fence_heuristic(SearchNode* node);
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

        std::pair<int, double> nn_query(SearchInstance* si, double& elapsed_time_micro);
};

}
