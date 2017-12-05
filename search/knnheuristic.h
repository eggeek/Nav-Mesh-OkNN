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
#include "rtree.h"
#include <queue>
#include <vector>
#include <ctime>

namespace polyanya {

class KnnHeuristic {
    typedef std::priority_queue<SearchNodePtr, std::vector<SearchNodePtr>,
                                PointerComp<SearchNode> > pq;
    typedef std::pair<Point, int> value;
    typedef bg::model::polygon<Point> polygon;
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
        std::vector<value> nn;

        // Pre-initialised variables to use in search().
        Successor* search_successors;
        SearchNode* search_nodes_to_push;
        bgi::rtree<value, bgi::rstar<16> > rtree;

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
        }

        int get_knn(const Point& p, int k) {
          nn.clear();
          rtree.query(bgi::nearest(p, k), std::back_inserter(nn));
          if (nn.empty()) return -1;
          else return nn.back().second;
        }

        int get_knn(const Point& l, const Point& r, int k=1) {
          nn.clear();
          bg::model::segment<Point> seg(l, r);
          rtree.query(bgi::nearest(seg, k), std::back_inserter(nn));
          if (nn.empty()) return -1;
          else return nn.back().second;
        }

        /*
        Boost R-tree doesn't support within(polygon) query, so this function doesn't work currently
        std::pair<int, double> get_min_hueristic(const Point& r, const Point& a, const Point& b, int k=1) {
            Point perp = perp_point(r, a, b);
            Point r2 = reflect_point(r, a, b);
            Point v = r - perp;
            double t = 1e8;
            Point a0 = a + t * v, b0 = b + t * v, a1 = a - t * v, b1 = b - t * v;
            Point w = a - b;
            Point p0 = a0 + t * w, p1 = a1 + t * w, p2 = b0 - t * w, p3 = b1 - t * w;
            polygon A {{a0, a, b, b0, a0}};
            polygon B {{a, a1, b1, b, a}};
            polygon C {{p0, p1, a1, a0, p0}};
            polygon D {{b0, b1, p3, p2, b0}};
            int res = -1;
            double curv = 1e18; //INF

            auto is_better = [&](double& oldv, int gid) {
              double newv = get_h_value(r, goals[gid], a, b);
              if (oldv > newv) {
                oldv = newv;
                return true;
              }
              return false;
            };

            nn.clear();
            rtree.query(bgi::within(A) && bgi::nearest(r2, k), nn);
            if (!nn.empty() && is_better(curv, nn.back().second)) res = nn.back().second;

            nn.clear();
            rtree.query(bgi::within(B) && bgi::nearest(r, k), nn);
            if (!nn.empty() && is_better(curv, nn.back().second)) res = nn.back().second;

            nn.clear();
            rtree.query(bgi::within(C) && bgi::nearest(a, k), nn);
            if (!nn.empty() && is_better(curv, nn.back().second)) res = nn.back().second;

            nn.clear();
            rtree.query(bgi::within(D) && bgi::nearest(b, k), nn);
            if (!nn.empty() && is_better(curv, nn.back().second)) res = nn.back().second;

            return {res, curv};
        }
        */

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

        KnnHeuristic() { }
        KnnHeuristic(MeshPtr m) : mesh(m) { init(); }
        KnnHeuristic(int k, MeshPtr m, Point s, std::vector<Point> gs) :
            K(k), mesh(m), start(s), goals(gs) { init(); }
        KnnHeuristic(KnnHeuristic const &) = delete;
        void operator=(KnnHeuristic const &x) = delete;
        ~KnnHeuristic() {
            if (node_pool) {
                delete node_pool;
            }
            delete[] search_successors;
            delete[] search_nodes_to_push;
        }

        void set_K(int k) { this->K = k; }

        void set_start_goal(Point s, std::vector<Point> gs) {
            start = s;
            goals = std::vector<Point>(gs);
            rtree.clear();
            final_nodes = std::vector<SearchNodePtr>();
            for (int i=0; i<(int)gs.size(); i++) {
              rtree.insert(std::make_pair(gs[i], i));
            }
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
