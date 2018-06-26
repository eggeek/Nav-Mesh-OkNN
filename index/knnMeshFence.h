#pragma once
#include "mesh.h"
#include "timer.h"
#include "searchnode.h"
#include "cpool.h"
#include "expansion.h"
#include "successor.h"
#include "point.h"
#include <queue>

using namespace std;

namespace polyanya {

struct Fence {
  double lb, ub;
  int gid;
  SearchNode s;
  Fence(double l, double r, int gid,  SearchNodePtr sn): lb(l), ub(r), gid(gid) {
    s.parent = nullptr;
    s.root = sn->root;
    s.g = sn->g;
    s.f = sn->f;
    s.left = sn->left;
    s.right = sn->right;
    s.left_vertex = sn->left_vertex;
    s.right_vertex = sn->right_vertex;
  };
};


class FloodFillNode {
public:
  SearchNode* snode;
  double lb, ub;
  int gid, polyid, left_vid;

  FloodFillNode(SearchNodePtr sp, double l, double u, int gid, int polyid, int vid):
    snode(sp), lb(l), ub(u), gid(gid), polyid(polyid), left_vid(vid) {};

  bool operator<(const FloodFillNode& other) const {
    if (this->lb == other.lb) {
      return this->ub < other.ub;
    }
    return this->lb < other.lb;
  }

  bool operator>(const FloodFillNode& other) const {
    if (this->lb == other.lb) {
      return this->ub > other.ub;
    }
    return this->lb > other.lb;
  }
};

class KnnMeshEdgeFence{

private:
  typedef priority_queue<FloodFillNode, vector<FloodFillNode>, greater<FloodFillNode> > pq;

  Mesh* mesh;
  map<pair<int, int>, vector<Fence>> fences;
  pq open_list;
  vector<Point> goals;
  warthog::timer timer;

  // root pruning
  vector<double> root_g_values;
  int search_id;
  vector<int> root_search_ids;
  warthog::mem::cpool* node_pool;
  Successor* search_successors;
  SearchNode* search_nodes_to_push;

  PointLocation get_point_location(Point p);

  void init_floodfill() {
    assert(node_pool);
    node_pool->reclaim();
    search_id++;
    open_list = pq();
    nodes_generated = 0;
    nodes_pushed = 0;
    nodes_popped = 0;
    nodes_pruned = 0;
    gen_initial_nodes();
  }

  void gen_initial_nodes();
  bool pass_fence(const FloodFillNode& fnode);
  int succ_to_node(
    SearchNodePtr parent, Successor* successors, int num_succ, SearchNodePtr nodes, int gid
  );

public:
  int nodes_generated;
  int nodes_pushed;
  int nodes_popped;
  int nodes_pruned;
  int fenceCnt;
  int edgecnt;
  bool verbose;
  KnnMeshEdgeFence(Mesh* m): mesh(m) {
    int nump = m->mesh_polygons.size();
    fences.clear();
    fenceCnt = 0;
    for (int i=0; i<nump; i++) {
      int numv = m->mesh_polygons[i].vertices.size();
      for (int j=0; j<numv; j++) {
        edgecnt++;
      }
    }
    //assert(edgecnt%2 == 0);
    edgecnt /= 2;
    init();
  }

  void print_node(const FloodFillNode& fnode, ostream& outfile);

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

  void set_goals(vector<Point> gs) {
    goals = vector<Point>(gs);
  }

  void floodfill();

  double get_processing_micro() {
    return timer.elapsed_time_micro();
  }

  const vector<Fence>& get_fences(int left_vid, int right_vid) {
    pair<int, int> key = {min(left_vid, right_vid), max(left_vid, right_vid)};
    return fences[key];
  }
};

} // end namespace polyanya
