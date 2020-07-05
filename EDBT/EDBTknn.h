#pragma once
#include "RStarTree.h"
#include "RStarTreeUtil.h"
#include "Data.h"
#include "timer.h"
#include "EDBTObstacles.h"
#include "Graph.h"
#include <set>
#include <chrono>


namespace EDBT {

class EDBTkNN {

public:

  std::chrono::steady_clock::time_point start; 
  double time_limit_micro = INF;
  vector<pPoint> goals;
  pPoint q;
  Graph g;
  rs::RStarTree* rte = nullptr; // rtree for entities
  vector<rs::LeafNodeEntry> rtEntries;
  rs::MinHeap heap;
  // < seg({vid, vid}), ... >
  set<pii> explored;
  set<int> exploredV;
  ObstacleMap* O;
  vector<vector<pPoint>> paths;

  EDBTkNN(ObstacleMap* mapPtr) {
    O = mapPtr;
  }

  EDBTkNN(vector<pPoint> goals, pPoint q, ObstacleMap* mapPtr): goals(goals), q(q) {
    O = mapPtr;
    g.init(O->vs, q, goals[0]);
    if (O->isVisible(g.start, g.goal)) {
      g.add_edge(g.sid(), g.tid(), g.start.distance(g.goal));
    }
  }

  void set_start(pPoint queryP) {
    q = queryP;
    g.init(O->vs, q, goals[0]);
    if (O->isVisible(g.start, g.goal)) {
      g.add_edge(g.sid(), g.tid(), g.start.distance(g.goal));
    }
    initSearch();
  }
  void set_goals(vector<pPoint> targets) {
    goals.clear();
    for (auto it: targets) goals.push_back(it);
    g.init(O->vs, q, goals[0]);
    if (O->isVisible(g.start, g.goal)) {
      g.add_edge(g.sid(), g.tid(), g.start.distance(g.goal));
    }
    initRtree();
  }

  void initSearch() {
    heap.clear();
    paths.clear();
    explored.clear();
    exploredV.clear();
		g.nodes_generated = 0;
    start = chrono::steady_clock::now();
  }

  void initRtree() {
    // rte can be initialized only once
    assert(rte == nullptr);
    rtEntries.clear();
    rte = new rs::RStarTree();
    for (auto& it: goals) {
      rs::Mbr mbr(it.x, it.x, it.y, it.y);
      rs::LeafNodeEntry leaf(mbr, (rs::Data_P)&it);
      rtEntries.push_back(leaf);
    }
    for (auto& it: rtEntries)
      rte->insertData(&it);
  }

  double ODC(Graph& g, pPtr p, double& curR);
  double get_search_micro() {
    return timer.elapsed_time_micro();
  }
  double get_current_micro() {
    auto cur = chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(cur - start).count();
  }
  void updateObstacles(set<pii> obs);
  void changeTarget(pPtr p);
  void enlargeExplored(double preR, double newR);
  vector<pair<pPtr, double>> OkNN(int k);
  inline pPoint getP(int vid) { return pPoint{(double)O->vs[vid].x, (double)O->vs[vid].y}; }
  inline ObstacleMap::Vertex getV(int vid) { return O->vs[vid]; }

  void get_path(int k, vector<pPoint>& outIter) {
    if (k >= (int)paths.size()) return;
    else {
      outIter.clear();
      for (pPoint i: paths[k])
        outIter.push_back(i);
    }
  }

  void set_time_limit_micro(double tlimit) {
    this->time_limit_micro = tlimit;
  }

  vector<pPoint> to_point_path(vector<int>& path_ids, pPtr last) {
    vector<pPoint> res;
    assert(path_ids.back() == g.tid());
    for (int i: path_ids) {
      if (i != g.tid())
        res.push_back(g.vs[i]);
      else
        res.push_back({last->x, last->y});
    }
    return res;
  }

private:
  warthog::timer timer;
};

} // namespace EDBT
