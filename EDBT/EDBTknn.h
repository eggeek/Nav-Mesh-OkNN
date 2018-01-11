#pragma once
#include "RStarTree.h"
#include "RStarTreeUtil.h"
#include "Data.h"
#include "timer.h"
#include "EDBTObstacles.h"
#include <set>


namespace EDBT {

namespace rs = rstar;
using namespace std;
typedef polyanya::Point pPoint;
typedef pair<int, int> pii;
typedef ObstacleMap::Vertex Vertex;
typedef const pPoint* pPtr;

class Graph {
public:
  vector<map<int, double>> es;
  vector<pPoint> vs;
  vector<int> pre;
  int vertNum;
  vector<double> dist;
  vector<int> path_ids;
  // start id: vertNum, target id: vertNum+1;
  pPoint start;
  pPoint goal;

  void add_edge(int from, int to, double w) {
    if (!es[from].count(to)) es[from][to] = w;
    else es[from][to] = min(es[from][to], w);
    if (!es[to].count(from)) es[to][from] = w;
    else es[to][from] = min(es[to][from], w);
  }

  void init(vector<Vertex>& vertices, pPoint s, pPoint t) {
    es.clear();
    vs.clear();
    vertNum = (int)vertices.size();
    vs.resize(vertNum + 2);
    es.resize(vertNum + 2);
    pre.resize(vertNum + 2);
    for (int i=0; i<vertNum + 2; i++) es[i].clear();
    dist.resize(vertNum + 2);
    for (const Vertex& v: vertices)
      vs[v.id] = pPoint{(double)v.x, (double)v.y};
    start = vs[sid()] = s;
    goal = vs[tid()] = t;
  }

  inline int sid() { return vertNum; }
  inline int tid() { return vertNum + 1; }

  double Dijkstra(double r);
};


class EDBTkNN {

public:
  vector<pPoint> goals;
  pPoint q;
  Graph g;
  rs::RStarTree* rte; // rtree for entities
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

  ~EDBTkNN() {
    if (rte) delete rte;
    if (O) delete O;
  }

  void set_start_goals(pPoint queryP, vector<pPoint> targets) {
    if (rte) delete rte;
    q = queryP;
    goals.clear();
    for (auto it: targets) goals.push_back(it);
    g.init(O->vs, q, goals[0]);
    if (O->isVisible(g.start, g.goal)) {
      g.add_edge(g.sid(), g.tid(), g.start.distance(g.goal));
    }
    initSearch();
  }

  void initSearch() {
    heap.clear();
    rtEntries.clear();
    paths.clear();
    explored.clear();
    exploredV.clear();
  }

  void initRtree() {
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
  void updateObstacles(set<pii> obs);
  void changeTarget(pPtr p);
  void enlargeExplored(double preR, double newR);
  vector<pair<pPtr, double>> OkNN(int k);
  vector<pair<pPtr, double>> Euclidean_NN(int k);
  pair<pPtr, double> next_Euclidean_NN();
  inline pPoint getP(int vid) { return pPoint{(double)O->vs[vid].x, (double)O->vs[vid].y}; }
  inline ObstacleMap::Vertex getV(int vid) { return O->vs[vid]; }

  void get_path(int k, vector<pPoint>& outIter) {
    if (k >= (int)paths.size()) return;
    else {
      for (pPoint i: paths[k])
        outIter.push_back(i);
    }
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
