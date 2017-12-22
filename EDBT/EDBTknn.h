#include "RStarTree.h"
#include "RStarTreeUtil.h"
#include "Data.h"
#include "EDBTObstacles.h"
#include <set>

namespace rs = rstar;

namespace EDBT {

using namespace std;
typedef polyanya::Point pPoint;
typedef pair<int, int> pii;
typedef ObstacleMap::Vertex Vertex;

class Graph {
public:
  vector<map<int, double>> es;
  vector<pPoint> vs;
  int vertNum;
  vector<double> dist;
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
  ObstacleMap O;
  vector<pPoint> goals;
  pPoint q;
  Graph g;
  rs::RStarTree rte; // entities
  rs::MinHeap heap;
  // < seg({vid, vid}), ... >
  set<pii> explored;
  set<int> exploredV;

  EDBTkNN(ObstacleMap oMap, vector<pPoint> goals, pPoint q): O(oMap), goals(goals), q(q) {
    g.init(O.vs, q, goals[0]);
    initRtree();
  }

  void initRtree() {
    for (auto& it: goals) {
      rs::Mbr mbr(it.x, it.x, it.y, it.y);
      rs::LeafNodeEntry leaf = rs::LeafNodeEntry(mbr, &it);
      rte.insertData(&leaf);
    }
  }

  double ODC(Graph& g, pPoint p, double& curR);
  void updateObstacles(set<pii> obs);
  void changeTarget(pPoint p);
  void enlargeExplored(double preR, double newR);
  vector<pPoint> OkNN(int k);
  vector<pair<pPoint, double>> Euclidean_NN(int k);
  pair<pPoint, double> next_Euclidean_NN();
  inline pPoint getP(int vid) { return pPoint{(double)O.vs[vid].x, (double)O.vs[vid].y}; }
  inline ObstacleMap::Vertex getV(int vid) { return O.vs[vid]; }
};

} // namespace EDBT
