#pragma once
#include "EDBTObstacles.h"

namespace EDBT {

using namespace std;
namespace rs = rstar;
typedef ObstacleMap::Vertex Vertex;

class Graph {
public:

  typedef 
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

  vector<map<int, double>> es;
  vector<pPoint> vs;
  vector<int> pre;
  int vertNum;
  vector<double> dist;
  vector<int> path_ids;
  // start id: vertNum, target id: vertNum+1;
  pPoint start;
  pPoint goal;
	pq open_list;
	int nodes_generated;

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
		nodes_generated = 0;
  }

  inline int sid() { return vertNum; }
  inline int tid() { return vertNum + 1; }

  double Dijkstra(double r, const set<int>& exploredV);
};

}
