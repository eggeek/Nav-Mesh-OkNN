#include "EDBTknn.h"
#include "RStarTreeUtil.h"
#include "consts.h"
#include <queue>

namespace EDBT {

using namespace std;
namespace rs = rstar;
typedef ObstacleMap::Vertex Vertex;
typedef polyanya::Point pPoint;
typedef ObstacleMap::Seg Seg;

void EDBTkNN::updateObstacles(set<pii> obs) {
  set<int> newV;
  for (pii it: obs) {
    if (!exploredV.count(it.first)) newV.insert(it.first);
    if (!exploredV.count(it.second)) newV.insert(it.second);
  }
  // add perimeters
  for (const auto it: obs) if (!explored.count(it)){
    explored.insert(it);
    const Vertex v1 = getV(it.first);
    const Vertex v2 = getV(it.second);
    g.add_edge(v1.id, v2.id, sqrt(Vertex::dist2(v1, v2)));
  }
  // add edges between vertices
  for (int vid: exploredV) {
    Vertex v = getV(vid);
    for (int newid: newV) {
      Vertex newv = getV(newid);
      if (O.isVisible({v, newv})) {
        g.add_edge(v.id, newv.id, sqrt(Vertex::dist2(v, newv)));
      }
    }
  }
  for (int vid: newV) {
    Vertex v = getV(vid);
    pPoint p = pPoint{(double)v.x, (double)v.y};
    // add edges between (s, v);
    if (O.isVisible(g.start, p)) {
      g.add_edge(v.id, g.sid(), p.distance(g.start));
    }
    // add edges between (t, v);
    if (O.isVisible(g.goal, p)) {
      g.add_edge(v.id, g.tid(), p.distance(g.goal));
    }
  }
  exploredV.insert(newV.begin(), newV.end());
}

void EDBTkNN::changeTarget(pPoint p) {
  // remove previous edges
  for (const auto& it: g.es[g.tid()]) {
    int to = it.first;
    assert(g.es[to].count(g.tid()));
    auto rmIt = g.es[to].find(g.tid());
    g.es[to].erase(rmIt);
  }
  g.es[g.tid()].clear();
  g.vs[g.tid()] = g.goal = p;
  for (int vid: exploredV) {
    Vertex v = getV(vid);
    pPoint vp = pPoint{(double)v.x, (double)v.y};
    if (O.isVisible(g.goal, vp)) {
      g.add_edge(vid, g.tid(), vp.distance(g.goal));
    }
  }
}

double Graph::Dijkstra(double r) {
  fill(dist.begin(), dist.end(), INF);
  dist[sid()] = 0;
  // <dist, vid>
  priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> q;
  q.push({0, sid()});
  double res = INF;
  while (!q.empty()) {
    pair<double, int> c = q.top(); q.pop();
    if (c.first - EPSILON > dist[c.second]) continue;
    if (c.second == tid()) {
      res = c.first;
      break;
    }
    // because all segments touch the ring will be retrieved
    // if all segments are strictly in explored area:
    //  1. a path has been found
    //  2. terminate with res=INF (not reachable)
    // otherwise there is a `c` in queue that c.first >= r
    if (c.first >= r) res = min(res, c.first);

    for (const auto& it: es[c.second]) {
      double nxtd = c.first + it.second;
      if (nxtd < dist[it.first]) {
        dist[it.first] = nxtd;
        q.push({nxtd, it.first});
      }
    }
  }
  while(!q.empty()) q.pop();
  return res;
}

void EDBTkNN::enlargeExplored(double preR, double newR) {
  vector<rs::Data_P> rawObs;
  rs::RStarTreeUtil::rangeQuery(O.rtree, rs::Point(q.x, q.y), preR, newR, rawObs);
  set<pii> obs;
  for (auto itPtr: rawObs) {
    Seg seg = *((Seg*)itPtr);
    obs.insert({seg.first.id, seg.second.id});
  }
  updateObstacles(obs);
}

double EDBTkNN::ODC(Graph& g, pPoint p, double curR) {
  // before call this function, Graph g must be initilized
  changeTarget(p);
  if (curR <= EPSILON) { // first time call ODC
    double r = p.distance(q);
    enlargeExplored(0, r);
  }
  double d = INF;
  do {
    double d = g.Dijkstra(curR);
    if (d <= curR) // find valid shortest path
      break;
    else if (fabs(d - INF) <= EPSILON) // not reachable
      break;
    else { // d > curR
      enlargeExplored(curR, d);
      curR = d;
    }
  } while (true);
  return d;
}

}// namespace EDBT
