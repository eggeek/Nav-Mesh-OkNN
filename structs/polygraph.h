#pragma once
#include "mesh.h"
#include "point.h"
#include <queue>
using namespace std;

namespace polyanya {

class PolyGraph {
  struct Edge {
    int to;
    double v;
  };

  const double INF = 1e18;
  const Mesh* mptr;
  int polynum, startpoly;
  vector< vector<Edge> > e;
  vector<double> dist;

public:
  PolyGraph(const Mesh* m) {
    mptr = m;
    polynum = (int)mptr->mesh_polygons.size();
    e.resize(polynum);
    dist.resize(polynum);
    for (int i=0; i<polynum; i++) e[i].clear();
    for (int i=0; i<polynum; i++) {
      for (int j: mptr->mesh_polygons[i].polygons) if (j != -1) {
        const Polygon& p = mptr->mesh_polygons[j];
        double d = (Point{p.min_x, p.min_y} - Point{p.max_x, p.max_y}).normal();
        e[i].push_back(Edge{j, d});
      }
    }
  }

  void set_start(int s) { startpoly = s; }

  void Dijkstra() {
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>> > q;
    fill(dist.begin(), dist.end(), INF);
    const Polygon& p = mptr->mesh_polygons[startpoly];
    dist[startpoly] = (Point{p.min_x, p.min_y} - Point{p.max_x, p.max_y}).normal();
    q.push({dist[startpoly], startpoly});
    while (!q.empty()) {
      pair<double, int> c = q.top(); q.pop();
      assert(c.first + EPSILON >= dist[c.second]);
      if (dist[c.second] != c.first) continue;
      for (const Edge& i: e[c.second]) {
        int nxt = i.to;
        if (dist[nxt] > dist[c.second] + i.v) {
          dist[nxt] = dist[c.second] + i.v;
          q.push({dist[nxt], nxt});
        }
      }
    }
  }

  double get_dist(int pid) {
    return dist[pid];
  }
};
}
