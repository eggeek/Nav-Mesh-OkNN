#include "Graph.h"

double EDBT::Graph::Dijkstra(double r, const set<int>& exploredV) {
  for (int i: exploredV) {
    dist[i] = INF;
    pre[i] = -1;
  }
  dist[sid()] = 0;
  dist[tid()] = INF;
  pre[sid()] = -1;
  pre[tid()] = -1;
  path_ids.clear();
  // <dist, vid>
	open_list = pq();
	this->nodes_generated++;
  open_list.push({0, sid()});
  double res = INF;
  while (!open_list.empty()) {
    pair<double, int> c = open_list.top(); open_list.pop();
    if (c.first - EPSILON > dist[c.second]) continue;
    if (c.second == tid()) {
      res = c.first;
      int last_id = c.second;
      while (last_id != -1) {
        path_ids.push_back(last_id);
        last_id = pre[last_id];
      }
      reverse(path_ids.begin(), path_ids.end());
      assert(path_ids.front() == sid());
      break;
    }
    // because all segments touch the ring will be retrieved
    // if all segments are strictly in explored area:
    //  1. a path has been found
    //  2. terminate with res=INF (not reachable)
    // otherwise there is a `c` in queue that c.first >= r
    if (c.first > r + EPSILON)
      res = min(res, c.first);

    for (const auto& it: es[c.second]) {
      double nxtd = c.first + it.second;
      if (nxtd < dist[it.first]) {
        dist[it.first] = nxtd;
        pre[it.first] = c.second;
				this->nodes_generated++;
        open_list.push({nxtd, it.first});
      }
    }
  }
	while (!open_list.empty()) open_list.pop();
  return res;
}

