#include "FastFilterPolyanya.h"
#include "geometry.h"
#include "vertex.h"
#include "point.h"
#include "consts.h"
#include "RStarTree.h"
#include "RStarTreeUtil.h"
#include <queue>
#include <vector>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <ctime>

using namespace std;

namespace polyanya {
  namespace rs = rstar; 
  vector<double> FastFilterPolyanya::search() {
    init_search();
    priority_queue<double, vector<double>> maxh;
    vector<pair<double, Point>> edists;
    vector<double> odists;
    rs::MinHeap heap;
    rs::Point P(start.x, start.y);
    double D = sqrt(rs::RStarTreeUtil::minDis2(P, rte->root->mbrn));
    heap.push(rs::MinHeapEntry(D, rte->root));
    rs::MinHeapEntry cur(INF, (rs::Entry_P)nullptr);
    while (true) {
      cur = rs::RStarTreeUtil::iNearestNeighbour(heap, P);
      if (cur.key == INF) // not found
        break;
      int gid = *((int*)cur.entryPtr->data);
      double de = goals[gid].distance(start);
      if ((int)maxh.size() == K && maxh.top() <= de)
        break;
      polyanya->set_start_goal(start, goals[gid]);
      bool found = polyanya->search();
      search_cost += polyanya->get_search_micro();
      nodes_generated += polyanya->nodes_generated;
      if (!found) continue;
      double dist = polyanya->get_cost();
      if ((int)maxh.size() < K) {
        maxh.push(dist);
      } else if (dist < maxh.top()) {
        maxh.pop();
        maxh.push(dist);
      }
    }
    while (!maxh.empty()) {
      odists.push_back(maxh.top());
      maxh.pop();
    }
    sort(odists.begin(), odists.end());
    return odists;
  }
}