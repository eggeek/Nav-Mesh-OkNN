#pragma once
#include "RStarTreeUtil.h"
#include "RStarTree.h"
#include "Data.h"
#include "EDBTObstacles.h"

namespace EDBT {

namespace rs = rstar;
using namespace std;

static vector<pair<pPtr, double>> Euclidean_NN(rs::MinHeap& heap, rs::RStarTree* rte, const pPoint & q, int k) {
  heap.clear();
  double d;
  rs::Point rq(q.x, q.y);
  d = sqrt(rs::RStarTreeUtil::dis2(rq, rte->root->mbrn));
  heap.push(rs::MinHeapEntry(d, rte->root));
  vector<pair<pPtr, double> > res;
  for (int i=0; i<k; i++) {
    rs::MinHeapEntry e = rs::RStarTreeUtil::iNearestNeighbour(heap, rs::Point(q.x, q.y));
    if (e.entryPtr == nullptr) break;
    pPtr p = (pPoint*)e.entryPtr->data;
    d = e.key;
    res.push_back({p, d});
  }
  return res;
}

static pair<pPtr, double> next_Euclidean_NN(rs::MinHeap& heap, const pPoint& q) {
  rs::MinHeapEntry e = rs::RStarTreeUtil::iNearestNeighbour(heap, rs::Point(q.x, q.y));
  if (e.entryPtr == nullptr) return {nullptr, INF};
  else {
    pPtr p = (pPoint*)e.entryPtr->data;
    return {p, e.key};
  }
}

}
