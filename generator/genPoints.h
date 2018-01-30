#pragma once
#include "point.h"
#include "mesh.h"
#include "EDBTObstacles.h"
#include <vector>
#include <string>
#include <sstream>
#include <random>
using namespace std;
namespace pl = polyanya;

namespace generator {

void gen_points_in_traversable(EDBT::ObstacleMap* oMap, const vector<vector<pl::Point>>& polys,
                               int num, vector<pl::Point>& out, bool verbose=false) {
  long long min_x, max_x, min_y, max_y;
  // ignore border
  min_x = max_x = polys[1][0].x;
  min_y = max_y = polys[1][0].y;
  for (size_t i=1; i<polys.size(); i++) {
    for (const auto& p: polys[i]) {
      min_x = min(min_x, (long long)p.x);
      max_x = max(max_x, (long long)p.x);
      min_y = min(min_y, (long long)p.y);
      max_y = max(max_y, (long long)p.y);
    }
  }

  if (verbose) {
    cerr << "border: " << polys[0][0] << " " << polys[0][1] << endl;
    cerr << "x: " << min_x << " " << max_x << endl;
    cerr << "y: " << min_y << " " << max_y << endl;
  }

  out.resize(num);
  random_device rd;
  mt19937 eng(rd());
  uniform_int_distribution<long long> distx(min_x, max_x);
  uniform_int_distribution<long long> disty(min_y, max_y);

  if (verbose) {
    cout << num << endl;
  }

  for (int i=0; i<num; i++) {
    long long x, y;
    do {
      x = distx(eng);
      y = disty(eng);
      pl::Point p{(double)x, (double)y};
      if (oMap->isCoveredByTraversable(p, p)) {
        out[i] = p;
        if (verbose) {
          cout << x << " " << y << endl;
        }
        break;
      }
    } while (true);
  }
}

}// namespace generator
