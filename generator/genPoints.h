#pragma once
#include "point.h"
#include "mesh.h"
#include "EDBTObstacles.h"
#include "expansion.h"
#include <vector>
#include <string>
#include <sstream>
#include <random>
using namespace std;
namespace pl = polyanya;

namespace generator {

void gen_points_in_traversable(EDBT::ObstacleMap* oMap, const vector<vector<pl::Point>>& polys,
                               int num, vector<pl::Point>& out, bool verbose=false) {
  double min_x, max_x, min_y, max_y;
  // ignore border
  min_x = oMap->mesh->get_minx();
  max_x = oMap->mesh->get_maxx();
  min_y = oMap->mesh->get_miny();
  max_y = oMap->mesh->get_maxy();

  if (verbose) {
    cerr << "border: " << polys[0][0] << " " << polys[0][1] << endl;
    cerr << "x: " << min_x << " " << max_x << endl;
    cerr << "y: " << min_y << " " << max_y << endl;
  }

  out.resize(num);
  random_device rd;
  mt19937 eng(rd());
  uniform_real_distribution<double> distx(min_x, max_x);
  uniform_real_distribution<double> disty(min_y, max_y);

  if (verbose) {
    cout << num << endl;
  }

  for (int i=0; i<num; i++) {
    double x, y;
    do {
      x = distx(eng);
      y = disty(eng);
      pl::Point p{x, y};
      if (oMap->isCoveredByPolys(p, p, oMap->traversableRtree) &&
          oMap->mesh->get_point_location(p).type == pl::PointLocation::IN_POLYGON) {
        out[i] = p;
        if (verbose) {
          cout << x << " " << y << endl;
        }
        break;
      }
    } while (true);
  }
}

/*
 * generate clusters around given targets
 */
void gen_clusters_in_traversable(
    EDBT::ObstacleMap* oMap, pl::Mesh* mesh,
    int maxNum,
    vector<pl::Point>& seeds,
    vector<pl::Point>& out, double radius=EPSILON, bool verbose=false)  {

  double min_x = mesh->get_minx();
  double max_x = mesh->get_maxx();
  double min_y = mesh->get_miny();
  double max_y = mesh->get_maxy();
  double dx, dy;
  if (fabs(radius-EPSILON) <= EPSILON) {
    dx = (max_x - min_x) / 100.0;
    dy = (max_y - min_y) / 100.0;
  } else {
    dx = dy = radius;
  }
  random_device rd;
  mt19937 eng(rd());
  uniform_int_distribution<> distnum(maxNum, maxNum);
  uniform_real_distribution<> distx(-dx, dx);
  uniform_real_distribution<> disty(-dy, dy);
  for (auto& t: seeds) {
    int num = distnum(eng);
    for (int i=0; i<num; i++) {
      double x, y;
      do {
        x = t.x + distx(eng);
        y = t.y + disty(eng);
        pl::Point p{x, y};
        if (oMap->isCoveredByPolys(p, p, oMap->traversableRtree) &&
            oMap->mesh->get_point_location(p).type == pl::PointLocation::IN_POLYGON) {
          out.push_back(p);
          break;
        } 
      } while (true); 
    }
  }
  if (verbose) {
    cout << out.size() << endl;
    for (auto it: out) cout << it.x << " " << it.y << endl;
  }
}

}// namespace generator
