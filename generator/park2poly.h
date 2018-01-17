#include "point.h"
#include "geometry.h"
#include <sstream>
#include <vector>
using namespace std;
namespace pl = polyanya;
namespace generator {

template<typename T>
void print_polygons(vector<vector<pl::Point>>& outPolys) {
  cout << "poly" << endl
       << "1" << endl
       << outPolys.size() << endl;
  for (int i=0; i<(int)outPolys.size(); i++) {
    cout << outPolys[i].size();
    for (int j=0; j<(int)outPolys[i].size(); j++) {
      cout << " " << (T)outPolys[i][j].x << " " << (T)outPolys[i][j].y;
    }
    cout << endl;
  }

}

vector<vector<pl::Point>> read_polys(istream& infile) {
  // format:
  // poly
  // $version
  // num of poly
  // {poly}
  vector<vector<polyanya::Point>> polys;
  string header;
  infile >> header;
  assert(header == "poly");
  int version;
  infile >> version;
  assert(version == 1);
  // ========================= read polygons
  int N;
  infile >> N;
  for (int i=0; i<N; i++) {
    int M;
    infile >> M;
    assert(M >= 3);
    vector<polyanya::Point> ps;
    for (int j=0; j<M; j++) {
      double x, y;
      infile >> x >> y;
      polyanya::Point p{x, y};
      ps.push_back(p);
    }
    polys.push_back(ps);
  }
  return polys;
}

void normalize_polys(const vector<vector<pl::Point>>& polys, vector<vector<pl::Point>>& normalized, double EPS) {
  double min_x, min_y, max_x, max_y;
  min_x = max_x = polys.front().front().x;
  min_y = max_y = polys.front().front().y;
  for (const auto& poly: polys) {
    for (const auto& pt: poly) {
      min_x = min(min_x, pt.x);
      max_x = max(max_x, pt.x);
      min_y = min(min_y, pt.y);
      max_y = max(max_y, pt.y);
    }
  }

  min_x -= 10 * EPS, max_x += 10 * EPS;
  min_y -= 10 * EPS, max_y += 10 * EPS;
  double rangex = max_x - min_x;
  double rangey = max_y - min_y;
  normalized.push_back({{0, 0}, {1.0, 0}, {1.0, 1.0}, {0, 1.0}});
  for (const auto& poly: polys) {
    int N = (int)poly.size();
    vector<pl::Point> normalPoly;
    for (int i=0; i<N; i++) {
      pl::Point newP = {(poly[i].x - min_x) / rangex, (poly[i].y - min_y) / rangey};
      if (normalPoly.empty() || normalPoly.back().distance(newP) > EPS)
        normalPoly.push_back(newP);
    }
    if (normalPoly.back().distance(normalPoly.front()) < EPS)
      normalPoly.pop_back();
    if (normalPoly.size() >= 3)
      normalized.push_back(normalPoly);
  }
}

void simplify_polys(const vector<vector<pl::Point>>& polys, vector<vector<pl::Point>>& simplified, double EPS) {
  for (const auto& poly: polys) {
    vector<pl::Point> simpPoly;
    for (const auto p: poly) {
      simpPoly.push_back({p.x / EPS, p.y / EPS});
    }
    simplified.push_back(simpPoly);
  }
}

}// namesapce geneartor
