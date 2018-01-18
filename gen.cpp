#include "EDBTObstacles.h"
#include "mesh.h"
#include "point.h"
#include "park2poly.h"
#include <string>
#include <sstream>
#include <random>
using namespace std;
namespace pl = polyanya;

void gen_vg(string polypath, string meshpath) {
  ifstream meshfile(meshpath);
  ifstream polyfile(polypath);
  polyanya::Mesh* mp = new polyanya::Mesh(meshfile);
  EDBT::ObstacleMap* oMap = new EDBT::ObstacleMap(polyfile, mp);
  oMap->printObsMap();
}

void gen_poly(string parkpath, double size, int num) {
  vector<vector<polyanya::Point>> polys;
  ifstream parkfile(parkpath);
  polys = generator::read_polys(parkfile);
  if (num != -1)
    generator::random_choose_polys(polys, num);
  cerr << "size of polys: " << polys.size() << endl;

  //vector<vector<pl::Point>> normalized;
  generator::normalize_polys(polys, size);

  vector<vector<pl::Point>> simplified;
  generator::simplify_polys(polys, simplified);
  generator::print_polygons<long long>(simplified);
}

void gen_entities_points(string polypath, string meshpath, int num) {
  ifstream polyfile(polypath);
  ifstream meshfile(meshpath);
  polyanya::Mesh* mp = new polyanya::Mesh(meshfile);
  EDBT::ObstacleMap* oMap = new EDBT::ObstacleMap(polyfile, mp);
  long long min_x, max_x, min_y, max_y;
  min_x = max_x = oMap->vs[0].x;
  min_y = max_y = oMap->vs[0].y;
  for (int i=0; i<(int)oMap->vs.size(); i++) {
    min_x = min(min_x, oMap->vs[i].x);
    max_x = max(max_x, oMap->vs[i].x);
    min_y = min(min_y, oMap->vs[i].y);
    max_y = max(max_y, oMap->vs[i].y);
  }
  random_device rd;
  mt19937 eng(rd());
  uniform_int_distribution<long long> distx(min_x, max_x);
  uniform_int_distribution<long long> disty(min_y, max_y);
  cout << num << endl;
  for (int i=0; i<num; i++) {
    long long x, y;
    do {
      x = distx(eng);
      y = disty(eng);
      polyanya::Point p{(double)x, (double)y};
      if (oMap->isCoveredByTraversable(p, p)) {
        cout << x << " " << y << endl;
        break;
      }
    } while (true);
  }
}

int main(int argc, char* argv[]) {
  if (argc > 1) {
    string t = string(argv[1]);
    if (t == "vg") {
      string polypath = string(argv[2]);
      string meshpath = string(argv[3]);
      gen_vg(polypath, meshpath);
    }
    else if (t == "pl") {
      string parkpath = string(argv[2]);
      string size = string(argv[3]);
      string num = string(argv[4]);
      gen_poly(parkpath, stod(size), stod(num));
    }
    else if (t == "pts") {
      string polypath = string(argv[2]);
      string meshpath = string(argv[3]);
      int num = atoi(argv[4]);
      gen_entities_points(polypath, meshpath, num);
    }
  }
  return 0;
}
