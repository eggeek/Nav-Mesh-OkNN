#include "EDBTObstacles.h"
#include "mesh.h"
#include "point.h"
#include <string>
#include <sstream>
#include <random>
using namespace std;

void gen_vg(string polypath, string meshpath) {
  ifstream meshfile(meshpath);
  ifstream polyfile(polypath);
  polyanya::Mesh* mp = new polyanya::Mesh(meshfile);
  EDBT::ObstacleMap* oMap = new EDBT::ObstacleMap(polyfile, mp);
  oMap->printObsMap();
}

void gen_real_poly(double EPS) {
  // format:
  // poly
  // $version
  // num of poly
  // {poly}
  vector<vector<polyanya::Point>> polys;
  string header;
  cin >> header;
  assert(header == "poly");
  int version;
  cin >> version;
  assert(version == 1);
  int N;
  cin >> N;
  for (int i=0; i<N; i++) {
    int M;
    cin >> M;
    vector<polyanya::Point> ps;
    for (int j=0; j<M; j++) {
      double x, y;
      cin >> x >> y;
      polyanya::Point p{x, y};
      if (ps.empty() || ps.back().distance(p) > EPS) {
        ps.push_back(p);
      }
    }
    if (ps.size() >= 3) {
      for (int j=0; j<(int)ps.size(); j++) {
        ps[j].x /= EPS;
        ps[j].y /= EPS;
      }
      polys.push_back(ps);
    }
  }
  cout << header << endl
       << version << endl
       << polys.size() << endl;
  for (int i=0; i<(int)polys.size(); i++) {
    cout << polys[i].size();
    for (int j=0; j<(int)polys[i].size(); j++) {
      cout << " " << (long long)polys[i][j].x << " " << (long long)polys[i][j].y;
    }
    cout << endl;
  }
}

void gen_entities_points(string polypath, string meshpath, int num) {
  ifstream polyfile(polypath);
  ifstream meshfile(meshpath);
  polyanya::Mesh* mp = new polyanya::Mesh(meshfile);
  EDBT::ObstacleMap* oMap = new EDBT::ObstacleMap(polyfile, mp);
  long long min_x, max_x, min_y, max_y;
  min_x = max_x = oMap->vs[0].x;
  min_y = max_y = oMap->vs[0].y;
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
      string eps = string(argv[2]);
      gen_real_poly(stod(eps));
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
