#define CATCH_CONFIG_RUNNER
#include "catch.hpp"
#include "expansion.h"
#include "genPoints.h"
#include "mesh.h"
#include "geometry.h"
#include "searchinstance.h"
#include "intervaHeuristic.h"
#include "targetHeuristic.h"
#include "fenceHeuristic.h"
#include "EDBTknn.h"
#include "park2poly.h"
#include "knnMeshFence.h"
using namespace std;
using namespace polyanya;

MeshPtr mp;
Mesh m;
EDBT::ObstacleMap* oMap;
EDBT::EDBTkNN* edbt;
SearchInstance* si;
OkNNIntervalHeuristic* ki;
OkNNIntervalHeuristic* ki0;
TargetHeuristic* hi;
TargetHeuristic* hi2;
FenceHeuristic* fi;
KnnMeshEdgeFence* meshFence;
vector<Scenario> scenarios;
vector<Point> pts;
vector<vector<Point>> polys;

void load_points(istream& infile ) {
  int N;
  infile >> N;
  pts.resize(N);
  for (int i=0; i<N; i++) {
    infile >> pts[i].x >> pts[i].y;
  }
}

void load_data() {
  string scenario_path, obs_path, polys_path, pts_path, mesh_path;
  cin >> mesh_path >> polys_path >> obs_path >> pts_path;

  ifstream scenfile(scenario_path);
  ifstream meshfile(mesh_path);
  ifstream obsfile(obs_path);
  ifstream ptsfile(pts_path);
  ifstream polysfile(polys_path);

  polys = generator::read_polys(polysfile);
  load_points(ptsfile);
  mp = new Mesh(meshfile);
  m = *mp;
  oMap = new EDBT::ObstacleMap(obsfile, &m);
  meshfile.close();
  si = new SearchInstance(mp);
  ki = new OkNNIntervalHeuristic(mp);
	ki0 = new OkNNIntervalHeuristic(mp); ki0->setZero(true);
  hi = new TargetHeuristic(mp);
  hi2 = new TargetHeuristic(mp);
  fi = new FenceHeuristic(mp);
  meshFence = new KnnMeshEdgeFence(mp);
  meshFence->set_goals(pts);
  meshFence->floodfill();
  fi->set_meshFence(meshFence);
  //edbt = new EDBT::EDBTkNN(oMap);
  printf("vertices: %d, polygons: %d\n", (int)m.mesh_vertices.size(), (int)m.mesh_polygons.size());
}

TEST_CASE("Test polyanya zero heuristic") {
  int N = 10;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts);

  for (Point& start: starts) {
    ki->set_K(pts.size());
    ki->set_start_goal(start, pts);

    ki0->set_K(pts.size());
    ki0->set_start_goal(start, pts);

    int res0 = ki0->search();
    int res = ki->search();
    REQUIRE(res0 == res);
    for (int i=0; i<res0; i++) {
      double d0, d;
      d0 = ki0->get_cost(i);
      d = ki->get_cost(i);
      REQUIRE(fabs(d0 - d) < EPSILON);
    }
  }
}

TEST_CASE("Test target heuristic") {
  int N = 10;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts);
  for (Point& start: starts) {
    hi->set_K(pts.size());
    hi->set_start(start);
    hi->set_goals(pts);

    ki->set_K(pts.size());
    ki->set_start_goal(start, pts);

    int reshi = hi->search();
    int reski = ki->search();
    REQUIRE(reski == reshi);
    for (int i=0; i<reshi; i++) {
      double dhi, dki;
      dhi = hi->get_cost(i);
      dki = ki->get_cost(i);
      REQUIRE(fabs(dhi - dki) < EPSILON);
    }
  }
}

TEST_CASE("Test brute force polyanya") {
  int N = 10;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts);
  for (Point& start: starts) {
    int k = min(5, (int)pts.size());
    double cost_poly = 0, gen_poly = 0;
    vector<double> dists = si->brute_force(start, pts, k, cost_poly, gen_poly);

    ki->set_K(k);
    ki->set_start_goal(start, pts);
    int reski = ki->search();
    REQUIRE(reski == (int)dists.size());
    for (int i=0; i<reski; i++) {
      double dki = ki->get_cost(i);
      REQUIRE(fabs(dki - dists[i]) < EPSILON);
    }
  }
}

TEST_CASE("Test fence heuristic") {
  int N = 10;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts);
  for (Point& start: starts) {
    int k = min(5, (int)pts.size());
    hi->set_K(k);
    hi->set_start(start);
    hi->set_goals(pts);

    fi->set_K(k);
    fi->set_start(start);
    fi->set_goals(pts);

    int resthi = hi->search();
    int restfi = fi->search();
    for (int i=0; i<resthi; i++) {
      double dhi = hi->get_cost(i);
      double dfi = fi->get_cost(i);
      REQUIRE(fabs(dhi - dfi) < EPSILON);
    }
  }
}

int main(int argv, char* args[]) {
	cout << "Loading data..." << endl;
  load_data();
	cout << "Running test cases..." << endl;
	Catch::Session session;
	int res = session.run(argv, args);
  return res;
}
