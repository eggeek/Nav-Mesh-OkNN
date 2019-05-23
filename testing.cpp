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
#include "IERPolyanya.h"
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
IntervalHeuristic* ki;
IntervalHeuristic* ki0;
TargetHeuristic* hi;
FenceHeuristic* fi;
IERPolyanya* ffp;
KnnMeshEdgeFence* meshFence;
vector<Point> pts;
vector<vector<Point>> polys;
string testfile, obs_path, polys_path, pts_path, mesh_path;

void load_points(istream& infile ) {
  int N;
  infile >> N;
  pts.resize(N);
  for (int i=0; i<N; i++) {
    infile >> pts[i].x >> pts[i].y;
  }
}

void dump(const vector<pl::Point> starts) {
  // write all test data to folder ./dump/
  // run the function when a test case fail
  ofstream file;
  string fname = "dump/" + to_string(polys.size()) + "polys"
    + to_string(pts.size()) + "pts";
  cerr << fname << endl;
  file.open(fname + ".in");
  file << mesh_path << endl;
  file << obs_path << endl;
  file << pts_path << endl;
  file << polys_path << endl;
  file.close();

  ofstream ptsfile;
  ptsfile.open(fname + "-starts.points");
  ptsfile << starts.size() << endl;
  for (size_t i=0; i<starts.size(); i++) {
    ptsfile << starts[i].x << " " << starts[i].y << endl;
  }
  ptsfile.close();

  ofstream targetsfile;
  targetsfile.open(fname + "-targets.points");
  targetsfile << pts.size() << endl;
  for (size_t i=0; i< pts.size(); i++) {
    targetsfile << pts[i].x << " " << pts[i].y << endl;
  }
  targetsfile.close();
}

void load_data(const string& testcase) {
  ifstream testin(testcase);

  testin >> mesh_path >> polys_path >> obs_path >> pts_path;

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
  ki = new IntervalHeuristic(mp);
	ki0 = new IntervalHeuristic(mp); ki0->setZero(true);
  hi = new TargetHeuristic(mp);
  new TargetHeuristic(mp);
  fi = new FenceHeuristic(mp);
  meshFence = new KnnMeshEdgeFence(mp);
  ffp = new IERPolyanya(si);
  fi->set_meshFence(meshFence);
  //edbt = new EDBT::EDBTkNN(oMap);
  printf("vertices: %d, polygons: %d\n", (int)m.mesh_vertices.size(), (int)m.mesh_polygons.size());
}

void print_path(const vector<Point>& path) {
  for (int i=0; i<(int)path.size(); i++) {
    cout << "(" << path[i].x << ", " << path[i].y << ")";
    if (i == (int)path.size()-1) cout << endl;
    else cout << " ";
  }
}

TEST_CASE("poly-h0") { // test zero heuristic
  load_data(testfile);
  int N = 10;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts); // generate start points

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

TEST_CASE("poly-ht") { // test target heuristic
  load_data(testfile);
  int N = 10;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts); // generate start points
  hi->set_K(pts.size());
  hi->set_goals(pts);

  ki->set_K(pts.size());
  for (Point& start: starts) {
    hi->set_start(start);

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

TEST_CASE("poly-bf") { // brute force polyanya
  load_data(testfile);
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

TEST_CASE("fence-nn") { // Fence preprocessing for NN query
  load_data(testfile);
  int N = 1000;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts);
  meshFence->set_goals(pts);
  meshFence->floodfill();
  hi->set_K(1);
  hi->set_goals(pts);
  fi->set_goals(pts);
  for (Point& start: starts) {
    fi->set_start(start);
    hi->set_start(start);
    double nn_cost = 0.0;
    pair<int, double> res = fi->nn_query(si, nn_cost); 
    int rest_hi = hi->search();
    if (rest_hi) {
      double dist_hi = hi->get_cost(0);
      REQUIRE(fabs(res.second - dist_hi) <= EPSILON);
    } else {
      REQUIRE(res.first == -1);
    }
  }
}

TEST_CASE("fence-h") { // Fence preoprocessing for kNN
  load_data(testfile);
  int N = 1000;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts);

  meshFence->set_goals(pts);
  meshFence->floodfill();
  hi->set_goals(pts);
  for (Point& start: starts) {
    int k = min(5, (int)pts.size());
    hi->set_K(k);
    hi->set_start(start);

    fi->set_K(k);
    fi->set_start(start);
    fi->set_goals(pts);

    int resthi = hi->search();
    int restfi = fi->search();
    REQUIRE(resthi == restfi);
    for (int i=0; i<resthi; i++) {
      double dhi = hi->get_cost(i);
      double dfi = fi->get_cost(i);
      if (fabs(dhi-dfi) > EPSILON) {
        vector<Point> path;
        hi->get_path_points(path, i);
        print_path(path);
        Point goal_hi = path.back();
        fi->get_path_points(path, i);
        print_path(path);
        Point goal_fi = path.back();
        cout << "Start: " << start.x << " " << start.y << endl;
        cout << "k: " << i << endl;
        cout << "Goal hi: " << goal_hi.x << " " << goal_hi.y << endl;
        cout << "Goal fi: " << goal_fi.x << " " << goal_fi.y << endl;
      }
      REQUIRE(fabs(dhi - dfi) < EPSILON);
    }
  }
}


TEST_CASE("fast-filter") { // test fast filter in EDBT paper
  load_data(testfile);
  int N = 10;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts);
  int k = min(5, (int)pts.size());

  ffp->set_K(k);
  ffp->set_goals(pts);
  for (Point& start: starts) {
    double cost_poly = 0, gen_poly = 0;
    vector<double> dists = si->brute_force(start, pts, k, cost_poly, gen_poly);

    ffp->set_start(start);
    vector<double> dists2 = ffp->search();
    ki->set_K(k);
    ki->set_start_goal(start, pts);
    int reski = ki->search();
    REQUIRE(reski == (int)dists.size());
    REQUIRE(reski == (int)dists2.size());
    for (int i=0; i<reski; i++) {
      double dki = ki->get_cost(i);
      REQUIRE(fabs(dki - dists[i]) < EPSILON);
      REQUIRE(fabs(dki - dists2[i]) < EPSILON);
    }
  }
}

int main(int argv, char* args[]) {
  using namespace Catch::clara;
  Catch::Session session;
  auto cli = session.cli() | Opt(testfile, "testfile")["--input"]("");
  session.cli(cli);
  int resCode = session.applyCommandLine(argv, args);
  if (resCode != 0)
    return resCode;

	cout << "Running test cases..." << endl;
	return session.run(argv, args);
}
