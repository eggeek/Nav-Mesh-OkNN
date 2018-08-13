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
TargetHeuristic* hi2;
FenceHeuristic* fi;
IERPolyanya* ffp;
KnnMeshEdgeFence* meshFence;
vector<Scenario> scenarios;
vector<Point> pts;
vector<vector<Point>> polys;
string obs_path, polys_path, pts_path, mesh_path;

void load_points(istream& infile ) {
  int N;
  infile >> N;
  pts.resize(N);
  for (int i=0; i<N; i++) {
    infile >> pts[i].x >> pts[i].y;
  }
}

void dump(const vector<pl::Point> starts) {
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
  ptsfile.open(fname + "starts.points");
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

void load_data() {
  cin >> mesh_path >> polys_path >> obs_path >> pts_path;

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
  hi2 = new TargetHeuristic(mp);
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


TEST_CASE("BFPolyanya") {
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

TEST_CASE("FENCE") {
  // data from input/brc202d-20.in
  Point start = {291, 302};
  //pts.clear();
  // int N = 1;
  //vector<Point> starts;
  //generator::gen_points_in_traversable(oMap, polys, N, starts);
  //Point start = starts.back();
  //meshFence->verbose = true;
  meshFence->set_goals(pts);
  meshFence->floodfill();
  int k = min(3, (int)pts.size());
  hi->set_K(k);
  hi->set_start(start);
  hi->set_goals(pts);
  int resthi = hi->search();
  for (int i=0; i<k; i++) {
    cout << hi->get_cost(i) << endl;
    vector<Point> path;
    hi->get_path_points(path, i);
    print_path(path);
  }

  //fi->verbose = true;
  fi->set_K(k);
  fi->set_start(start);
  fi->set_goals(pts);

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
      cout << "dist hi: " << dhi << ", dist fi: " << dfi << endl;
    }
    REQUIRE(fabs(dhi - dfi) < EPSILON);
  }
}

TEST_CASE("FenceNN") {
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

TEST_CASE("fence_heuristic") {
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

TEST_CASE("fence_knn") {
  int N = 1000;
  vector<Point> starts;
  pts.clear();
  generator::gen_points_in_traversable(oMap, polys, (int)polys.size(), pts);
  generator::gen_points_in_traversable(oMap, polys, N, starts);
  meshFence->set_goals(pts);
  meshFence->floodfill();
  dump(starts);
  for (auto& start: starts) {
    for (int i=1; i<=min(50, (int)pts.size()); i += 5) {
      ki->set_start_goal(start, pts);
      ki->set_K(i);
      int cntki = ki->search();
      fi->set_start(start);
      fi->set_K(i);
      fi->set_goals(pts);
      int cntfi = fi->search();
      REQUIRE(cntki == cntfi);
      for (int j=0; j<cntki; j++) {
        double dist_ki = ki->get_cost(j);
        double dist_fi = fi->get_cost(j);
        if (fabs(dist_ki - dist_fi) > EPSILON) {
          cout << "Start: " << start.x << " " << start.y << endl;
        }
        REQUIRE(fabs(dist_ki - dist_fi) <= EPSILON);
      }
    }
  }  
}

TEST_CASE("no_reassign") {
  int N = 10;
  int k = min(2, (int)pts.size());
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts);
  hi->set_K(k);
  hi->set_goals(pts);
  hi->set_reassign(true);
  hi2->set_K(k);
  hi2->set_goals(pts);
  hi2->set_reassign(false);
  for (Point& start: starts) {
    hi->set_start(start);
    hi2->set_start(start);
    int res = hi->search();
    int res2 = hi2->search();
    REQUIRE(res == res2);
    for (int i=0; i<res; i++) {
      double d = hi->get_cost(i);
      double d2 = hi2->get_cost(i); 
      REQUIRE(fabs(d - d2) < EPSILON);
    }
  }
}

TEST_CASE("FastFilter") {
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
	cout << "Loading data..." << endl;
  load_data();
	cout << "Running test cases..." << endl;
	Catch::Session session;
	int res = session.run(argv, args);
  return res;
}
