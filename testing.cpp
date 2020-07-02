#define CATCH_CONFIG_RUNNER
#include "EDBTObstacles.h"
#include "catch.hpp"
#include "consts.h"
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
#include "RStarTree.h"
using namespace std;
using namespace polyanya;

MeshPtr mp;
Mesh m;
EDBT::ObstacleMap* oMap;
EDBT::EDBTkNN* edbt;
SearchInstance* poly_p2p;
IntervalHeuristic* poly_hi;
IntervalHeuristic* poly_hi0;
TargetHeuristic* poly_ht;
FenceHeuristic* poly_fence;
IERPolyanya* IERpoly;
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
  poly_p2p = new SearchInstance(mp);
  poly_hi = new IntervalHeuristic(mp);
	poly_hi0 = new IntervalHeuristic(mp); poly_hi0->setZero(true);
  poly_ht = new TargetHeuristic(mp);
  poly_fence = new FenceHeuristic(mp);
  meshFence = new KnnMeshEdgeFence(mp);
  IERpoly = new IERPolyanya(poly_p2p);
  poly_fence->set_meshFence(meshFence);
  edbt = new EDBT::EDBTkNN(oMap);
  printf("vertices: %d, polygons: %d\n", (int)m.mesh_vertices.size(), (int)m.mesh_polygons.size());
}

void print_path(const vector<Point>& path) {
  for (int i=0; i<(int)path.size(); i++) {
    cout << "(" << path[i].x << ", " << path[i].y << ")";
    if (i == (int)path.size()-1) cout << endl;
    else cout << " ";
  }
}


TEST_CASE("edbt") { // test competitor in EDBT paper
  load_data(testfile);
  int N = 10;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts);

  int k;
  k = min(5, (int)pts.size());
  //k = (int)pts.size();
  IERpoly->set_K(k);
  IERpoly->set_goals(pts);

  edbt->set_goals(pts);
  // Visibility Graph based approach
  // is slow, set time limit to speed up testing
  edbt->time_limit_micro = 1e6;
  for (Point& start: starts) {
    edbt->set_start(start);
    vector<pair<EDBT::pPtr, double>> res = edbt->OkNN(k);

    IERpoly->set_start(start);
    vector<double> dist = IERpoly->search();

    REQUIRE(res.size() == dist.size());
    for (int i=0; i<(int)dist.size(); i++) {
      if (res[i].second  == INF) continue;
      // this assertion may fail due to the time limit
      REQUIRE(fabs(dist[i] - res[i].second) < EPSILON);
    }
  }
}

TEST_CASE("rtree-del") {
  load_data(testfile);
  polyanya::rs::RStarTree* tree = nullptr;
  int repeat = 10;
  for (int i=0; i<repeat; i++) {
    REQUIRE(tree == nullptr);
    int N = 1000;
    vector<Point> starts;
    generator::gen_points_in_traversable(oMap, polys, N, starts);
    vector<rs::LeafNodeEntry> leaves;
    for (auto& it: starts) {
      rs::Mbr mbr(it.x, it.x, it.y, it.y);
      rs::LeafNodeEntry leaf(mbr, (rs::Data_P)&it);
      leaves.push_back(leaf);
    }
    tree = new rs::RStarTree();
    for (auto& it: leaves)
      tree->insertData(&it);
    delete tree;
  }
}

TEST_CASE("poly-h0") { // test zero heuristic
  load_data(testfile);
  int N = 10;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts); // generate start points

  for (Point& start: starts) {
    poly_hi->set_K(pts.size());
    poly_hi->set_start_goal(start, pts);

    poly_hi0->set_K(pts.size());
    poly_hi0->set_start_goal(start, pts);

    int res0 = poly_hi0->search();
    int res = poly_hi->search();
    REQUIRE(res0 == res);
    for (int i=0; i<res0; i++) {
      double d0, d;
      d0 = poly_hi0->get_cost(i);
      d = poly_hi->get_cost(i);
      REQUIRE(fabs(d0 - d) < EPSILON);
    }
  }
}

TEST_CASE("poly-ht") { // test target heuristic
  load_data(testfile);
  int N = 10;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts); // generate start points
  poly_ht->set_K(pts.size());
  poly_ht->set_goals(pts);

  poly_hi->set_K(pts.size());
  for (Point& start: starts) {
    poly_ht->set_start(start);

    poly_hi->set_start_goal(start, pts);

    int reshi = poly_ht->search();
    int reski = poly_hi->search();
    REQUIRE(reski == reshi);
    for (int i=0; i<reshi; i++) {
      double dhi, dki;
      dhi = poly_ht->get_cost(i);
      dki = poly_hi->get_cost(i);
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
    vector<double> dists = poly_p2p->brute_force(start, pts, k, cost_poly, gen_poly);

    poly_hi->set_K(k);
    poly_hi->set_start_goal(start, pts);
    int reski = poly_hi->search();
    REQUIRE(reski == (int)dists.size());
    for (int i=0; i<reski; i++) {
      double dki = poly_hi->get_cost(i);
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
  poly_ht->set_K(1);
  poly_ht->set_goals(pts);
  poly_fence->set_goals(pts);
  for (Point& start: starts) {
    poly_fence->set_start(start);
    poly_ht->set_start(start);
    double nn_cost = 0.0;
    pair<int, double> res = poly_fence->nn_query(poly_p2p, nn_cost);
    int rest_hi = poly_ht->search();
    if (rest_hi) {
      double dist_hi = poly_ht->get_cost(0);
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
  poly_ht->set_goals(pts);
  for (Point& start: starts) {
    int k = min(5, (int)pts.size());
    poly_ht->set_K(k);
    poly_ht->set_start(start);

    poly_fence->set_K(k);
    poly_fence->set_start(start);
    poly_fence->set_goals(pts);

    int resthi = poly_ht->search();
    int restfi = poly_fence->search();
    REQUIRE(resthi == restfi);
    for (int i=0; i<resthi; i++) {
      double dhi = poly_ht->get_cost(i);
      double dfi = poly_fence->get_cost(i);
      if (fabs(dhi-dfi) > EPSILON) {
        vector<Point> path;
        poly_ht->get_path_points(path, i);
        print_path(path);
        Point goal_hi = path.back();
        poly_fence->get_path_points(path, i);
        print_path(path);
        Point goal_fi = path.back();
        cout << "Start: " << start.x << " " << start.y << endl;
        cout << "k: " << i << endl;
        cout << "Goal poly_ht: " << goal_hi.x << " " << goal_hi.y << endl;
        cout << "Goal poly_fence: " << goal_fi.x << " " << goal_fi.y << endl;
      }
      REQUIRE(fabs(dhi - dfi) < EPSILON);
    }
  }
}


TEST_CASE("poly-IER") { // test polyanya with incremental euclidean restriction
  load_data(testfile);
  int N = 10;
  vector<Point> starts;
  generator::gen_points_in_traversable(oMap, polys, N, starts);
  int k = min(5, (int)pts.size());

  IERpoly->set_K(k);
  IERpoly->set_goals(pts);
  for (Point& start: starts) {
    double cost_poly = 0, gen_poly = 0;
    vector<double> dists = poly_p2p->brute_force(start, pts, k, cost_poly, gen_poly);

    IERpoly->set_start(start);
    vector<double> dists2 = IERpoly->search();
    poly_hi->set_K(k);
    poly_hi->set_start_goal(start, pts);
    int reski = poly_hi->search();
    REQUIRE(reski == (int)dists.size());
    REQUIRE(reski == (int)dists2.size());
    for (int i=0; i<reski; i++) {
      double dki = poly_hi->get_cost(i);
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
