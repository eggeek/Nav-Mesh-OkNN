// various testing functions
#include "expansion.h"
#include "genPoints.h"
#include "mesh.h"
#include "RStarTree.h"
#include "geometry.h"
#include "scenario.h"
#include "searchinstance.h"
#include "knninstance.h"
#include "knnheuristic.h"
#include "RStarTreeUtil.h"
#include "EDBTknn.h"
#include "park2poly.h"
#include <stdio.h>
#include <sstream>
#include <iomanip>
#include <time.h>
#include <random>
#include <iostream>
#include <fstream>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/box.hpp>

using namespace std;
using namespace polyanya;

MeshPtr mp;
Mesh m;
Point tp;
EDBT::ObstacleMap* oMap;
EDBT::EDBTkNN* edbt;
SearchInstance* si;
KnnInstance* ki;
KnnHeuristic* hi;
vector<Scenario> scenarios;
vector<Point> pts;
vector<vector<Point>> polys;
Point global_start;

const int MIN_X = 0, MAX_X = 1024, MIN_Y = 0, MAX_Y = 768;
const int MAX_ITER = 10000;

uniform_real_distribution<double> unif(-10, 10);
uniform_real_distribution<double> unix;
uniform_real_distribution<double> uniy;
default_random_engine engine;
std::random_device rd;
std::mt19937 mt(rd());

#define rand_point() {unif(engine), unif(engine)}


void test_containment(Point test_point)
{
    for (int i = 0; i < (int) m.mesh_polygons.size(); i++)
    {
        cout << setw(4) << i << setw(0) << " "
             << m.poly_contains_point(i, test_point) << endl;
    }
}

void benchmark_point_lookup_single(Point test_point)
{
    clock_t t;
    t = clock();
    PointLocation pl = m.get_point_location(test_point);
    t = clock() - t;
    cout << "Took " << t << " clock ticks." << endl;
    cout << "That should be " << (t/1.0/CLOCKS_PER_SEC * 1e6)
        << " microseconds." << endl;
    cout << "Point " << test_point << " returns:" << endl;
    cout << pl << endl;
    cout << "from get_point_location." << endl;
}

void benchmark_point_lookup_average()
{
    clock_t t;
    Point p;
    t = clock();
    for (int y = MIN_Y; y <= MAX_Y; y++)
    {
        for (int x = MIN_X; x <= MAX_X; x++)
        {
            p.x = x;
            p.y = y;
            m.get_point_location(p);
        }
    }
    t = clock() - t;
    const double total_micro = (t/1.0/CLOCKS_PER_SEC * 1e6);
    const double average_micro = total_micro / ((MAX_X + 1) * (MAX_Y + 1));
    cout << "Point lookup took " << average_micro << "us on average." << endl;
}

void test_point_lookup_correct()
{
    cout << "Confirming that get_point_location is correct" << endl
         << "(this will take a while)..." << endl;
    for (int y = MIN_Y; y <= MAX_Y; y++)
    {
        cout << "y=" << y << endl;
        for (int x = MIN_X; x <= MAX_X; x++)
        {
            Point test_point = {(double)x, (double)y};
            PointLocation pl       = m.get_point_location(test_point),
                          pl_naive = m.get_point_location_naive(test_point);
            // Do some checks: continue if it's the same.
            if (pl != pl_naive)
            {
                cout << "Found discrepancy at " << test_point << endl;
                cout << "Method gives " << pl << endl;
                cout << "Naive gives " << pl_naive << endl;
            }
        }
    }
}

void test_projection_asserts()
{
    Point a, b, c, d;
    double d1, d2, d3;
    for (int i = 0; i < MAX_ITER; i++)
    {
        a = rand_point();
        b = rand_point();
        c = rand_point();
        d = rand_point();
        line_intersect_time(a, b, c, d, d1, d2, d3);
    }
}

void test_reflection_asserts()
{
    Point a, b, c;
    for (int i = 0; i < MAX_ITER; i++)
    {
        a = rand_point();
        b = rand_point();
        c = rand_point();
        reflect_point(a, b, c);
    }
}

void test_h_value_asserts()
{
    Point a, b, c, d;
    for (int i = 0; i < MAX_ITER; i++)
    {
        a = rand_point();
        b = rand_point();
        c = rand_point();
        d = rand_point();
        get_h_value(a, b, c, d);
    }
}

void print_path(vector<Point>& path, int idx) {
  const int n = (int) path.size();
  cout << "path " << idx <<"; ";
  for (int i=0; i<n; i++) {
    cout << path[i];
    if (i != n-1) cout << " ";
  }
  cout << endl;
}

void get_path_knn(int idx, int top=0) {
  vector<Point> path;
  ki->get_path_points(path, top);
  print_path(path, idx);
}

void get_path_hknn(int idx, int top=0) {
  vector<Point> path;
  hi->get_path_points(path, top);
  print_path(path, idx);
}

void get_path_si(int idx) {
  vector<Point> path;
  si->get_path_points(path);
  print_path(path, idx);
}

void test_run_scenario(int idx, Scenario scen) {
  si->verbose = false;
  si->set_start_goal(scen.start, scen.goal);
  si->search();
  //get_path_si(idx);
  cout << "finish polyanya search, cost: " << si->get_search_micro() << "ms" << endl;

  hi->verbose = false;
  hi->set_start_goal(scen.start, {scen.goal});
  hi->set_K(1);
  hi->search();
  //get_path_knn(idx);
  cout << "finish knn search, cost: " << hi->get_search_micro() << "ms" << endl;
  if (true) {
    assert(abs(si->get_cost() - hi->get_cost(0)) < EPSILON);
    printf("---------------------------------------------------------------------\n");
    printf("%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s\n","index", "time", "succ call",
        "node gen", "node push", "node pop", "node prune", "cost");
    printf("%10d,%10.6lf,%10d,%10d,%10d,%10d,%10d,%10.6lf\n",
        idx, si->get_search_micro(), si->successor_calls, si->nodes_generated,
        si->nodes_pushed, si->nodes_popped, si->nodes_pruned_post_pop, si->get_cost());
    printf("%10d,%10.6lf,%10d,%10d,%10d,%10d,%10d,%10.6lf\n",
        idx, hi->get_search_micro(), hi->successor_calls, hi->nodes_generated,
        hi->nodes_pushed, hi->nodes_popped, hi->nodes_pruned_post_pop, hi->get_cost(0));
    printf("---------------------------------------------------------------------\n");
  }
}

void test_polyanya(int idx,Scenario scen) {
  si->verbose = true;
  si->set_start_goal(scen.start, scen.goal);
  si->search();
  get_path_si(idx);
}

void test_knn(int idx, Scenario scen) {
  ki->verbose = true;
  ki->set_start_goal(scen.start, {scen.goal});
  ki->search();
  //get_path_knn(idx);
  if (true) {
    printf("---------------------------------------------------------------------\n");
    printf("%10d,%10.6lf,%10d,%10d,%10d,%10d,%10d,%10.6lf\n",
        idx, ki->get_search_micro(), ki->successor_calls, ki->nodes_generated,
        ki->nodes_pushed, ki->nodes_popped, ki->nodes_pruned_post_pop, ki->get_cost(0));
    printf("---------------------------------------------------------------------\n");
  }
}

void test_search() {

  //int i = 2965;
  //test_polyanya(i, scenarios[i]);
  //test_knn(i, scenarios[i]);
  //test_run_scenario(i, scenarios[i]);
}

void load_points(istream& infile ) {
  int N;
  infile >> N;
  pts.resize(N);
  for (int i=0; i<N; i++) {
    infile >> pts[i].x >> pts[i].y;
  }
  infile >> global_start.x >> global_start.y;
}

void load_data() {
  string scenario_path, obs_path, polys_path, pts_path, mesh_path;
  cin >> scenario_path >> obs_path >> polys_path >> pts_path >> mesh_path;

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
  unix = uniform_real_distribution<double>(m.get_minx(), m.get_maxx());
  uniy = uniform_real_distribution<double>(m.get_miny(), m.get_maxy());
  meshfile.close();
  si = new SearchInstance(mp);
  ki = new KnnInstance(mp);
  hi = new KnnHeuristic(mp);
  edbt = new EDBT::EDBTkNN(oMap);
  load_scenarios(scenfile, scenarios);
  printf("vertices: %d, polygons: %d\n", (int)m.mesh_vertices.size(), (int)m.mesh_polygons.size());
}

Point gen_rand_point_in_mesh() {
  return {unix(mt), uniy(mt)};
}

void test_knn_multi_goals(int idx) {
  //Point start = gen_rand_point_in_mesh();
  Point start = scenarios[idx].start;
  vector<Point> gs;
  //int N = 1000;
  int N = (int)scenarios.size() / 10;
  for (int i=0; i<N; i++) {
    //gs.push_back(gen_rand_point_in_mesh());
    gs.push_back(scenarios[i].goal);
    ki->verbose = false;
    ki->set_start_goal(start, {gs.back()});
    int cnt1 = ki->search();
    si->verbose = false;
    si->set_start_goal(start, gs.back());
    int cnt2 = (int)si->search();
    if (cnt1 == 0) {
      if (cnt2) assert(false);
    } else {
      double diff = ki->get_cost(0) - si->get_cost();
      if (abs(diff) > EPSILON) {
        printf("got wrong case.\n");
        printf("start(%.5lf, %.5lf) goal(%.5lf, %.5lf) diff:%.5lf\n", start.x, start.y, gs.back().x, gs.back().y, diff);
        assert(false);
      }
    }
  }
  //int top = (int)gs.size() / 2;
  int top = 6;
  ki->verbose = false;
  ki->set_K(top);
  ki->set_start_goal(start, gs);
  int actual = ki->search();
  for (int i=0; i<actual; i++) {
    vector<Point> out;
    ki->get_path_points(out, i);
    Point goal = out.back();
    si->set_start_goal(start, goal);
    si->search();
    double diff = ki->get_cost(i) - si->get_cost();
    if (abs(diff) > EPSILON) {
      printf("got wrong case.\n");
      get_path_knn(i);
      get_path_si(i);
      assert(false);
    }
  }
}

int debug_heuristic_knn(int idx, bool verbose=false) {
  //Point start = scenarios[idx].start;
  int N = min((int)pts.size(), 5);
  vector<Point> gs;
  generator::gen_points_in_traversable(oMap, polys, N, gs);
  Point start = pts[idx];
  //random_shuffle(gs.begin(), gs.end());
  //if (N < (int)gs.size())
  //  gs.erase(gs.begin() + N, gs.end());
  if (verbose) {
    cout << N << endl;
    for (auto i: gs)
      cout << i.x << " " << i.y << endl;
  }
  int top = 1;
  ki->verbose = verbose;
  ki->set_K(top);
  ki->set_start_goal(start, gs);
  int actual = ki->search();
  double cost_ki = ki->get_search_micro();
  if (verbose) {
    for (int i=0; i<actual; i++)
      get_path_knn(i, i);
  }

  hi->verbose = verbose;
  hi->set_K(top);
  hi->set_start_goal(start, gs);
  int actual2 = hi->search();
  double cost_hi = hi->get_search_micro();

  //printf("ki:%.5lf, hi:%.5lf, %.5lf faster\n", cost_ki, cost_hi, cost_ki / cost_hi);
  //printf("ki: node_gen: %d, nodes_pushed:%d, nodes_popped:%d\n", ki->nodes_generated, ki->nodes_pushed, ki->nodes_popped);
  //printf("hi: node_gen: %d, nodes_pushed:%d, nodes_popped:%d\n", hi->nodes_generated, hi->nodes_pushed, hi->nodes_popped);


  if (actual != actual2) {
    printf("got different results, actual:%d, actual2:%d.\n", actual, actual2);
    assert(false);
  }
  int cnt = 0;
  for (int i=0; i<actual; i++) {
    //vector<Point> out;
    //int gid = ki->get_gid(i);
    //int gord = hi->get_gid(gid);
    //if (gord == -1) continue;
    //
    cout << setw(3) << idx << ",";
    cout << setw(3) << i << ",";
    cout << setw(3) << top << ",";
    cout << setw(15) << ki->get_cost(i) << ",";
    cout << setw(15) << hi->get_cost(i) << ",";
    cout << setw(15) << cost_ki << ",";
    cout << setw(15) << cost_hi << ",";
    cout << setw(15) << cost_ki / cost_hi << ",";
    cout << setw(15) << hi->get_heuristic_micro() << ",";
    cout << setw(15) << hi->get_angle_micro() << endl;

    if (verbose) {
      get_path_knn(i, i);
      get_path_hknn(i, i);
    }
    double diff = ki->get_cost(i) - hi->get_cost(i);
    if (abs(diff) > EPSILON) {
      printf("got wrong case, ki: %6lf, hi: %6lf, diff: %.3lf\n",
          ki->get_cost(i), hi->get_cost(i), diff);
      cnt++;
      get_path_knn(i, i);
      get_path_hknn(i, i);
    }
  }
  if (cnt)
    assert(false);
  return 1;
}

void test_edbt(int idx, Scenario scen) {
  edbt->set_start_goals(scen.start, {scen.goal});
  pair<EDBT::pPtr, double> res = edbt->OkNN(1).back();
  vector<Point> path;
  edbt->get_path(0, path);
  printf("idx:%d, cost: %5lf, dist: %.5lf\n", idx, edbt->get_search_micro(), res.second);
  print_path(path, idx);
}

int compare_edbt_polyanya(int idx, Point start, int K=1, int verbose=false) {
  ki->verbose = verbose;
  ki->set_K(K);
  ki->set_start_goal(start, pts);
  ki->search();
  if (verbose) {
    cout << "knn-polyanya:" << endl;
    for (int i=0; i<K; i++) {
      get_path_knn(idx, i);
      cout << setw(10) << idx << "/" << i << ",";
      cout << setw(20) << ki->get_cost(i) << ",";
      cout << setw(20) << ki->get_search_micro()<< endl;
    }
  }

  hi->verbose = verbose;
  hi->set_K(K);
  hi->set_start_goal(start, pts);
  hi->search();
  if (verbose) {
    cout << "heuristic-polyanya:" << endl;
    for (int i=0; i<K; i++) {
      get_path_hknn(idx, i);
      cout << setw(10) << idx << "/" << i << ",";
      cout << setw(20) << hi->get_cost(i) << ",";
      cout << setw(20) << hi->get_search_micro()<< endl;
    }
  }

  edbt->set_start_goals(start, pts);
  vector<pair<EDBT::pPtr, double>> res =  edbt->OkNN(K);
  if (verbose) {
    cout << "EDBT:" << endl;
    for (int i=0; i<K; i++) {
      vector<Point> path;
      edbt->get_path(i, path);
      print_path(path, idx);
      cout << setw(10) << idx << "/" << i << ",";
      cout << setw(20) << res[i].second << ",";
      cout << setw(20) << edbt->get_search_micro()<< endl;
    }
  }

  int cnt = 0;
  for (int i=0; i<K; i++) {
    double dist_ki = ki->get_cost(i);
    double cost_ki = ki->get_search_micro();

    double dist_edbt = res[i].second;
    double cost_edbt = edbt->get_search_micro();

    double dist_hi = hi->get_cost(i);
    double cost_hi = hi->get_search_micro();

    vector<Point> path;
    ki->get_path_points(path, i);
    int vnum_ki = (int)path.size();

    edbt->get_path(i, path);
    int vnum_edbt = (int)path.size();
    cout << setw(10) << idx << ",";
    cout << setw(5) << K << ",";
    cout << setw(5) << i << ",";
    cout << setw(20) << dist_ki << ",";
    cout << setw(20) << dist_hi << ",";
    cout << setw(20) << dist_edbt << ",";
    cout << setw(20) << cost_ki << ",";
    cout << setw(20) << cost_hi << ",";
    cout << setw(20) << cost_edbt << ",";
    cout << setw(10) << vnum_ki << ",";
    cout << setw(10) << vnum_edbt << endl;
    if (fabs(dist_hi - dist_ki) > EPSILON) {
      cout << "Wrong: " << endl;
      ki->get_path_points(path, i);
      print_path(path, idx);
      path.clear();
      hi->get_path_points(path, i);
      print_path(path, idx);
      //assert(false);
    }
    else cnt++;
  }
  return cnt;
}

void run_knnheuristic() {
  int top = 5;
  int N = 10;
  Point start = pts[0];
  vector<Point> gs = vector<Point>(pts.begin(), pts.begin() + N);
  hi->set_K(top);
  hi->set_start_goal(start, gs);
  hi->search();
}

int main(int argv, char* args[]) {
  load_data();
  //run_knnheuristic();
  //return 0;
  if (argv == 3) {
  // example1: ./bin/test knn 3
  // example2: ./bin/test poly 3
    string t = string(args[1]);
    int idx = std::atoi(args[2]);
    if (t == "knn") {
      test_knn(idx, scenarios[idx]);
    }
    else if (t == "poly") {
      test_polyanya(idx, scenarios[idx]);
    }
    else if (t == "edbt") {
      test_edbt(idx, scenarios[idx]);
    }
    else if (t == "cmp") {
      if (idx >= 0)
        compare_edbt_polyanya(idx, pts[idx], 5, true);
      else {
        printf("%10s,%5s,%5s,%20s,%20s,%20s,%20s,%20s,%20s,%10s,%10s\n",
            "idx","K","order","dist_ki","dist_hi","dist_edbt","cost_ki","cost_hi","cost_edbt", "vnum_ki", "vnum_edbt");
        int total = 0;
        for (int K=5; K<=20; K+=5) {
          int cnt = 0;
          for (int i = 0; i < (int) pts.size(); i++) {
            //test_run_scenario(i, scenarios[i]);
            cnt += compare_edbt_polyanya(i, pts[i], K);
            total++;
          }
          printf("Total: %d, Correct: %d\n", total, cnt);
        }
      }
    }
    else if (t == "db") {
      if (idx != -1)
        debug_heuristic_knn(idx, true);
      else {
        printf("%3s,%3s,%3s,%15s,%15s,%15s,%15s,%15s,%15s,%15s\n",
               "idx","order","K","dist_ki","dist_hi","cost_ki", "cost_hi", "faster", "heuristic_using", "angle_using");
        int tot = 0, cnt = 0;
        random_shuffle(pts.begin(), pts.end());
        int N = min((int)pts.size(), 500);
        for (int i=0; i<N; i++) {
          tot ++;
          cnt += debug_heuristic_knn(i);
        }
        cout << "Total: " << tot << ", Cnt: " << cnt << endl;
      }
    }
  }

  // test_containment(tp);
  // test_point_lookup_correct();
  // benchmark_point_lookup_average();
  // benchmark_point_lookup_single(tp);
  // test_projection_asserts();
  // test_reflection_asserts();
  // test_h_value_asserts();
  if (edbt) delete edbt;
  if (si) delete si;
  if (ki) delete ki;
  if (hi) delete hi;
  if (mp) delete mp;
  if (oMap) delete oMap;
  return 0;
}
