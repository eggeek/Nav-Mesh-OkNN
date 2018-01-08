// various testing functions
#include "expansion.h"
#include "mesh.h"
#include "RStarTree.h"
#include "geometry.h"
#include "scenario.h"
#include "searchinstance.h"
#include "knninstance.h"
#include "knnheuristic.h"
#include "RStarTreeUtil.h"
#include "rtree.h"
#include "EDBTknn.h"
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

void load_data() {
  string mesh_path = "/Users/eggeek/project/nav-mesh-ornn/meshes/arena.mesh";
  string scenario_path = "/Users/eggeek/project/nav-mesh-ornn/scenarios/arena.scen";
  string obs_path = "/Users/eggeek/project/nav-mesh-ornn/polygons/arena.poly";
  //string mesh_path = "/Users/eggeek/project/nav-mesh-ornn/meshes/aurora-merged.mesh";
  //string scenario_path = "/Users/eggeek/project/nav-mesh-ornn/scenarios/aurora.scen";
  //string obs_path = "/Users/eggeek/project/nav-mesh-ornn/polygons/aurora.poly";
  ifstream scenfile(scenario_path);
  ifstream meshfile(mesh_path);
  ifstream obsfile(obs_path);

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
  int top = 5;
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

void test_heuristic_knn(int idx) {
  Point start = scenarios[idx].start;
  vector<Point> gs;
  //int N = 1000;
  int N = (int)scenarios.size() / 10;
  for (int i=0; i<N; i++) {
    //gs.push_back(gen_rand_point_in_mesh());
    gs.push_back(scenarios[i].goal);
  }
  //int top = (int)gs.size() / 2;
  int top = 5;
  ki->verbose = false;
  ki->set_K(top);
  ki->set_start_goal(start, gs);
  int actual = ki->search();
  double cost_ki = ki->get_search_micro();
  hi->verbose = false;
  hi->set_K(top);
  hi->set_start_goal(start, gs);
  int actual2 = hi->search();
  double cost_hi = hi->get_search_micro();

  printf("ki:%.5lf, hi:%.5lf, %.5lf faster\n", cost_ki, cost_hi, cost_ki / cost_hi);
  printf("ki: node_gen: %d, nodes_pushed:%d, nodes_popped:%d\n", ki->nodes_generated, ki->nodes_pushed, ki->nodes_popped);
  printf("hi: node_gen: %d, nodes_pushed:%d, nodes_popped:%d\n", hi->nodes_generated, hi->nodes_pushed, hi->nodes_popped);

  if (actual != actual2) {
    printf("got different results, actual:%d, actual2:%d.\n", actual, actual2);
    assert(false);
  }
  int cnt = 0;
  for (int i=0; i<actual; i++) {
    vector<Point> out;
    int gid = ki->get_gid(i);
    int gord = hi->get_goal_ord(gid);
    if (gord == -1) continue;
    double diff = ki->get_cost(i) - hi->get_cost(gord);
    if (abs(diff) > EPSILON) {
      printf("got wrong case, diff: %.3lf, ratio: %.3lf%%\n", diff, diff / ki->get_cost(i) * 100 );
      cnt++;
      get_path_knn(i, i);
      get_path_hknn(i, i);
      if (diff > 0) assert(false);
    }
  }
  printf("total: %d, wrong: %d\n", actual, cnt);
}

void test_edbt(int idx, Scenario scen) {
  edbt->set_start_goals(scen.start, {scen.goal});
  pair<EDBT::pPtr, double> res = edbt->OkNN(1).back();
  vector<Point> path;
  edbt->get_path(0, path);
  printf("idx:%d, cost: %5lf, dist: %.5lf\n", idx, edbt->get_search_micro(), res.second);
  print_path(path, idx);
}

bool compare_edbt_polyanya(int idx, Scenario scen, int verbose=false) {
  ki->verbose = verbose;
  ki->set_K(1);
  ki->set_start_goal(scen.start, {scen.goal});
  ki->search();
  double dist_ki = ki->get_cost(0);
  double cost_ki = ki->get_search_micro();

  edbt->set_start_goals(scen.start, {scen.goal});
  vector<pair<EDBT::pPtr, double>> res =  edbt->OkNN(1);
  double dist_edbt = res.back().second;
  double cost_edbt = edbt->get_search_micro();

  printf("%10d,%10.5lf,%10.5lf,%10.5lf,%10.5lf\n", idx, dist_ki, dist_edbt, cost_ki, cost_edbt);
  if (fabs(dist_edbt - dist_ki) > EPSILON) {
    get_path_knn(idx);

    vector<Point> path;
    edbt->get_path(0, path);
    print_path(path, idx);
    return false;
  }
    //assert(false);
  return true;
}

int main(int argv, char* args[]) {
  load_data();
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
        compare_edbt_polyanya(idx, scenarios[idx], true);
      else {
        //for (int i=0; i<(int)scenarios.size(); i++) test_heuristic_knn(i);
        printf("%10s,%10s,%10s,%10s,%10s\n","idx","dist_ki","dist_edbt","cost_ki","cost_edbt");
        int cnt = 0;
        for (int i = 0; i < (int) scenarios.size(); i++) {
          //test_run_scenario(i, scenarios[i]);
          cnt += compare_edbt_polyanya(i, scenarios[i]);
        }
        printf("Total: %d, Correct: %d\n", (int)scenarios.size(), cnt);
      }
    }
  }
  else {
    Scenario scen = scenarios[0];
    // (3, 15) (19, 18)
    scen.start = Point{1, 10};
    scen.goal = Point{45, 10};
    compare_edbt_polyanya(-1, scen);
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
  return 0;
}
