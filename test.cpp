// various testing functions
#include "expansion.h"
#include "mesh.h"
#include "geometry.h"
#include "scenario.h"
#include "searchinstance.h"
#include "knninstance.h"
#include "knnheuristic.h"
#include "rtree.h"
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

Mesh m;
Point tp;
SearchInstance* si;
KnnInstance* ki;
KnnHeuristic* hi;
vector<Scenario> scenarios;

const int MIN_X = 0, MAX_X = 1024, MIN_Y = 0, MAX_Y = 768;
const int MAX_ITER = 10000;

uniform_real_distribution<double> unif(-10, 10);
default_random_engine engine;
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

void get_path_knn(int idx, int top=0) {
  vector<Point> path;
  ki->get_path_points(path, top);
  const int n = (int) path.size();
  cout << "path " << idx <<"; ";
  for (int i=0; i<n; i++) {
    cout << path[i];
    if (i != n-1) cout << " ";
  }
  cout << endl;
}

void get_path_si(int idx) {
  vector<Point> path;
  si->get_path_points(path);
  const int n = (int) path.size();
  cout << "path " << idx <<"; ";
  for (int i=0; i<n; i++) {
    cout << path[i];
    if (i != n-1) cout << " ";
  }
  cout << endl;
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
  for (int i = 0; i < (int) scenarios.size(); i++) {
    //if (i == 0) break;
    test_run_scenario(i, scenarios[i]);
  }
}

void test_get_knn_h_value() {
  printf("calc:%.6lf, expected:%.6lf\n", get_knn_h_value({1, 1}, {0, 0}, {0, 2}), 1.0);
  printf("calc:%.6lf, expected:%.6lf\n", get_knn_h_value({0, 0}, {0, 0}, {0, 2}), 0.0);
  printf("calc:%.6lf, expected:%.6lf\n", get_knn_h_value({-1.0, 0}, {0, 0}, {0, 2}), 1.0);
  printf("calc:%.6lf, expected:%.6lf\n", get_knn_h_value({-1.0, 1.0}, {0, 0}, {0, 2}), 1.0);
  printf("calc:%.6lf, expected:%.6lf\n", get_knn_h_value({-1.0, 1.0}, {0, 0}, {0, 0}), sqrt(2.0));
  printf("calc:%.6lf, expected:%.6lf\n", get_knn_h_value({-1.0, 1.0}, {0, 0}, {2, 0}), sqrt(2.0));
}

void load_data() {
  //string mesh_path = "/Users/eggeek/project/nav-mesh-ornn/meshes/arena.mesh";
  //string scenario_path = "/Users/eggeek/project/nav-mesh-ornn/scenarios/arena.scen";
  string mesh_path = "/Users/eggeek/project/nav-mesh-ornn/meshes/aurora-merged.mesh";
  string scenario_path = "/Users/eggeek/project/nav-mesh-ornn/scenarios/aurora.scen";
  ifstream scenfile(scenario_path);
  ifstream meshfile(mesh_path);

  MeshPtr mp = new Mesh(meshfile);
  meshfile.close();
  si = new SearchInstance(mp);
  ki = new KnnInstance(mp);
  hi = new KnnHeuristic(mp);
  load_scenarios(scenfile, scenarios);
}

void test_knn_multi_goals() {
  Point start = scenarios[0].start;
  vector<Point> gs;
  for (int i=0; i<(int)scenarios.size(); i++) {
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
  int top = (int)gs.size();
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
  return;
  hi->verbose = false;
  hi->set_K(top);
  hi->set_start_goal(start, gs);
  int actual2 = hi->search();
  if (actual != actual2) {
    printf("got different results.\n");
    assert(false);
  }
  int cnt = 0;
  for (int i=0; i<actual2; i++) {
    vector<Point> out;
    double diff = ki->get_cost(i) - hi->get_cost(i);
    if (abs(diff) > EPSILON) {
      printf("got wrong case, diff: %.3lf, ratio: %.3lf%%\n", diff, diff / ki->get_cost(i) * 100 );
      cnt++;
      //assert(false);
    }
  }
  printf("total: %d, wrong: %d\n", actual, cnt);
}

void test_rtree() {
  bgi::rtree<std::pair<Point, int>, bgi::rstar<16> > rtree;
  pl::Point start = scenarios[0].start;
  for (int i=0; i<(int)scenarios.size(); i++) {
    rtree.insert(std::make_pair(scenarios[i].goal, i));
  }
  std::vector<std::pair<Point, int> > res;
  rtree.query(bgi::nearest(start, 20), std::back_inserter(res));
  reverse(res.begin(), res.end());
  for (auto it: res) {
    printf("(%.3lf, %.3lf) dist: %.3lf\n", it.first.x, it.first.y, it.first.distance(start));
  }
  Point p1 = rand_point();
  Point p2 = rand_point();
  bg::model::segment<Point> seg(p1, p2);
  typedef bg::model::box<Point> box;
  box b(Point{0, 0}, Point{scenarios[0].goal.x + EPSILON, scenarios[0].goal.y + EPSILON});
  res.clear();
  rtree.query(bgi::within(b) && bgi::nearest(seg, 10), std::back_inserter(res));
  reverse(res.begin(), res.end());
  printf("top1:\n");
  printf("box: (%.5lf, %.5lf) (%.5lf, %.5lf)\n", b.min_corner().x, b.min_corner().y, b.max_corner().x, b.max_corner().y);
  for (auto it: res) {
    printf("seg:[(%.3lf, %.3lf), (%.3lf, %.3lf)] to point (%.3lf, %.3lf) dist: %.3lf\n",
        seg.first.x, seg.first.y, seg.second.x, seg.second.y, it.first.x, it.first.y, it.first.distance_to_seg(seg.first, seg.second));
  }
  res.clear();
  rtree.query(bgi::nearest(seg, 10), std::back_inserter(res));
  reverse(res.begin(), res.end());
  printf("top10:\n");
  for (auto it: res) {
    printf("seg:[(%.3lf, %.3lf), (%.3lf, %.3lf)] to point (%.3lf, %.3lf) dist: %.3lf\n",
        seg.first.x, seg.first.y, seg.second.x, seg.second.y, it.first.x, it.first.y, it.first.distance_to_seg(seg.first, seg.second));
  }

  printf("size of rtree: %d\n", (int)rtree.size());
  for (int i=0; i<(int)scenarios.size(); i++) {
    size_t f = remove_ids_bulk(rtree, i);
    if (f <= 0) assert(false);
  }
  printf("size of rtree: %d\n", (int)rtree.size());
  assert(rtree.size() == 0);
  int N = 50000;
  std::vector<std::pair<Point, int> > nodes;
  for (int i=0; i<N; i++) {
    Point p = rand_point();
    nodes.push_back(std::make_pair(p, i));
  }
  for (int i=0; i<N; i++) {
    rtree.insert(nodes[i]);
  }


  printf("size of rtree: %d\n", (int)rtree.size());
  for (int i=0; i<N; i++) {
    int f = rtree.remove(std::make_pair(nodes[i].first, i));
    if (f <= 0) assert(false);
  }
  printf("size of rtree: %d\n", (int)rtree.size());
}

int main(int argv, char* args[]) {
  load_data();
  test_rtree();
  return 0;
  if (argv == 3) {
  // example1: ./bin/test knn 3
  // example2: ./bin/test poly 3
    string t = string(args[1]);
    int idx = std::atoi(args[2]);
    if (t == "knn") {
      test_knn(idx, scenarios[idx]);
    } else {
      test_polyanya(idx, scenarios[idx]);
    }
  }
  else {
    test_knn_multi_goals();
    //test_search();
  }
  //test_get_knn_h_value();
  // test_containment(tp);
  // test_point_lookup_correct();
  // benchmark_point_lookup_average();
  // benchmark_point_lookup_single(tp);
  // test_projection_asserts();
  // test_reflection_asserts();
  // test_h_value_asserts();
  return 0;
}
