// various testing functions
#include "expansion.h"
#include "mesh.h"
#include "geometry.h"
#include "scenario.h"
#include "searchinstance.h"
#include "knninstance.h"
#include <stdio.h>
#include <sstream>
#include <iomanip>
#include <time.h>
#include <random>
#include <iostream>
#include <fstream>

using namespace std;
using namespace polyanya;

Mesh m;
Point tp;
SearchInstance* si;
KnnInstance* ki;
vector<Scenario> scenarios;

const int MIN_X = 0, MAX_X = 1024, MIN_Y = 0, MAX_Y = 768;
const int MAX_ITER = 10000;

uniform_real_distribution<double> unif(-10, 10);
default_random_engine engine;
#define rand_point() {unif(engine), unif(engine)}

void test_io(int argc, char* argv[])
{
  m = Mesh(cin);
  if (argc == 3)
  {
      stringstream ss;
      ss << argv[1] << " " << argv[2];
      ss >> tp.x >> tp.y;
  }
  else
  {
      tp = {0, 0};
  }
  cout << "using point " << tp << endl;
  m.print(cout);
}

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
  get_path_si(idx);
  cout << "finish polyanya search, cost: " << si->get_search_micro() << "ms" << endl;

  ki->verbose = false;
  ki->set_start_goal(scen.start, {scen.goal});
  ki->search();
  get_path_knn(idx);
  cout << "finish knn search, cost: " << ki->get_search_micro() << "ms" << endl;
  if (true) {
    assert(abs(si->get_cost() - ki->get_cost(0)) < EPSILON);
    printf("---------------------------------------------------------------------\n");
    printf("%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s\n","index", "time", "succ call",
        "node gen", "node push", "node pop", "node prune", "cost");
    printf("%10d,%10.6lf,%10d,%10d,%10d,%10d,%10d,%10.6lf\n",
        idx, si->get_search_micro(), si->successor_calls, si->nodes_generated,
        si->nodes_pushed, si->nodes_popped, si->nodes_pruned_post_pop, si->get_cost());
    printf("%10d,%10.6lf,%10d,%10d,%10d,%10d,%10d,%10.6lf\n",
        idx, ki->get_search_micro(), ki->successor_calls, ki->nodes_generated,
        ki->nodes_pushed, ki->nodes_popped, ki->nodes_pruned_post_pop, ki->get_cost(0));
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
  get_path_knn(idx);
  if (false) {
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

bool is_eq(double lhs, double rhs) {
  return (abs(lhs - rhs) < EPSILON);
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
  string mesh_path = "/Users/eggeek/project/nav-mesh-ornn/meshes/aurora-merged.mesh";
  string scenario_path = "/Users/eggeek/project/nav-mesh-ornn/scenarios/aurora.scen";
  ifstream scenfile(scenario_path);
  ifstream meshfile(mesh_path);

  MeshPtr mp = new Mesh(meshfile);
  meshfile.close();
  si = new SearchInstance(mp);
  ki = new KnnInstance(mp);
  load_scenarios(scenfile, scenarios);
}

int main(int argv, char* args[]) {
  load_data();
  //test_knn(2965, scenarios[2965]);
  if (argv == 3) {
    string t = string(args[1]);
    int idx = std::atoi(args[2]);
    if (t == "knn") {
      test_knn(idx, scenarios[idx]);
    } else {
      test_polyanya(idx, scenarios[idx]);
    }
  }
  else {
    test_search();
  }
  //test_get_knn_h_value();
  // test_io(argc, argv);
  // test_containment(tp);
  // test_point_lookup_correct();
  // benchmark_point_lookup_average();
  // benchmark_point_lookup_single(tp);
  // test_projection_asserts();
  // test_reflection_asserts();
  // test_h_value_asserts();
  return 0;
}
