#include "park2poly.h"
#include "EDBTknn.h"
#include "knnheuristic.h"
#include "knninstance.h"
#include "searchinstance.h"
#include "genPoints.h"
#include "mesh.h"
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <fstream>
using namespace std;
namespace pl = polyanya;
namespace vg = EDBT;

pl::MeshPtr mp;
pl::SearchInstance* si;
pl::KnnInstance* ki;
pl::KnnHeuristic* hi;
vg::ObstacleMap* oMap;
vg::EDBTkNN* edbt;
vector<pl::Point> pts;
vector<vector<pl::Point>> polys;
string obs_path, polys_path, pts_path, mesh_path;
vector<pl::Point> starts;
string globalT;
int globalK;

void load_points(istream& infile) {
  int N;
  infile >> N;
  pts.resize(N);
  for (int i=0; i<N; i++) {
    infile >> pts[i].x >> pts[i].y;
  }
}

void load_data() {
  cin >> mesh_path >> polys_path >> obs_path >> pts_path;

  ifstream meshfile(mesh_path);
  ifstream polysfile(polys_path);
  ifstream obsfile(obs_path);
  ifstream ptsfile(pts_path);

  mp = new pl::Mesh(meshfile);
  polys = generator::read_polys(polysfile);
  oMap = new vg::ObstacleMap(obsfile, mp);
  load_points(ptsfile);

  si = new pl::SearchInstance(mp);
  ki = new pl::KnnInstance(mp);
  hi = new pl::KnnHeuristic(mp);
  edbt = new vg::EDBTkNN(oMap);
}

void dump() {
  fstream file;
  string fname = "dump-" + globalT + "-poly" + to_string(polys.size())
    + "-pts" + to_string(pts.size()) + "-k" + to_string(globalK);
  file.open(fname + ".in");
  file << mesh_path << endl;
  file << obs_path << endl;
  file << pts_path << endl;
  file << polys_path << endl;
  file.close();

  fstream ptsfile;
  ptsfile.open(fname + ".points");
  ptsfile << starts.size() << endl;
  for (size_t i=0; i<starts.size(); i++) {
    ptsfile << starts[i].x << " " << starts[i].y << endl;
  }
  ptsfile.close();
}

void edbt_vs_polyanya(pl::Point start, int k, bool verbose=false) {

  vector<pl::Point> path;

  ki->verbose = verbose;
  ki->set_K(k);
  ki->set_start_goal(start, pts);
  ki->search();

  edbt->set_start(start);
  vector<pair<vg::pPtr, double>> res = edbt->OkNN(k);

  double dist_ki, dist_edbt, cost_ki, cost_edbt;
  double density = (double) pts.size() / (double)polys.size();

  cost_ki = ki->get_search_micro();
  cost_edbt = edbt->get_search_micro();
  for (int i=0; i<k; i++) {
    dist_ki = ki->get_cost(i);
    dist_edbt = res[i].second;
    if (fabs(dist_ki - dist_edbt) > EPSILON) {
      dump();
      assert(false);
      exit(1);
    }
    ki->get_path_points(path, i);
    int vnum = (int)path.size();
    cout << k << "," << i+1 << "," << dist_ki << "," << cost_ki << "," << cost_edbt << "," << vnum << "," << density << endl;
  }
}

void heuristic_vs_polyanya(pl::Point start, int k, bool verbose=false) {

  vector<pl::Point> path;

  ki->verbose = verbose;
  ki->set_K(k);
  ki->set_start_goal(start, pts);
  int actual = ki->search();

  hi->verbose = verbose;
  hi->set_K(k);
  hi->set_start(start);
  int actual2 = hi->search();

  double dist_ki, dist_hi, cost_hi, cost_ki, h_cost, cost_polyanya;
  int gen_poly=0, push_poly=0, pop_poly=0;
  int gen_ki, push_ki, pop_ki, prune_ki, suc_call_ki;
  int gen_hi, push_hi, pop_hi, prune_hi, suc_call_hi;

  cost_polyanya = 0;
  for (pl::Point p: pts) {
    si->set_start_goal(start, p);
    si->search();
    cost_polyanya += si->get_search_micro();
    gen_poly += si->nodes_generated;
    push_poly += si->nodes_pushed;
    pop_poly += si->nodes_popped;
  }

  gen_ki = ki->nodes_generated;
  push_ki = ki->nodes_pushed;
  pop_ki = ki->nodes_popped;
  prune_ki = ki->nodes_pruned_post_pop;
  suc_call_ki = ki->successor_calls;

  gen_hi = hi->nodes_generated;
  push_hi = hi->nodes_pushed;
  pop_hi = hi->nodes_popped;
  prune_hi = hi->nodes_pruned_post_pop;
  suc_call_hi = hi->successor_calls;

  if (actual != actual2) {
    dump();
    assert(false);
    exit(1);
  }
  cost_ki = ki->get_search_micro();
  cost_hi = hi->get_search_micro();
  h_cost = hi->get_heuristic_micro();

  int i = actual - 1;

  if (i >= 0) {
    dist_ki= ki->get_cost(i);
    dist_hi= hi->get_cost(i);
    if (fabs(dist_ki - dist_hi) > EPSILON) {
      dump();
      assert(false);
      exit(1);
    }
    ki->get_path_points(path, i);
    int vnum = (int)path.size();
    cout << k << "," << i+1 << "," << dist_ki << ","
         << cost_polyanya << "," << cost_ki << "," << cost_hi << ","
         << h_cost << "," << vnum << ","
         << gen_poly << "," << push_poly << "," << pop_poly << ","
         << gen_ki << "," << push_ki << "," << pop_ki << "," << prune_ki << "," << suc_call_ki << ","
         << gen_hi << "," << push_hi << "," << pop_hi << "," << prune_hi << "," << suc_call_hi << ","
         << hi->heuristic_call << "," << hi->nodes_reevaluate << ","
         << pts.size() << "," << polys.size() << endl;
  }
}

int main(int argv, char* args[]) {
  load_data();
  if (argv == 3) { // ./bin/test [s1/s2]
    string t = string(args[1]);
    int k = atoi(args[2]);
    globalT = t;
    globalK = k;
    if (t == "s1") { // edbt vs polyanya
      edbt->set_goals(pts);

      string header = "K,order,dist,cost_ki,cost_edbt,vnum,density";
      cout << header << endl;

      int N = 200;
      generator::gen_points_in_traversable(oMap, polys, N, starts);
      for (int i=0; i<N; i++)
        edbt_vs_polyanya(starts[i], k);
    }
    else if (t == "s2") { // hueristic vs polyanya
      hi->set_goals(pts);

      string header = "K,order,dist,cost_polyanya,cost_ki,cost_hi,h_cost,vnum,gen_poly,push_poly,pop_poly,gen_ki,push_ki,pop_ki,prune_ki,suc_call_ki,gen_hi,push_hi,pop_hi,prune_hi,suc_call_hi,heuristic_call,reevaluate,pts,polys";
      cout << header << endl;

      int N = 200;
      generator::gen_points_in_traversable(oMap, polys, N, starts);
      for (int i=0; i<N; i++)
        heuristic_vs_polyanya(starts[i], k);
    }
    else assert(false);
  }
  else assert(false);
}