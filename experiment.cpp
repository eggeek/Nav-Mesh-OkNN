#include "park2poly.h"
#include "EDBTknn.h"
#include "knnheuristic.h"
#include "knninstance.h"
#include "searchinstance.h"
#include "genPoints.h"
#include "knnMeshEdge.h"
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
pl::KnnHeuristic* hi2;
pl::KnnMeshEdgeDam* meshDam;
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
  hi2 = new pl::KnnHeuristic(mp);
  edbt = new vg::EDBTkNN(oMap);
  meshDam = new pl::KnnMeshEdgeDam(mp);
}

void dump() {
  ofstream file;
  string fname = "dump-" + globalT + "-poly" + to_string(polys.size())
    + "-pts" + to_string(pts.size()) + "-k" + to_string(globalK);
  file.open(fname + ".in");
  file << mesh_path << endl;
  file << obs_path << endl;
  file << pts_path << endl;
  file << polys_path << endl;
  file.close();

  ofstream ptsfile;
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

  double dist_ki, dist_edbt, dist_hi, dist_hi2, cost_ki, cost_edbt;
  double density = (double) pts.size() / (double)polys.size();

  cost_ki = ki->get_search_micro();
  cost_edbt = edbt->get_search_micro();
  for (int i=0; i<k; i++) {
    dist_ki = ki->get_cost(i);
    dist_hi = hi->get_cost(i);
    dist_hi2 = hi2->get_cost(i);
    dist_edbt = res[i].second;
    if (fabs(dist_ki - dist_edbt) > EPSILON || fabs(dist_hi - dist_hi2) > EPSILON ||
        fabs(dist_ki - dist_hi) > EPSILON) {
      dump();
      assert(false);
      exit(1);
    }
    ki->get_path_points(path, i);
    int vnum = (int)path.size();
    cout << k << "," << i+1 << "," << dist_ki << "," << cost_ki << "," << cost_edbt << "," << vnum << "," << density << endl;
  }
}

void heuristic_vs_polyanya(pl::Point start, int k, vector<string>& cols, bool verbose=false) {

  vector<pl::Point> path;

  ki->verbose = verbose;
  ki->set_K(k);
  ki->set_start_goal(start, pts);
  int actual = ki->search();

  hi->verbose = verbose;
  hi->set_K(k);
  hi->set_start(start);
  int actual2 = hi->search();

  hi2->verbose = verbose;
  hi2->set_K(k);
  hi2->set_start(start);

  double dist_si, dist_ki, dist_hi, dist_hi2, cost_hi, cost_hi2, cost_ki, h_cost, h_cost2, cost_polyanya, precost;
  int gen_poly=0, push_poly=0, pop_poly=0;
  int gen_ki, push_ki, pop_ki, prune_ki;
  int gen_hi, push_hi, pop_hi, prune_hi, hcall, reevaluate;
  int gen_hi2, push_hi2, pop_hi2, prune_hi2, hcall2, reevaluate2;
  int gen_pre;
  int edgecnt, damcnt;

  precost = meshDam->get_processing_micro();
  edgecnt = meshDam->edgecnt;
  damcnt = meshDam->damcnt;
  gen_pre = meshDam->nodes_generated;


  cost_polyanya = 0;
  dist_si = INF;
  for (pl::Point p: pts) {
    si->set_start_goal(start, p);
    si->search();
    dist_si = min(dist_si, si->get_cost());
    cost_polyanya += si->get_search_micro();
    gen_poly += si->nodes_generated;
    push_poly += si->nodes_pushed;
    pop_poly += si->nodes_popped;
  }

  gen_ki = ki->nodes_generated;
  push_ki = ki->nodes_pushed;
  pop_ki = ki->nodes_popped;
  prune_ki = ki->nodes_pruned_post_pop;

  gen_hi = hi->nodes_generated;
  push_hi = hi->nodes_pushed;
  pop_hi = hi->nodes_popped;
  prune_hi = hi->nodes_pruned_post_pop;
  hcall = hi->heuristic_call;
  reevaluate = hi->nodes_reevaluate;

  gen_hi2 = hi2->nodes_generated;
  push_hi2 = hi2->nodes_pushed;
  pop_hi2 = hi2->nodes_popped;
  prune_hi2 = hi2->nodes_pruned_post_pop;
  hcall2 = hi2->heuristic_call;
  reevaluate2 = hi2->nodes_reevaluate;

  if (actual != actual2) {
    dump();
    assert(false);
    exit(1);
  }
  cost_ki = ki->get_search_micro();
  cost_hi = hi->get_search_micro();
  h_cost = hi->get_heuristic_micro();
  h_cost2 = hi2->get_heuristic_micro();

  int i = actual - 1;

  cost_hi2 = 0;
  if (i >= 0) {
    dist_ki= ki->get_cost(i);
    dist_hi= hi->get_cost(i);
    dist_hi2 = hi2->nn_query(si, cost_hi2).second;
    if (fabs(dist_ki - dist_hi) > EPSILON ||
        fabs(dist_hi - dist_hi2) > EPSILON) {
      ki->get_path_points(path, i);
      cost_hi2 = 0;
      hi2->nn_query(si, cost_hi2);
      dump();
      assert(false);
      exit(1);
    }
    ki->get_path_points(path, i);
    int vnum = (int)path.size();
    map<string, double> row;
    row["k"] = k, row["order"] = i+1, row["dist"] = dist_ki;
    row["cost_polyanya"] = cost_polyanya, row["cost_ki"] = cost_ki, row["cost_hi"] = cost_hi, row["cost_hi2"] = cost_hi2;
    row["h_cost"] = h_cost, row["h_cost2"] = h_cost2, row["vnum"] = vnum;
    row["gen_poly"] = gen_poly, row["push_poly"] = push_poly;
    row["gen_ki"] = gen_ki, row["push_ki"] = push_ki, row["prune_ki"] = prune_ki;
    row["gen_hi"] = gen_hi, row["push_hi"] = push_hi, row["prune_hi"] = prune_hi,
    row["hcall"] = hcall, row["reevaluate"] = reevaluate;
    row["gen_hi2"] = gen_hi2, row["push_hi2"] = push_hi2, row["prune_hi2"] = prune_hi2,
    row["hcall2"] = hcall2, row["reevaluate2"] = reevaluate2;
    row["precost"] = precost, row["gen_pre"] = gen_pre, row["edgecnt"] = edgecnt, row["damcnt"] = damcnt;
    row["pts"] = pts.size(), row["polys"] = polys.size();

    for (int i=0; i<(int)cols.size(); i++) {
      cout << row[cols[i]];
      if (i+1 == (int)cols.size()) cout << endl;
      else cout << ",";
    }
  }
}

int main(int argv, char* args[]) {
  load_data();
  meshDam->set_goals(pts);
  meshDam->floodfill();
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
      hi2->set_goals(pts);
      hi2->set_meshDam(meshDam);

      vector<string> cols = {
      "k","order","dist","cost_polyanya","cost_ki","cost_hi", "cost_hi2", "h_cost","h_cost2", "vnum",
      "gen_poly", "push_poly",
      "gen_ki", "push_ki", "prune_ki",
      "gen_hi", "push_hi",  "prune_hi", "hcall", "reevaluate",
      "gen_hi2", "push_hi2", "prune_hi2", "hcall2", "reevaluate2",
      "precost", "gen_pre", "edgecnt", "damcnt",
      "pts", "polys"};
      for (int i=0; i<(int)cols.size(); i++) {
        cout << cols[i];
        if (i+1 == (int)cols.size()) cout << endl;
        else cout << ",";
      }
      int N = 200;
      generator::gen_points_in_traversable(oMap, polys, N, starts);
      for (int i=0; i<N; i++)
        heuristic_vs_polyanya(starts[i], k, cols);
    }
    else if (t == "blind") {
      pl::Point start = pts.back();
      pts.pop_back();
      ki->verbose = true;
      ki->set_K(k);
      ki->set_start_goal(start, pts);
      ki->search();
    }
    else if (t == "heuristic") {
      pl::Point start = pts.back();
      pts.pop_back();
      hi->verbose = true;
      hi->set_goals(pts);
      hi->set_K(k);
      hi->set_start(start);
      hi->search();
    }
    else if (t == "polyanya") {
      pl::Point start = pts.back();
      pts.pop_back();
      pl::Point goal = pts[2];
      si->verbose = true;
      si->set_start_goal(start, goal);
      si->search();
    }
    else assert(false);
  }
  else assert(false);
}
