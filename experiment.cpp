#include "park2poly.h"
#include "EDBTknn.h"
#include "targetHeuristic.h"
#include "intervaHeuristic.h"
#include "fenceHeuristic.h"
#include "searchinstance.h"
#include "genPoints.h"
#include "knnMeshFence.h"
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
pl::IntervalHeuristic* ki;
pl::IntervalHeuristic* ki0;
pl::TargetHeuristic* hi;
pl::TargetHeuristic* hi2;
pl::FenceHeuristic* fi;
pl::KnnMeshEdgeFence* meshFence;
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

  meshFence= new pl::KnnMeshEdgeFence(mp);
  si = new pl::SearchInstance(mp);
  ki = new pl::IntervalHeuristic(mp);
	ki0 = new pl::IntervalHeuristic(mp); ki0->setZero(true);
  hi = new pl::TargetHeuristic(mp);
  hi2 = new pl::TargetHeuristic(mp);
  fi = new pl::FenceHeuristic(mp);
  fi->set_meshFence(meshFence);
  edbt = new vg::EDBTkNN(oMap);
}

void dump() {
  ofstream file;
  string fname = "dump/" + globalT + "-poly" + to_string(polys.size())
    + "-pts" + to_string(pts.size()) + "-k" + to_string(globalK);
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

void dense_experiment(pl::Point start, int k, vector<string>& cols, bool verbose=false) {

  map<string, double> row;
  vector<pl::Point> path;

	double dist_ki0, cost_ki0;
	int gen_ki0, cnt_ki0;

	double dist_ki, cost_ki;
	int gen_ki, cnt_ki;

	double dist_hi, cost_hi, hcost;
	int gen_hi, cnt_hi, hcall, reevaluate;

  double dist_fi, cost_fi, cost_pre;
  int gen_fi, cnt_fi, gen_pre, edgecnt, fencecnt;

	double dist_edbt, cost_edbt;
	int gen_edbt;

  double cost_poly, gen_poly;
  vector<double> odists;

	int ptsnum, polynum;
	ptsnum = (int)pts.size(); polynum = (int)polys.size();

  ki->verbose = verbose;
  ki->set_K(k);
  ki->set_start_goal(start, pts);
  cnt_ki = ki->search();
	cost_ki = ki->get_search_micro();
	gen_ki = ki->nodes_generated;
  row["cost_ki"] = cost_ki;
  row["gen_ki"] = gen_ki;

	ki0->verbose = verbose;
	ki0->set_K(k);
	ki0->set_start_goal(start, pts);
	cnt_ki0 = ki0->search();
  cost_ki0 = ki0->get_search_micro();
  gen_ki0 = ki0->nodes_generated;
  row["cost_ki0"] = cost_ki0;
  row["gen_ki0"] = gen_ki0;

	hi->set_start(start);
	hi->set_goals(pts);
	hi->set_K(k);
	cnt_hi = hi->search();
  cost_hi = hi->get_search_micro();
  gen_hi = hi->nodes_generated;
  reevaluate = hi->nodes_reevaluate;
  hcall = hi->heuristic_call;
  hcost = hi->get_heuristic_micro();
  row["cost_hi"] = cost_hi;
  row["gen_hi"] = gen_hi;
  row["hcost"] = hcost;
  row["hcall"] = hcall;
  row["reevaluate"] = reevaluate;

  fi->set_start(start);
  fi->set_goals(pts);
  fi->set_K(k);
  cost_pre = meshFence->get_processing_micro();
  gen_pre = meshFence->nodes_generated;
  edgecnt = meshFence->edgecnt;
  fencecnt = meshFence->fenceCnt;
  cnt_fi = fi->search();
  cost_fi = fi->get_search_micro();
  gen_fi = fi->nodes_generated;
  row["cost_fi"] = cost_fi;
  row["gen_fi"] = gen_fi;
  row["cost_pre"] = cost_pre;
  row["gen_pre"] = gen_pre;
  row["edgecnt"] = edgecnt;
  row["fencecnt"] = fencecnt;

  edbt->set_goals(pts);
  edbt->set_start(start);
  vector<pair<vg::pPtr, double>> res = edbt->OkNN(k);
  cost_edbt = edbt->get_search_micro();
  gen_edbt = edbt->g.nodes_generated;
  cost_edbt = edbt->get_search_micro();

  cost_poly = gen_poly = 0;
  odists = si->brute_force(start, pts, k, cost_poly, gen_poly);
  row["cost_poly"] = cost_poly;
  row["gen_poly"] = gen_poly;

  if (!(cnt_ki == cnt_ki0 && cnt_ki == cnt_hi && cnt_ki == cnt_fi &&
        cnt_ki == (int)res.size())) {
    dump();
    assert(false);
    exit(1);
  }

  for (int i=0; i<cnt_ki; i++) {
    dist_ki = ki->get_cost(i);
		dist_ki0 = ki0->get_cost(i);
    dist_hi = hi->get_cost(i);
    dist_fi = fi->get_cost(i);
    dist_edbt = res[i].second;
    if (fabs(dist_ki - dist_edbt) > EPSILON ||
        fabs(dist_ki - dist_hi) > EPSILON  ||
				fabs(dist_ki - dist_ki0 > EPSILON) ||
        fabs(dist_ki - dist_fi) > EPSILON) {
      dump();
      assert(false);
      exit(1);
    }
  }

  if (cnt_ki -1 >= 0) {
    row["k"] = k;
    row["dist"] = ki->get_cost(cnt_ki-1);
    row["polys"] = polys.size();
    row["pts"] = pts.size();
    for (int i=0; i<(int)cols.size(); i++) {
      cout << setw(10) << row[cols[i]];
      if (i+1 == (int)cols.size()) cout << endl;
      else cout << ",";
    }
  }
}

void sparse_experiment(pl::Point start, int k, vector<string>& cols, bool verbose=false) {

	double dist_ki, cost_ki, gen_ki;
	double dist_ki0, cost_ki0, gen_ki0;
	double dist_hi, cost_hi, gen_hi, hcost, hcall, reevaluate;
	double gen_pre, cost_pre, edgecnt, fenceCnt;
  map<string, double> row;
  vector<pl::Point> path;

	ki0->verbose = verbose;
	ki0->set_K(k);
	ki0->set_start_goal(start, pts);
	int actual0 = ki0->search();

  ki->verbose = verbose;
  ki->set_K(k);
  ki->set_start_goal(start, pts);
  int actual = ki->search();

  hi->verbose = verbose;
  hi->set_goals(pts);
  hi->set_K(k);
  hi->set_start(start);
  int actual2 = hi->search();

  hi2->verbose = verbose;
  hi2->set_reassign(false);
  hi2->set_goals(pts);
  hi2->set_K(k);
  hi2->set_start(start);
  int actual3 = hi2->search();

  fi->verbose = verbose;
  fi->set_goals(pts);
  fi->set_K(k);
  fi->set_start(start);
  int actual4 = fi->search();

  double cost_poly = 0, gen_poly = 0;
  vector<double> odists = si->brute_force(start, pts, k, cost_poly, gen_poly);

  cost_pre = meshFence->get_processing_micro();
  edgecnt = (double)meshFence->edgecnt;
  fenceCnt = (double)meshFence->fenceCnt;
  gen_pre = (double)meshFence->nodes_generated;
	row["cost_pre"] = cost_pre;
	row["edgecnt"] = edgecnt;
	row["fencecnt"] = fenceCnt;
	row["gen_pre"] = gen_pre;
  row["cost_fi"] = fi->get_search_micro();
  row["gen_fi"] = fi->nodes_generated;


	row["cost_poly"] = cost_poly;
	row["gen_poly"] = gen_poly;

	gen_ki0 = (double)ki0->nodes_generated;
	cost_ki0 = (double)ki0->get_search_micro();
	row["gen_ki0"] = gen_ki0;
	row["cost_ki0"] = cost_ki0;

  gen_ki = (double)ki->nodes_generated;
	cost_ki = (double)ki->get_search_micro();
	row["gen_ki"] = gen_ki;
	row["cost_ki"] = cost_ki;

  gen_hi = (double)hi->nodes_generated;
	cost_hi = (double)hi->get_search_micro();
  hcall = (double)hi->heuristic_call;
	hcost = (double)hi->get_heuristic_micro();
  reevaluate = (double)hi->nodes_reevaluate;
	row["gen_hi"] = gen_hi;
	row["cost_hi"] = cost_hi;
	row["hcall"] = hcall;
	row["hcost"] = hcost;
	row["reevaluate"] = reevaluate;

  int gen_hi2 = hi2->nodes_generated;
  double cost_hi2 = hi2->get_search_micro();
  int hcall2 =  hi2->heuristic_call;
  double hcost2 = hi2->get_heuristic_micro();
  row["gen_hi2"] = gen_hi2;
  row["cost_hi2"] = cost_hi2;
  row["hcall2"] = hcall2;
  row["hcost2"] = hcost2;

  row["gen_fi"] = fi->nodes_generated;
  row["cost_fi"] = fi->get_search_micro();

  if (actual != actual0 || 
      actual != actual2 ||
      actual != actual3 ||
      actual != actual4) {
    dump();
    assert(false);
    exit(1);
  }

	for (int i=0; i<actual; i++) {
		dist_hi = ki->get_cost(i);
    dist_ki= ki->get_cost(i);
		dist_ki0 = ki0->get_cost(i);
    if (fabs(dist_ki - dist_hi) > EPSILON ||
				fabs(dist_ki - dist_ki0) > EPSILON ||
        fabs(dist_ki - odists[i]) > EPSILON) {
      dump();
      assert(false);
      exit(1);
    }
	}

  if (actual-1 >= 0) {

		row["k"] = k;
		row["dist"] = ki->get_cost(actual-1);
		row["polys"] = polys.size();
		row["pts"] = pts.size();
    for (int i=0; i<(int)cols.size(); i++) {
      cout << setw(10) << row[cols[i]];
      if (i+1 == (int)cols.size()) cout << endl;
      else cout << ",";
    }
  }
}

void nn_experiment(pl::Point start,
    const vector<pl::Point>& targets,
    const vector<string>& cols,
    bool verbose=false) {
  map<string, double> row;
  int k = 1;

  ki0->verbose = verbose;
  ki0->set_K(k);
  ki0->set_start_goal(start, targets);
  int foundki0 = ki0->search();
  row["gen_ki0"] = ki0->nodes_generated;
  row["cost_ki0"] = ki0->get_search_micro();

  ki->verbose = verbose;
  ki->set_K(k);
  ki->set_start_goal(start, targets);
  int foundki = ki->search();
  row["gen_ki"] = ki->nodes_generated;
  row["cost_ki"] = ki->get_search_micro();

  hi->verbose = verbose;
  hi->set_goals(targets);
  hi->set_K(k);
  hi->set_start(start);
  int foundhi = hi->search();
  row["gen_hi"] = hi->nodes_generated;
  row["cost_hi"] = hi->get_search_micro();
  row["hcall"] = hi->heuristic_call;
  row["hcost"] = hi->get_heuristic_micro();
  row["reevaluate"] = hi->nodes_reevaluate;

  hi2->verbose = verbose;
  hi2->set_reassign(false);
  hi2->set_goals(targets);
  hi2->set_K(k);
  hi2->set_start(start);
  int foundhi2 = hi2->search();
  row["gen_hi2"] = hi2->nodes_generated;
  row["cost_hi2"] = hi2->get_search_micro();
  row["hcall2"] = hi2->heuristic_call;
  row["hcost2"] = hi2->get_heuristic_micro();

  fi->verbose = verbose;
  fi->set_goals(targets);
  fi->set_start(start);
  double nn_cost = 0.0;
  pair<int, double> res = fi->nn_query(si, nn_cost);
  row["gen_fi"] = fi->nodes_generated;
  row["cost_fi"] = nn_cost;
  row["gen_pre"] = meshFence->nodes_generated;
  row["cost_pre"] = meshFence->get_processing_micro();
  row["edgecnt"] = meshFence->edgecnt;
  row["fencecnt"] = meshFence->fenceCnt;

  double cost_poly = 0, gen_poly = 0;
  vector<double> odists = si->brute_force(start, targets, 1, cost_poly, gen_poly);
  row["cost_poly"] = cost_poly;
  row["gen_poly"] = gen_poly;

  if (res.first == -1) {
    if (foundki0 || foundki ||
        foundhi || foundhi2 || (int)odists.size()) {
      dump();
      assert(false);
      exit(1);
    }
  }
  if (res.first != -1) {
    if (!foundki0 || !foundki || !foundhi ||
        !foundhi2 || !(int)odists.size()) {
      dump();
      assert(false);
      exit(1);
    }
  }
  for (int i=0; i<foundhi; i++) {
    double dist_ki0 = ki0->get_cost(i);
    double dist_ki = ki->get_cost(i);
    double dist_hi = hi->get_cost(i);
    double dist_hi2 = hi2->get_cost(i);
    double dist_fi = res.second;
    double dist_poly = odists[i];
    if (fabs(dist_ki0 - dist_ki) > EPSILON ||
        fabs(dist_hi - dist_ki) > EPSILON ||
        fabs(dist_hi2 - dist_ki) > EPSILON ||
        fabs(dist_fi - dist_ki) > EPSILON ||
        fabs(dist_poly - dist_ki) > EPSILON ) {
      dump();
      assert(false);
      exit(1);
    }
  }
  if (foundki >= 1) {
    row["k"] = k;
    row["dist"] = ki->get_cost(foundki-1);
    row["polys"] = polys.size();
    row["pts"] = targets.size();
    for (int i=0; i<(int)cols.size(); i++) {
      cout << setw(10) << row[cols[i]];
      if (i+1 == (int)cols.size()) cout << endl;
      else cout << ",";
    }
  }
}

int main(int argv, char* args[]) {
  load_data();
  vector<string> cols = {
    "k", "dist", 
    "cost_edbt", "gen_edbt",
    "cost_ki0", "gen_ki0", 
    "cost_ki", "gen_ki", 
    "cost_poly", "gen_poly",
    "cost_hi", "gen_hi", "hcost", "hcall", "reevaluate",
    "cost_fi", "gen_fi", "cost_pre", "gen_pre", "edgecnt", "fencecnt",
    "pts", "polys" 
  };
  if (argv == 3) { // ./bin/test [s1/s2]
    string t = string(args[1]);
    int k = atoi(args[2]);
    // print header
    for (int i=0; i<(int)cols.size(); i++) {
      cout << setw(10) << cols[i];
      if (i+1 == (int)cols.size()) cout << endl;
      else cout << ",";
    }
    globalT = t;
    globalK = k;
    int N = 1000;
    generator::gen_points_in_traversable(oMap, polys, N, starts);
    meshFence->set_goals(pts);
    meshFence->floodfill();
    if (t == "s1") { // dense experiment
      // ./bin/experiment s1 {k} < {input file}
      for (int i=0; i<N; i++)
        dense_experiment(starts[i], k, cols);
    }
    else if (t == "s2") { // sparse experiment
      // ./bin/experiment s2 {k} < {input file}
      for (int i=0; i<N; i++)
        sparse_experiment(starts[i], k, cols);
    }
    else if (t == "nn") { // preprocessing experiment 
      // ./bin/experiment nn {num of target} < {input file}
      int targetRatio = k;
      int targetSize = (int)polys.size() * targetRatio  / 100 + 1;
      int N = 1000;
      generator::gen_points_in_traversable(oMap, polys, N, starts);
      pts.clear();
      generator::gen_points_in_traversable(oMap, polys, targetSize, pts);
      meshFence->set_goals(pts);
      meshFence->floodfill();
      for (pl::Point& start: starts) {
        nn_experiment(start, pts, cols);
      }
    }
    else assert(false);
  }
  else assert(false);
}
