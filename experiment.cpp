#include "park2poly.h"
#include "EDBTknn.h"
#include "targetHeuristic.h"
#include "intervaHeuristic.h"
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
pl::OkNNIntervalHeuristic* ki;
pl::OkNNIntervalHeuristic* ki0;
pl::TargetHeuristic* hi;
pl::TargetHeuristic* hi2;
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

  si = new pl::SearchInstance(mp);
  ki = new pl::OkNNIntervalHeuristic(mp);
	ki0 = new pl::OkNNIntervalHeuristic(mp); ki0->setZero(true);
  hi = new pl::TargetHeuristic(mp);
  hi2 = new pl::TargetHeuristic(mp);
  edbt = new vg::EDBTkNN(oMap);
  meshFence= new pl::KnnMeshEdgeFence(mp);
}

void dump() {
  ofstream file;
  string fname = "dump/" + globalT + "-poly" + to_string(polys.size())
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
	int cnt_ki, cnt_ki0, cnt_hi;

	double dist_ki0, cost_ki0;
	int gen_ki0;

	double dist_ki, cost_ki;
	int gen_ki;

	double dist_hi, cost_hi;
	int gen_hi;

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

	ki0->verbose = verbose;
	ki0->set_K(k);
	ki0->set_start_goal(start, pts);
	cnt_ki0 = ki0->search();

	hi->set_start(start);
	hi->set_goals(pts);
	hi->set_K(k);
	cnt_hi = hi->search();

  edbt->set_start(start);
  vector<pair<vg::pPtr, double>> res = edbt->OkNN(k);

  cost_poly = gen_poly = 0;
  odists = si->brute_force(start, pts, k, cost_poly, gen_poly);

	assert(cnt_ki == cnt_ki0 && cnt_ki == cnt_hi && (int)res.size());

	cost_ki0 = ki0->get_search_micro();
  cost_ki = ki->get_search_micro();
	cost_hi = hi->get_search_micro();
  cost_edbt = edbt->get_search_micro();
	int i = cnt_ki-1;
  if (i>=0) {
    dist_ki = ki->get_cost(i);
		cost_ki = ki->get_search_micro();
		gen_ki = ki->nodes_generated;

		dist_ki0 = ki0->get_cost(i);
		cost_ki0 = ki0->get_search_micro();
		gen_ki0 = ki0->nodes_generated;

    dist_hi = hi->get_cost(i);
		cost_hi = hi->get_search_micro();
		gen_hi = hi->nodes_generated;

    dist_edbt = res[i].second;
		cost_edbt = edbt->get_search_micro();
		gen_edbt = edbt->g.nodes_generated;

    if (fabs(dist_ki - dist_edbt) > EPSILON ||
        fabs(dist_ki - dist_hi) > EPSILON  ||
				fabs(dist_ki - dist_ki0 > EPSILON)) {
      dump();
      assert(false);
      exit(1);
    }
		cout << k << "," << dist_ki << ","
				 << cost_ki0 << "," << cost_ki << "," << cost_hi << "," << cost_edbt << ","
				 << gen_ki0 << "," << gen_ki << "," << gen_hi << "," << gen_edbt << ","
         << gen_poly << "," << cost_poly << ","
				 << ptsnum << "," << polynum << endl;
  }
}

void heuristic_vs_polyanya(pl::Point start, int k, vector<string>& cols, bool verbose=false) {

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
  hi->set_K(k);
  hi->set_start(start);
  int actual2 = hi->search();


	double cost_poly, gen_poly;
	double dist_ki, cost_ki, gen_ki;
	double dist_ki0, cost_ki0, gen_ki0;
	double dist_hi, cost_hi, gen_hi, hcost, hcall, reevaluate;
	double gen_pre, cost_pre, edgecnt, fenceCnt;
  map<string, double> row;

  cost_pre = meshFence->get_processing_micro();
  edgecnt = (double)meshFence->edgecnt;
  fenceCnt = (double)meshFence->fenceCnt;
  gen_pre = (double)meshFence->nodes_generated;
	row["cost_pre"] = cost_pre;
	row["edgecnt"] = edgecnt;
	row["fenceCnt"] = fenceCnt;
	row["gen_pre"] = gen_pre;

  cost_poly = 0;
	gen_poly = 0;
  vector<double> odists = si->brute_force(start, pts, k, cost_poly, gen_poly);

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

  if (actual != actual2 || actual0 != actual2) {
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

int main(int argv, char* args[]) {
  load_data();
  meshFence->set_goals(pts);
  meshFence->floodfill();
  if (argv == 3) { // ./bin/test [s1/s2]
    string t = string(args[1]);
    int k = atoi(args[2]);
    globalT = t;
    globalK = k;
    if (t == "s1") { // edbt vs polyanya
      edbt->set_goals(pts);

			vector<string> cols = {
				"k", "dist", "cost_ki0", "cost_ki", "cost_hi", "cost_edbt",
				"gen_ki0", "gen_ki", "gen_hi", "gen_edbt", "gen_poly", "cost_poly", "pts", "polys" 
			};
			// print header
			for (int i=0; i<(int)cols.size(); i++) {
					cout << cols[i];
					if (i + 1 == (int) cols.size()) cout << endl;
					else cout << ",";
			}

      int N = 1000;
      generator::gen_points_in_traversable(oMap, polys, N, starts);
      for (int i=0; i<N; i++)
        edbt_vs_polyanya(starts[i], k);
    }
    else if (t == "s2") { // hueristic vs polyanya
      hi->set_goals(pts);
      hi2->set_goals(pts);
      hi2->set_meshFence(meshFence);

      vector<string> cols = {
				"k", "dist",
				"cost_poly","gen_poly",
				"cost_hi", "gen_hi", "hcost", "hcall", "reevaluate",
				"cost_ki", "gen_ki",
				"cost_ki0", "gen_ki0",
				"cost_pre", "gen_pre", "edgecnt", "fenceCnt",
				"pts", "polys"
			};
      for (int i=0; i<(int)cols.size(); i++) {
        cout << setw(10) << cols[i];
        if (i+1 == (int)cols.size()) cout << endl;
        else cout << ",";
      }
      int N = 1000;
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
