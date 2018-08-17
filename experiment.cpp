#include "park2poly.h"
#include "EDBTknn.h"
#include "targetHeuristic.h"
#include "intervaHeuristic.h"
#include "fenceHeuristic.h"
#include "searchinstance.h"
#include "genPoints.h"
#include "knnMeshFence.h"
#include "mesh.h"
#include "IERPolyanya.h"
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
pl::TargetHeuristic* hi;
pl::FenceHeuristic* fi;
pl::IERPolyanya* ffp;
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

void load_starts(istream& infile) {
  int N;
  infile >> N;
  starts.resize(N);
  for (int i=0; i<N; i++) {
    infile >> starts[i].x >> starts[i].y;
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
  hi = new pl::TargetHeuristic(mp);
  fi = new pl::FenceHeuristic(mp);
  ffp = new pl::IERPolyanya(si);
  fi->set_meshFence(meshFence);
  edbt = new vg::EDBTkNN(oMap);
}

map<int, int> get_euclidean_rank(pl::Point start,  const vector<pl::Point> ts) {
  vector<pair<int, double>> id_dists;
  map<int, int> res;
  for (int i=0; i<(int)ts.size(); i++) {
    id_dists.push_back({i, start.distance(ts[i])});
  }

  auto cmp = [&](pair<int, double> a, pair<int, double> b) {
    return a.second < b.second;
  };
  sort(id_dists.begin(), id_dists.end(), cmp);
  for (int i=0; i<(int)ts.size(); i++) {
    res[id_dists[i].first] = i+1;
  }
  return res;
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

pair<int, int> count_miss_and_false(const map<int, double>& correct_dist, const map<int, double>& actual_dist) {
  int miss = 0;
  for (const auto& i: correct_dist) {
    bool flag = false;
    for (const auto& j: actual_dist) {
      if (fabs(i.second - j.second) <= EPSILON) {
        flag = true;
        break;
      }
    }
    if (!flag) miss++;
  }

  return {miss, miss};
} 

void dense_experiment(pl::Point start, int k, vector<string>& cols, bool verbose=false) {

  if (start.distance({52722.2, 84437.8}) <= EPSILON) {
    assert(true);
  }
  map<string, double> row;
  vector<pl::Point> path;
  map<int, int> rank = get_euclidean_rank(start, pts);
  int vnum = 0;

	double dist_ki, cost_ki;
	int gen_ki, cnt_ki;

	double dist_hi, cost_hi, hcost;
	int gen_hi, cnt_hi, hcall, reevaluate;

  double dist_fi, cost_fi, cost_pre;
  int gen_fi, cnt_fi, gen_pre, edgecnt, fencecnt;

	//double dist_edbt, cost_edbt;
	//int gen_edbt;

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

	hi->set_start(start);
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
  row["hreuse"] = hi->heuristic_reuse;

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
  row["keycnt"] = meshFence->get_active_edge_cnt();

  edbt->set_start(start);
  vector<pair<vg::pPtr, double>> res = edbt->OkNN(k);
  row["cost_edbt"] = edbt->get_search_micro();
  row["gen_edbt"] = edbt->g.nodes_generated;

  ffp->set_start(start);
  ffp->set_K(k);
  vector<double> odists = ffp->search();
  row["cost_ffp"] = ffp->search_cost;
  row["gen_ffp"] = ffp->nodes_generated;
  row["rtree_cost"] = ffp->rtree_cost;

  double cost_poly=0, gen_poly=0;
  si->brute_force(start, pts, k, cost_poly, gen_poly);
  row["cost_poly"] = cost_poly;
  row["gen_poly"] = gen_poly;
  row["sort_cost"] = si->sort_cost;

  if (!(cnt_ki == cnt_hi &&
        cnt_ki == (int)odists.size())) {
    dump();
    assert(false);
    exit(1);
  }

  map<int, double> gid_dist_ki;
  map<int, double> gid_dist_fi;
  for (int i=0; i<cnt_fi; i++) {
    int gid = fi->get_gid(i);
    gid_dist_fi[gid] = fi->get_cost(i);
  }
  for (int i=0; i<cnt_ki; i++) {
    int gid = ki->get_gid(i);
    gid_dist_ki[gid] = ki->get_cost(i);
    dist_ki = ki->get_cost(i);
    dist_hi = hi->get_cost(i);
    dist_fi = fi->get_cost(i);
    //dist_edbt = res[i].second;
    row["max_rank"] = max(row["max_rank"], (double)rank[ki->get_gid(i)]);
    row["tot_hit"] = ffp->tot_hit;
    if (fabs(dist_ki - dist_hi) > EPSILON  ||
        fabs(dist_ki - odists[i]) > EPSILON) {
      dump();
      assert(false);
      exit(1);
    }
    vector<pl::Point> path;
    ki->get_path_points(path, i);
    vnum += (int)path.size();
  }

  if (cnt_ki -1 >= 0) {
    row["k"] = k;
    row["rank"] = rank[ki->get_gid(cnt_ki-1)];
    assert(row["rank"]>0);
    row["dist"] = ki->get_cost(cnt_ki-1);
    row["polys"] = polys.size();
    row["pts"] = pts.size();
    row["vnum"] = vnum;
    pair<int, int> miss_false = count_miss_and_false(gid_dist_ki, gid_dist_fi);
    row["miss"] = miss_false.first;
    row["vertices"] = mp->mesh_vertices.size();
    row["false_retrieval"] = miss_false.second;
    row["x"] = start.x, row["y"] = start.y;
    for (int i=0; i<(int)cols.size(); i++) {
      cout << setw(10) << row[cols[i]];
      if (i+1 == (int)cols.size()) cout << endl;
      else cout << ",";
    }
  }
}

void nn_experiment(pl::Point start, vector<string>& cols, bool verbose=false) {
  int k = 1;

  map<string, double> row;
  vector<pl::Point> path;
  map<int, int> rank = get_euclidean_rank(start, pts);
  int vnum = 0;

	double dist_ki, cost_ki;
	int gen_ki, cnt_ki;

	double dist_hi, cost_hi, hcost;
	int gen_hi, cnt_hi, hcall, reevaluate;

  double dist_fi, cost_fi, cost_pre;
  int gen_fi, cnt_fi, gen_pre, edgecnt, fencecnt;

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

	hi->set_start(start);
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
  row["hreuse"] = hi->heuristic_reuse;


  double cost_fc = 0;
  fi->set_start(start);
  fi->set_goals(pts);
  fi->nn_query(si, cost_fc);
  row["cost_fc"] = cost_fc;
  row["gen_fc"] = fi->nodes_generated;

  fi->set_start(start);
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
  row["keycnt"] = meshFence->get_active_edge_cnt();

  ffp->set_start(start);
  ffp->set_K(k);
  vector<double> odists = ffp->search();
  row["cost_ffp"] = ffp->search_cost;
  row["gen_ffp"] = ffp->nodes_generated;

  if (!(cnt_ki == cnt_hi &&
        cnt_ki == (int)odists.size())) {
    dump();
    assert(false);
    exit(1);
  }

  map<int, double> gid_dist_ki;
  map<int, double> gid_dist_fi;
  for (int i=0; i<cnt_fi; i++) {
    int gid = fi->get_gid(i);
    gid_dist_fi[gid] = fi->get_cost(i);
  }
  for (int i=0; i<cnt_ki; i++) {
    int gid = ki->get_gid(i);
    gid_dist_ki[gid] = ki->get_cost(i);
    dist_ki = ki->get_cost(i);
    dist_hi = hi->get_cost(i);
    dist_fi = fi->get_cost(i);
    row["max_rank"] = max(row["max_rank"], (double)rank[ki->get_gid(i)]);
    row["tot_hit"] = ffp->tot_hit;
    if (fabs(dist_ki - dist_hi) > EPSILON  ||
        fabs(dist_ki - odists[i]) > EPSILON) {
      dump();
      assert(false);
      exit(1);
    }
    vector<pl::Point> path;
    ki->get_path_points(path, i);
    vnum += (int)path.size();
  }

  if (cnt_ki -1 >= 0) {
    row["k"] = k;
    row["rank"] = rank[ki->get_gid(cnt_ki-1)];
    assert(row["rank"]>0);
    row["dist"] = ki->get_cost(cnt_ki-1);
    row["polys"] = polys.size();
    row["pts"] = pts.size();
    row["vnum"] = vnum;
    pair<int, int> miss_false = count_miss_and_false(gid_dist_ki, gid_dist_fi);
    row["miss"] = miss_false.first;
    row["vertices"] = mp->mesh_vertices.size();
    row["false_retrieval"] = miss_false.second;
    row["x"] = start.x, row["y"] = start.y;
    for (int i=0; i<(int)cols.size(); i++) {
      cout << setw(10) << row[cols[i]];
      if (i+1 == (int)cols.size()) cout << endl;
      else cout << ",";
    }
  }
}

void knn_experiment(pl::Point start, int k, vector<string>& cols, bool verbose=false) {

	double dist_ki, cost_ki, gen_ki;
	double dist_hi, cost_hi, gen_hi, hcost, hcall, reevaluate;
	double gen_pre, cost_pre, edgecnt, fenceCnt;
  int vnum = 0;
  map<string, double> row;
  vector<pl::Point> path;
  map<int, int> rank = get_euclidean_rank(start, pts);

  ki->verbose = verbose;
  ki->set_K(k);
  ki->set_start_goal(start, pts);
  int actual = ki->search();

  hi->verbose = verbose;
  hi->set_K(k);
  hi->set_start(start);
  int actual2 = hi->search();

  fi->verbose = verbose;
  fi->set_goals(pts);
  fi->set_K(k);
  fi->set_start(start);
  int actual4 = fi->search();

  ffp->set_start(start);
  ffp->set_K(k);
  vector<double> odists2 = ffp->search();
  row["cost_ffp"] = ffp->search_cost;
  row["gen_ffp"] = ffp->nodes_generated;

  cost_pre = meshFence->get_processing_micro();
  edgecnt = (double)meshFence->edgecnt;
  fenceCnt = (double)meshFence->fenceCnt;
  gen_pre = (double)meshFence->nodes_generated;
	row["cost_pre"] = cost_pre;
	row["edgecnt"] = edgecnt;
	row["fencecnt"] = fenceCnt;
  row["keycnt"] = meshFence->get_active_edge_cnt();
	row["gen_pre"] = gen_pre;
  row["cost_fi"] = fi->get_search_micro();
  row["gen_fi"] = fi->nodes_generated;

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
  row["hreuse"] = hi->heuristic_reuse;

  row["gen_fi"] = fi->nodes_generated;
  row["cost_fi"] = fi->get_search_micro();

  if (actual != actual2) {
    dump();
    cerr << "faild" << endl;
    assert(false);
    exit(1);
  }

  map<int, double> gid_dist_ki;
  map<int, double> gid_dist_fi;
  for (int i=0; i<actual4; i++) {
    int gid = fi->get_gid(i);
    gid_dist_fi[gid] = fi->get_cost(i);
  }

	for (int i=0; i<actual; i++) {
		dist_hi = ki->get_cost(i);
    dist_ki= ki->get_cost(i);
    int gid = ki->get_gid(i);
    gid_dist_ki[gid] = dist_ki;
    row["max_rank"] = max(row["max_rank"], (double)rank[ki->get_gid(i)]);
    row["tot_hit"] = ffp->tot_hit;
    if (fabs(dist_ki - dist_hi) > EPSILON ||
        fabs(dist_ki - odists2[i]) > EPSILON) {
      dump();
      assert(false);
      exit(1);
    }
    vector<pl::Point> path;
    ki->get_path_points(path, i);
    vnum += (int)path.size();
	}

  if (actual-1 >= 0) {

		row["k"] = k;
    row["rank"] = rank[ki->get_gid(actual-1)];
    assert(row["rank"]>0);
		row["dist"] = ki->get_cost(actual-1);
		row["polys"] = polys.size();
		row["pts"] = pts.size();
    row["vnum"] = vnum;
    pair<int, int> miss_false = count_miss_and_false(gid_dist_ki, gid_dist_fi);
    row["miss"] = miss_false.first;
    row["vertices"] = mp->mesh_vertices.size();
    row["false_retrieval"] = miss_false.second;
    row["x"] = start.x, row["y"] = start.y;
    for (int i=0; i<(int)cols.size(); i++) {
      cout << setw(10) << row[cols[i]];
      if (i+1 == (int)cols.size()) cout << endl;
      else cout << ",";
    }
  }
  else cerr << "target not found" << endl;
}

void print_header(const vector<string>& cols) {
  for (int i=0; i<(int)cols.size(); i++) {
    cout << setw(10) << cols[i];
    if (i+1 == (int)cols.size()) cout << endl;
    else cout << ",";
  }
}

int main(int argv, char* args[]) {
  load_data();
  vector<string> cols = {
    "k", "dist", "rank", "max_rank", "vnum", "tot_hit",
    "cost_edbt", "gen_edbt",
    "cost_ki", "gen_ki", 
    "cost_ffp", "gen_ffp", "rtree_cost",
    "cost_poly", "gen_poly", "sort_cost",
    "cost_hi", "gen_hi", "hcost", "hcall", "hreuse", "reevaluate",
    "cost_fc" , "gen_fc",
    "cost_fi", "gen_fi", "cost_pre", "gen_pre", "edgecnt", "fencecnt", "keycnt",
    "miss", "false_retrieval",
    "pts", "polys", "vertices", "x", "y",
  };
  if (argv >= 3) { // ./bin/test [s1/s2]
    string t = string(args[1]);

    if (t == "nn") { // nn query with preprocessing experiment 
      // ./bin/experiment nn {num of target} < {input file}
      double ratio = atof(args[2]);
      int targetSize = mp->mesh_vertices.size() * ratio + 1;
      int N = 1000;
      generator::gen_points_in_traversable(oMap, polys, N, starts);
      pts.clear();
      generator::gen_points_in_traversable(oMap, polys, targetSize, pts);
      meshFence->set_goals(pts);
      meshFence->floodfill();
      hi->set_goals(pts);
      ffp->set_goals(pts);
      print_header(cols);
      for (pl::Point& start: starts) {
        nn_experiment(start, cols);
      }
    }
    else if (t == "knn") {
      double ratio = atof(args[2]);
      int targetSize = mp->mesh_vertices.size() * ratio + 1;
      int k = atoi(args[3]);
      int N = 1000;
      generator::gen_points_in_traversable(oMap, polys, N, starts);
      pts.clear();
      generator::gen_points_in_traversable(oMap, polys, targetSize, pts); 
      meshFence->set_goals(pts);
      meshFence->floodfill();
      hi->set_goals(pts);
      ffp->set_goals(pts);
      print_header(cols);
      for (auto& start: starts) {
        knn_experiment(start, k, cols);
      }
    }
    else if (t == "k") {
      globalT = t;
      int N = 1000;
      int targetNum = mp->mesh_vertices.size() / 100 + 1;
      generator::gen_points_in_traversable(oMap, polys, N, starts);
      pts.clear();
      generator::gen_points_in_traversable(oMap, polys, targetNum, pts);
      meshFence->set_goals(pts);
      meshFence->floodfill();
      hi->set_goals(pts);
      ffp->set_goals(pts);
      edbt->set_goals(pts);
      print_header(cols);
      int ks[] = {1, 5, 10, 25, 50};
      for (auto& start: starts) {
        for (int i=0; i< 5; i++) {
          int k = ks[i];
          globalK = k;
          dense_experiment(start, k, cols);
        }
      }
    }
    else if (t == "t") {
      int N = 1000;
      int k = 5;
      double ratio = atof(args[2]);
      int targetNum = mp->mesh_vertices.size() * ratio + 1;
      generator::gen_points_in_traversable(oMap, polys, N, starts);
      pts.clear();
      generator::gen_points_in_traversable(oMap, polys, targetNum, pts);
      meshFence->set_goals(pts);
      meshFence->floodfill();
      hi->set_goals(pts);
      ffp->set_goals(pts);
      edbt->set_goals(pts);
      print_header(cols);
      for (auto& start: starts) {
        globalK = k;
        dense_experiment(start, k, cols);
      }
    }
    else if (t == "dense") {
      int ks[] = {1, 5, 10, 25, 50};
      //ifstream startsfile("./points/9000-start-cluster.points");
      //load_starts(startsfile);;
      int N = 1000;
      generator::gen_points_in_traversable(oMap, polys, N, starts);
      meshFence->set_goals(pts);
      meshFence->floodfill();
      hi->set_goals(pts);
      ffp->set_goals(pts);
      edbt->set_goals(pts);
      print_header(cols);
      for (auto& start: starts) {
        for (int i=0; i<5; i++) {
          int k = ks[i];
          globalK = k;
          dense_experiment(start, k, cols);
        }
      }
    }
    else if (t == "fence") { // preproessing experiment
      string para = "random";
      if (argv >= 4) {
        para = string(args[3]); 
      }
      vector<pl::Point> targets;
      if (para == "cluster") {
        double clusterRatio = atof(args[2]);
        int clusterNum = (int)mp->mesh_vertices.size() * clusterRatio + 1;
        vector<pl::Point> clusters;
        generator::gen_points_in_traversable(oMap, polys, clusterNum, clusters);
        generator::gen_clusters_in_traversable(oMap, mp, 10, clusters, targets);
      }
      else {
        double targetRatio = atof(args[2]);
        int targetSize = (int)mp->mesh_vertices.size() * targetRatio + 1;
        generator::gen_points_in_traversable(oMap, polys, targetSize, targets);
      }
      meshFence->set_goals(targets);
      meshFence->floodfill();
      vector<string> headers = {
        "edgecnt", "totalfence", "keycnt", "pts", "polys", "gen", "cost", "fencecnt"
      };
      print_header(headers);
      const auto fences = meshFence->get_all_fences();
      for (const auto& it: fences) {
        map<string, double> row;
        row["fencecnt"] = (int)it.second.size();
        row["edgecnt"] = meshFence->edgecnt;
        row["totalfence"] = meshFence->fenceCnt;
        row["keycnt"] = meshFence->get_active_edge_cnt();
        row["pts"] = targets.size();
        row["polys"] = polys.size();
        row["gen"] = meshFence->nodes_generated;
        row["cost"] = meshFence->get_processing_micro();
        for (int i=0; i<(int)headers.size(); i++) {
          cout << setw(10) << row[headers[i]];
          if (i+1 == (int)headers.size()) cout << endl;
          else cout << ",";
        }
      }
    }
    else if (t == "cluster") {
      string starts_path = string(args[2]);
      int k = atoi(args[3]);
      ifstream startsfile(starts_path);
      load_starts(startsfile);

      meshFence->set_goals(pts);
      meshFence->floodfill();
      hi->set_goals(pts);
      ffp->set_goals(pts);
      print_header(cols);
      for (auto& start: starts) {
        globalK = k;
        globalT = t;
        knn_experiment(start, k, cols);
      }
    }
    else assert(false);
  }
  else assert(false);
}
