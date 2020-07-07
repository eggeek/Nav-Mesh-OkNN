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
string polys_path, pts_path, mesh_path, query_path;
vector<pl::Point> queries;
string globalT;
int globalK;

void load_targets(istream& infile) {
  int N;
  infile >> N;
  pts.resize(N);
  for (int i=0; i<N; i++) {
    infile >> pts[i].x >> pts[i].y;
  }
}

void load_queries(istream& infile) {
  int N;
  infile >> N;
  queries.resize(N);
  for (int i=0; i<N; i++) {
    infile >> queries[i].x >> queries[i].y;
  }
}

void load_data() {
  cin >> mesh_path >> polys_path >> pts_path >> query_path;

  ifstream meshfile(mesh_path);
  ifstream polysfile(polys_path);
  ifstream ptsfile(pts_path);
  ifstream queryfile(query_path);

  load_targets(ptsfile);
  load_queries(queryfile);

  mp = new pl::Mesh(meshfile);
  oMap = new vg::ObstacleMap(polysfile, mp);
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
  file << polys_path << endl;
  file << pts_path << endl;
  file << query_path << endl;
  file.close();
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

inline void fill_intervalH(map<string, double>& row, pl::IntervalHeuristic* iPtr) {
  row["cost_ki"] = iPtr->get_search_micro();
  row["gen_ki"] = iPtr->nodes_generated;
}

inline void fill_targetH(map<string, double>& row, pl::TargetHeuristic* tPtr) {
  row["cost_hi"] = tPtr->get_search_micro();
  row["gen_hi"] = tPtr->nodes_generated;
  row["hcost"] = tPtr->get_heuristic_micro();
  row["hcall"] = tPtr->heuristic_call;
  row["reevaluate"] = tPtr->nodes_reevaluate;
  row["hreuse"] = tPtr->heuristic_reuse;
}

inline void fill_fenceH(map<string, double>& row, pl::FenceHeuristic* fPtr) {
  row["cost_fi"] = fPtr->get_search_micro();
  row["gen_fi"] = fPtr->nodes_generated;
}

inline void fill_IERPolyanya(map<string, double>& row, pl::IERPolyanya* ierPtr) {
  row["cost_ffp"] = ierPtr->search_cost;
  row["gen_ffp"] = ierPtr->nodes_generated;
  row["rtree_cost"] = ierPtr->rtree_cost;
}

inline void fill_meshFence(map<string, double>& row, pl::KnnMeshEdgeFence* mPtr) {
  row["cost_pre"] = mPtr->get_processing_micro();
  row["gen_pre"] = mPtr->nodes_generated;
  row["edgecnt"] = mPtr->edgecnt;
  row["fencecnt"] = mPtr->fenceCnt;
  row["keycnt"] = mPtr->get_active_edge_cnt();
}

inline void fill_LVG(map<string, double>& row, vg::EDBTkNN* lPtr) {
  row["cost_edbt"] = lPtr->get_search_micro();
  row["gen_edbt"] = lPtr->g.nodes_generated;
}

inline void fill_extra_info(pl::Point& start, int k, map<string, double>& row, int cnt_fi, int cnt_ki, map<int, int> rank) {
  map<int, double> gid_dist_ki;
  map<int, double> gid_dist_fi;
  int vnum = 0;
  for (int i=0; i<cnt_fi; i++) {
    int gid = fi->get_gid(i);
    gid_dist_fi[gid] = fi->get_cost(i);
  }
  for (int i=0; i<cnt_ki; i++) {
    int gid = ki->get_gid(i);
    gid_dist_ki[gid] = ki->get_cost(i);
    row["max_rank"] = max(row["max_rank"], (double)rank[ki->get_gid(i)]);
    row["tot_hit"] = ffp->tot_hit;
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
  }
}

// run polyanya based and visibility graph based
void poly_vg(pl::Point start, int k, vector<string>& cols, bool verbose=false) {

  map<string, double> row;
  vector<pl::Point> path;
  map<int, int> rank = get_euclidean_rank(start, pts);
	int cnt_ki, cnt_hi, cnt_fi; 
	//double dist_edbt, cost_edbt;
	//int gen_edbt;

	int ptsnum, polynum;
	ptsnum = (int)pts.size(); polynum = (int)polys.size();

  ki->verbose = verbose;
  ki->set_K(k);
  ki->set_start_goal(start, pts);
  cnt_ki = ki->search();
  fill_intervalH(row, ki);

	hi->set_start(start);
	hi->set_K(k);
	cnt_hi = hi->search();
  fill_targetH(row, hi);

  fi->set_start(start);
  fi->set_K(k);
  cnt_fi = fi->search();
  fill_fenceH(row, fi);
  fill_meshFence(row, meshFence);

  edbt->set_start(start);
  vector<pair<vg::pPtr, double>> res = edbt->OkNN(k);
  fill_LVG(row, edbt);

  ffp->set_start(start);
  ffp->set_K(k);
  vector<double> odists = ffp->search();
  fill_IERPolyanya(row, ffp);

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

  fill_extra_info(start, k, row, cnt_fi, cnt_fi, rank);
  if (cnt_ki -1 >= 0) {
    for (int i=0; i<(int)cols.size(); i++) {
      cout << setw(10) << row[cols[i]];
      if (i+1 == (int)cols.size()) cout << endl;
      else cout << ",";
    }
  }
}

// run approaches for nn query
void nn_experiment(pl::Point start, vector<string>& cols, bool verbose=false) {
  int k = 1;

  map<string, double> row;
  map<int, int> rank = get_euclidean_rank(start, pts);

	int cnt_ki, cnt_hi, cnt_fi;

	int ptsnum, polynum;
	ptsnum = (int)pts.size(); polynum = (int)polys.size();

  ki->verbose = verbose;
  ki->set_K(k);
  ki->set_start_goal(start, pts);
  cnt_ki = ki->search();
  fill_intervalH(row, ki);

	hi->set_start(start);
	hi->set_K(k);
	cnt_hi = hi->search();
  fill_targetH(row, hi);


  double cost_fc = 0;
  fi->set_start(start);
  fi->nn_query(si, cost_fc);
  row["cost_fc"] = cost_fc;
  row["gen_fc"] = fi->nodes_generated;

  fi->set_start(start);
  fi->set_K(k);
  cnt_fi = fi->search();
  fill_fenceH(row, fi);
  fill_meshFence(row, meshFence);

  ffp->set_start(start);
  ffp->set_K(k);
  vector<double> odists = ffp->search();
  fill_IERPolyanya(row, ffp);

  if (!(cnt_ki == cnt_hi &&
        cnt_ki == (int)odists.size())) {
    dump();
    assert(false);
    exit(1);
  }
  fill_extra_info(start, k, row, cnt_fi, cnt_ki, rank);
  if (cnt_ki - 1 >= 0) {
    for (int i=0; i<(int)cols.size(); i++) {
      cout << setw(10) << row[cols[i]];
      if (i+1 == (int)cols.size()) cout << endl;
      else cout << ",";
    }
  }
}

// run polyanya based approaches
void poly_only(pl::Point start, int k, vector<string>& cols, bool verbose=false) {

  map<string, double> row;
  map<int, int> rank = get_euclidean_rank(start, pts);

  ki->verbose = verbose;
  ki->set_K(k);
  ki->set_start_goal(start, pts);
  int actual = ki->search();
  fill_intervalH(row, ki);

  hi->verbose = verbose;
  hi->set_K(k);
  hi->set_start(start);
  int actual2 = hi->search();
  fill_targetH(row, hi);

  fi->verbose = verbose;
  fi->set_K(k);
  fi->set_start(start);
  int actual4 = fi->search();
  fill_fenceH(row, fi);
  fill_meshFence(row, meshFence);

  ffp->set_start(start);
  ffp->set_K(k);
  vector<double> odists2 = ffp->search();
  fill_IERPolyanya(row, ffp);

  if (actual != actual2) {
    dump();
    cerr << "faild" << endl;
    assert(false);
    exit(1);
  }

  fill_extra_info(start, k, row, actual4, actual2, rank);
  if (actual-1 >= 0) {
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

void pre_process() {
  vector<string> cols = {
    "edgelabels",
    //"map", "domain", "category"
  };
  meshFence->set_goals(pts);
  meshFence->floodfill();
  print_header(cols);
  for (const auto& it: meshFence->get_all_fences()) {
    map<string, double> row;

    row["edgelabels"] = it.second.size();
    fill_meshFence(row, meshFence);

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
  string t = string(args[1]);

  if (t == "nn") { // nn query with preprocessing experiment 
    // ./bin/experiment nn < {input file}
    globalK = 1;
    meshFence->set_goals(pts);
    meshFence->floodfill();
    hi->set_goals(pts);
    ffp->set_goals(pts);
    fi->set_goals(pts);
    print_header(cols);
    queries.resize(100);
    for (pl::Point& query: queries) {
      nn_experiment(query, cols);
    }
  }
  else if (t == "small") {
    // ./bin/experiment small 5 < {input file}
    int k = atoi(args[2]);
    globalK = k;
    meshFence->set_goals(pts);
    meshFence->floodfill();
    hi->set_goals(pts);
    fi->set_goals(pts);
    ffp->set_goals(pts);
    edbt->set_goals(pts);
    print_header(cols);
    queries.resize(100);
    for (auto& query: queries) {
      poly_vg(query, k, cols);
    }
  }
  else if (t == "random" || t == "cluster") {
    // ./bin/experiment random/cluster < {input file}
    globalT = t;
    meshFence->set_goals(pts);
    meshFence->floodfill();
    hi->set_goals(pts);
    ffp->set_goals(pts);
    fi->set_goals(pts);
    print_header(cols);
    queries.resize(100);
    int ks[] = {1, 5, 10, 25, 50};
    for (auto& query: queries) {
      for (int i=0; i< 5; i++) {
        int k = ks[i];
        globalK = k;
        poly_only(query, k, cols);
      }
    }
  }
  else if (t == "pre") {
    pre_process();
  }
  else assert(false);
}
