// various testing functions
#include "expansion.h"
#include "genPoints.h"
#include "mesh.h"
#include "geometry.h"
#include "searchinstance.h"
#include "knninstance.h"
#include "knnheuristic.h"
#include "EDBTknn.h"
#include "park2poly.h"
#include "knnMeshEdge.h"

using namespace std;
using namespace polyanya;

MeshPtr mp;
Mesh m;
Point tp;
EDBT::ObstacleMap* oMap;
EDBT::EDBTkNN* edbt;
SearchInstance* si;
KnnInstance* ki;
KnnInstance* ki0;
KnnHeuristic* hi;
KnnHeuristic* hi2;
KnnMeshEdgeDam* meshDam;
vector<Scenario> scenarios;
vector<Point> pts;
vector<vector<Point>> polys;

void load_points(istream& infile ) {
  int N;
  infile >> N;
  pts.resize(N);
  for (int i=0; i<N; i++) {
    infile >> pts[i].x >> pts[i].y;
  }
}

void load_data() {
  string scenario_path, obs_path, polys_path, pts_path, mesh_path;
  cin >> mesh_path >> polys_path >> obs_path >> pts_path;

  ifstream scenfile(scenario_path);
  ifstream meshfile(mesh_path);
  ifstream obsfile(obs_path);
  ifstream ptsfile(pts_path);
  ifstream polysfile(polys_path);

  polys = generator::read_polys(polysfile);
  load_points(ptsfile);
  mp = new Mesh(meshfile);
  m = *mp;
  //oMap = new EDBT::ObstacleMap(obsfile, &m);
  meshfile.close();
  si = new SearchInstance(mp);
  ki = new KnnInstance(mp);
	ki0 = new KnnInstance(mp); ki0->setZero(true);
  hi = new KnnHeuristic(mp);
  hi2 = new KnnHeuristic(mp);
  //edbt = new EDBT::EDBTkNN(oMap);
  meshDam = new KnnMeshEdgeDam(mp);
  printf("vertices: %d, polygons: %d\n", (int)m.mesh_vertices.size(), (int)m.mesh_polygons.size());
}

void test_knn(Point start, int k, bool verbose=false) {
	ki->set_K(k);
	ki->set_start_goal(start, pts);
	ki->verbose = verbose;
	ki0->set_K(k);
	ki0->set_start_goal(start, pts);
	ki0->verbose = verbose;
	int res0, res;
	res0 = ki0->search();
	res = ki->search();
	assert(res0 == res);
	cout << "cost_ki,cost_ki0,dist,len" << endl;
	vector<Point> path;
	for (int i=0; i<res0; i++) {
		assert(fabs(ki->get_cost(i) - ki0->get_cost(i)) < EPSILON);
		double cost_ki = ki->get_search_micro();
		double cost_ki0 = ki0->get_search_micro();
		double dist = ki->get_cost(i);
		ki->get_path_points(path, i);
		cout << cost_ki << "," << cost_ki0 << "," << dist << "," << path.size() << endl;
	}
}

int main(int argv, char* args[]) {
  load_data();
	if (argv == 3) {
		string t = string(args[1]);
		if (t == "knn") {
			int k = std::atoi(args[2]);
			Point start = pts.back();
			pts.pop_back();
			test_knn(start, k);
		}
	}
  return 0;
}
