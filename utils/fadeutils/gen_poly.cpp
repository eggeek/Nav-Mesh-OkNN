#include "Fade_2D.h"
#include <random>
using namespace GEOM_FADE2D;
using namespace std;

namespace gm = GEOM_FADE2D;

const int MaxSeg = 10;
const int Scale = 500;

int rng(int lb, int ub) {
  int res = std::rand() % (ub - lb + 1) + lb;
  return res;
}

void gen_main () {
  cout <<"\n\t--------------Generate random polygon----------" <<endl;
  vector<Segment2> p;
  vector<Segment2> p2;
  std::srand(0);
  gm::generateRandomPolygon(rng(3, MaxSeg), -1 * rng(-Scale, Scale), rng(-Scale, Scale), p, 0);
  gm::generateRandomPolygon(rng(3, MaxSeg), -1 * rng(-Scale, Scale), rng(-Scale, Scale), p2, 1);

  Fade_2D dt;
  dt.insert(Point2(-Scale, -Scale));
  dt.insert(Point2(-Scale, Scale));
  dt.insert(Point2(Scale, -Scale));
  dt.insert(Point2(Scale, Scale));

  set<Point2> ps;
  for (auto &it: p) {
    ps.insert(it.getSrc());
    ps.insert(it.getTrg());
  }

  for (const auto& it: p2) {
    ps.insert(it.getSrc());
    ps.insert(it.getTrg());
  }

  for (const auto& it: ps) dt.insert(it);

  ConstraintGraph2* cg1 = dt.createConstraint(p, CIS_CONSTRAINED_DELAUNAY);
  ConstraintGraph2* cg2 = dt.createConstraint(p2, CIS_CONSTRAINED_DELAUNAY);

  dt.applyConstraintsAndZones();

  // create outside zone
  Zone2* z0(dt.createZone(cg1, ZL_OUTSIDE));
  Zone2* z1(dt.createZone(cg2, ZL_OUTSIDE));
  Zone2* tot(zoneIntersection(z0, z1));
  tot->show("pic/zoneOutside.ps", false, true);

  // create inside zone
  Zone2* z2(dt.createZone(cg1, gm::ZL_INSIDE));
  Zone2* z3(dt.createZone(cg2, gm::ZL_INSIDE));
  Zone2* tot2(gm::zoneUnion(z2, z3));
  tot2->show("pic/zoneInside.ps", false, true);

  vector<Segment2> sb;
  tot2->getBoundarySegments(sb);
  Visualizer2 vis("pic/mergedPolygon.ps");
  for (auto &i: sb)
    vis.addObject(i, gm::CPURPLE);
  vis.writeFile();

  Visualizer2 vis2("pic/orginalPolygon.ps");
  for (auto &it: p) vis2.addObject(it, gm::CBLUE);
  for (auto &it: p2) vis2.addObject(it, gm::CBLACK);
  vis2.writeFile();
  //Visualizer2 vis("rndPolygon.ps");
  //vis.addObject(p, Color(CBLACK));
  //vis.addObject(p2, Color(CGREEN));
  //vis.writeFile();
}
