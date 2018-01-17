#pragma once
#include "RStarTree.h"
#include "geometry.h"
#include "mesh.h"
#include "rtree.h"
#include "timer.h"
#include <vector>
#include <queue>
#include <map>
#include <string>
#include <cassert>
#include <iostream>

namespace EDBT {

namespace rs = rstar;
namespace pl = polyanya;
using namespace std;

enum struct PolySegPosition {
  WITHIN,
  INTERSECT,
  SEPARATED,
  ONBORDER,
};

enum struct ObsSegPosition {
  INTERSECT,
  DISJOINT,
  ONBORDER,
};

class ObstacleMap {
  public:
  struct Vertex {
    long long x, y;
    int id;
    public:
    Vertex(long long xx, long long yy, int ID): x(xx), y(yy), id(ID) {};
    inline static double dist2(const Vertex& lhs, const Vertex& rhs) {
      double dx = lhs.x - rhs.x;
      double dy = lhs.y - rhs.y;
      return dx*dx + dy*dy;
    }
  };
  typedef pair<long long, long long> pii;
  // Obstacle: <vid, vid, ...>
  typedef vector<int> Obstacle;
  typedef pair<Vertex, Vertex> Seg;
  typedef pl::Mesh* MeshPtr;
  typedef pl::Point pPoint;
  typedef pl::Polygon Polygon;
  typedef bg::model::polygon<pPoint> boostPoly;
  typedef pl::SegIntPos SegIntPos;

  int nxt_id = 0;
  map<pii, int> vert_id;
  vector<Vertex> vs;
  vector<Seg> perimeters;
  // <vid: oid>: find which obstacle vs[vid] belong to
  map<int, int> vid_oid;
  // visObs[oid][i][j] = true: the the i-th and j-th vertices of obstacle[oid] are co-visible
  vector<set<pii>> visObs;
  // obs: <obstacle, obstacle, ...>
  vector<Obstacle> obs;
  rs::RStarTree* rtree;
  vector<rs::LeafNodeEntry> rtEntries;

  MeshPtr mesh;
  rs::RStarTree* traversableRtree;
  vector<rs::LeafNodeEntry> traversableEntries;
  ObstacleMap(istream& infile, MeshPtr mp): mesh(mp) {
    string header;
    int version;

    if (!(infile >> header))
    {
        fail("Error reading header");
    }
    if (header != "poly")
    {
        cerr << "Got header '" << header << "'" << endl;
        fail("Invalid header (expecting 'poly')");
    }

    if (!(infile >> version))
    {
        fail("Error getting version number");
    }
    if (version != 1 && version != 2)
    {
        cerr << "Got file with version " << version << endl;
        fail("Invalid version (expecting 1)");
    }

    int N;
    if (!(infile >> N))
    {
        fail("Error getting number of polys");
    }
    if (N < 1)
    {
        cerr << "Got " << N << "polys" << endl;
        fail("Invalid number of polys");
    }

    visObs.clear();
    visObs.resize(N);
    for (int i = 0; i < N; i++)
    {

      Obstacle cur;
      int M;
      if (!(infile >> M))
      {
          fail("Error parsing map (can't get number of points of poly)");
      }
      if (M < 3)
      {
          cerr << "Got " << N << "points" << endl;
          fail("Invalid number of points in poly");
      }
      for (int j = 0; j < M; j++)
      {
          long long x, y;
          if (!(infile >> x >> y))
          {
              fail("Error parsing map (can't get point)");
          }
          int vid = AddVert(x, y);
          cur.push_back(vid);
          vid_oid[vid] = obs.size();
      }
      assert((int)cur.size() == M);
      obs.push_back(cur);
      if (version == 2) {
        int K;
        if (!(infile >> K))
        {
          fail("Error parsing map (can't get visbility edges num)");
        }
        for (int k=0; k<K; k++) {
          int u, v;
          if (!(infile >> u >> v))
          {
            fail("Error parsing map (can't get visibility edges)");
          }
          assert(u < v);
          visObs[i].insert({u, v});
        }
      }
    }
    assert((int)obs.size() == N);

    int temp;
    if (infile >> temp)
    {
        fail("Error parsing map (read too much)");
    }
    rtree = new rs::RStarTree();
    traversableRtree = new rs::RStarTree();
    initRtree();
    initTraversableRtree();
    if (version != 2)
      initVisObs();
  }
  ObstacleMap() { }

  ~ObstacleMap() {
    if (rtree) delete rtree;
    if (traversableRtree) delete traversableRtree;
  }

  void initVisObs() {
    visObs.resize(obs.size());
    double tot = 0.0;
    for (size_t i=0; i<obs.size(); i++) {
      cerr << "obs: " << i << ", size: " << obs[i].size();
      timer.start();
      for (size_t j=0; j<obs[i].size(); j++) {
        for (size_t k=j+1; k<obs[i].size(); k++) {
          if (isObsVisible(obs[i], j, k)) {
            int vid0 = min(obs[i][j], obs[i][k]);
            int vid1 = max(obs[i][j], obs[i][k]);
            if (vid0 == vid1) {
              cerr << "duplicated index" << endl;
              exit(1);
            }
            visObs[i].insert({vid0, vid1});
          }
        }
      }
      timer.stop();
      double t = timer.elapsed_time_micro();
      tot += t;
      cerr  << ", time: " << t / 1000.00 << "ms" << endl;
    }
    cerr << "total cost: " << tot / 1000.00 << "ms" << endl;
  }

  void initTraversableRtree() {
    for (size_t i=0; i<mesh->mesh_polygons.size(); i++) {
      Polygon p = mesh->mesh_polygons[i];
      rs::Mbr mbr = rs::Mbr(p.min_x, p.max_x, p.min_y, p.max_y);
      rs::LeafNodeEntry leaf(mbr, (rs::Data_P)(&mesh->mesh_polygons.at(i)));
      traversableEntries.push_back(leaf);
    }
    for (auto& it: traversableEntries)
      traversableRtree->insertData(&it);
  }

  void initRtree() {
    for (int i=0; i<(int)obs.size(); i++) {
      for (int j=1; j<(int)obs[i].size(); j++) {
        const Vertex& v1 = vs[obs[i][j-1]];
        const Vertex& v2 = vs[obs[i][j]];
        perimeters.push_back(Seg(v1, v2));
      }
      const Vertex& v1 = vs[obs[i].back()];
      const Vertex& v2 = vs[obs[i][0]];
      perimeters.push_back(Seg(v1, v2));
    }
    for (size_t i=0; i<perimeters.size(); i++) {
      rs::LeafNodeEntry leaf(genSegMbr(perimeters[i].first, perimeters[i].second), (rs::Data_P)(&perimeters[i]));
      rtEntries.push_back(leaf);
    }
    for (auto& it: rtEntries)
      rtree->insertData(&it);
  }

  bool isVisible(const Seg& seg) {
    // if vertices of the segment belong to same obstacle
    // return the precomputed result
    if (vid_oid[seg.first.id] == vid_oid[seg.second.id]) {
      int oid = vid_oid[seg.first.id];
      pii p = {min(seg.first.id, seg.second.id), max(seg.first.id, seg.second.id)};
      if (visObs[oid].find(p) != visObs[oid].end())
        return true;
      return false;
    }
    // otherwise they are covisible only if there is no obstacle segment between them
    pl::Point p0{(double)seg.first.x, (double)seg.first.y};
    pl::Point p1{(double)seg.second.x, (double)seg.second.y};
    ObsSegPosition sPos = getObsSegPosition(p0, p1);
    if (sPos == ObsSegPosition::INTERSECT)
      return false;
    if (sPos == ObsSegPosition::DISJOINT) {
      if (vid_oid[seg.first.id] != vid_oid[seg.second.id])
        return true;
      else
        return isCoveredByTraversable(p0, p1);
    }
    else { // sPos == ObsSegPosition::ONBORDER
      assert(vid_oid[seg.first.id] == vid_oid[seg.second.id]);
      return true;
    }
  }

  bool isVisible(const pl::Point& p0, const pl::Point& p1) {
    ObsSegPosition sPos = getObsSegPosition(p0, p1);
    if (sPos == ObsSegPosition::INTERSECT)
      return false;
    else
      return isCoveredByTraversable(p0, p1);

  }

  bool isCoveredByTraversable(const pPoint& p0, const pPoint& p1) {
    vector<const Polygon*> polys;
    if (!isCollidePolys(p0, p1, polys))
      return false;
    vector<pair<pPoint, pPoint>> stack;
    stack.push_back({{p0.x, p0.y}, {p1.x, p1.y}});
    bool res = true;
    while (!stack.empty()) {
      pair<pPoint, pPoint> cur = stack.back(); stack.pop_back();
      if (cur.first.distance(cur.second) < EPSILON) // the seg is a point, ignore it.
        continue;
      bool covered = false;
      bool updated = false;

      //printf("pop out seg: (%.9lf, %.9lf) (%.9lf, %.9lf)\n", cur.first.x, cur.first.y, cur.second.x, cur.second.y);
      for (const auto& p: polys) {
        //#ifndef NDEBUG
        //for (size_t i=0; i< p->vertices.size(); i++) {
        //  pl::Vertex pv = mesh->mesh_vertices[p->vertices[i]];
        //  printf("(%.9lf, %.9lf) ", pv.p.x, pv.p.y);
        //}
        //pl::Vertex pv0 = mesh->mesh_vertices[p->vertices[0]];
        //printf("(%.9lf, %.9lf)\n", pv0.p.x, pv0.p.y);
        //#endif
        PolySegPosition segPos = getPolySegPosition(cur.first, cur.second, *p);
        if (segPos == PolySegPosition::SEPARATED)
          continue;
        if (segPos == PolySegPosition::WITHIN) {
          covered = true;
          break;
        }
        if (segPos == PolySegPosition::INTERSECT || segPos == PolySegPosition::ONBORDER) {
          vector<pair<pPoint, pPoint>> segs;
          covered |= updateUncoveredSeg(segs, p, {cur.first.x, cur.first.y}, {cur.second.x, cur.second.y});
          for (const auto& it: segs) if (it.first.distance(it.second) > EPSILON) {
            //printf("push seg: (%.9lf, %.9lf) (%.9lf, %.9lf)\n", it.first.x, it.first.y, it.second.x, it.second.y);
            stack.push_back(it);
            updated = true;
          }
          if (updated) break;
        }
      }
      if (!covered) {
        res = false;
        break;
      }
    }
    while (!stack.empty()) stack.pop_back();
    return res;
  }

  bool updateUncoveredSeg(vector<pair<pPoint, pPoint>>& segs, const Polygon* p, const pPoint& p0, const pPoint& p1) {
    bool covered = false;
    pPoint I0, I1;
    pPoint u = p1 - p0; // |u| > EPSILON
    double tmin = INF, tmax=EPSILON;
    for (size_t i=0; i<p->vertices.size(); i++) {
      const pPoint& q0 = mesh->mesh_vertices[p->vertices[i]].p;
      const pPoint& q1 = mesh->mesh_vertices[p->vertices[(i+1) % p->vertices.size()]].p;
      SegIntPos sPos = intersect2D_2Segments(p0, p1, q0, q1, I0, I1);
      if (sPos == SegIntPos::DISJOINT)
        continue;
      if (sPos == SegIntPos::INTERSECT) {
        double t = (I0 - p0).normal() / u.normal();
        tmin = min(tmin, t);
        tmax = max(tmax, t);
      }
      if (sPos == SegIntPos::OVERLAP) {
        double t0 = (I0 - p0).normal() / u.normal();
        double t1 = (I1 - p0).normal() / u.normal();
        tmin = min(tmin, min(t0, t1));
        tmax = max(tmax, max(t0, t1));
        break;
      }
    }
    if (tmin < INF) { // has at least 1 intersection
      double lenSeg = p0.distance(p1);
      if ((tmax - tmin) > EPSILON) { // intersect at two point
        // [c.first, I0] and [I1, c.second] are uncovered
        I0 = p0 + tmin * u;
        I1 = p0 + tmax * u;
        pair<pPoint, pPoint> newS1 = {{p0.x, p0.y}, {I0.x, I0.y}};
        pair<pPoint, pPoint> newS2 = {{I1.x, I1.y}, {p1.x, p1.y}};
        double l1 = newS1.first.distance(newS1.second);
        double l2 = newS2.first.distance(newS2.second);
        double diff = fabs(lenSeg - (l1 + l2));
        if (diff > EPSILON) {
          segs.push_back(newS1);
          segs.push_back(newS2);
          covered = true;
        }
      } else { // otherwise, only has one intersection
        boostPoly bpoly = toBoostPoly(*p);
        if (bg::covered_by(p0, bpoly)) { // p0 in/on poly
          I1 = p0 + tmax*u;
          pair<pPoint, pPoint> newS = {{I1.x, I1.y}, {p1.x, p1.y}};
          double l = newS.first.distance(newS.second);
          double diff = fabs(lenSeg - l);
          if (diff > EPSILON) {
            segs.push_back(newS);
            covered = true;
          }
        }
        else if (bg::covered_by(p1, bpoly)) { // p1 in/on poly
          I0 = p0 + tmin*u;
          pair<pPoint, pPoint> newS = {{p0.x, p0.y}, {I0.x, I0.y}};
          double l = newS.first.distance(newS.second);
          double diff = fabs(lenSeg - l);
          if (diff > EPSILON) {
            segs.push_back(newS);
            covered = true;
          }
        }
      }
    }
    return covered;
  }

  inline bool isCollideWithMbr(const pPoint& p0, const pPoint& p1, const rs::Mbr& mbr) {
    if (mbr.coord[0][0] <= p0.x && p0.x <= mbr.coord[0][1] &&
        mbr.coord[0][0] <= p1.x && p1.x <= mbr.coord[0][1] &&
        mbr.coord[1][0] <= p0.y && p0.y <= mbr.coord[1][1] &&
        mbr.coord[1][0] <= p1.y && p1.y <= mbr.coord[1][1]) return true;
    pPoint a,b,c,d;
    a = {mbr.coord[0][0], mbr.coord[1][0]};
    b = {mbr.coord[0][1], mbr.coord[1][0]};
    c = {mbr.coord[0][1], mbr.coord[1][1]};
    d = {mbr.coord[0][0], mbr.coord[1][1]};
    bool flag = false;
    flag |= is_intersect(p0, p1, a, b);
    flag |= is_intersect(p0, p1, b, c);
    flag |= is_intersect(p0, p1, c, d);
    flag |= is_intersect(p0, p1, d, a);
    return flag;
  }

  bool isCollidePolys(const pPoint& pi, const pPoint& pj, vector<const Polygon*>& polys) {

    rs::Node_P_V stack;
    stack.push_back(traversableRtree->root);
    while (!stack.empty()) {
      rs::Node_P c = stack.back(); stack.pop_back();
      if (!isCollideWithMbr(pi, pj, c->mbrn))
        continue;
      if (c->level) { // interior node
        rs::Node_P_V& children = *c->children;
        for (const auto& i: children) stack.push_back(i);
      } else { // leaf
        rs::Entry_P_V& entires = *c->entries;
        for (const auto& it: entires) {
          const Polygon* p = (Polygon*)(it->data);
          PolySegPosition ploc = getPolySegPosition(pi, pj, *p);
          if (ploc != PolySegPosition::SEPARATED) {
            polys.push_back(p);
          }
        }
      }
    }
    while (!stack.empty()) stack.pop_back();
    return !polys.empty();
  }

  void printObsMap() {
    printf("poly\n2\n");
    int N = (int)obs.size();
    printf("%d\n", N);
    for (int i=0; i<N; i++) {
      Obstacle ob = obs[i];
      int M = (int)ob.size();
      printf("%d", M);
      for (size_t j=0; j<ob.size(); j++) {
        printf(" %lld %lld", vs[ob[j]].x, vs[ob[j]].y);
      }
      printf("\n");
      int K = (int)visObs[i].size();
      printf("%d\n", K);
      for (const auto& it: visObs[i]) {
        printf("%lld %lld\n", it.first, it.second);
      }
    }
  }

  private:
    warthog::timer timer;

  static void fail(const string& msg) {
    cerr << msg << endl;
    exit(1);
  }

  int AddVert(long long x, long long y) {
    if (!vert_id.count({x, y})) {
      vs.push_back(Vertex(x, y, nxt_id));
      vert_id[{x, y}] = nxt_id++;
    }
    return vert_id[{x, y}];
  }

  rs::Mbr genSegMbr(const Vertex& v1, const Vertex& v2) {
    int minx = min(v1.x, v2.x);
    int maxx = max(v1.x, v2.x);
    int miny = min(v1.y, v2.y);
    int maxy = max(v1.y, v2.y);
    return rs::Mbr(minx, maxx, miny, maxy);
  }

  bool isObsVisible(Obstacle& ob, int i, int j) {
    // check whether i-th and j-th vertices of the obstacle are visible
    // if <i, j> intersect with a obstacle border -> false
    pPoint pi = pPoint{(double)vs[ob[i]].x, (double)vs[ob[i]].y};
    pPoint pj = pPoint{(double)vs[ob[j]].x, (double)vs[ob[j]].y};
    return isCoveredByTraversable(pi, pj);
  }

  PolySegPosition getPolySegPosition(const pPoint& p0, const pPoint& p1, const Polygon& poly) {
    bool flag1 = covered_by(poly, p0);
    bool flag2 = covered_by(poly, p1);
    if (flag1 && flag2)
      return PolySegPosition::WITHIN;
    for (size_t i=0; i<poly.vertices.size(); i++) {
      const pPoint& q0 = mesh->mesh_vertices[poly.vertices[i]].p;
      const pPoint& q1 = mesh->mesh_vertices[poly.vertices[(i+1) % poly.vertices.size()]].p;
      pPoint I0, I1;
      SegIntPos sPos = pl::intersect2D_2Segments(p0, p1, q0, q1, I0, I1);
      if (sPos == SegIntPos::DISJOINT)
        continue;
      if (sPos == SegIntPos::OVERLAP)
        return PolySegPosition::ONBORDER;
      if (sPos == SegIntPos::INTERSECT)
        return PolySegPosition::INTERSECT;
    }
    return PolySegPosition::SEPARATED;
  }

  bool covered_by(const Polygon& poly, const pPoint& p) {
    for (size_t i=0; i<poly.vertices.size(); i++) {
      const pPoint& q0 = mesh->mesh_vertices[poly.vertices[i]].p;
      const pPoint& q1 = mesh->mesh_vertices[poly.vertices[(i+1) % poly.vertices.size()]].p;
      pPoint I0, I1;
      SegIntPos sPos = intersect2D_2Segments(p, p, q0, q1, I0, I1);
      if (sPos == SegIntPos::INTERSECT || sPos == SegIntPos::OVERLAP)
        return true;
    }
    boostPoly bpoly = toBoostPoly(poly);
    bool res = bg::covered_by(p, bpoly);
    return res;
  }

  public:
  ObsSegPosition getObsSegPosition(const pPoint& p0, const pPoint& p1) {
    rs::Node_P_V stack;
    stack.push_back(rtree->root);
    ObsSegPosition res = ObsSegPosition::DISJOINT;
    while (!stack.empty()) {
      rs::Node_P c = stack.back(); stack.pop_back();
      rs::Mbr& mbr = c->mbrn;
      if (!isCollideWithMbr(p0, p1, mbr)) continue;
      if (c->level) {
        rs::Node_P_V& children = *c->children;
        for (const auto& i: children) stack.push_back(i);
      }
      else {
        rs::Entry_P_V& entires = *c->entries;
        for (const auto& i: entires) {
          Seg* seg = (Seg*)i->data;
          pPoint v0 = pPoint{(double)seg->first.x, (double)seg->first.y};
          pPoint v1 = pPoint{(double)seg->second.x, (double)seg->second.y};
          pPoint I0, I1;
          SegIntPos sPos = intersect2D_2Segments(p0, p1, v0, v1, I0, I1);
          if (sPos == SegIntPos::DISJOINT) // joint at one vertex, regard this case as disjoint
            continue;
          if (sPos == SegIntPos::INTERSECT) {
            if (I0.distance(p0) < EPSILON || I0.distance(p1) < EPSILON)
              continue;
            else
              res = ObsSegPosition::INTERSECT;
            break;
          }
          if (sPos == SegIntPos::OVERLAP) {
            res = ObsSegPosition::ONBORDER;
            break;
          }
        }
      }
    }
    while (!stack.empty()) stack.pop_back();
    return res;
  }

  boostPoly toBoostPoly(const Polygon& poly) {
    boostPoly res;
    vector<pPoint> ps;
    for (int vid: poly.vertices) {
      const pPoint& p = mesh->mesh_vertices[vid].p;
      ps.push_back(p);
    }
    ps.push_back(ps.front());
    bg::append(res, ps);
    return res;
  }
};


} // namespace EDBT
