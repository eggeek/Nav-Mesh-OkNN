#pragma once
#include "RStarTree.h"
#include "geometry.h"
#include "mesh.h"
#include "rtree.h"
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
  NOT_INTERSECT,
  COLLINEAR,
};

class ObstacleMap {
  public:
  struct Vertex {
    int x, y, id;
    public:
    Vertex(int xx, int yy, int ID): x(xx), y(yy), id(ID) {};
    inline static double dist2(const Vertex& lhs, const Vertex& rhs) {
      double dx = lhs.x - rhs.x;
      double dy = lhs.y - rhs.y;
      return dx*dx + dy*dy;
    }
  };
  typedef pair<int, int> pii;
  // Obstacle: <vid, vid, ...>
  typedef vector<int> Obstacle;
  typedef pair<Vertex, Vertex> Seg;
  typedef pl::Mesh* MeshPtr;
  typedef pl::Point pPoint;
  typedef pl::Polygon Polygon;
  typedef bg::model::polygon<pPoint> boostPoly;

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
    if (version != 1)
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
          int x, y;
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
    initVisObs();
  }
  ObstacleMap() { }

  ~ObstacleMap() {
    if (rtree) delete rtree;
    if (traversableRtree) delete traversableRtree;
  }

  void initVisObs() {
    visObs.resize(obs.size());
    for (size_t i=0; i<obs.size(); i++) {
      for (size_t j=0; j<obs[i].size(); j++) {
        for (size_t k=j+1; k<obs[i].size(); k++) {
          if (isObsVisible(obs[i], j, k))
            visObs[i].insert({j, k});
        }
      }
    }
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
    if ((seg.first.id == 8 || seg.first.id == 6) &&
        (seg.second.id == 6 || seg.second.id == 8))
      assert(true);
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
    if (isVisible(p0, p1))
      return true;
    else
      return false;
  }

  bool isVisible(const pl::Point& p0, const pl::Point& p1) {
    ObsSegPosition sloc = getObsSegPosition(p0, p1);
    if (sloc == ObsSegPosition::COLLINEAR)
      return true;
    if (sloc == ObsSegPosition::INTERSECT)
      return false;
    else
      return isCollidePolys(p0, p1);
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

  bool isCollidePolys(const pPoint& pi, const pPoint& pj) {

    rs::Node_P_V stack;
    stack.push_back(traversableRtree->root);
    bool flag = false;
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
          const Polygon& p = *(Polygon*)(it->data);
          PolySegPosition ploc = getPolySegPosition(pi, pj, p);
          if (ploc == PolySegPosition::WITHIN ||
              ploc == PolySegPosition::INTERSECT ||
              ploc == PolySegPosition::ONBORDER) {
            flag = true;
            break;
          }
        }
      }
    }
    while (!stack.empty()) stack.pop_back();
    return flag;
  }

  private:

  static void fail(const string& msg) {
    cerr << msg << endl;
    exit(1);
  }

  int AddVert(int x, int y) {
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
    pPoint a{2, 3}, b{3, 2};
    if ((a.distance(pi) < EPSILON || a.distance(pj) < EPSILON) &&
        (b.distance(pj) < EPSILON || b.distance(pj) < EPSILON))
      assert(true);
    Seg seg(vs[ob[i]], vs[ob[j]]);
    ObsSegPosition sloc = getObsSegPosition(pi, pj);
    if (sloc == ObsSegPosition::INTERSECT)
      return false;
    if (sloc == ObsSegPosition::COLLINEAR)
      return true;
    // otherwise if <i, j> collide with a traversable polygon of mesh -> true
    return isCollidePolys(pi, pj);
  }

  PolySegPosition getPolySegPosition(const pPoint& p0, const pPoint& p1, const Polygon& poly) {
    boostPoly bpoly = toBoostPoly(poly);
    if (bg::covered_by(p0, bpoly) && bg::covered_by(p1, bpoly))
      return PolySegPosition::WITHIN;
    int cnt = 0;
    vector<pPoint> collinears;
    for (size_t i=0; i<poly.vertices.size(); i++) {
      const pPoint& q0 = mesh->mesh_vertices[poly.vertices[i]].p;
      const pPoint& q1 = mesh->mesh_vertices[poly.vertices[(i+1) % poly.vertices.size()]].p;
      if (is_collinear(q0, q1, p0) && is_collinear(q0, q1, p1))
        return PolySegPosition::ONBORDER;
      if (is_collinear(p0, p1, q0)) {
        if (collinears.empty() || q0.distance(collinears.front()) > EPSILON) {
          cnt++;
          collinears.push_back(q0);
        }
        continue;
      }
      if (is_collinear(p0, p1, q1)) {
        if (collinears.empty() || q1.distance(collinears.front()) > EPSILON) {
          cnt++;
          collinears.push_back(q1);
        }
        continue;
      }

      if (is_intersect(p0, p1, q0, q1))
        return PolySegPosition::INTERSECT;
    }
    if (cnt > 1)
      return PolySegPosition::INTERSECT;
    return PolySegPosition::SEPARATED;
  }

  ObsSegPosition getObsSegPosition(const pPoint& p0, const pPoint& p1) {
    rs::Node_P_V stack;
    stack.push_back(rtree->root);
    ObsSegPosition res = ObsSegPosition::NOT_INTERSECT;
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
          pl::Point v0 = pl::Point{(double)seg->first.x, (double)seg->first.y};
          pl::Point v1 = pl::Point{(double)seg->second.x, (double)seg->second.y};
          if (is_collinear(p0, p1, v0) && is_collinear(p0, p1, v1)) {
            res = ObsSegPosition::COLLINEAR;
            break;
          }
          if (is_collinear(p0, p1, v0) ||
              is_collinear(p0, p1, v1) ||
              is_collinear(v0, v1, p0) ||
              is_collinear(v0, v1, p1))
            continue;

          if (is_intersect(p0, p1, v0, v1)) {
            res = ObsSegPosition::INTERSECT;
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
