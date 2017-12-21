#include "RStarTree.h"
#include "geometry.h"
#include <vector>
#include <queue>
#include <map>
#include <cassert>
#include <iostream>

namespace rs = rstar;
namespace pl = polyanya;

namespace EDBT {

using namespace std;

void fail(const string& msg) {
  cerr << msg << endl;
  exit(1);
}

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
  int nxt_id = 0;
  map<pii, int> vert_id;
  vector<Vertex> vs;
  // <vid: oid>: find which obstacle vs[vid] belong to
  map<int, int> vid_oid;
  // obs: <obstacle, obstacle, ...>
  vector<Obstacle> obs;
  rs::RStarTree rtree;

  void read(istream& infile) {
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
  }

  void initRtree() {
    for (int i=0; i<(int)obs.size(); i++) {
      for (int j=1; j<(int)obs[i].size(); j++) {
        const Vertex& v1 = vs[obs[i][j-1]];
        const Vertex& v2 = vs[obs[i][j]];
        Seg seg(v1, v2);
        rs::LeafNodeEntry leaf = rs::LeafNodeEntry(genSegMbr(v1, v2), &seg);
        rtree.insertData(&leaf);
      }
    }
  }

  bool isVisible(const Seg& seg) {
    if (vid_oid[seg.first.id] == vid_oid[seg.second.id]) return false;
    pl::Point p0{(double)seg.first.x, (double)seg.first.y};
    pl::Point p1{(double)seg.second.x, (double)seg.second.y};
    return isVisible(p0, p1);
  }

  bool isVisible(const pl::Point& p0, const pl::Point& p1) {
    rs::Node_P_V stack;
    stack.push_back(rtree.root);
    bool flag = false;
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
          if (is_intersect(p0, p1, v0, v1)) {
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

  inline bool isCollideWithMbr(const pl::Point& p0, const pl::Point& p1, const rs::Mbr& mbr) {
    if (mbr.coord[0][0] <= p0.x && p0.x <= mbr.coord[0][1] &&
        mbr.coord[0][0] <= p1.x && p1.x <= mbr.coord[0][1] &&
        mbr.coord[1][0] <= p0.y && p0.y <= mbr.coord[1][1] &&
        mbr.coord[1][0] <= p1.y && p1.y <= mbr.coord[1][1]) return true;
    pl::Point a,b,c,d;
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
};


} // namespace EDBT
