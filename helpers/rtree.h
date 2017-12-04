#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include "point.h"
#include "scenario.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
namespace pl = polyanya;

namespace boost { namespace geometry { namespace traits {
  template<> struct tag<pl::Point> {
    typedef point_tag type;
  };

  template<> struct coordinate_type<pl::Point> {
    typedef double type;
  };

  template<> struct coordinate_system<pl::Point> {
    typedef cs::cartesian type;
  };

  template<> struct dimension<pl::Point> : boost::mpl::int_<2> {};

  template<> struct access<pl::Point, 0> {
    static double get(pl::Point const& p) {
      return p.x;
    }

    static void set(pl::Point& p, double const& value) {
      p.x = value;
    }
  };

  template<> struct access<pl::Point, 1> {
    static double get(pl::Point const& p) {
      return p.y;
    }

    static void set(pl::Point& p, double const& value) {
      p.y = value;
    }
  };

} } }

bgi::rtree< pl::Point, bgi::rstar<16> > rtree;

void test_rtree() {
  std::string scenario_path = "/Users/eggeek/project/nav-mesh-ornn/scenarios/aurora.scen";
  std::ifstream scenfile(scenario_path);
  std::vector<pl::Scenario> scens;
  pl::load_scenarios(scenfile, scens);
  pl::Point start = scens[0].start;
  for (auto i: scens) {
    rtree.insert(i.goal);
  }
  std::vector<pl::Point> res;
  rtree.query(bgi::nearest(start, 1), std::back_inserter(res));
  for (auto it: res) {
    printf("(%.3lf, %.3lf) dist: %.3lf\n", it.x, it.y, it.distance(start));
  }
}
