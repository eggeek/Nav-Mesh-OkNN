#pragma once
#include "point.h"

namespace polyanya
{

void line_intersect_time(const Point& a, const Point& b,
                         const Point& c, const Point& d,
                         double& ab_num, double& cd_num, double& denom);

// Returns the line intersect between ab and cd as fast as possible.
// Uses a and b for the parameterisation.
// ASSUMES NO COLLINEARITY.
inline Point line_intersect(const Point& a, const Point& b,
                            const Point& c, const Point& d)
{
    const Point ab = b - a;
    return a + ab * (((c - a) * (d - a)) / (ab * (d - c)));
}

enum struct ZeroOnePos
{
    LT_ZERO,  // n < 0
    EQ_ZERO,  // n = 0
    IN_RANGE, // 0 < n < 1
    EQ_ONE,   // n = 1
    GT_ONE,   // n > 1
};

ZeroOnePos line_intersect_bound_check(const double num, const double denom);
// Given two points a, b and a number t, compute the point
//  a + (b-a) * t
inline Point get_point_on_line(const Point& a, const Point& b, const double t)
{
    return a + (b - a) * t;
}
Point reflect_point(const Point& p, const Point& l, const Point& r);

enum struct Orientation
{
	CCW,      // counterclockwise
	COLLINEAR, // collinear
	CW,       // clockwise
};

Orientation get_orientation(const Point& a, const Point& b, const Point& c);
inline bool is_collinear(const Point& a, const Point& b, const Point& c)
{
    return std::abs((b - a) * (c - b)) < EPSILON;
}

}
