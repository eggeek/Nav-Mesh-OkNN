#pragma once
#include "point.h"

namespace polyanya
{

void line_intersect_time(const Point& a, const Point& b,
						const Point& c, const Point& d,
						double& ab_num, double& cd_num, double& denom);
int line_intersect_bound_check(const double num, const double denom);
Point get_point_on_line(const Point& a, const Point& b, const double t);

}
