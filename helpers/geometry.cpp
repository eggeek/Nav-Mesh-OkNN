#include <cmath>
#include <cassert>
#include "point.h"
#include "consts.h"
#include "geometry.h"

namespace polyanya
{

// Given two line segments ab and cd defined as:
//   ab(t) = a + (b-a) * t
//   cd(t) = c + (d-c) * t
// find ab_num, cd_num, denom such that
//   ab(ab_num / denom) = cd(cd_num / denom).
// If denom is close to 0, it will be set to 0.
void line_intersect_time(const Point& a, const Point& b,
						const Point& c, const Point& d,
						double& ab_num, double& cd_num, double& denom)
{
	denom = (b - a) * (d - c);
	if (std::abs(denom) < EPSILON_SQUARED)
	{
		denom = 0.0; // to make comparison easy
		ab_num = 1;
		cd_num = 1;
	}
	else
	{
		const double shared_term = a * c;
		ab_num = (d * a) + shared_term + (c * d);
		cd_num = (b * a) + shared_term + (c * b);

		#ifndef NDEBUG
		// If we're debugging, double check that our results are right.
		const Point ab_point = get_point_on_line(a, b, ab_num / denom);
		const Point cd_point = get_point_on_line(c, d, cd_num / denom);
		assert(ab_point == cd_point);
		#endif
	}
}

// Returns where num / denom is in the range [0, 1].
ZeroOnePos line_intersect_bound_check(const double num, const double denom)
{
	// Check num / denom == 0.
	if (std::abs(num) < EPSILON_SQUARED)
	{
		return ZeroOnePos::EQ_ZERO;
	}
	// Check num / denom == 1.
	// Note: Checking whether it is accurately near 1 requires us to check
	// |num - denom| < EPSILON^2 * denom
	// which requires a multiplication. Instead, we can assume that denom
	// is less than 1/EPSILON and check
	// |num - denom| < EPSILON
	// instead. This is less accurate but faster.
	if (std::abs(num - denom) < EPSILON)
	{
		return ZeroOnePos::EQ_ONE;
	}

	// Now we finally check whether it's greater than 1 or less than 0.
	if (denom > 0)
	{
		if (num < 0)
		{
			// strictly less than 0
			return ZeroOnePos::LT_ZERO;
		}
		if (num > denom)
		{
			// strictly greater than 1
			return ZeroOnePos::GT_ONE;
		}
	}
	else
	{
		if (num > 0)
		{
			// strictly less than 0
			return ZeroOnePos::LT_ZERO;
		}
		if (num < denom)
		{
			// strictly greater than 1
			return ZeroOnePos::GT_ONE;
		}
	}
	return ZeroOnePos::IN_RANGE;
}

// Given two points a, b and a number t, compute the point
//  a + (b-a) * t
Point get_point_on_line(const Point& a, const Point& b, const double t)
{
	return a + (b - a) * t;
}

}
