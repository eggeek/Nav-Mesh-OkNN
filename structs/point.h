#pragma once
#include <iostream>
#include "consts.h"

namespace polyanya
{

// An (x, y) pair.
struct Point
{
	double x, y;

	bool operator==(const Point& other) const
	{
		#define square(x) (x)*(x)
		return square(x-other.x) + square(y-other.y) < EPSILON_SQUARED;
		#undef square
	}

	Point operator+(const Point& other) const
	{
		return {x + other.x, y + other.y};
	}

	Point operator-() const
	{
		return {-x, -y};
	}

	Point operator-(const Point& other) const
	{
		return *this + (-other);
	}
};

inline std::ostream& operator<< (std::ostream& stream, const Point& p)
{
	return stream << "(" << p.x << ", " << p.y << ")";
}

}
