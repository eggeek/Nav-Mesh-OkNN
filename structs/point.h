#pragma once
#include <iostream>
#include <cmath>
#include "consts.h"

namespace polyanya
{

// An (x, y) pair.
struct Point
{
	double x, y;

	bool operator==(const Point& other) const
	{
		return this->distance_sq(other) < EPSILON_SQUARED;
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

	double distance_sq(const Point& other) const
	{
		#define square(x) (x)*(x)
		return square(x-other.x) + square(y-other.y);
		#undef square
	}

	double distance(const Point& other) const
	{
		return std::sqrt(this->distance_sq(other));
	}
};

inline std::ostream& operator<< (std::ostream& stream, const Point& p)
{
	return stream << "(" << p.x << ", " << p.y << ")";
}

}
