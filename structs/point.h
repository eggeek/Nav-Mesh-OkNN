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

	Point operator*(const double& mult) const
	{
		return {mult * x, mult * y};
	}

	friend Point operator*(const double& mult, const Point& p)
	{
		return p * mult;
	}

	friend std::ostream& operator<<(std::ostream& stream, const Point& p)
	{
		return stream << "(" << p.x << ", " << p.y << ")";
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

	// Reflects the point across the line lr.
	Point reflect(const Point& l, const Point& r) const
	{
		const double D = r.distance_sq(l);

		// I have no idea how the below works.
		// Let X be the array [ly, rx, py, lx, ry, px].
		// magic is equal to the the alternating sum of (product of pair)
		// for all adjacent pairs (wrapping around) of X.
		#define p this
		const double magic = (p->x * l.y) - (l.y * r.x) + (r.x * p->y) 
							- (p->y * l.x) + (l.x * r.y) - (r.y * p->x);
		#undef p

		// The vector r - l rotated 90 degrees counterclockwise.
		// Can imagine "multiplying" the vector by the imaginary constant.
		const Point delta_rotated = {l.y - r.y, r.x - l.x};

		return *this - (2.0 * magic / D) * delta_rotated;
	}
};

}
