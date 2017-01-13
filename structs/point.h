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

	// Cross product.
	// Returns the z component (as we are working in 2D).
	double operator*(const Point& other) const
	{
		return x * other.y - y * other.x;
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
		// I have no idea how the below works.
		const double denom = r.distance_sq(l);

		const Point p = *this;
		const double numer = (p * r) + (r * l) + (l * p);

		// The vector r - l rotated 90 degrees counterclockwise.
		// Can imagine "multiplying" the vector by the imaginary constant.
		const Point delta_rotated = {l.y - r.y, r.x - l.x};

		return *this + (2.0 * numer / denom) * delta_rotated;
	}
};

}
