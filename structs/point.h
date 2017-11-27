#pragma once
#include "consts.h"
#include <iostream>
#include <cmath>
#include <cassert>

namespace polyanya
{

// An (x, y) pair.
struct Point
{
    double x, y;

    bool operator==(const Point& other) const
    {
        return (std::abs(x - other.x) < EPSILON) &&
               (std::abs(y - other.y) < EPSILON);
    }

    bool operator!=(const Point& other) const
    {
        return !((*this) == other);
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
        return {x - other.x, y - other.y};
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

    double distance_to_seg(const Point& l, const Point& r) const {
      Point rl = r - l;
      if (std::abs(rl.x) < EPSILON && std::abs(rl.y) < EPSILON) {
        // distance to a point
        return this->distance(l);
      }
      Point pl = (*this) - l;
      double t = (pl.x * rl.x + pl.y * rl.y) / (rl.x*rl.x + rl.y*rl.y);
      Point d;
      if (t < 0) {
        d = (*this) - l;
      }
      else if (t > 1.0) {
        d = (*this) - r;
      }
      else {
        d = (*this) - (l + t * rl);
      }
      return std::sqrt(d.x*d.x + d.y+d.y);
    }
};

}
