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

    double dot(const Point& other) const {
      return this->x * other.x + this->y * other.y;
    }

    double normal() {
      return std::sqrt(this->x * this->x + this->y * this->y);
    }

    double normal2() {
      return this->x * this->x + this->y * this->y;
    }

    double distance_to_seg(const Point& l, const Point& r) const {
      Point b = r - l, p = *this - l;
      if (std::abs(b.x) < EPSILON && std::abs(b.y) < EPSILON)
        return this->distance(l);
      double t = b.dot(p) / b.normal2();
      Point d;
      if (t < 0) {
        d = (*this) - l;
      }
      else if (t > 1) {
        d = (*this) - r;
      }
      else {
        d = (*this) - (l + t * b);
      }
      return d.normal();
    }
};

}
