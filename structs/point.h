#pragma once
#include <iostream>

namespace polyanya
{

// An (x, y) pair.
struct Point
{
	double x, y;
};

inline std::ostream& operator<< (std::ostream& stream, const Point& p)
{
	return stream << "(" << p.x << ", " << p.y << ")";
}

}
