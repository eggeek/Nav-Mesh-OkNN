#pragma once
#include <vector>

namespace polyanya
{

struct Polygon
{
	// "int" here means an array index.
	std::vector<int> vertices;
	std::vector<int> polygons;
};

}
