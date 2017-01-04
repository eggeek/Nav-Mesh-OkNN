#pragma once
#include <vector>
#include <iostream>
#include "vertex.h"
#include "polygon.h"

namespace polyanya
{

class Mesh
{
	public:
		Mesh() { }
		Mesh(std::istream& infile);
		std::vector<Vertex> mesh_vertices;
		std::vector<Polygon> mesh_polygons;

		void print(std::ostream& outfile);
		int poly_contains_point(int poly, Point& p, int& special_index);


	// TODO: "get polygon index from Point"
};

}
