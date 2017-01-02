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
		Mesh(std::istream& infile);
		std::vector<Vertex> mesh_vertices;
		std::vector<Polygon> mesh_polygons;

		void print(std::ostream& outfile);

	// TODO: "get polygon index from Point"
};

}
