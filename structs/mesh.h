#pragma once
#include <vector>
#include <iostream>
#include <map>
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
		std::map<double, std::vector<int>* > slabs;

		void read(std::istream& infile);
		void precalc_point_location();
		void print(std::ostream& outfile);
		int poly_contains_point(int poly, Point& p, int& special_index);
		void get_point_location(Point& p, int& out1, int& out2);
		void get_point_location_naive(Point& p, int& out1, int& out2);


	// TODO: "get polygon index from Point"
};

}
