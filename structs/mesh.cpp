#include "mesh.h"
#include <iostream>
#include <string>
#include <cmath>
#include <cassert>
#include <algorithm>

namespace polyanya
{

Mesh::Mesh(std::istream& infile)
{
	read(infile);
	precalc_point_location();
}

void Mesh::read(std::istream& infile)
{
	#define fail(message) std::cerr << message << std::endl; exit(1);
	std::string header;
	int version;

	if (!(infile >> header))
	{
		fail("Error reading header");
	}
	if (header != "mesh")
	{
		std::cerr << "Got header '" << header << "'" << std::endl;
		fail("Invalid header (expecting 'mesh')");
	}

	if (!(infile >> version))
	{
		fail("Error getting version number");
	}
	if (version != 1)
	{
		std::cerr << "Got file with version " << version << std::endl;
		fail("Invalid version (expecting 1)");
	}

	int V, P;
	if (!(infile >> V >> P))
	{
		fail("Error getting V and P");
	}
	if (V < 1)
	{
		std::cerr << "Got " << V << " vertices" << std::endl;
		fail("Invalid number of vertices");
	}
	if (P < 1)
	{
		std::cerr << "Got " << P << " polygons" << std::endl;
		fail("Invalid number of polygons");
	}

	mesh_vertices.resize(V);
	mesh_polygons.resize(P);


	for (int i = 0; i < V; i++)
	{
		Vertex& v = mesh_vertices[i];
		v.is_corner = false;
		if (!(infile >> v.p.x >> v.p.y))
		{
			fail("Error getting vertex point");
		}
		int neighbours;
		if (!(infile >> neighbours))
		{
			fail("Error getting vertex neighbours");
		}
		if (neighbours < 2)
		{
			std::cerr << "Got " << neighbours << " neighbours" << std::endl;
			fail("Invalid number of neighbours around a point");
		}
		v.polygons.resize(neighbours);
		for (int j = 0; j < neighbours; j++)
		{
			int polygon_index;
			if (!(infile >> polygon_index))
			{
				fail("Error getting a vertex's neighbouring polygon");
			}
			if (polygon_index >= P)
			{
				std::cerr << "Got a polygon index of " \
					<< polygon_index << std::endl;
				fail("Invalid polygon index when getting vertex");
			}
			v.polygons[j] = polygon_index;
			if ((!v.is_corner) && (polygon_index != -1))
			{
				v.is_corner = true;
			}
		}
	}


	for (int i = 0; i < P; i++)
	{
		Polygon& p = mesh_polygons[i];
		int n;
		if (!(infile >> n))
		{
			fail("Error getting number of vertices of polygon");
		}
		if (n < 3)
		{
			std::cerr << "Got " << n << " vertices" << std::endl;
			fail("Invalid number of vertices in polygon");
		}
		p.vertices.resize(n);
		p.polygons.resize(n);

		for (int j = 0; j < n; j++)
		{
			int vertex_index;
			if (!(infile >> vertex_index))
			{
				fail("Error getting a polygon's vertex");
			}
			if (vertex_index >= V)
			{
				std::cerr << "Got a vertex index of " \
					<< vertex_index << std::endl;
				fail("Invalid vertex index when getting polygon");
			}
			p.vertices[j] = vertex_index;
			if (j == 0)
			{
				p.min_x = mesh_vertices[vertex_index].p.x;
				p.min_y = mesh_vertices[vertex_index].p.y;
				p.max_x = mesh_vertices[vertex_index].p.x;
				p.max_y = mesh_vertices[vertex_index].p.y;
			}
			else
			{
				p.min_x = std::min(p.min_x, mesh_vertices[vertex_index].p.x);
				p.min_y = std::min(p.min_y, mesh_vertices[vertex_index].p.y);
				p.max_x = std::max(p.max_x, mesh_vertices[vertex_index].p.x);
				p.max_y = std::max(p.max_y, mesh_vertices[vertex_index].p.y);
			}
		}

		for (int j = 0; j < n; j++)
		{
			int polygon_index;
			if (!(infile >> polygon_index))
			{
				fail("Error getting a polygon's neighbouring polygon");
			}
			if (polygon_index >= P)
			{
				std::cerr << "Got a polygon index of " \
					<< polygon_index << std::endl;
				fail("Invalid polygon index when getting polygon");
			}
			p.polygons[j] = polygon_index;
		}
	}

	double temp;
	if (infile >> temp)
	{
		fail("Error parsing mesh (read too much)");
	}
	#undef fail
}

void Mesh::precalc_point_location()
{
	for (Vertex& v : mesh_vertices)
	{
		slabs[v.p.x] = new std::vector<int>;
	}
	for (int i = 0; i < (int) mesh_polygons.size(); i++)
	{
		const Polygon& p = mesh_polygons[i];
		const auto low_it = slabs.lower_bound(p.min_x);
		const auto high_it = slabs.upper_bound(p.max_x);

		for (auto it = low_it; it != high_it; it++)
		{
			it->second->push_back(i);
		}
	}
	for (auto pair : slabs)
	{
		const auto vec_ref = pair.second;
		std::sort(vec_ref->begin(), vec_ref->end(),
			[&](const int& a, const int& b) -> bool
			{
				// Sorts based on the midpoints.
				// If tied, sort based on width of poly.
				const Polygon& ap = mesh_polygons[a], bp = mesh_polygons[b];
				const double as = ap.min_y + ap.max_y, bs = bp.min_y + ap.max_y;
				if (as == bs) {
					return (ap.max_y - ap.min_y) > (bp.max_y - bp.min_y);
				}
				return as < bs;
			}
		);
	}
}

// Finds out whether the polygon specified by "poly" contains point P.
// Returns:
//   0 if does not contain
//   1 if contains point
//   2 if contains point and point lies on a polygon edge
//   3 if contains point and point lies on a polygon vertex
// If 2 is returned, special_index contains the index of the adjacent polygon.
// If 3 is returned, special_index contains the index of the point it lies on.
int Mesh::poly_contains_point(int poly, Point& p, int& special_index)
{
	// The below is taken from
	// "An Efficient Test for a Point to Be in a Convex Polygon"
	// from the Wolfram Demonstrations Project
	// demonstrations.wolfram.com/AnEfficientTestForAPointToBeInAConvexPolygon/

	// Assume points are in counterclockwise order.
	const Polygon& poly_ref = mesh_polygons[poly];
	if (p.x < poly_ref.min_x - EPSILON || p.x > poly_ref.max_x + EPSILON ||
		p.y < poly_ref.min_y - EPSILON || p.y > poly_ref.max_y + EPSILON)
	{
		return 0;
	}
	const Point& last_point_in_poly = mesh_vertices[poly_ref.vertices.back()].p;
	const Point ZERO = {0, 0};

	Point last = last_point_in_poly - p;
	if (last == ZERO)
	{
		special_index = poly_ref.vertices.back();
		return 3;
	}

	for (int i = 0; i < (int) poly_ref.vertices.size(); i++)
	{
		const int point_index = poly_ref.vertices[i];
		const Point cur = mesh_vertices[point_index].p - p;
		if (cur == ZERO)
		{
			special_index = point_index;
			return 3;
		}
		const double cur_a = last * cur;
		if (std::abs(cur_a) < EPSILON)
		{
			// The line going from cur to last goes through p.
			// This means that they are colinear.
			// The associated polygon should actually be polygons[i-1]
			// according to the file format.

			// Ensure that cur = c*last where c is negative.
			// If not, this means that the point is definitely outside.
			if (cur.x)
			{
				if (!((cur.x > 0) ^ (last.x > 0)))
				{
					return 0;
				}
			}
			else
			{
				if (!((cur.y > 0) ^ (last.y > 0)))
				{
					return 0;
				}
			}
			if (i == 0)
			{
				special_index = poly_ref.polygons.back();
			}
			else
			{
				special_index = poly_ref.polygons[i-1];
			}
			return 2;
		}

		// Because we assume that the points are counterclockwise,
		// we can immediately terminate when we see a negatively signed area.
		if (cur_a < 0)
		{
			return 0;
		}
		last = cur;
	}
	return 1;
}

// Finds where the point P lies in the mesh. Returns (out1, out2).
// Returns:
//   (-1, -1) if P does not lie on the mesh.
//   (-2, a)  if P is strictly contained within polygon a
//   (a, b)   if P lies on the edge of polygons a and b.
//            Note that b can be -1 (if P lies on border of mesh).
//   (-3, c)  if P lies on a corner c.
void Mesh::get_point_location(Point& p, int& out1, int& out2)
{
	// TODO: Find a better way of doing this without going through every poly.
	auto slab = slabs.upper_bound(p.x);
	if (slab != slabs.begin())
	{
		slab--;
		const std::vector<int>* polys = slab->second;
		const auto close_it = std::lower_bound(polys->begin(), polys->end(), p.y,
			[&](const int& poly_index, const double& y_coord) -> bool
			{
				// Sorts based on the midpoints.
				// If tied, sort based on width of poly.
				const Polygon& poly = mesh_polygons[poly_index];
				return poly.min_y + poly.max_y < y_coord * 2;
			}
		);
		const int close_index = close_it - polys->begin();
		// The plan is to take an index and repeatedly do:
		// +1, -2, +3, -4, +5, -6, +7, -8, ...
		// until it hits the edge. If it hits an edge, instead iterate normally.
		const int ps = polys->size();
		int i = close_index;
		int next_delta = 1;
		int walk_delta = 0; // way to go when walking normally

		while (i >= 0 && i < ps)
		{
			const int polygon = (*polys)[i];
			int special = -999;
			const int result = poly_contains_point(polygon, p, special);
			switch (result)
			{
				case 0:
					// Does not contain: try the next one.
					break;

				case 1:
					// This one strictly contains the point.
					out1 = -2;
					out2 = polygon;
					return;

				case 2:
					// This one lies on the edge.
					out1 = polygon;
					out2 = special;
					return;

				case 3:
					// This one lies on a corner.
					out1 = -3;
					out2 = special;
					return;

				default:
					// This should not be reachable
					assert(false);
			}


			// do stuff
			if (walk_delta == 0)
			{
				const int next_i = i + next_delta * (2 * (next_delta & 1) - 1);
				if (next_i < 0)
				{
					// was going to go too far to the left.
					// start going right
					walk_delta = 1;
				}
				else if (next_i >= ps)
				{
					walk_delta = -1;
				}
				else
				{
					i = next_i;
					next_delta++;
				}
			}

			if (walk_delta != 0)
			{
				i += walk_delta;
			}
		}
	}
	// Haven't returned yet, therefore P does not lie on the mesh.
	out1 = -1;
	out2 = -1;
	return;
}

void Mesh::get_point_location_naive(Point& p, int& out1, int& out2)
{
	// TODO: Find a better way of doing this without going through every poly.
	for (int i = 0; i < (int) mesh_polygons.size(); i++)
	{
		int special = -999;
		const int result = poly_contains_point(i, p, special);
		switch (result)
		{
			case 0:
				// Does not contain: try the next one.
				break;

			case 1:
				// This one strictly contains the point.
				out1 = -2;
				out2 = i;
				return;

			case 2:
				// This one lies on the edge.
				out1 = i;
				out2 = special;
				return;

			case 3:
				// This one lies on a corner.
				out1 = -3;
				out2 = special;
				return;

			default:
				// This should not be reachable
				assert(false);
		}
	}
	// Haven't returned yet, therefore P does not lie on the mesh.
	out1 = -1;
	out2 = -1;
	return;
}

void Mesh::print(std::ostream& outfile)
{
	outfile << "mesh with " << mesh_vertices.size() << " vertices, " \
		<< mesh_polygons.size() << " polygons" << std::endl;
	outfile << "vertices:" << std::endl;
	for (Vertex vertex : mesh_vertices)
	{
		outfile << vertex.p << " " << vertex.is_corner << std::endl;
	}
	outfile << std::endl;
	outfile << "polygons:" << std::endl;
	for (Polygon polygon : mesh_polygons)
	{
		for (int vertex : polygon.vertices)
		{
			outfile << mesh_vertices[vertex].p << " ";
		}
		outfile << std::endl;
	}
}

}
