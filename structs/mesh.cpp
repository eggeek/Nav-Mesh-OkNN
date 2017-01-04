#include "mesh.h"
#include <iostream>
#include <string>
#include <cmath>

namespace polyanya
{

Mesh::Mesh(std::istream& infile)
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
		const double cur_a = last.x * cur.y - last.y * cur.x;
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
