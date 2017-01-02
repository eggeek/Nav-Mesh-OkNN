#include "mesh.h"
#include <iostream>
#include <string>

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

void Mesh::print(std::ostream& outfile)
{
	outfile << "mesh with " << mesh_vertices.size() << " vertices, " \
		<< mesh_polygons.size() << " polygons" << std::endl;
	outfile << "vertices:" << std::endl;
	for (Vertex vertex : mesh_vertices)
	{
		outfile << vertex.p << std::endl;
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
