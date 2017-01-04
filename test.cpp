// various testing functions
#include "mesh.h"
#include <stdio.h>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace polyanya;

Mesh m;

void test_io()
{
	m.print(cout);
}

void test_containment(Point test_point)
{
	for (int i = 0; i < (int) m.mesh_polygons.size(); i++)
	{
		cout << setw(4) << i << setw(0) << " ";
		int special = -1;
		cout << m.poly_contains_point(i, test_point, special);
		if (special != -1)
		{
			for (auto p : m.mesh_polygons[i].vertices)
			{
				cout << " " << m.mesh_vertices[p].p;
			}
			cout << " " << special;
		}
		cout << endl;
	}
}

void test_point_lookup(Point test_point)
{
	int a, b;
	m.get_point_location(test_point, a, b);
	cout << "Point " << test_point << " returns:" << endl;
	cout << a << " " << b << endl;
	cout << "from get_point_location." << endl;
}

int main(int argc, char* argv[])
{
	m = Mesh(cin);
	Point tp;
	if (argc == 3)
	{
		stringstream ss;
		ss << argv[1] << " " << argv[2];
		ss >> tp.x >> tp.y;
	}
	else
	{
		tp = {0, 0};
	}
	cout << "using point " << tp << endl;
	// test_io();
	// test_containment(tp);
	test_point_lookup(tp);
	return 0;
}
