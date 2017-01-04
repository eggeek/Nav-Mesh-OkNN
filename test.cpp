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

int main(int argc, char* argv[])
{
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
	m = Mesh(cin);
	test_io();
	test_containment(tp);
	return 0;
}
