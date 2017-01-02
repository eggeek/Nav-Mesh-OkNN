// test_io: confirms that the mesh is read in correctly
#include "mesh.h"
#include <stdio.h>

using namespace std;
using namespace polyanya;

int main()
{
	Mesh m(cin);
	m.print(cout);
	return 0;
}
