#include <Fade_2D.h>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>

using namespace std;
using namespace GEOM_FADE2D;

vector<vector<Point2>> polys;

typedef ConstraintGraph2* CGPointer;
vector<CGPointer> constraint_graphs;
typedef Zone2* ZonePointer;
ZonePointer traversable;

Fade_2D dt;

// Retrieves the triangles of pZone and highlights them in the triangulation.
void highlightTriangles(Fade_2D& dt,Zone2* pZone, const string& filename)
{
	// 1) Show the full triangulation
	Visualizer2 vis(filename);
	dt.show(&vis);

	// 2) Fetch the triangles from the zone
	vector<Triangle2*> vZoneT;
	pZone->getTriangles(vZoneT);

	// 3) Highlight them in red
	Color colorRed(1, 0, 0, 0.01, true); // The true parameter means 'fill'
	for(size_t i=0; i<vZoneT.size(); i++)
	{
		vis.addObject(*vZoneT[i], colorRed);
	}
	vis.writeFile();
}

void fail(const string& message)
{
	cerr << message << endl;
	exit(1);
}

void read_polys(const string& filename)
{
	ifstream infile(filename);
	if (!infile.is_open())
	{
		fail("Unable to open file");
	}

	string header;
	int version;

	if (!(infile >> header))
	{
		fail("Error reading header");
	}
	if (header != "poly")
	{
		cerr << "Got header '" << header << "'" << endl;
		fail("Invalid header (expecting 'poly')");
	}

	if (!(infile >> version))
	{
		fail("Error getting version number");
	}
	if (version != 1)
	{
		cerr << "Got file with version " << version << endl;
		fail("Invalid version (expecting 1)");
	}

	int N;
	if (!(infile >> N))
	{
		fail("Error getting number of polys");
	}
	if (N < 1)
	{
		cerr << "Got " << N << "polys" << endl;
		fail("Invalid number of polys");
	}

	for (int i = 0; i < N; i++)
	{
		int M;
		if (!(infile >> M))
		{
			fail("Error parsing map (can't get number of points of poly)");
		}
		if (M < 3)
		{
			cerr << "Got " << N << "points" << endl;
			fail("Invalid number of points in poly");
		}
		vector<Point2> cur_poly;
		for (int j = 0; j < M; j++)
		{
			int x, y;
			if (!(infile >> x >> y))
			{
				fail("Error parsing map (can't get point)");
			}
			cur_poly.push_back(Point2(x, y));
		}
		polys.push_back(cur_poly);
	}
}

void create_constraint_graphs()
{
	for (auto poly : polys)
	{
		vector<Segment2> segments;
		for (int i = 0; i < ((int)poly.size()) - 1; i++)
		{
			const Point2& p0 = poly[i];
			const Point2& p1 = poly[i+1];
			segments.push_back(Segment2(p0, p1));
		}
		segments.push_back(Segment2(poly.back(), poly.front()));
		CGPointer cg = dt.createConstraint(segments, CIS_CONSTRAINED_DELAUNAY);
		constraint_graphs.push_back(cg);
	}
}

void print_points()
{
	for (auto poly : polys)
	{
		cout << poly.size() << " ";
		for (auto p : poly)
		{
			double x, y;
			p.xy(x, y);
			cout << x << " " << y << " ";
		}
		cout << endl;
	}
}

int main(int argc, char* argv[])
{
	if (argc != 2)
	{
		cerr << "usage: " << argv[0] << " <file>" << endl;
		return 1;
	}
	string filename = argv[1];
	read_polys(filename);
	create_constraint_graphs();
	print_points();
	dt.applyConstraintsAndZones();
	for (auto x : constraint_graphs)
	{
		assert(x->isPolygon());
	}
	dt.show(filename + "-nozone.ps");

	vector<ZonePointer> zones;
	for (auto x : constraint_graphs)
	{
		zones.push_back(dt.createZone(x, ZL_INSIDE));
	}
	assert(!constraint_graphs.empty());
	traversable = zones.front();
	for (int i = 1; i < ((int)zones.size()); i++)
	{
		traversable = zoneSymmetricDifference(traversable, zones[i]);
	}
	highlightTriangles(dt, traversable, filename + "-traversable.ps");

	return 0;
}
