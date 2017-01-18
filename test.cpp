// various testing functions
#include "mesh.h"
#include "geometry.h"
#include <stdio.h>
#include <sstream>
#include <iomanip>
#include <time.h>
#include <random>

using namespace std;
using namespace polyanya;

Mesh m;

const int MIN_X = 0, MAX_X = 1024, MIN_Y = 0, MAX_Y = 768;
const int MAX_ITER = 10000;

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

void benchmark_point_lookup_single(Point test_point)
{
    int a, b;
    clock_t t;
    t = clock();
    m.get_point_location(test_point, a, b);
    t = clock() - t;
    cout << "Took " << t << " clock ticks." << endl;
    cout << "That should be " << (t/1.0/CLOCKS_PER_SEC * 1e6)
        << " microseconds." << endl;
    cout << "Point " << test_point << " returns:" << endl;
    cout << a << " " << b << endl;
    cout << "from get_point_location." << endl;
}

void benchmark_point_lookup_average()
{
    clock_t t;
    int a, b;
    Point p;
    t = clock();
    for (int y = MIN_Y; y <= MAX_Y; y++)
    {
        for (int x = MIN_X; x <= MAX_X; x++)
        {
            p.x = x;
            p.y = y;
            m.get_point_location(p, a, b);
        }
    }
    t = clock() - t;
    const double total_micro = (t/1.0/CLOCKS_PER_SEC * 1e6);
    const double average_micro = total_micro / ((MAX_X + 1) * (MAX_Y + 1));
    cout << "Point lookup took " << average_micro << "us on average." << endl;
}

void test_point_lookup_correct()
{
    cout << "Confirming that get_point_location is correct" << endl
         << "(this will take a while)..." << endl;
    for (int y = MIN_Y; y <= MAX_Y; y++)
    {
        cout << "y=" << y << endl;
        for (int x = MIN_X; x <= MAX_X; x++)
        {
            Point test_point = {(double)x, (double)y};
            int a, b, naive_a, naive_b;
            m.get_point_location(test_point, a, b);
            m.get_point_location_naive(test_point, naive_a, naive_b);
            if (a != naive_a || b != naive_b)
            {
                if (a == naive_b && b == naive_a) continue;
                cout << "Found discrepancy at " << test_point << endl;
                cout << "Method gives " << a << " " << b << endl;
                cout << "Naive gives " << naive_a << " " << naive_b << endl;
            }
        }
    }
}

void test_projection_correct()
{
    // Projection has asserts inside it so we just stress test those
    uniform_real_distribution<double> unif(-10, 10);
    default_random_engine engine;

    #define rand_point {unif(engine), unif(engine)}

    Point a, b, c, d;
    double d1, d2, d3;
    for (int i = 0; i < MAX_ITER; i++)
    {
        a = rand_point;
        b = rand_point;
        c = rand_point;
        d = rand_point;
        line_intersect_time(a, b, c, d, d1, d2, d3);
    }

    #undef rand_point
}

void test_reflection_correct()
{
    // Reflection has asserts inside it so we just stress test those
    uniform_real_distribution<double> unif(-10, 10);
    default_random_engine engine;

    #define rand_point {unif(engine), unif(engine)}

    Point a, b, c;
    for (int i = 0; i < MAX_ITER; i++)
    {
        a = rand_point;
        b = rand_point;
        c = rand_point;
        reflect_point(a, b, c);
    }

    #undef rand_point
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
    // test_point_lookup_correct();
    // benchmark_point_lookup_average();
    // benchmark_point_lookup_single(tp);
    test_projection_correct();
    test_reflection_correct();
    return 0;
}
