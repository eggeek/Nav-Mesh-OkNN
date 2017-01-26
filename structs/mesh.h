#pragma once
#include "polygon.h"
#include "vertex.h"
#include <vector>
#include <iostream>
#include <map>
#include <memory>

namespace polyanya
{

struct PolyContainment
{
    enum Type
    {
        // Does not use any ints.
        OUTSIDE,

        // Does not use any ints.
        INSIDE,

        // Uses adjacent_poly, vertex1 and vertex2.
        ON_EDGE,

        // Uses vertex1.
        ON_VERTEX,
    };

    Type type;

    int adjacent_poly;

    // If on edge, vertex1/vertex2 represents the left/right vertices of the
    // edge when looking from a point in the poly.
    int vertex1, vertex2;

    friend std::ostream& operator<<(std::ostream& stream,
                                    const PolyContainment& pc)
    {
        switch (pc.type)
        {
            case PolyContainment::OUTSIDE:
                return stream << "OUTSIDE";

            case PolyContainment::INSIDE:
                return stream << "INSIDE";

            case PolyContainment::ON_EDGE:
                return stream << "ON_EDGE (poly " << pc.adjacent_poly
                              << ", vertices " << pc.vertex1 << ", "
                              << pc.vertex2 << ")";

            case PolyContainment::ON_VERTEX:
                return stream << "ON_VERTEX (" << pc.vertex1 << ")";

            default:
                assert(false);
                return stream;
        }
    }
};

typedef std::shared_ptr<int> IntPtr;

class Mesh
{
    public:
        Mesh() { }
        Mesh(std::istream& infile);
        std::vector<Vertex> mesh_vertices;
        std::vector<Polygon> mesh_polygons;
        std::map<double, std::vector<int>> slabs;
        double min_x, max_x, min_y, max_y;

        void read(std::istream& infile);
        void precalc_point_location();
        void print(std::ostream& outfile);
        PolyContainment poly_contains_point(int poly, Point& p);
        void get_point_location(Point& p, int& out1, int& out2,
                                IntPtr left_vertex = nullptr,
                                IntPtr right_vertex = nullptr);
        void get_point_location_naive(Point& p, int& out1, int& out2);
        int get_any_poly_from_point(Point p, int& bonus,
                                    IntPtr left_vertex = nullptr,
                                    IntPtr right_vertex = nullptr);


    // TODO: "get polygon index from Point"
};

}
