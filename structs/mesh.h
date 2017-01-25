#pragma once
#include "polygon.h"
#include "vertex.h"
#include <vector>
#include <iostream>
#include <map>

namespace polyanya
{

enum struct PolyContainment
{
    OUTSIDE,
    INSIDE,
    ON_EDGE,
    ON_VERTEX,
};

inline std::ostream& operator<<(std::ostream& stream, const PolyContainment& pc)
{
    const std::string lookup[] = {"OUTSIDE", "INSIDE", "ON_EDGE", "ON_VERTEX"};
    return stream << lookup[static_cast<int>(pc)];
}

class Mesh
{
    public:
        Mesh() { }
        Mesh(std::istream& infile);
        std::vector<Vertex> mesh_vertices;
        std::vector<Polygon> mesh_polygons;
        std::map<double, std::vector<int>* > slabs;
        double min_x, max_x, min_y, max_y;

        void read(std::istream& infile);
        void precalc_point_location();
        void print(std::ostream& outfile);
        PolyContainment poly_contains_point(int poly, Point& p,
                                            int& special_index);
        void get_point_location(Point& p, int& out1, int& out2);
        void get_point_location_naive(Point& p, int& out1, int& out2);
        int get_any_poly_from_point(Point p);


    // TODO: "get polygon index from Point"
};

}
