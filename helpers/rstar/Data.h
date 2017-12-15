#ifndef DATA_H
#define DATA_H

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include "RStarTree.h"

namespace rstar {
const double NORMALIZE_SCALE = 100000;

typedef struct Point
{
    Coord coord[DIM];

    friend istream& operator>>(istream& in, Point& p);

    bool operator< (const Point& p) const;

    Point();
    Point(Coord c[DIM]);
    Point(Coord x, Coord y);

    void print();

    double distance2(Point& p);
}Point;

inline double Point::distance2(Point& p)
{
    double dis = 0;
    for(size_t dim = 0; dim < DIM; dim++)
    {
        double diff = coord[dim] - p.coord[dim];
        dis += diff * diff;
    }
    return dis;
}

typedef vector<Point> Point_V;
typedef vector<LeafNodeEntry> Entry_V;

typedef struct Data
{
    static Point_V facilities;
    static Point_V clients;
    static Point_V queries;

    static RStarTree* fTree;
    static RStarTree* cTree;
    static Point* query;
    static size_t indexq;

    static void buildFTree();
    static void buildCTree();
    static void loadQueries();
    static size_t getNumOfQueries();
    static void next();

    Data();
    ~Data();

    private:
        static Entry_V fEntries;
        static Entry_V cEntries;

        static void buildTree(Point_V& points, Entry_V& entries, RStarTree* tree);
        static void loadPoint(ifstream& in, Point_V& points, string fileName);
}Data;

typedef struct Result
{
    Result()
        : aggregate(0)
        { }

    Node_P_V nodes;
    Entry_P_V entries;
    size_t aggregate;

    void insert(Node_P np);
    void insert(Entry_P ep);
}Result;

inline void Result::insert(Node_P np)
{
    nodes.push_back(np);
    aggregate += np->aggregate;
}

inline void Result::insert(Entry_P ep)
{
    entries.push_back(ep);
    aggregate++;
}

typedef struct ConvexPolygon
{
    ConvexPolygon()
        : phi(false)
        { }

    HalfSpace_V hSpaces;
    bool phi;

    void init();
    bool prune(HalfSpace& halfSpace);
    void getVertex(Vertex_S& vs);
    void print();
}ConvexPolygon;

inline void ConvexPolygon::init()
{
    HalfSpace topHS, bottomHS, leftHS, rightHS;

    topHS.line.b = NORMALIZE_SCALE;
    topHS.from.x = 0;
    topHS.from.y = NORMALIZE_SCALE;
    topHS.to.x = NORMALIZE_SCALE;
    topHS.to.y = NORMALIZE_SCALE;

    bottomHS.line.b = 0;
    bottomHS.from.x = 0;
    bottomHS.from.y = 0;
    bottomHS.to.x = NORMALIZE_SCALE;
    bottomHS.to.y = 0;

    leftHS.line.vertical = true;
    leftHS.line.c = 0;
    leftHS.from.x = 0;
    leftHS.from.y = 0;
    leftHS.to.x = 0;
    leftHS.to.y = NORMALIZE_SCALE;

    rightHS.line.vertical = true;
    rightHS.line.c = NORMALIZE_SCALE;
    rightHS.from.x = NORMALIZE_SCALE;
    rightHS.from.y = 0;
    rightHS.to.x = NORMALIZE_SCALE;
    rightHS.to.y = NORMALIZE_SCALE;

    hSpaces.push_back(topHS);
    hSpaces.push_back(bottomHS);
    hSpaces.push_back(leftHS);
    hSpaces.push_back(rightHS);
}

inline bool ConvexPolygon::prune(HalfSpace& halfSpace)
{
    if(phi)
        return false;

    size_t cnt = 0;
    Vertex cross[3];
    for(HalfSpace_V::iterator iterHS = hSpaces.begin(); iterHS != hSpaces.end(); iterHS++)
    {
        if(!Util2D::lineCross(iterHS->line, halfSpace.line, cross[cnt]))
            continue;
        if(iterHS->line.vertical)
        {
            if( (iterHS->from.y + EPS < cross[cnt].y && cross[cnt].y + EPS < iterHS->to.y)
               || (iterHS->from.y > cross[cnt].y + EPS && cross[cnt].y > iterHS->to.y + EPS ) )
            {
                assert(cnt < 2);
                cnt++;
            }
        }
        else
        {
            if( (iterHS->from.x + EPS < cross[cnt].x && cross[cnt].x + EPS < iterHS->to.x)
               || (iterHS->from.x > cross[cnt].x + EPS && cross[cnt].x > iterHS->to.x + EPS) )
            {
                assert(cnt < 2);
                cnt++;
            }
        }
    }

    if(cnt == 0)
    {
        if(Util2D::isAboveLine(halfSpace.line, hSpaces.begin()->from) == halfSpace.isAbove)
        {
            phi = true;
            hSpaces.clear();
        }
        else
            ; // no effect
    }
    else if(cnt == 2)
    {
        HalfSpace_V newhSpaces;
        for(HalfSpace_V::iterator iterHS = hSpaces.begin(); iterHS != hSpaces.end(); iterHS++)
        {
            if(Util2D::isAboveLine(halfSpace.line, iterHS->from) == halfSpace.isAbove
               && Util2D::isAboveLine(halfSpace.line, iterHS->to) == halfSpace.isAbove)
               ;
            else
            {
                Vertex intersection;
                if(Util2D::lineCross(iterHS->line, halfSpace.line, intersection))
                {
                    if(Util2D::isAboveLine(halfSpace.line, iterHS->from) == halfSpace.isAbove)
                    {
                        iterHS->from.x = intersection.x;
                        iterHS->from.y = intersection.y;
                        cnt--;
                        assert(Util2D::isAboveLine(halfSpace.line, iterHS->to) != halfSpace.isAbove);
                    }
                    else if(Util2D::isAboveLine(halfSpace.line, iterHS->to) == halfSpace.isAbove)
                    {
                        iterHS->to.x = intersection.x;
                        iterHS->to.y = intersection.y;
                        cnt--;
                    }
                }
                newhSpaces.push_back(*iterHS);
            }
        }
        assert(cnt == 0);
        halfSpace.from = cross[0];
        halfSpace.to = cross[1];
        newhSpaces.push_back(halfSpace);
        hSpaces.clear();
        hSpaces.swap(newhSpaces);
        return true;
    }
/*
    else
    {
        assert(false);
    }
*/

    return false;
}

inline void ConvexPolygon::getVertex(Vertex_S& vs)
{
    for(size_t ihs = 0; ihs < hSpaces.size(); ihs++)
    {
        HalfSpace halfSpace = hSpaces.at(ihs);
        vs.insert(halfSpace.from);
        vs.insert(halfSpace.to);
    }
}

inline void ConvexPolygon::print()
{
    ofstream crOFS("data/debug/cr.txt");
    assert(crOFS.good());
    for(size_t ihs = 0; ihs < hSpaces.size(); ihs++)
    {
        HalfSpace halfSpace = hSpaces.at(ihs);
        crOFS << halfSpace.from.x << " " << halfSpace.from.y << " " << halfSpace.to.x << " " << halfSpace.to.y << endl;
    }
    crOFS.close();
}

#endif // DATA_H
}


