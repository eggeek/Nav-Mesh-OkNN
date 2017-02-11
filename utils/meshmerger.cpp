// Takes mesh from stdin, outputs to stdout.
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <cassert>
#include <numeric>
using namespace std;

// We need union find!
struct UnionFind
{
    vector<int> parent;

    UnionFind(int n) : parent(n)
    {
        iota(parent.begin(), parent.end(), 0);
    }

    int find(int x)
    {
        if (parent[x] != x)
        {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }

    // can't use "union" as that's a keyword!
    // also: don't use union by rank as we need find(x) == x after merge.
    void merge(int x, int y)
    {
        x = find(x);
        y = find(y);
        parent[y] = x;
    }
};

// We need a circular linked list of sorts.
// This is going to be used a lot - for polys around point and for merging the
// two polygon arrays together.
// We'll just use a std::shared_ptr to handle our memory...
struct ListNode
{
    shared_ptr<ListNode> next;
    int val;

    shared_ptr<ListNode> go(int n) const
    {
        shared_ptr<ListNode> out = next;
        for (int i = 1; i < n; i++)
        {
            out = out->next;
        }
        return out;
    }
};

typedef shared_ptr<ListNode> ListNodePtr;
#define make_node(next, val) ListNodePtr(new ListNode {next, val})

struct Point
{
    double x, y;

    Point operator+(const Point& other) const
    {
        return {x + other.x, y + other.y};
    }

    Point operator-(const Point& other) const
    {
        return {x - other.x, y - other.y};
    }

    double operator*(const Point& other) const
    {
        return x * other.y - y * other.x;
    }
};

struct Vertex
{
    Point p;
    int num_polygons;
    ListNodePtr polygons;
};

struct Polygon
{
    int num_vertices;
    int num_traversable;
    ListNodePtr vertices;
    // Stores the original polygons.
    // To get the actual polygon, do polygon_unions.find on the polygon you get.
    ListNodePtr polygons;
};

// We'll keep all vertices, but we may throw them out in the end if num_polygons
// is 0.
// We'll figure it out once we're finished.
vector<Vertex> mesh_vertices;

// We'll also keep all polygons, but we'll throw them out like above.
vector<Polygon> mesh_polygons;

UnionFind polygon_unions(0);


// taken from structs/mesh.cpp
void read_mesh(istream& infile)
{
    #define fail(message) cerr << message << endl; exit(1);
    string header;
    int version;

    if (!(infile >> header))
    {
        fail("Error reading header");
    }
    if (header != "mesh")
    {
        cerr << "Got header '" << header << "'" << endl;
        fail("Invalid header (expecting 'mesh')");
    }

    if (!(infile >> version))
    {
        fail("Error getting version number");
    }
    if (version != 2)
    {
        cerr << "Got file with version " << version << endl;
        fail("Invalid version (expecting 2)");
    }

    int V, P;
    if (!(infile >> V >> P))
    {
        fail("Error getting V and P");
    }
    if (V < 1)
    {
        cerr << "Got " << V << " vertices" << endl;
        fail("Invalid number of vertices");
    }
    if (P < 1)
    {
        cerr << "Got " << P << " polygons" << endl;
        fail("Invalid number of polygons");
    }

    mesh_vertices.resize(V);
    mesh_polygons.resize(P);
    polygon_unions = UnionFind(P);


    for (int i = 0; i < V; i++)
    {
        Vertex& v = mesh_vertices[i];
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
            cerr << "Got " << neighbours << " neighbours" << endl;
            fail("Invalid number of neighbours around a point");
        }

        v.num_polygons = neighbours;
        // Guaranteed to have 2 or more.
        ListNodePtr cur_node = nullptr;
        for (int j = 0; j < neighbours; j++)
        {
            int polygon_index;
            if (!(infile >> polygon_index))
            {
                fail("Error getting a vertex's neighbouring polygon");
            }
            if (polygon_index >= P)
            {
                cerr << "Got a polygon index of " \
                          << polygon_index << endl;
                fail("Invalid polygon index when getting vertex");
            }

            ListNodePtr new_node = make_node(nullptr, polygon_index);

            if (j == 0)
            {
                cur_node = new_node;
                v.polygons = cur_node;
            }
            else
            {
                cur_node->next = new_node;
                cur_node = new_node;
            }
        }
        cur_node->next = v.polygons;
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
            cerr << "Got " << n << " vertices" << endl;
            fail("Invalid number of vertices in polygon");
        }

        p.num_vertices = n;

        ListNodePtr cur_node = nullptr;
        for (int j = 0; j < n; j++)
        {
            int vertex_index;
            if (!(infile >> vertex_index))
            {
                fail("Error getting a polygon's vertex");
            }
            if (vertex_index >= V)
            {
                cerr << "Got a vertex index of " \
                          << vertex_index << endl;
                fail("Invalid vertex index when getting polygon");
            }
            ListNodePtr new_node = make_node(nullptr, vertex_index);

            if (j == 0)
            {
                cur_node = new_node;
                p.vertices = cur_node;
            }
            else
            {
                cur_node->next = new_node;
                cur_node = new_node;
            }
        }
        cur_node->next = p.vertices;

        // don't worry: the old one is still being pointed to
        cur_node = nullptr;
        p.num_traversable = 0;
        for (int j = 0; j < n; j++)
        {
            int polygon_index;
            if (!(infile >> polygon_index))
            {
                fail("Error getting a polygon's neighbouring polygon");
            }
            if (polygon_index >= P)
            {
                cerr << "Got a polygon index of " \
                          << polygon_index << endl;
                fail("Invalid polygon index when getting polygon");
            }

            if (polygon_index != -1)
            {
                p.num_traversable++;
            }
            ListNodePtr new_node = make_node(nullptr, polygon_index);

            if (j == 0)
            {
                cur_node = new_node;
                p.polygons = cur_node;
            }
            else
            {
                cur_node->next = new_node;
                cur_node = new_node;
            }
        }
        cur_node->next = p.polygons;
    }

    double temp;
    if (infile >> temp)
    {
        fail("Error parsing mesh (read too much)");
    }
    #undef fail
}

inline bool cw(const Point& a, const Point& b, const Point& c)
{
    return (b - a) * (c - b) < -1e-8;
}

// Can polygon x merge with the polygon adjacent to the edge
// (v->next, v->next->next)?
// (The reason for this is because we don't have back pointers, and we need
// to have the vertex before the edge starts).
// Assume that v and p are "aligned", that is, they have been offset by the
// same amount.
// This also means that the actual polygon used will be p->next->next.
// Also assume that x is a valid non-merged polygon.
bool can_merge(int x, ListNodePtr v, ListNodePtr p)
{
    const int merge_index = p->go(2)->val;
    if (merge_index == -1)
    {
        return false;
    }
    const Polygon& to_merge = mesh_polygons[polygon_unions.find(merge_index)];

    // Define (v->next, v->next->next).
    const int A = v->go(1)->val;
    const int B = v->go(2)->val;

    // We want to find (B, A) inside to_merge's vertices.
    // In fact, we want to find the one BEFORE B. We'll call this merge_end.
    // Assert that we have good data - that is, if B appears, A must be next.
    // Also, we can't iterate for more than to_merge.num_vertices.
    ListNodePtr merge_end_v = to_merge.vertices;
    ListNodePtr merge_end_p = to_merge.polygons;
    int counter;
    counter = 0;
    while (merge_end_v->next->val != B)
    {
        merge_end_v = merge_end_v->next;
        merge_end_p = merge_end_p->next;
        counter++;
        assert(counter <= to_merge.num_vertices);
    }
    // Ensure that A comes after B.
    assert(merge_end_v->go(2)->val == A);
    // Ensure that the neighbouring polygon is x.
    assert(polygon_unions.find(merge_end_p->go(2)->val) == x);

    // The merge will change
    // (v, A, B) to (v, A, [3 after merge_end_v]) and
    // (A, B, [3 after v]) to (merge_end_v, B, [3 after v]).
    // If the new ones are clockwise, we must return false.
    #define P(ptr) mesh_vertices[(ptr)->val].p
    if (cw(P(v), P(v->go(1)), P(merge_end_v->go(3))))
    {
        return false;
    }

    if (cw(P(merge_end_v), P(v->go(2)), P(v->go(3))))
    {
        return false;
    }

    #undef P

    return true;
}

void check_correct()
{
    for (int i = 0; i < (int) mesh_vertices.size(); i++)
    {
        Vertex& v = mesh_vertices[i];
        if (v.num_polygons == 0)
        {
            continue;
        }

        int count = 1;
        ListNodePtr cur_node = v.polygons->next;
        while (cur_node != v.polygons)
        {
            assert(count < v.num_polygons);
            cur_node = cur_node->next;
            count++;
        }
        assert(count == v.num_polygons);
    }

    for (int i = 0; i < (int) mesh_polygons.size(); i++)
    {
        Polygon& p = mesh_polygons[i];
        if (polygon_unions.find(i) != i || p.num_vertices == 0)
        {
            // Has been merged.
            continue;
        }

        {
            #define P(ptr) mesh_vertices[(ptr)->val].p
            int count = 1;

            assert(!cw(P(p.vertices), P(p.vertices->next),
                       P(p.vertices->next->next)));
            can_merge(i, p.vertices, p.polygons);

            ListNodePtr cur_node_v = p.vertices->next;
            ListNodePtr cur_node_p = p.polygons->next;
            while (cur_node_v != p.vertices)
            {
                assert(count < p.num_vertices);
                assert(!cw(P(cur_node_v), P(cur_node_v->next),
                           P(cur_node_v->next->next)));
                can_merge(i, cur_node_v, cur_node_p);

                cur_node_v = cur_node_v->next;
                cur_node_p = cur_node_p->next;
                count++;
            }

            assert(count == p.num_vertices);

            #undef P
        }

        {
            int count = 1;
            ListNodePtr cur_node = p.polygons->next;
            while (cur_node != p.polygons)
            {
                assert(count < p.num_vertices);
                cur_node = cur_node->next;
                count++;
            }
            assert(count == p.num_vertices);
        }
    }
}

void print_mesh(ostream& outfile)
{
    outfile << "mesh\n";
    outfile << "2\n";

    outfile << "\n";

    vector<int> vertex_mapping;
    vertex_mapping.resize(mesh_vertices.size());
    {
        // We need to create a mapping from old-vertex to new-vertex.
        int next_index = 0;
        for (int i = 0; i < (int) mesh_vertices.size(); i++)
        {
            vertex_mapping[i] = next_index;
            if (mesh_vertices[i].num_polygons != 0)
            {
                next_index++;
            }
        }
    }

    vector<int> polygon_mapping;
    polygon_mapping.resize(mesh_polygons.size());
    {
        // We need to create a mapping from old-vertex to new-vertex.
        int next_index = 0;
        for (int i = 0; i < (int) mesh_polygons.size(); i++)
        {
            polygon_mapping[i] = next_index;
            if (mesh_polygons[i].num_vertices != 0)
            {
                next_index++;
            }
        }
    }

    #define get_v(v) ((v) == -1 ? -1 : vertex_mapping[v]);
    #define get_p(p) ((p) == -1 ? -1 : polygon_mapping[p]);

    outfile << vertex_mapping.back() + 1 << " " << polygon_mapping.back() + 1
            << "\n";

    outfile << "\n";

    for (int i = 0; i < (int) mesh_vertices.size(); i++)
    {
        Vertex& v = mesh_vertices[i];
        if (v.num_polygons == 0)
        {
            continue;
        }
        outfile << v.p.x << " " << v.p.y << "\t";
        outfile << v.num_polygons << "\t";

        outfile << get_p(v.polygons->val);
        {
            int count = 1;
            ListNodePtr cur_node = v.polygons->next;
            while (cur_node != v.polygons)
            {
                assert(count < v.num_polygons);
                outfile << " " << get_p(cur_node->val);
                cur_node = cur_node->next;
                count++;
            }
            assert(count == v.num_polygons);
        }
        outfile << "\n";
    }

    outfile << "\n";

    for (int i = 0; i < (int) mesh_polygons.size(); i++)
    {
        Polygon& p = mesh_polygons[i];
        if (p.num_vertices == 0)
        {
            continue;
        }
        outfile << p.num_vertices << "\t";

        outfile << get_v(p.vertices->val);
        {
            ListNodePtr cur_node = p.vertices->next;
            while (cur_node != p.vertices)
            {
                outfile << " " << get_v(cur_node->val);
                cur_node = cur_node->next;
            }
        }
        outfile << "\t";

        outfile << get_p(p.polygons->val);
        {
            ListNodePtr cur_node = p.polygons->next;
            while (cur_node != p.polygons)
            {
                outfile << " " << get_p(cur_node->val);
                cur_node = cur_node->next;
            }
        }
        outfile << "\n";
    }

    #undef get_p
    #undef get_v
}

int main()
{
    read_mesh(cin);
    check_correct();
    print_mesh(cout);
    return 0;
}
