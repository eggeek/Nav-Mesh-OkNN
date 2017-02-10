// Takes mesh from stdin, outputs to stdout.
#include <iostream>
#include <vector>
#include <string>
#include <memory>
using namespace std;

// We need a circular linked list of sorts.
// This is going to be used a lot - for polys around point and for merging the
// two polygon arrays together.
// We'll just use a std::shared_ptr to handle our memory...
struct ListNode
{
    shared_ptr<ListNode> next;
    int val;
};

typedef shared_ptr<ListNode> ListNodePtr;
#define make_node(next, val) ListNodePtr(new ListNode {next, val})

struct Vertex
{
    double x, y;
    int num_polygons;
    ListNodePtr polygons;
};

struct Polygon
{
    int num_vertices;
    ListNodePtr vertices;
    ListNodePtr polygons;
};

// We'll keep all vertices, but we may throw them out in the end if num_polygons
// is 0.
// We'll figure it out once we're finished.
vector<Vertex> mesh_vertices;

// We'll also keep all polygons, but we'll throw them out like above.
vector<Polygon> mesh_polygons;


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


    for (int i = 0; i < V; i++)
    {
        Vertex& v = mesh_vertices[i];
        if (!(infile >> v.x >> v.y))
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
        outfile << v.x << " " << v.y << "\t";
        outfile << v.num_polygons << "\t";

        outfile << get_p(v.polygons->val);
        {
            ListNodePtr cur_node = v.polygons->next;
            while (cur_node != v.polygons)
            {
                outfile << " " << get_p(cur_node->val);
                cur_node = cur_node->next;
            }
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
    print_mesh(cout);
    return 0;
}
