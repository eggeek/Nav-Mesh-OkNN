/*
Converts a gridmap into a mesh made of big rectangles.
Uses "clearance" values, very very similar to
http://harabor.net/data/papers/harabor-botea-cig08.pdf.

Firstly - all orientation is based on
Clearance is biggest square you can make with bottom-right corner at that cell.
Area is the total area of the square, plus if you extend the square right/left.
When we take a rectangle, mark all of the squares of that rectangle
non-traversable and with the rectangle ID. Store that rectangle somewhere as
well.
*/
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <cassert>
#include <iomanip>

using namespace std;

typedef vector<bool> vbool;
typedef vector<int> vint;


// Everything here is [y][x]!
vector<vbool> map_traversable;
vector<vint> clearance;
vector<vint> area;
vector<vint> rectangle_id;
int map_width;
int map_height;



void fail(string msg)
{
    cerr << msg << endl;
    exit(1);
}

void read_map(istream& infile)
{
    // Most of this code is from dharabor's warthog.
    // read in the whole map. ensure that it is valid.
    unordered_map<string, string> header;

    // header
    for (int i = 0; i < 3; i++)
    {
        string hfield, hvalue;
        if (infile >> hfield)
        {
            if (infile >> hvalue)
            {
                header[hfield] = hvalue;
            }
            else
            {
                fail("err; map has bad header");
            }
        }
        else
        {
            fail("err; map has bad header");
        }
    }

    if (header["type"] != "octile")
    {
        fail("err; map type is not octile");
    }

    // we'll assume that the width and height are less than INT_MAX
    map_width = atoi(header["width"].c_str());
    map_height = atoi(header["height"].c_str());

    if (map_width == 0 || map_height == 0)
    {
        fail("err; map has bad dimensions");
    }

    // we now expect "map"
    string temp_str;
    infile >> temp_str;
    if (temp_str != "map")
    {
        fail("err; map does not have 'map' keyword");
    }


    // basic checks passed. initialse the map
    map_traversable = vector<vbool>(map_height, vbool(map_width));
    clearance = vector<vint>(map_height, vint(map_width, 0));
    rectangle_id = vector<vint>(map_height, vint(map_width, -1));
    area = vector<vint>(map_height, vint(map_width, -1));
    // so to get (x, y), do map_traversable[y][x]
    // 0 is nontraversable, 1 is traversable

    // read in map_data
    int cur_y = 0;
    int cur_x = 0;

    char c;
    while (infile.get(c))
    {
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
        {
            // whitespace.
            // cannot put in the switch statement below as we need to check
            // "too many chars" before anything else
            continue;
        }

        if (cur_y == map_height)
        {
            fail("err; map has too many characters");
        }

        switch (c)
        {
            case 'S':
            case 'W':
            case 'T':
            case '@':
            case 'O':
                // obstacle
                map_traversable[cur_y][cur_x] = 0;
                break;
            default:
                // traversable
                map_traversable[cur_y][cur_x] = 1;
                break;
        }

        cur_x++;
        if (cur_x == map_width)
        {
            cur_x = 0;
            cur_y++;
        }
    }

    if (cur_y != map_height || cur_x != 0)
    {
        fail("err; map has too few characters");
    }
}

int get_clearance(int y, int x)
{
    if (x < 0 || y < 0)
    {
        return 0;
    }
    assert(y < map_height);
    assert(x < map_width);
    if (!map_traversable[y][x])
    {
        return clearance[y][x] = 0;
    }
    if (clearance[y][x])
    {
        return clearance[y][x];
    }
    return clearance[y][x] = min(
        get_clearance(y-1, x),
        min(get_clearance(y,x-1),
            get_clearance(y-1, x-1))) + 1;
}

void calculate_clearance(int bottom_y, int bottom_x)
{
    // Bottom up DP.
    // Invalidate our cache and run get_clearance.
    // Go [bottom_x+1, end) for y from [0, bottom_y+1)
    // and then go [0, end) for y from [bottom_y+1, end)
    for (int y = 0; y < bottom_y+1; y++)
    {
        for (int x = bottom_x; x < map_width; x++)
        {
            get_clearance(y, x);
        }
    }
    for (int y = bottom_y+1; y < map_height; y++)
    {
        for (int x = 0; x < map_width; x++)
        {
            get_clearance(y, x);
        }
    }
}

void print_clearance()
{
    for (auto& x : clearance)
    {
        for (auto y : x)
        {
            if (y)
            {
                cout << setfill(' ') << setw(3) << y;
            }
            else
            {
                cout << "   ";
            }
        }
        cout << "\n";
    }
}

void print_traversable()
{
    for (auto& x : map_traversable)
    {
        for (auto y : x)
        {
            cout << "@."[y];
        }
        cout << "\n";
    }
}

int main()
{
    read_map(cin);
    calculate_clearance(-1, -1);
    print_clearance();
    print_traversable();
    return 0;
}
