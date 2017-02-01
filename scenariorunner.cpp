// various testing functions
#include "scenario.h"
#include "searchinstance.h"
#include "point.h"
#include "mesh.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

using namespace std;
using namespace polyanya;

SearchInstance* si;

bool get_path = false;

void print_header()
{
    cout << "index;micro;expanded;generated;length;gridcost" << endl;
}

void run_scenario(int index, Scenario scen)
{
    si->set_start_goal(scen.start, scen.goal);
    si->search();
    if (get_path)
    {
        vector<Point> path;
        si->get_path_points(path);
        const int n = (int) path.size();
        for (int i = 0; i < n; i++)
        {
            cout << path[i];
            if (i != n-1)
            {
                cout << " ";
            }
            else
            {
                cout << endl;
            }
        }
    }
    else
    {
        cout << index << ";"
             << si->get_search_micro() << ";"
             << si->get_nodes_expanded() << ";"
             << si->get_nodes_generated() << ";"
             << setprecision(10) << fixed << si->get_cost() << ";"
             << setprecision(6) << defaultfloat << scen.gridcost << endl;
    }
}

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        cerr << "usage: " << argv[0] << " <mesh> <scenario>" << endl;
        return 1;
    }

    string temp = argv[1];;
    ifstream meshfile(temp);
    if (!meshfile.is_open())
    {
        cerr << "Unable to open mesh" << endl;
        return 1;
    }
    MeshPtr m = MeshPtr(new Mesh(meshfile));
    meshfile.close();

    si = new SearchInstance(m);

    vector<Scenario> scenarios;
    temp = argv[2];
    ifstream scenfile(temp);
    if (!scenfile.is_open())
    {
        cerr << "Unable to open scenarios" << endl;
        return 1;
    }
    load_scenarios(scenfile, scenarios);
    scenfile.close();

    if (!get_path)
    {
        print_header();
    }
    for (int i = 0; i < (int) scenarios.size(); i++)
    {
        run_scenario(i, scenarios[i]);
    }

    delete si;
    return 0;
}
