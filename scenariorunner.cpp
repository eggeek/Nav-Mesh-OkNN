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

SearchInstance search;

bool get_path = true;

void print_header()
{
    cout << "index;length;gridcost" << endl;
}

void run_scenario(int index, Scenario scen)
{
    search.set_start_goal(scen.start, scen.goal);
    search.search();
    if (get_path)
    {
        vector<Point> path;
        search.get_path_points(path);
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
        double final_cost = search.get_cost();
        cout << setprecision(10) << fixed;
        cout << index << ";" << final_cost << ";" << scen.gridcost << endl;
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

    search = SearchInstance(m);

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
    return 0;
}
