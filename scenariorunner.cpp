// various testing functions
#include "scenario.h"
#include "searchinstance.h"
#include "point.h"
#include "mesh.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;
using namespace polyanya;

SearchInstance search;

void print_header()
{
    cout << "index;length;gridcost" << endl;
}

void run_scenario(int index, Scenario scen)
{
    search.set_start_goal(scen.start, scen.goal);
    search.search();
    double final_cost = search.get_cost();
    cout << index << ";" << final_cost << ";" << scen.gridcost << endl;
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

    print_header();
    for (int i = 0; i < (int) scenarios.size(); i++)
    {
        run_scenario(i, scenarios[i]);
    }
    return 0;
}