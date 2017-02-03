// various testing functions
#include "scenario.h"
#include "searchinstance.h"
#include "point.h"
#include "mesh.h"
#include "cfg.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

using namespace std;
using namespace polyanya;

SearchInstance* si;

int get_path = 0;

void print_header()
{
    cout << "index;micro;generated;pushed;popped;pruned_post_pop;length;"
         << "gridcost" << endl;
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
        cout << index << "; ";
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
             << si->nodes_generated << ";"
             << si->nodes_pushed << ";"
             << si->nodes_popped << ";"
             << si->nodes_pruned_post_pop << ";"
             << setprecision(16) << si->get_cost() << ";"
             << setprecision(8) << scen.gridcost << endl;
    }
}

int main(int argc, char* argv[])
{
    warthog::util::param valid_args[] =
    {
        {"path", no_argument, &get_path, 1}
    };

    warthog::util::cfg cfg;
    cfg.parse_args(argc, argv, valid_args);

    if (argc - optind != 2)
    {
        cerr << "usage: " << argv[0] << "[--path] <mesh> <scenario>" << endl;
        return 1;
    }

    string temp = argv[optind];;
    ifstream meshfile(temp);
    if (!meshfile.is_open())
    {
        cerr << "Unable to open mesh" << endl;
        return 1;
    }
    Mesh* m = new Mesh(meshfile);
    meshfile.close();

    si = new SearchInstance(m);

    vector<Scenario> scenarios;
    temp = argv[optind+1];
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
    delete m;
    return 0;
}
