#include "searchnode.h"
#include "successor.h"
#include "mesh.h"
#include "point.h"
#include <vector>

namespace polyanya
{

// Gets the h value of a search node with interval l-r and root "root",
// given a goal.
double get_h_value(const Point& root, Point goal,
                   const Point& l, const Point& r);

// Generates the successors of the search node and appends them to the successor
// vector.
void get_successors(SearchNode& node, const Point& start, const Mesh& mesh,
                    std::vector<Successor>& successors);
}
