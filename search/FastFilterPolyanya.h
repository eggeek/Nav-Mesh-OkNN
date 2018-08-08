#pragma once
#include "searchinstance.h"
#include "point.h"
#include "timer.h"
#include "RStarTree.h"
#include "RStarTreeUtil.h"
#include "knnMeshFence.h"
#include <chrono>
#include <queue>
#include <vector>
#include <ctime>

using namespace std;

namespace polyanya {

namespace rs = rstar;

class FastFilterPolyanya {
    typedef std::priority_queue<SearchNodePtr, std::vector<SearchNodePtr>,
                                PointerComp<SearchNode> > pq;
    private:
        int K = 1;
        Point start;
        std::vector<Point> goals;
        warthog::timer timer;
        KnnMeshEdgeFence* meshFence;
        SearchInstance* polyanya;

        void init() {
            meshFence= nullptr;
            verbose = false;
        }

        void initRtree() {
          assert(rte == nullptr);
          rte = new rs::RStarTree();
          rtEntries.clear();
          gids.clear();
          for (int i=0; i<(int)goals.size(); i++) gids.push_back(i);
          for (int& i: gids) {
            const Point& it = goals[i];
            rs::Mbr mbr(it.x, it.x, it.y, it.y);
            rs::LeafNodeEntry leaf(mbr, (rs::Data_P)(&i));
            rtEntries.push_back(leaf);
          }

          for (auto& it: rtEntries)
            rte->insertData(&it);
        }

        void init_search() {
          search_cost = 0;
          rtree_cost = 0;
          nodes_generated = 0;
          nodes_pushed = 0;
          nodes_popped = 0;
          tot_hit = 0;
        }

    public:
        int nodes_generated;        // Nodes stored in memory
        int nodes_pushed;           // Nodes pushed onto open
        int nodes_popped;           // Nodes popped off open
        int tot_hit;
        double search_cost;
        double rtree_cost;
        bool verbose;
        rs::RStarTree* rte = nullptr;
        std::vector<rs::LeafNodeEntry> rtEntries;
        std::vector<int> gids;

        FastFilterPolyanya() { }
        FastFilterPolyanya(SearchInstance* si): polyanya(si) {
          rte = nullptr; 
        };
        FastFilterPolyanya(FastFilterPolyanya const &) = delete;
        void operator=(FastFilterPolyanya const &x) = delete;
        ~FastFilterPolyanya() {
            if (rte)
              delete rte;
        }

        void set_K(int k) { this->K = k; }
        void set_goals(std::vector<Point> gs) {
          goals = std::vector<Point>(gs);
          initRtree();
        }
        void set_start(Point s) { start = s; }
        void set_meshFence(KnnMeshEdgeFence* meshFence) { this->meshFence= meshFence; }
        vector<double> search();        
};

}
