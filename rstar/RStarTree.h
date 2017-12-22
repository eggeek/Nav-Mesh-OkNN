#pragma once
#include <algorithm>
#include "Util2D.h"

namespace rstar {

#define MAX_CHILD (pageSize - sizeof(RTreeNode)) / sizeof(RTreeNode)

const size_t DIM = 2;
const size_t LEAF_LEVEL = 0;
const size_t PAGE_SIZE = 4096;

const double SPLIT_FACTOR = 0.4;
const double REINSERT_FACTOR = 0.3;
const size_t NEAR_MINIMUM_OVERLAP_FACTOR = 32;

/*******************************************************************************
 * Mbr - minimum bounding rectangle
 ******************************************************************************/

typedef struct Mbr
{
    Coord coord[DIM][2];

    Mbr();
    Mbr(Coord c[DIM]);
    Mbr(Coord c[DIM][2]);
    Mbr(Coord xmin, Coord xmax, Coord ymin, Coord ymax);

    void print();

    static Mbr getMbr(Mbr &mbr1, Mbr &mbr2);
    static double getOverlap(Mbr &mbr1, Mbr &mbr2);
    static double getEnlarge(Mbr &base, Mbr &add);

    double getMargin() const;
    double getArea() const;
    inline double getCenter(size_t dim);

    void enlarge(Mbr &add);
    void init();
}Mbr;

/*******************************************************************************
 * RTreeNode
 ******************************************************************************/

typedef void* Data_P;

typedef struct LeafNodeEntry
{
    Mbr mbre;
    Data_P data;
    double value;

    LeafNodeEntry(Mbr mbre, Data_P data);

    void print(string ident);
}LeafNodeEntry;

typedef LeafNodeEntry* Entry_P;
typedef vector<Entry_P> Entry_P_V;

typedef struct RTreeNode RTreeNode;

typedef RTreeNode* Node_P;
typedef vector<Node_P> Node_P_V;
typedef list<Node_P> Node_P_L;

struct RTreeNode
{
    size_t level; // Level of the node. All leaf nodes are at level 0.
    size_t aggregate;
    Mbr mbrn;
    Node_P_V* children;
    Entry_P_V* entries;
    Node_P parent;
    double value;

    RTreeNode(size_t level);
    ~RTreeNode();

    bool operator < (const RTreeNode& node) const;

    void print(string ident);

    size_t size();
    void insert(Node_P childPtr);
    void insert(Entry_P entryPtr);
    void take(RTreeNode &source, size_t from); // Copy and remove elements from source.
    void resize(size_t from);
    void adjustMbr();
    void enlargeMbr(Mbr &mbr);
    void decreaseAggregate(size_t amount);
    void increaseAggregate(size_t amount);
    void prepareEnlargementSort(Mbr &add);
    void prepareDisSort();
};

/*******************************************************************************
 * Sorting
 ******************************************************************************/

typedef struct AxisSort
{
    size_t axis;

    AxisSort(size_t axis);

    bool operator() (Node_P child1, Node_P child2);
    bool operator() (Entry_P entry1, Entry_P entry2);
}AxisSort;

/*******************************************************************************
 * RStarTree
 ******************************************************************************/

typedef struct RStarTree
{
    static size_t pageSize; // Page size in bytes.
    static size_t maxChild; // M -- node capacity -- maximum number of entries
    static size_t minChild; // m -- minimum number of entries in a node

    RTreeNode* root;

    RStarTree();
    ~RStarTree();

    void print();

    /// Insertion
    void insertData(Entry_P entryPtr);

    private:
        /// Insertion
        void insert(Node_P childPtr, Entry_P entryPtr, size_t desiredLevel, size_t &overflowLevel); // with OverflowTreatment

        /// Insertion - ChooseSubtree
        Node_P chooseSubtree(Mbr &mbr, size_t desiredLevel);

        /// Insertion - OverflowTreatment - ReInsert
        void reInsert(RTreeNode &node, size_t &overflowLevel);

        /// Insertion - OverflowTreatment - Split
        void split(RTreeNode &node);
        size_t chooseSplitAxis(RTreeNode &node);
        double computeS(RTreeNode &node);
        size_t chooseSplitIndex(RTreeNode &node, size_t axis);
}RStarTree;

}


