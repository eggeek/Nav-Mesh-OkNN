#include "RStarTreeUtil.h"

namespace rstar {

void RStarTreeUtil::validate(RStarTree& tree)
{
    size_t cnt = 0;
    MinHeap heap;
    heap.push(MinHeapEntry(1, tree.root));
    while(!heap.isEmpty())
    {
        MinHeapEntry e = heap.pop();
        if(e.nodePtr)
        {
            //Mbr& mbr = e.nodePtr->mbrn;
            if(e.nodePtr->level)
            {
                Node_P_V& children = *e.nodePtr->children;
                for(size_t ic = 0; ic < children.size(); ic++)
                {
                    Node_P childPtr = children.at(ic);
                    heap.push(MinHeapEntry(1, childPtr));
                    //Mbr& mbrn = childPtr->mbrn;
                    //assert(mbr.coord[0][0] <= mbrn.coord[0][0] && mbr.coord[0][1] >= mbrn.coord[0][1]);
                    //assert(mbr.coord[1][0] <= mbrn.coord[1][0] && mbr.coord[1][1] >= mbrn.coord[1][1]);
                }
            }
            else
            {
                Entry_P_V& entries = *e.nodePtr->entries;
                for(size_t ie = 0; ie < entries.size(); ie++)
                {
                    //Entry_P entryPtr = entries.at(ie);
                    //Mbr& mbre = entryPtr->mbre;
                    //assert(mbr.coord[0][0] <= mbre.coord[0][0] && mbr.coord[0][1] >= mbre.coord[0][1]);
                    //assert(mbr.coord[1][0] <= mbre.coord[1][0] && mbr.coord[1][1] >= mbre.coord[1][1]);
                    cnt++;
                }
            }
        }
    }
    cout << cnt << " data points validated" << endl;
}

size_t RStarTreeUtil::rangeQuery2(RStarTree& tree, Point& center, double r, size_t k)
{
    MaxHeap heap;
    heap.push(MaxHeapEntry(0, tree.root));
    size_t cnt = 0;
    while(!heap.isEmpty())
    {
        MaxHeapEntry e = heap.pop();
        if(minDis2(center, e.nodePtr->mbrn) > r)
            cnt += e.nodePtr->aggregate;
        if(maxDis2(center, e.nodePtr->mbrn) > r)
        {
            if(e.nodePtr->level)
            {
                Node_P_V& children = *e.nodePtr->children;
                for(size_t ic = 0; ic < children.size(); ic++)
                {
                    Node_P childPtr = children.at(ic);
                    heap.push(MaxHeapEntry(0, childPtr));
                }
            }
            else
            {
                Entry_P_V& entries = *e.nodePtr->entries;
                for(size_t ie = 0; ie < entries.size(); ie++)
                {
                    Entry_P entryPtr = entries.at(ie);
                    if(dis2(center, entryPtr->mbre) > r)
                    {
                        cnt++;
                        if(cnt >= k)
                            return cnt;
                    }
                }
            }
        }
    }
    return cnt;
}

size_t RStarTreeUtil::rangeQuery2SmallTree(RStarTree& tree, Point& center, double r, size_t k)
{
    MaxHeap heap;
    heap.push(MaxHeapEntry(0, tree.root));
    size_t cnt = 0;
    while(!heap.isEmpty())
    {
        MaxHeapEntry e = heap.pop();
        if(maxDis2(center, e.nodePtr->mbrn) > r)
        {
            if(e.nodePtr->level)
            {
                Node_P_V& children = *e.nodePtr->children;
                for(size_t ic = 0; ic < children.size(); ic++)
                {
                    Node_P childPtr = children.at(ic);
                    heap.push(MaxHeapEntry(0, childPtr));
                }
            }
            else
            {
                Entry_P_V& entries = *e.nodePtr->entries;
                for(size_t ie = 0; ie < entries.size(); ie++)
                {
                    Entry_P entryPtr = entries.at(ie);
                    if(dis2(center, entryPtr->mbre) > r)
                    {
                        cnt++;
                        if(cnt >= k)
                            return cnt;
                    }
                }
            }
        }
    }
    return cnt;
}

bool RStarTreeUtil::find(RStarTree& tree, Coord point[DIM])
{
    Node_P_V queue;
    queue.push_back(tree.root);
    while(queue.size() > 0)
    {
        Node_P nodePtr = queue.back();
        queue.pop_back();
        Mbr& mbrn = nodePtr->mbrn;

        bool contain = true;
        for(size_t dim = 0; dim < DIM; dim++)
            if(mbrn.coord[dim][0] > point[dim] || mbrn.coord[dim][1] < point[dim])
            {
                contain = false;
                break;
            }

        if(!contain)
            continue;

        if(nodePtr->level)
        {
            Node_P_V& children = *nodePtr->children;
            for(size_t ic = 0; ic < children.size(); ic++)
                queue.push_back(children.at(ic));
        }
        else
        {
            Entry_P_V& entries = *nodePtr->entries;
            for(size_t ie = 0; ie < entries.size(); ie++)
            {
                Mbr& mbre = entries.at(ie)->mbre;

                bool identical = true;
                for(size_t dim = 0; dim < DIM; dim++)
                    if(mbre.coord[dim][0] != point[dim])
                    {
                        identical = false;
                        break;
                    }

                if(identical)
                    return true;
            }
        }
    }
    return false;
}

}


