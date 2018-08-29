#ifndef VERTEX_LABEL_ORACLES_LCATREE_H
#define VERTEX_LABEL_ORACLES_LCATREE_H

#include <vector>
#include "precision.h"
#include "hierarchy_oracle_base.h" // Edge struct

class LCATree {
private:
    std::vector<W> distance;
    std::vector<int> e, l, r;
    std::vector<std::vector<int>> st;
public:
    LCATree(int _size, int _root, std::vector<std::vector<Edge>> &edges);
    W query(int s, int t);
};


#endif //VERTEX_LABEL_ORACLES_LCATREE_H
