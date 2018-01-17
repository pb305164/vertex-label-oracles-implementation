#ifndef GEN_GRAPH_KDTREE_H
#define GEN_GRAPH_KDTREE_H
#include <vector>
#include <tuple>

#include "common.h"
#include "mpreal.h"

class KDTree {
private:
    KDTree(std::vector<Node> &vnodes, int lvl);
    void find_nearest(mpfr::mpreal lat, mpfr::mpreal lon, mpfr::mpreal &best_dist, Node &best_node);
    void print(int indent);

public:
    int level;
    Node node;
    KDTree *LTree, *RTree;

    static KDTree create(std::vector<Node> &vnodes);
    KDTree(Node n);
    ~KDTree();
    void print();
    std::pair<Node, mpfr::mpreal> find_nearest(mpfr::mpreal lat, mpfr::mpreal lon);
};


#endif //GEN_GRAPH_KDTREE_H
