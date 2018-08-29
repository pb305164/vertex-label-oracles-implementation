#ifndef VERTEX_LABEL_ORACLES_LCAORACLE_H
#define VERTEX_LABEL_ORACLES_LCAORACLE_H

#include <vector>
#include <unordered_map>
#include <set>

#include "LCATree.h"

class LCAOracle {
private:
    std::vector<LCATree> shortest_paths;
    std::vector<int> labels;
    std::unordered_map<int, std::set<int>> lbl_to_ver;

public:
    LCAOracle(std::vector<std::pair<int, int>> &_edges, std::vector<W> &_weights, std::vector<char> &_types,
              std::vector<int> &_labels, std::vector<std::pair<double, double> > &_coords, int _number_of_trees = 24);

    float distanceToVertex(int s, int t);
    std::pair<float, int> distanceToLabel(int s, int l);
    std::pair<float, std::pair<int, int> > distanceBetweenLabels(int l1, int l2);
    void setLabel(int v, int l);
    int labelOf(int v);
};


#endif //VERTEX_LABEL_ORACLES_LCAORACLE_H
