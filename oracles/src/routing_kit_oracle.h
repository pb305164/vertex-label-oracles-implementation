#ifndef VERTEX_LABEL_ORACLES_ROUTEKITORACLE_H
#define VERTEX_LABEL_ORACLES_ROUTEKITORACLE_H

#include <routingkit/contraction_hierarchy.h>
#include "precision.h"

#include <unordered_map>
#include <set>
#include <vector>

class RoutingKitOracle {

private:
    std::vector<int> labels;
    std::unordered_map<int, std::set<int>> lbl_to_ver;
    RoutingKit::ContractionHierarchy ch;

public:
    RoutingKitOracle(char *pbf_file, std::vector<int> &_labels);

    float distanceToVertex(int s, int t);
    std::pair<float, int> distanceToLabel(int s, int l);
    std::pair<float, std::pair<int, int> > distanceBetweenLabels(int l1, int l2);
    void setLabel(int v, int l);
    int labelOf(int v);
};


#endif //VERTEX_LABEL_ORACLES_ROUTEKITORACLE_H