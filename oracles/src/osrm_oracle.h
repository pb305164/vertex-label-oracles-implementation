#ifndef OSRM_OSRMORACLE_H
#define OSRM_OSRMORACLE_H



#include "osrm/engine_config.hpp"

#include "osrm/osrm.hpp"

#include <cstdio>
#include <utility>
#include <vector>
#include <experimental/propagate_const>
#include <memory>

using std::pair;
using std::vector;

class OsrmOracle {
private:
    osrm::EngineConfig config;
    const osrm::OSRM osrm;
    vector<pair<float, float> > coords;
    vector<vector<int>> lbl_to_ver;

public:
    OsrmOracle(char *osrm_file, int max_label, std::vector<pair<float, float> > &_coords, std::vector<int> labels);

    float distanceToVertex(int s, int t);
    pair<float, int> distanceToLabel(int s, int l);
    pair<float, pair<int, int> > distanceBetweenLabels(int l1, int l2);
};


#endif //OSRM_OSRMORACLE_H
