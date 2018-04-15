#ifndef OSRM_OSRMORACLE_H
#define OSRM_OSRMORACLE_H

#include "osrm/engine_config.hpp"
#include "osrm/osrm.hpp"

#include <unordered_map>
#include <vector>
#include <set>


class OsrmOracle {
private:
    osrm::EngineConfig config;
    const osrm::OSRM osrm;
    std::vector<std::pair<float, float> > coords;
    std::vector<int> labels;
    std::unordered_map<int, std::set<int>> lbl_to_ver;

public:
    OsrmOracle(char *osrm_file, std::vector<std::pair<float, float> > &_coords, std::vector<int> &_labels);

    float distanceToVertex(int s, int t);
    std::pair<float, int> distanceToLabel(int s, int l);
    std::pair<float, std::pair<int, int> > distanceBetweenLabels(int l1, int l2);
    void setLabel(int v, int l);
    int labelOf(int v);
};


#endif //OSRM_OSRMORACLE_H
