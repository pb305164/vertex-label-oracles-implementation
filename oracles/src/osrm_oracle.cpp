//
// Created by banan on 28/09/17.
//

#include "osrm_oracle.h"
#include "osrm/status.hpp"
#include "osrm/json_container.hpp"
#include "osrm/coordinate.hpp"
#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include <exception>
#include <utility>

using std::make_pair;
using namespace osrm;


EngineConfig prepareConfig(char *osrm_file, EngineConfig &config) {
    config.storage_config = {osrm_file};
    config.use_shared_memory = false;

//     We support two routing speed up techniques:
//     - Contraction Hierarchies (CH): requires extract+contract pre-processing
//     - Multi-Level Dijkstra (MLD): requires extract+partition+customize pre-processing

    config.algorithm = EngineConfig::Algorithm::CH;
//    config.algorithm = EngineConfig::Algorithm::MLD;
    return config;
}


OsrmOracle::OsrmOracle(char *osrm_file, int max_label, std::vector<pair<float, float> > &_coords, std::vector<int> labels):
        osrm{(prepareConfig(osrm_file, config), config)},
        coords(_coords.size()),
        lbl_to_ver(max_label) {
    coords.resize(_coords.size());
    for (int i=0; i < (int)labels.size(); i++) {
        lbl_to_ver[labels[i]].push_back(i);
    }
    NearestParameters params;
    json::Object result;

    params.coordinates.resize(1);
    for (size_t i=0; i < _coords.size(); i++) {
        auto c = _coords[i];
        params.coordinates[0] = {FloatLongitude{c.second}, FloatLatitude{c.first}};
        const auto status = osrm.Nearest(params, result);
        assert(status == Status::Ok);
        auto loc = result.values["waypoints"].get<json::Array>().values.at(0).get<json::Object>().values["location"].get<json::Array>().values;
        coords[i]= make_pair((float)loc.at(1).get<json::Number>().value, (float)loc.at(0).get<json::Number>().value);
    }
}

float OsrmOracle::distanceToVertex(int s, int t) {
    RouteParameters params;
    params.coordinates.push_back({FloatLongitude{coords[s].second}, FloatLatitude{coords[s].first}});
    params.coordinates.push_back({FloatLongitude{coords[t].second}, FloatLatitude{coords[t].first}});
    json::Object result;
    const auto status = osrm.Route(params, result);
    if (status == Status::Ok) {
        auto &routes = result.values["routes"].get<json::Array>();
        auto &route = routes.values.at(0).get<json::Object>();
        // TODO sprawdzić czy czas jest dobrze liczony ?
        return (float)route.values["duration"].get<json::Number>().value/3.6;
    } else {
        return -1;
    }
}

pair<float, int> OsrmOracle::distanceToLabel(int s, int l) {
    TableParameters params;
    params.coordinates.push_back({FloatLongitude{coords[s].second}, FloatLatitude{coords[s].first}});
    for (int i: lbl_to_ver[l]) {
        params.coordinates.push_back({FloatLongitude{coords[i].second}, FloatLatitude{coords[i].first}});
    }
    params.sources.push_back(0);
    for (size_t i=1; i<=lbl_to_ver[l].size(); i++) {
        params.destinations.push_back(i);
    }
    json::Object result;
    assert(params.IsValid());
    const auto status = osrm.Table(params, result);
    if (status == Status::Ok) {
        int min_i = -1;
        float min_d = std::numeric_limits<float>::max();
        auto durations = result.values["durations"].get<json::Array>().values.at(0).get<json::Array>().values;
        for (size_t i=0; i < lbl_to_ver[l].size(); i++) {
            if (durations.at(i).is<json::Number>()) {
                float d = (float) durations.at(i).get<json::Number>().value;
                min_d = d;
                min_i = i;
            }
        }
        // TODO sprawdzić czy czas jest dobrze liczony ?
        if (min_i != -1) {
            return make_pair((float) (min_d / 3.6), min_i);
        }
    }
    return make_pair(-1, -1);
}

pair<float, pair<int, int> > OsrmOracle::distanceBetweenLabels(int l1, int l2) {
    TableParameters params;
    size_t k=0, source_count = lbl_to_ver[l1].size(), dest_count = lbl_to_ver[l2].size();
    for (int i: lbl_to_ver[l1]) {
        params.coordinates.push_back({FloatLongitude{coords[i].second}, FloatLatitude{coords[i].first}});
        params.sources.push_back(k);
        k++;
    }
    for (int i: lbl_to_ver[l2]) {
        params.coordinates.push_back({FloatLongitude{coords[i].second}, FloatLatitude{coords[i].first}});
        params.destinations.push_back(k);
        k++;
    }
    json::Object result;
    const auto status = osrm.Table(params, result);
    if (status == Status::Ok) {
        int min_s = -1;
        int min_t = -1;
        float min_d = std::numeric_limits<float>::max();
        auto full_array = result.values["durations"].get<json::Array>();
        for (unsigned long i=0; i < source_count; i++) {
            auto durations = full_array.values.at(i).get<json::Array>().values;
            for (unsigned long j=0; j < dest_count; j++) {
                if (durations.at(j).is<json::Number>()) {
                    float d = (float) durations.at(j).get<json::Number>().value;
                    min_d = d;
                    min_s = i;
                    min_t = j;
                }
            }
        }
        // TODO sprawdzić czy czas jest dobrze liczony ?
        if (min_s != -1) {
            return make_pair((float) (min_d / 3.6), make_pair(min_s, min_t));
        }
    }
    return make_pair(-1, make_pair(-1, -1));}

