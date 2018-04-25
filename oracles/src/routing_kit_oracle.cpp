#include "routing_kit_oracle.h"
#include <routingkit/osm_simple.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/inverse_vector.h>

using namespace RoutingKit;
using namespace std;

RoutingKitOracle::RoutingKitOracle(char *pbf_file, std::vector<int> &_labels, vector<pair<int, int>> &_edges, vector<W> &weights): labels(_labels) {

    for (int i=0; i < (int)labels.size(); i++) {
        if (labels[i] > 0) {
            lbl_to_ver[labels[i]].insert(i);
        }
    }

    edges.resize(labels.size());
    for (int i = 0; i < (int)_edges.size(); i++) {
        int s, t;
        tie(s, t) = _edges[i];
        edges[s].push_back(make_pair(t, weights[i]));
        edges[t].push_back(make_pair(s, weights[i]));
    }

    // Load a car routing graph from OpenStreetMap-based data
    auto graph = simple_load_osm_car_routing_graph_from_pbf(pbf_file);
    auto tail = invert_inverse_vector(graph.first_out);

    // Build the shortest path index
    ch = ContractionHierarchy::build(
            graph.node_count(),
            tail, graph.head,
            graph.travel_time
    );

    ch_query = ContractionHierarchyQuery(ch);
}


float RoutingKitOracle::distanceToVertex(int s, int t) {
    ch_query.reset().add_source(s).add_target(t).run();
    vector<unsigned> path = ch_query.get_node_path();
    W got = verify_path(path);
    return got;
}


pair<float, int> RoutingKitOracle::distanceToLabel(int s, int l) {
    if (lbl_to_ver[l].size() > 0) {
        ch_query.reset();
        ch_query.add_source(s);
        for (auto &v: lbl_to_ver[l]) {
            ch_query.add_target(v);
        }
        ch_query.run();
        vector<unsigned> path = ch_query.get_node_path();
        W got = verify_path(path);
        return make_pair(got, (int)ch_query.get_used_target());
    }
    return make_pair(-1, -1);
}


pair<float, pair<int, int> > RoutingKitOracle::distanceBetweenLabels(int l1, int l2) {
    if (lbl_to_ver[l1].size() > 0 && lbl_to_ver[l2].size() > 0) {
        ch_query.reset();
        for (auto &v: lbl_to_ver[l1]) {
            ch_query.add_source(v);
        }
        for (auto &v: lbl_to_ver[l2]) {
            ch_query.add_target(v);
        }
        ch_query.run();
        return make_pair((float)((float)ch_query.get_distance()/1000.0), make_pair((int)ch_query.get_used_source(), (int)ch_query.get_used_target()));
    }
    return make_pair(-1, make_pair(-1, -1));
}

W RoutingKitOracle::verify_path(std::vector<unsigned> &path) {
    int prev = -1;
    W distance = 0;
    for (auto x: path) {
        if (prev != -1) {
            bool has = false;
            for (auto p: edges[prev]) {
                if (p.first == (int)x) {
                    has = true;
                    distance += p.second;
                }
            }
            assert(has);
        }
        prev = x;
    }
    return distance;
}

void RoutingKitOracle::setLabel(int v, int l) {
    lbl_to_ver[labels[v]].erase(v);
    labels[v] = l;
    lbl_to_ver[l].insert(v);
}

int RoutingKitOracle::labelOf(int v) {
    return labels[v];
}