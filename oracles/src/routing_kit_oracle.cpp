#include "routing_kit_oracle.h"
#include <routingkit/osm_simple.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/inverse_vector.h>

using namespace RoutingKit;
using namespace std;

RoutingKitOracle::RoutingKitOracle(char *pbf_file, std::vector<int> &_labels): labels(_labels) {
    for (int i=0; i < (int)labels.size(); i++) {
        if (labels[i] > 0) {
            lbl_to_ver[labels[i]].insert(i);
        }
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
}


float RoutingKitOracle::distanceToVertex(int s, int t) {
    ContractionHierarchyQuery ch_query(ch);
    ch_query.reset().add_source(s).add_target(t).run();
    return (float)((float)ch_query.get_distance()/1000.0);
}


pair<float, int> RoutingKitOracle::distanceToLabel(int s, int l) {
    ContractionHierarchyQuery ch_query(ch);
    if (lbl_to_ver[l].size() > 0) {
        ch_query.reset();
        ch_query.add_source(s);
        for (auto &v: lbl_to_ver[l]) {
            ch_query.add_target(v);
        }
        ch_query.run();
        return make_pair((float)((float)ch_query.get_distance()/1000.0), (int)ch_query.get_used_target());
    }
    return make_pair(-1, -1);
}


pair<float, pair<int, int> > RoutingKitOracle::distanceBetweenLabels(int l1, int l2) {
    ContractionHierarchyQuery ch_query(ch);
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

void RoutingKitOracle::setLabel(int v, int l) {
    lbl_to_ver[labels[v]].erase(v);
    labels[v] = l;
    lbl_to_ver[l].insert(v);
}

int RoutingKitOracle::labelOf(int v) {
    return labels[v];
}