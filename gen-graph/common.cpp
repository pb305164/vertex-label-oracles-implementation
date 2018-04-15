#include <GeographicLib/Geodesic.hpp>

#include "common.h"
#include "mpreal.h"

using mpfr::mpreal;

GeographicLib::Geodesic geodesic(EARTH_RADIUS, 1/INVERSE_FLATTENING);


// Calculate distance between given coordinates
mpreal calc_distance(mpreal lat1, mpreal lon1, mpreal lat2, mpreal lon2) {
    double dist;
    geodesic.Inverse(lat1.toDouble(), lon1.toDouble(), lat2.toDouble(), lon2.toDouble(), dist);
    return mpreal(dist);
}


// Return string representation of mpreal
std::string str(mpreal mpr) {
    return mpr.toString("%.10RNf");
}


// Round mpreal
mpreal mpreal_round(mpreal mpr) {
    if (mpr != MAX_DOUBLE && mpr != -MAX_DOUBLE) {
        mpreal ret = str(mpr).c_str();
        return ret;
    }
    return mpr;
}


// Check graph edges for errors
void check_edges(unordered_mapB<int, std::set<Edge> > &edges) {
    // Check edges are in both nodes
    for (auto &edge_set: edges) {
        for (Edge e: edge_set.second) {
            // Check no loops
            assert(e.source != e.dest);
            assert(e.source == edge_set.first);
            int tmp = e.source;
            e.source = e.dest;
            e.dest = tmp;
            assert(edges[e.source].find(e) != edges[e.source].end());
            assert(e.distance > 0);
        }
    }
}


// Check graph nodes for errors
void check_nodes(unordered_mapB<int, Node> &nodes) {
    std::set<int> labels;
    // check ids are continuous 0...size()-1
    for (int i=0; i<(int)nodes.size(); i++) {
        assert(nodes.find(i) != nodes.end());
        labels.insert(nodes[i].label);
    }

    // check lables are continuous
    for (int i=-1; i<(int)labels.size()-1; i++) {
        assert(labels.find(i) != labels.end());
    }

    // check no two different nodes have the same lat, lon
    std::set<Node> nodes_set;
    for (auto &node : nodes) {
        assert(nodes_set.find(node.second) == nodes_set.end());
        nodes_set.insert(node.second);
    }
}


// Check graph for errors
void check_graph(unordered_mapB<int, Node> &nodes, unordered_mapB<int, std::set<Edge> > &edges) {
    check_edges(edges);
    check_nodes(nodes);
}
