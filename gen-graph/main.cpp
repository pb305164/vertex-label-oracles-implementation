#include <iostream>
#include <vector>
#include <set>
#include <boost/graph/boyer_myrvold_planar_test.hpp>

#include "mpreal.h"
#include "common.h"
#include "parse_osm.h"
#include "make_planar.h"

double max_max_speed;

std::vector<std::string> HIGHWAY_FILTER = {
        "motorway",
        "motorway_link",
        "trunk",
        "trunk_link",
        "primary",
        "primary_link",
        "secondary",
        "secondary_link"
        "tertiary",
        "tertiary_link", // też trochę ważne
        "unclassified",
        "residential",
        "service",
        "living_street",
};

Graph get_boost_graph(unordered_map<int, Node> &nodes, unordered_map<int, std::set<Edge> > &edges) {
    Graph g(nodes.size());
    for (auto edge_set : edges) {
        for (auto edge : edge_set.second) {
            if (edge.source < edge.dest) boost::add_edge(edge.source, edge.dest, g);
        }
    }
    return g;
}

std::vector<std::pair<int, int> > get_kuratowski(unordered_map<int, Node> &nodes, unordered_map<int, std::set<Edge> > &edges) {
    kuratowski_edges_t kuratowski_edges;
    std::vector<std::pair<int, int> > kur;
    Graph g = get_boost_graph(nodes, edges);
    boost::property_map<Graph, boost::edge_index_t>::type e_index = boost::get(boost::edge_index, g);
    boost::graph_traits<Graph>::edge_iterator ei, ei_end;
    int edge_count = 0;
    for(boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei) {
        put(e_index, *ei, edge_count++);
    }

    if (!boost::boyer_myrvold_planarity_test(boost::boyer_myrvold_params::graph = g, boost::boyer_myrvold_params::kuratowski_subgraph = std::back_inserter(kuratowski_edges))) {
        kuratowski_edges_t::iterator ki, ki_end;
        ki_end = kuratowski_edges.end();
        for (ki = kuratowski_edges.begin(); ki != ki_end; ++ki) {
            kur.push_back(std::pair<int, int>(boost::source(*ki, g), boost::target(*ki, g)));
        }
    }
    return kur;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage ./gen-graph <path_to_osm_file>\n");
        return 1;
    }


    mpfr::mpreal::set_default_prec(512);
    mpfr::mpreal max_max_speed = FOOT_SPEED;

    char *osm_path = argv[1];
    unordered_map <int, Node> nodes;
    unordered_map <int, std::set<Edge> > edges;
    std::vector<std::pair<int, int> > kur;

    std::tie(nodes, edges) = read_graph(osm_path, HIGHWAY_FILTER, max_max_speed, false, true);
    for (kur = get_kuratowski(nodes, edges); kur.size()>0; kur = get_kuratowski(nodes, edges)) {
        make_planar(nodes, edges, kur);
    }

    long unsigned int ecount = 0; //, ecount2 = 0;
    for (auto es : edges) {
        ecount += es.second.size();
//        for (Edge e : es.second) {
//            assert(e.source != e.dest);
//        }
    }
    assert(ecount%2 == 0);
    printf("%lu %lu %lf\n", nodes.size(), ecount/2, max_max_speed.toDouble());
    for (long unsigned i=0; i<nodes.size(); i++) {
        printf("%d %lf %lf\n", nodes[i].label+1, nodes[i].lat.toDouble(), nodes[i].lon.toDouble());
    }
    for (auto es : edges) {
        for(auto e : es.second) {
            if (e.source < e.dest) {
//                ecount2++;
                printf("%d %d %d %f %s\n", e.source, e.dest, e.type, e.max_speed, e.distance.toString("%.10RNf").c_str());
            }
        }
    }
//    assert(ecount/2 == ecount2);
    return 0;
}
