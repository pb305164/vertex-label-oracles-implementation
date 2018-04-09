#include <vector>
#include <set>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include <cstdlib>

#include "mpreal.h"
#include "common.h"
#include "parse_osm.h"
#include "make_planar.h"


using namespace std;
using mpfr::mpreal;

Graph get_boost_graph(unordered_mapB<int, Node> &nodes, unordered_mapB<int, set<Edge> > &edges) {
    Graph g(nodes.size());
    for (auto edge_set : edges) {
        for (auto edge : edge_set.second) {
            if (edge.source < edge.dest) boost::add_edge(edge.source, edge.dest, g);
        }
    }
    return g;
}

vector<pair<int, int> > get_kuratowski(unordered_mapB<int, Node> &nodes, unordered_mapB<int, set<Edge> > &edges) {
    kuratowski_edges_t kuratowski_edges;
    vector<pair<int, int> > kur;
    Graph g = get_boost_graph(nodes, edges);
    boost::property_map<Graph, boost::edge_index_t>::type e_index = boost::get(boost::edge_index, g);
    boost::graph_traits<Graph>::edge_iterator ei, ei_end;
    int edge_count = 0;
    for(boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei) {
        put(e_index, *ei, edge_count++);
    }

    if (!boost::boyer_myrvold_planarity_test(
            boost::boyer_myrvold_params::graph = g,
            boost::boyer_myrvold_params::kuratowski_subgraph = back_inserter(kuratowski_edges)))
    {
        kuratowski_edges_t::iterator ki, ki_end;
        ki_end = kuratowski_edges.end();
        for (ki = kuratowski_edges.begin(); ki != ki_end; ++ki) {
            kur.emplace_back(boost::source(*ki, g), boost::target(*ki, g));
        }
    }
    return kur;
}


// Remove edges with travel time lesser than min_tavel_time, merge incidental nodes
void merge_nodes(unordered_mapB<int, Node> &nodes, unordered_mapB<int, set<Edge>> &edges, mpreal min_travel_time) {
    int max_id = (int)nodes.size();
    for (int i = 0; i < max_id; i++) {
        if (edges.find(i) != edges.end()) {
            for (auto it = edges[i].begin(); it != edges[i].end(); it++) {
                Edge e = *it;
                if (e.distance / e.max_speed < min_travel_time) {
                    vector<Edge> vedges;

                    // Remove all edges from source
                    for (auto itt = edges[e.source].begin(); itt != edges[e.source].end(); itt = edges[e.source].erase(itt)) {
                        Edge edge = *itt;
                        // Save all other edges then edge currently beeing removed to add them back
                        if (edge.dest != e.dest) {
                            vedges.push_back(edge);
                            swap(edge.source, edge.dest);
                            edges[edge.source].erase(edge);
                        }
                    }

                    // Remove all edges from dest
                    for (auto itt = edges[e.dest].begin(); itt != edges[e.dest].end(); itt = edges[e.dest].erase(itt)) {
                        Edge edge = *itt;
                        if (edge.dest != e.source) {
                            vedges.push_back(edge);
                            swap(edge.source, edge.dest);
                            edges[edge.source].erase(edge);
                        }
                    }

                    // Select node to remove, tying not to remove nodes with label
                    int new_node_id;
                    if (nodes[e.dest].label != -1) {
                        // If current node (e.source == i) is removed move to next
                        new_node_id = e.dest;
                        nodes.erase(e.source);
                        edges.erase(e.source);
                    } else {
                        // If not current node is removed will need to come back to the same node to check rest of edges
                        i--;
                        new_node_id = e.source;
                        edges.erase(e.dest);
                        nodes.erase(e.dest);
                    }

                    // Sort edges by travel time
                    auto sort_cmp = [](const Edge &e1, const Edge &e2) { return e1.distance / e1.max_speed < e2.distance / e2.max_speed; };
                    sort(vedges.begin(), vedges.end(), sort_cmp);

                    // Add edges back
                    for (Edge edge: vedges) {
                        // All readded edges need to incorporate half of removed edge
                        e.distance /= 2;
                        // Calculate new max_speed
                        edge.max_speed = (edge.distance + e.distance) /
                                         ((edge.distance / edge.max_speed) + (e.distance / e.max_speed));
                        edge.distance += e.distance;

                        // Ignore if edge already exists, shorter travel time will be added first
                        if (edge.source != new_node_id) {
                            edge.source = new_node_id;
                        }
                        edges[edge.source].insert(edge);
                        swap(edge.source, edge.dest);
                        edges[edge.source].insert(edge);
                    }

                    // After altering edges need to start again or move to next node, also iterator it becomes invalid
                    break;
                }
            }
        }
    }
}

bool merge_edges = false, add_labels = true, planarize = true;
mpreal min_travel_time = 0;
char *osm_path = nullptr;

void print_help() {
    fprintf(stderr,
            "Usage: ./gen-graph [options] path_to_osm_file [min dist]\n\n"
            "Options:\n"
            "  -h             : Print this help message\n\n"
            "  -me            : Merge edges by removing nodes with degree = 2\n"
            "                   This method desn't remove nodes with assigned labels,\n"
            "                   and doesn't create self loops. Be default edges are not merged.\n\n"
            "  -mn min_dist   : Merge nodes that are closer to each other then distance. By default nodes are not merged.\n"
            "                   To keep compatibility with previous verison min_dist can be also given as last argument"
            "  -P             : DON'T planarazie graph. By default graph is planarized\n\n"
            "  -L             : DON'T add labeled nodes to graph. By default labeled nodes are added to graph\n\n"
    );

}

void parse_program_options(int argc, char* argv[]) {
    for (int i=1; i<argc;) {
        if (strcmp("-h", argv[i]) == 0) {
            print_help();
            exit(0);
        } else if (strcmp("-me", argv[i]) == 0) {
            merge_edges = true;
        } else if (strcmp("-mn", argv[i]) == 0) {
            i++;
            float m=-1;
            if (sscanf(argv[i], "%f", &m) && m >= 0) {
                min_travel_time = m;
            } else {
                fprintf(stderr, "Error while parsing mn option, no distance specified or invalid value\n\n");
                print_help();
                exit(1);
            }
        } else if (strcmp("-P", argv[i]) == 0) {
            planarize = false;
        } else if (strcmp("-L", argv[i]) == 0) {
            add_labels = false;
        } else {
            if (osm_path == nullptr) {
                osm_path = argv[i];
            } else {
                if (min_travel_time == 0) {
                    // Legacy mode
                    float m=-1;
                    if (sscanf(argv[i], "%f", &m) && m >= 0) {
                        min_travel_time = m;
                    } else {
                        fprintf(stderr, "Error while parsing arguments %d\n\n", i);
                        print_help();
                        exit(1);
                    }
                } else {
                    fprintf(stderr, "Error while parsing arguments %d\n", i);
                    exit(1);
                }
            }
        }
        i++;
    }
    if (osm_path == nullptr) {
        fprintf(stderr, "Error no path to osm file specified\n\n");
        print_help();
        exit(1);
    }
}


int main(int argc, char* argv[]) {

    parse_program_options(argc, argv);

    mpfr::mpreal::set_default_prec(512);
    mpfr::mpreal::set_default_rnd(MPFR_RNDN);
    mpreal max_max_speed = FOOT_SPEED;

    unordered_mapB<int, Node> nodes;
    unordered_mapB<int, set<Edge> > edges;
    vector<pair<int, int> > kur;

    // Read graph, merge and add nodes with labels as requested
    std::tie(nodes, edges, max_max_speed) = read_graph(osm_path, merge_edges, add_labels);
//    check_graph(nodes, edges);

    // Planarize it
    if (planarize) {
        for (kur = get_kuratowski(nodes, edges); !kur.empty(); kur = get_kuratowski(nodes, edges)) {
            make_planar(nodes, edges, kur);
            // check_graph(nodes, edges);
        }
    }
//    check_graph(nodes, edges);

    // Merge close nodes, remove very short edges
    if (min_travel_time > 0) {
        merge_nodes(nodes, edges, min_travel_time);
        remap_ids(nodes, edges);
    }
//    check_graph(nodes, edges);

    // Print graph
    int edge_count = 0;
    for (auto es : edges) {
        edge_count += es.second.size();
    }
    printf("%lu %d %lf\n", nodes.size(), edge_count/2, max_max_speed.toDouble());
    for (long unsigned i=0; i<nodes.size(); i++) {
        printf("%d %lf %lf\n", nodes[i].label+1, nodes[i].lat.toDouble(), nodes[i].lon.toDouble());
    }
    for (auto es : edges) {
        for(auto e : es.second) {
            if (e.source < e.dest) {
                printf("%d %d %d %f %s\n", e.source, e.dest, e.type, e.max_speed.toFloat(), e.distance.toString("%.10RNf").c_str());
            }
        }
    }
    return 0;
}
