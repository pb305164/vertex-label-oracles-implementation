#ifndef GEN_GRAPH_COMMON_H
#define GEN_GRAPH_COMMON_H

#include <vector>

#include <boost/unordered_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include "mpreal.h"

#define DEBUG 0

#define printfd(...) \
            do { if (DEBUG>0) fprintf(stderr, ##__VA_ARGS__); } while (0)
#define printfdd(...) \
            do { if (DEBUG>1) fprintf(stderr, ##__VA_ARGS__); } while (0)
#define printfddd(...) \
            do { if (DEBUG>2) fprintf(stderr, ##__VA_ARGS__); } while (0)

#define FOOT_SPEED 5 // Speed of walking on foot, used as default speed and to define max speed of edges to nodes with labels
#define EARTH_RADIUS 6378137.0
#define INVERSE_FLATTENING 298.257223563


template <typename K, typename V> using unordered_mapB = boost::unordered::unordered_map<K, V>;
typedef unsigned long long int llu;

typedef boost::property<boost::vertex_index_t, int, boost::property<boost::vertex_index1_t, int> > VertexProperty;

typedef boost::adjacency_list<
        boost::vecS,
        boost::vecS,
        boost::undirectedS,
        VertexProperty,
        boost::property<boost::edge_index_t, int>
> Graph;

typedef std::vector<boost::graph_traits<Graph>::edge_descriptor > kuratowski_edges_t;

typedef std::vector<std::vector<boost::graph_traits<Graph>::edge_descriptor> > planar_embedding_storage_t;

typedef boost::iterator_property_map< planar_embedding_storage_t::iterator,
        boost::property_map<Graph, boost::vertex_index_t>::type>
        planar_embedding_t;

const mpfr::mpreal MAX_DOUBLE = std::numeric_limits<mpfr::mpreal>::max();

mpfr::mpreal calc_distance(mpfr::mpreal lat1, mpfr::mpreal lon1, mpfr::mpreal lat2, mpfr::mpreal lon2);

mpfr::mpreal mpreal_round(mpfr::mpreal mpr);
std::string str(mpfr::mpreal mpr);


struct Node {
    mpfr::mpreal lat;
    mpfr::mpreal lon;
    int id;
    int label;
    long long int osm_id;

    Node() {id = -1;}

    Node(mpfr::mpreal _lat, mpfr::mpreal _lon, int _id, int _label, long long int _osm_id):
            lat(mpreal_round(_lat)), lon(mpreal_round(_lon)), id(_id), label(_label), osm_id(_osm_id) {}

    Node(mpfr::mpreal _lat, mpfr::mpreal _lon):
            lat(mpreal_round(_lat)), lon(mpreal_round(_lon)), id(-1),  label(-1), osm_id(-1) {}

    // Nodes are compared by coordinates
    bool operator <(const Node &n) const {
        if (lat != n.lat) return lat < n.lat;
        return lon < n.lon;
    }
};

struct Edge {
    int source, dest;
    mpfr::mpreal max_speed;
    char type;
    mpfr::mpreal distance;

    Edge(int _source, int _dest):
            source(_source), dest(_dest), max_speed(-1), type(-1), distance(-1) {}

    Edge(int _source, int _dest, mpfr::mpreal _max_speed, char _type):
            source(_source), dest(_dest), max_speed(_max_speed), type(_type), distance(-1) {}

    Edge(int _source, int _dest, mpfr::mpreal _max_speed, char _type, mpfr::mpreal _distance):
            source(_source), dest(_dest), max_speed(_max_speed), type(_type), distance(_distance) {}


    bool operator<(const Edge &e) const {
        if (source != e.source) return source < e.source;
        return dest < e.dest;
    }

    bool operator==(const Edge &e) const {
        return source == e.source && dest == e.dest;
    }
};

void check_edges(unordered_mapB<int, std::set<Edge> > &edges);
void check_nodes(unordered_mapB<int, Node> &nodes);
void check_graph(unordered_mapB<int, Node> &nodes, unordered_mapB<int, std::set<Edge> > &edges);


#endif
