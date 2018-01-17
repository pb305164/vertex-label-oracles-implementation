#ifndef GEN_GRAPH_COMMON_H
#define GEN_GRAPH_COMMON_H

#include <vector>

#include <boost/unordered_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/timer/timer.hpp>
#include <chrono>
#include <ostream>

//class Timer {
//    typedef std::chrono::high_resolution_clock high_resolution_clock;
//    typedef std::chrono::milliseconds nanoseconds;
//public:
//    explicit Timer(bool run = false)
//    {
//        if (run)
//            Reset();
//    }
//    void Reset()
//    {
//        _start = high_resolution_clock::now();
//    }
//    nanoseconds Elapsed() const
//    {
//        return std::chrono::duration_cast<nanoseconds>(high_resolution_clock::now() - _start);
//    }
//    template <typename T, typename Traits>
//    friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const Timer& timer)
//    {
//        return out << timer.Elapsed().count();
//    }
//private:
//    high_resolution_clock::time_point _start;
//};


#include "mpreal.h"

#define DEBUG 0

#define printfd(...) \
            do { if (DEBUG>0) fprintf(stderr, ##__VA_ARGS__); } while (0)
#define printfdd(...) \
            do { if (DEBUG>1) fprintf(stderr, ##__VA_ARGS__); } while (0)
#define printfddd(...) \
            do { if (DEBUG>2) fprintf(stderr, ##__VA_ARGS__); } while (0)

#define FOOT_SPEED 5 // Prędkość poruszania się na nogach (tzn z najbliższego noda z drogi do noda z labelem)
#define EARTH_RADIUS 6378137.0
#define INVERSE_FLATTENING 298.257223563


mpfr::mpreal calc_distance(mpfr::mpreal lat1, mpfr::mpreal lon1, mpfr::mpreal lat2, mpfr::mpreal lon2);

mpfr::mpreal mpreal_round(mpfr::mpreal mpr);

typedef unsigned long long int llu;

template <typename K, typename V> using unordered_map = boost::unordered::unordered_map<K, V>;
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

std::string str(mpfr::mpreal mpr);

class Node {
public:
    int id;
    long long int osm_id;
    mpfr::mpreal lat;
    mpfr::mpreal lon;
    int label;

    Node() {id = -1;}
    Node(int _id, long long int _osm_id, mpfr::mpreal _lat, mpfr::mpreal _lon, int _label): id(_id), osm_id(_osm_id), lat(_lat), lon(_lon), label(_label) {}
    Node(int _id, long long int _osm_id, mpfr::mpreal _lat, mpfr::mpreal _lon): id(_id), osm_id(_osm_id), lat(_lat), lon(_lon), label(-1) {}
    Node(int _id, mpfr::mpreal _lat, mpfr::mpreal _lon, int _label): id(_id), osm_id(-1), lat(_lat), lon(_lon), label(_label) {}
//    Node(mpfr::mpreal _lat, mpfr::mpreal _lon): id(-1), osm_id(-1), lat(_lat), lon(_lon), label(-1) {}

    bool operator<(const Node &n) const {
        if (lat != n.lat) return lat < n.lat;
        return lon < n.lon;
    }
};

class Edge {
public:
    int source, dest;
    float max_speed;
    char type;
    mpfr::mpreal distance;

    Edge(int _source, int _dest): source(_source), dest(_dest), max_speed(-1), type(-1), distance(-1) {}
    Edge(int _source, int _dest, float _max_speed, char _type): source(_source), dest(_dest), max_speed(_max_speed), type(_type), distance(-1) {}
    Edge(int _source, int _dest, float _max_speed, char _type, mpfr::mpreal _distance): source(_source), dest(_dest), max_speed(_max_speed), type(_type), distance(_distance) {}

    bool operator<(const Edge& e) const {
        if (source != e.source) return source < e.source;
        return dest < e.dest;
    }

    bool operator==(const Edge e) const {
        return source == e.source && dest == e.dest;
    }
};

#endif
