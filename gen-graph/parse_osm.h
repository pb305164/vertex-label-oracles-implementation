#ifndef GEN_GRAPH_PARSE_OSM_H
#define GEN_GRAPH_PARSE_OSM_H

std::pair<unordered_map<int, Node>, unordered_map<int, std::set<Edge> > > read_graph(
        char* file_name,
        std::vector<std::string> &way_filter,
        mpfr::mpreal &max_max_speed,
        bool bmerge_edges=true,
        bool badd_labels=true
);

#endif //GEN_GRAPH_PARSE_OSM_H
