#ifndef GEN_GRAPH_PARSE_OSM_H
#define GEN_GRAPH_PARSE_OSM_H

void remap_ids(unordered_mapB<int, Node> &nodes, unordered_mapB<int, std::set<Edge> > &edges);
std::tuple<unordered_mapB<int, Node>, unordered_mapB<int, std::set<Edge>>, mpfr::mpreal> read_graph(
        char* file_name,
        bool bmerge_edges=true,
        bool badd_labels=true
);

#endif //GEN_GRAPH_PARSE_OSM_H
