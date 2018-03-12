
#ifndef GEN_GRAPH_MAKE_PLANAR_H
#define GEN_GRAPH_MAKE_PLANAR_H

void make_planar(
        unordered_mapB<int, Node> &nodes,
        unordered_mapB<int, std::set<Edge> > &edges,
        std::vector<std::pair<int, int> > &kuratowski
);

#endif //GEN_GRAPH_MAKE_PLANAR_H
