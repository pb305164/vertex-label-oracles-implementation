
#ifndef GEN_GRAPH_MAKE_PLANAR_H
#define GEN_GRAPH_MAKE_PLANAR_H

//void make_planar(unordered_map<int, Node> &nodes, unordered_map<int, std::set<Edge> > &edges);
void make_planar(unordered_map<int, Node> &nodes, unordered_map<int, std::set<Edge> > &edges, std::vector<std::pair<int, int> > &kuratowski);
//void hmm();

#endif //GEN_GRAPH_MAKE_PLANAR_H
