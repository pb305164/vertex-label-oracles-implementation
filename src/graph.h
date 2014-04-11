#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "precision.h"
#include <utility>
#include <vector>

using std::vector;
using std::pair;

struct Graph { 
    struct Edge {
        int v;
        W w;
        
        Edge(int v, W w) : v(v), w(w) {}
    };

    int n;
    vector< vector<Edge> > edges;

    Graph() : Graph(0, vector< pair<int, int> >(), vector<W>()) {};
    Graph(int n, vector< pair<int, int> > eedges, vector<W> weights) :
            n(n), edges(n) {
        for (int i=0; i<(int)eedges.size(); ++i) {
            int u = eedges[i].first, v = eedges[i].second;
            W w = weights[i];
            edges[u].push_back(Edge(v,w));
            edges[v].push_back(Edge(u,w));
        }        
    }
};

#endif
