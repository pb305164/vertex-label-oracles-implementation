#ifndef _PLANAR_H_
#define _PLANAR_H_

#include "precision.h"

#include <vector>
#include <utility>
#include <limits>
#include <cassert>

using std::vector;
using std::pair;

class PlanarGraph {
public:
    struct Edge {
        Edge(int uu, int vv, W ww) : 
            u(uu), v(vv), w(ww), uNext(-1), vNext(-1) {
                assert(w != 0);    
        }
        Edge(int uu, int vv) : Edge(uu, vv, infinity) {}

        int u, v;
        W w;
        int uNext, vNext;
    };

    struct Vertex {
        Vertex() : edges() {}

        vector<int> edges;
    };

    vector<Vertex> vertices;
    vector<Edge> edges;

    PlanarGraph() {}
    PlanarGraph(int n) : vertices(n) {}
    PlanarGraph(int n, vector< pair<int, int> > edges, vector<W> weights) : vertices(n) {
        for (int i=0; i<(int)edges.size(); ++i) {
            add_edge(edges[i].first, edges[i].second, weights[i]);
        }
    }

    vector<Vertex>& vs() {
        return vertices;
    }

    const vector<Vertex>& vs() const {
        return vertices;
    }

    vector<Edge>& es() {
        return edges;
    }

    const vector<Edge>& es() const {
        return edges;
    }
    
    int add_edge(int u, int v, W w = infinity) {
        assert(u != v);
        /*
        for (auto &e: es()) {
            assert(u != e.u || v != e.v);
            assert(v != e.u || u != e.v);
        }
        */
        int res = es().size();
        es().push_back(Edge(u, v, w));
        vs()[u].edges.push_back(res);
        vs()[v].edges.push_back(res);
        return res;
    }

    int opp(int u, int e) const {
        if (es()[e].u == u) return es()[e].v;
        return es()[e].u;
    }

    int& eNext(int u, int e) {
        if (es()[e].u == u) return es()[e].uNext;
        return es()[e].vNext;
    }

    int eNext(int u, int e) const {
        if (es()[e].u == u) return es()[e].uNext;
        return es()[e].vNext;
    }
};

bool isPlanar(const PlanarGraph& g);
void embed(PlanarGraph& g);
void triangulate(PlanarGraph& g);

#endif
