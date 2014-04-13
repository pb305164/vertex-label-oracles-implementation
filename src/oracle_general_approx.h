#ifndef _ORACLE_GENERAL_APPROX_H_
#define _ORACLE_GENERAL_APPROX_H_

#include "precision.h"
#include "graph.h"

#include <vector>
#include <algorithm>
#include <unordered_map>
#include <queue>
#include <set>

using std::pair;
using std::make_pair;
using std::vector;
using std::set;
using std::unordered_map;
using std::random_shuffle;
using std::max;
using std::priority_queue;
using std::greater;

class OracleGeneralApprox {
protected:    

// Fields
    Graph g;

    vector<int> portalNumbers;
    vector<int> portalIndices;
    
    struct Portal {
        vector< W > D_v;
        vector< set< pair<W, int> > > N_l;
    };
    vector<Portal> portals;

    struct Vertex {
        int label; 
        int p;
        W pd;
        vector< pair<W, int> > dist;
    };
    vector<Vertex> vertices;

// Methods

    virtual
    void initializeWithLabels(const vector<int> &llabels, int ro) {
        vertices.resize(g.n);
        for (int v=0; v<g.n; ++v) {
            vertices[v].label = llabels[v];
        }

        if (ro == -1) ro = max(1, (int)sqrt(g.n));
        selectPortals(ro);
        portals.resize(ro);
        
        for (int i=0; i<(int)portalNumbers.size(); ++i)
            initializePortals(i);
        
        for (int v=0; v<g.n; ++v)
            initializeDistances(v);

        for (int v=0; v<g.n; ++v)
            crossPieces(v);

        initializeStructures();
    }

    virtual
    void selectPortals(int ro) {
        for (int i=0; i<g.n; ++i) {
            portalNumbers.push_back(i);
        }
        random_shuffle(portalNumbers.begin(), portalNumbers.end());
        portalNumbers.resize(ro);
        portalIndices = vector<int>(g.n, -1);
        for (int i=0; i<ro; ++i) {
            portalIndices[portalNumbers[i]] = i;
        }
    }

    virtual
    void initializePortals(int pi) {
        int p = portalNumbers[pi];
        
        typedef pair<W, int> QEl;
        priority_queue< QEl, vector<QEl>, greater<QEl> > queue;
        vector<W> dist(g.n, infinity);
        
        queue.push(make_pair(0, p));
        dist[p] = 0;

        while (!queue.empty()) {
            QEl curr = queue.top(); queue.pop();
            W ud = curr.first;
            int u = curr.second;
            if (ud != dist[u]) continue;

            for (int i=0; i<(int)g.edges[u].size(); ++i) {
                W wd = ud + g.edges[u][i].w;
                int w = g.edges[u][i].v;

                if (wd < dist[w]) {
                    dist[w] = wd;
                    queue.push(make_pair(wd,w));
                }
            }
        }

        portals[pi].N_l.resize(g.n);
        for (int v=0; v<g.n; ++v) {
            portals[pi].N_l[vertices[v].label].insert(make_pair(dist[v], v));
        }
        swap(portals[pi].D_v, dist);
    }
    
    virtual
    void initializeDistances(int v) {
        typedef pair<W, int> QEl;
        priority_queue< QEl, vector<QEl>, greater<QEl> > queue;
        vector<W> dist(g.n, infinity);

        queue.push(make_pair(0, v));
        dist[v] = 0;

        while (!queue.empty()) {
            QEl curr = queue.top(); queue.pop();
            W ud = curr.first;
            int u = curr.second;
            if (ud != dist[u]) continue;

            if (portalIndices[u] != -1) {
                vertices[v].pd = ud;
                vertices[v].p = portalIndices[u];
                break;
            }
            vertices[v].dist.push_back(curr);

            for (int i=0; i<(int)g.edges[u].size(); ++i) {
                W wd = ud + g.edges[u][i].w;
                int w = g.edges[u][i].v;

                if (wd < dist[w]) {
                    dist[w] = wd;
                    queue.push(make_pair(wd,w));
                }
            }
        }
    }

    virtual
    void crossPieces(int v) {
        vector< pair<W, int> > truncated;
        for (auto curr: vertices[v].dist) {
            
            if (make_pair(vertices[curr.second].pd, portalNumbers[vertices[curr.second].p])
                > make_pair(curr.first, v)) {
              truncated.push_back(curr);
            }
        }
        swap(vertices[v].dist, truncated);
    }

    virtual
    void initializeStructures() {
        printf("Base class called!\n");
    };

    virtual
    void applyLabel(int v, int l) = 0;

    virtual
    void purgeLabel(int v) = 0;

    OracleGeneralApprox() {}

public:
    OracleGeneralApprox(
            int n, 
            const vector< pair<int, int> > edges, 
            const vector<W> weights,
            int ro = -1) :
        g(n, edges, weights)
    {
        vector<int> labels;
        for (int i=0; i<n; ++i) labels.push_back(i);
        initializeWithLabels(labels, ro);
    }
    
    OracleGeneralApprox(
            int n, 
            const vector< pair<int, int> > &edges, 
            const vector<W> &weights, 
            vector<int> labels,
            int ro = -1) :
        g(n, edges, weights)
    {
        initializeWithLabels(labels, ro);
    }

    virtual
    ~OracleGeneralApprox() {}

    void setLabel(int v, int l) {
        purgeLabel(v);
        applyLabel(v, l);
    }

    int labelOf(int v) {
        return vertices[v].label;
    }
};

#endif
