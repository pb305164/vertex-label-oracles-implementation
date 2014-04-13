#ifndef _ORACLE_GENERAL_H_
#define _ORACLE_GENERAL_H_

#include "precision.h"
#include "graph.h"

#include <vector>
#include <algorithm>
#include <unordered_map>
#include <queue>
#include <set>
#include <map>

using std::pair;
using std::make_pair;
using std::vector;
using std::set;
using std::map;
using std::unordered_map;
using std::random_shuffle;
using std::max;
using std::priority_queue;
using std::greater;

#include <iostream>
using namespace std;

#define VAR(v,i) __typeof(i) v = (i)
#define FORE(i, c) for(VAR(i, (c).begin()); i != (c).end(); ++i)
#define debug(x) { cerr << #x <<" = " << (x) << endl; }
#define debugv(x) { cerr << #x << " = "; for(auto it: x) { cerr << it << ", "; } cerr << endl;  }

template<class T1, class T2> ostream& operator<<(ostream &out, pair<T1, T2> pair) { return out << "(" << pair.first << ", "<< pair.second << ")"; };

class OracleGeneral {
    
// Fields
    Graph g;

    vector<int> portalNumbers;
    vector<int> portalIndices;
    
    struct Portal {
        vector< W > D_v;
        vector< set< pair<W, int> > > N_l;
    };
    vector<Portal> portals;

    struct Label {
        unordered_map< int, set< pair<W, int> > > S_v;
        unordered_map< int, set< pair<W, pair<int, int> > > > P_l;
    };
    vector<Label> labels;

    struct Vertex {
        int label;
        
        pair<W, int> p;
        vector< pair<W, int> > dist;
    };
    vector<Vertex> vertices;

// Methods

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
        
        labels.resize(g.n);
        for (int v=0; v<g.n; ++v)
            initializeDistances(v);

        for (int v=0; v<g.n; ++v)
            crossPieces(v);

        initializeStructures();
    }

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
                vertices[v].p = make_pair(ud, portalIndices[u]);
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

    void crossPieces(int v) {
        vector< pair<W, int> > truncated;
        for (auto curr: vertices[v].dist) {
            if (vertices[curr.second].p > make_pair(curr.first, v)) {
                truncated.push_back(curr);
            }
        }
        swap(vertices[v].dist, truncated);
    }

    void initializeStructures() {
        for (int v=0; v<g.n; ++v) {
            int l = vertices[v].label;
            for (auto curr: vertices[v].dist) {
                W du = curr.first;
                int u = curr.second;
                labels[l].S_v[u].insert(make_pair(du, v));
            }
        }

        for (auto &label: labels) {
            for (auto &S: label.S_v) {
                int v = S.first;
                int l2 = vertices[v].label;
                for (auto &u: S.second) {
                    label.P_l[l2].insert(make_pair(u.first, make_pair(v, u.second)));
                }
            }
        }
    }

    void applyLabel(int v, int l) {
        vertices[v].label = l;
        
        for (int pi=0; pi<(int)portals.size(); ++pi) {
            portals[pi].N_l[l].insert(make_pair(portals[pi].D_v[v], v));
        }
        
        for (pair<W, int> &curr: vertices[v].dist) {
            W du = curr.first;
            int u = curr.second;
            int ll = vertices[u].label;

            auto it = labels[l].S_v[u].begin();
            labels[l].S_v[u].insert(make_pair(du, v));

            if (it != labels[l].S_v[u].begin()) {
                if (it != labels[l].S_v[u].end()) {
                    labels[l].P_l[ll].erase(make_pair(it->first, make_pair(u, it->second)));
                }
                labels[l].P_l[ll].insert(make_pair(du, make_pair(u, v)));
            }
            
            it = labels[ll].S_v[v].begin();
            labels[ll].P_l[l].insert(make_pair(it->first, make_pair(v, it->second)));
        }
    }

    void purgeLabel(int v) {
        int l = vertices[v].label;
        
        for (int pi=0; pi<(int)portals.size(); ++pi) {
            portals[pi].N_l[l].erase(make_pair(portals[pi].D_v[v], v));
        }

        for (pair<W, int> &curr: vertices[v].dist) {
            W du = curr.first;
            int u = curr.second;
            int ll = vertices[u].label;
            
            auto it = labels[ll].S_v[v].begin();
            labels[ll].P_l[l].erase(make_pair(it->first, make_pair(v, it->second)));
            if (labels[ll].P_l[l].empty()) labels[ll].P_l.erase(l);
            
            it = labels[l].S_v[u].begin();
            labels[l].S_v[u].erase(make_pair(du, v));

            if (it != labels[l].S_v[u].begin()) {
                labels[l].P_l[ll].erase(make_pair(du, make_pair(u, v)));

                it = labels[l].S_v[u].begin();
                if (it != labels[l].S_v[u].end()) {
                    labels[l].P_l[ll].insert(make_pair(it->first, make_pair(u, it->second)));
                }
            }

            if (labels[l].S_v[u].empty()) {
                labels[l].S_v.erase(u);
            }
            if (labels[l].P_l[ll].empty()) {
                labels[l].P_l.erase(ll);
            }
            
        }
    }

public:
    OracleGeneral(
            int n, 
            const vector< pair<int, int> > edges, 
            const vector<W> weights,
            int ro = -1) :
        g(n, edges, weights)
    {
        vector<int> labels;
        for(int i=0; i<n; ++i) labels.push_back(i);
        initializeWithLabels(labels, ro);
    }
    
    OracleGeneral(
            int n, 
            const vector< pair<int, int> > &edges, 
            const vector<W> &weights, 
            vector<int> labels,
            int ro = -1) :
        g(n, edges, weights)
    {
        initializeWithLabels(labels, ro);
    }

    void setLabel(int v, int l) {
        purgeLabel(v);
        applyLabel(v, l);
    }

    pair<W, int> distanceToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
        for (auto &curr: vertices[v].dist) {
            if (vertices[curr.second].label == l) {
                result = curr;
                break;
            }
        }
        pair<W, int> p = vertices[v].p;
        auto it = portals[p.second].N_l[l].begin();
        if (it != portals[p.second].N_l[l].end()) 
            result = min(result, make_pair(p.first + it->first, it->second));
        return result;
    }

    pair<W, pair<int, int> > distanceBetweenLabels(int l1, int l2) {
        pair<W, pair<int, int> > result(infinity, make_pair(-1, -1));

        for (auto &p: portals) {
            if (p.N_l[l1].empty()) continue;
            if (p.N_l[l2].empty()) continue;
            auto v = *p.N_l[l1].begin();
            auto u = *p.N_l[l2].begin();
            result = min(result, make_pair(v.first + u.first, make_pair(v.second, u.second)));
        }

        auto it = labels[l1].P_l.find(l2);
        if (it != labels[l1].P_l.end()) {
            result = min(result, *it->second.begin());
        }

        return result;
    }

    void print() {
        for (int i=0; i<(int)portals.size(); ++i) {
            cout << "portal " << portalNumbers[i] << endl;
            for (int j=0; j<(int)portals[i].N_l.size(); ++j) {
                cout << j << ": ";
                debugv(portals[i].N_l[j]);
            }
        }

        for (int i=0; i<(int)labels.size(); ++i) {
            cout << "label " << i << endl;
            for (auto S: labels[i].S_v) {
                debug(S.first);
                debugv(S.second);
            }
            for (auto P: labels[i].P_l) {
                debug(P.first);
                debugv(P.second);
            }
        }
        cout << endl;
    }
};

#endif
