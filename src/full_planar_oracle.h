#ifndef _FULL_PLANAR_ORACLE_H_
#define _FULL_PLANAR_ORACLE_H_

#include "planar_oracle.h"
#include "find_union.h"

#include <iostream>
using namespace std;

class FullPlanarOracle : public PlanarOracle {
    
    struct Vertex {
        int label;
        vector< pair<W, int> > portals, rportals;
        vector< pair<W, int> > rdist;
    };
    vector< Vertex > vertices;
    
    struct Label {
        unordered_map< int, set< pair<W, int> > > S_v;
    }; 
    vector< Label > labels;

    struct Portal {
        unordered_map<int, set< pair<W, int> > > N_l;
        Portal() {}
    };
    vector< Portal > portals;
    
    virtual
    void processLeaf(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<bool>& source) {
  
        vector<W> distances;
        for (int v=0; v<(int)pg.vs().size(); ++v) {
            int vv = mapping[v];
            if (vv == -1) continue;
            if (!source[v]) continue;

            getDistances(pg, v, distances);

            for (int u=0; u<(int)pg.vs().size(); ++u) {
                int uu = mapping[u];
                if (uu == -1) continue;
                if (distances[u] == infinity) continue;
                vertices[uu].rdist.push_back(make_pair(distances[u], vv));
            }
        }
    }

    virtual
    void processPortals(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<int>& newPortals,
            const vector<bool>& source) {
    
        vector<W> distances;
        for (int p: newPortals) {
            getDistances(pg, p, distances);
            portals.push_back(Portal());
            for (int j=0; j<(int)mapping.size(); ++j) {
                int v = mapping[j];
                if (v == -1) continue;
                if (distances[j] == infinity) continue;

                portals.back().N_l[ vertices[v].label ].insert(make_pair(distances[j], v));
		vertices[v].rportals.push_back(make_pair(distances[j], portals.size()-1));
                if (source[j]) {
                    vertices[v].portals.push_back(make_pair(distances[j], portals.size()-1));
                }
            }
        }
    }

    virtual
    void initializeStructures() {
        for (auto &V: vertices) {
            sort(V.rdist.begin(), V.rdist.end());
            auto it = unique(V.rdist.begin(), V.rdist.end());
            V.rdist.resize(it - V.rdist.begin());
        }

        for (int v=0; v<(int)vertices.size(); ++v) {
            int l = vertices[v].label;
            for (auto &curr: vertices[v].rdist) {
                W du = curr.first;
                int u = curr.second;
                labels[l].S_v[u].insert(make_pair(du,v));
            }
        }
    }

    virtual
    void applyLabel(int v, int l) {
        vertices[v].label = l;
        for (auto &p: vertices[v].rportals) {
            portals[p.second].N_l[l].insert(make_pair(p.first, v));
        }

        for (pair<W, int> &curr: vertices[v].rdist) {
            W du = curr.first;
            int u = curr.second;
            labels[l].S_v[u].insert(make_pair(du,v));
        }
    }

    virtual
    void purgeLabel(int v) {
        int l = vertices[v].label;

        for (auto &p: vertices[v].rportals) {
            portals[p.second].N_l[l].erase(make_pair(p.first, v));
        }

        for (pair<W, int> &curr: vertices[v].rdist) {
            W du = curr.first;
            int u = curr.second;
            auto it3 = labels[l].S_v.find(u);
            it3->second.erase(make_pair(du, v));
            if (it3->second.empty()) {
                labels[l].S_v.erase(it3);
            }
        }
    }

public:
    FullPlanarOracle(
            int n,
            const vector< pair< int, int > >& edges, 
            const vector< W >& weights,
            const vector< int > llabels,
            W eps = .5) : labels(n) {
        ro = min(n, 3);
        vertices = vector<Vertex>(n);
        for (int i=0; i<n; ++i) vertices[i].label = llabels[i];
        initialize(n, edges, weights, eps);
        initializeStructures();

	long long sum = 0;
        for (auto &v: vertices) {
		sum += (int)v.portals.size();
        }
	cerr << sum << " / " << (int)portals.size() << " = " << (float)sum/portals.size() << endl;
	cerr << sum << " / " << (int)vertices.size() << " = " << (float)sum/vertices.size() << endl;
    }

    virtual
    int labelOf(int v) {
        return vertices[v].label;
    }

    virtual
    void setLabel(int v, int l) {
        purgeLabel(v);
        applyLabel(v, l);
    }

    virtual
    pair<W, int> distanceToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
        for (auto &p: vertices[v].portals) {
            auto it = portals[p.second].N_l[l].begin();
            if (it == portals[p.second].N_l[l].end()) continue;
            result = min(result, make_pair(p.first + it->first, it->second));
        }
        auto it = labels[l].S_v.find(v);
        if (it != labels[l].S_v.end()) {
            result = min(result, *it->second.begin());
        }
        return result;
    }
};

class FullPlanarOracle2 : public FullPlanarOracle {
public:
    FullPlanarOracle2(
            int n,
            const vector< pair< int, int > >& edges, 
            const vector< W >& weights,
            const vector< int > llabels,
            W eps = 1.) : FullPlanarOracle(n, edges, weights, llabels, eps) {}
};

#endif
