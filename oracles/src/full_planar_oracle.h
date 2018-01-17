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
            auto it = portals[p.second].N_l.find(l);
            it->second.erase(make_pair(p.first, v));
            if (it->second.empty()) {
                portals[p.second].N_l.erase(it);
            }
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
            W eps = 1.) : labels(n) {
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

class FullFullPlanarOracle : public PlanarOracle {
    
    struct Vertex {
        int label;
        vector< pair<W, int> > portals;
        vector< pair<W, int> > dist;
    };
    vector< Vertex > vertices;
    
    struct Label {
        unordered_map< int, set< pair<W, int> > > S_v;
        unordered_map< int, set< pair<W, pair<int, int> > > > P_l;
    }; 
    vector< Label > labels;

    struct Portal {
        map<int, set< pair<W, int> > > N_l;
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

            getDistances(pg, v, distances);

            for (int u=0; u<(int)pg.vs().size(); ++u) {
                int uu = mapping[u];
                if (uu == -1) continue;
                if (distances[u] == infinity) continue;
                vertices[uu].dist.push_back(make_pair(distances[u], vv));
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
		vertices[v].portals.push_back(make_pair(distances[j], portals.size()-1));
            }
        }
    }

    virtual
    void initializeStructures() {
        for (auto &V: vertices) {
            sort(V.dist.begin(), V.dist.end());
            auto it = unique(V.dist.begin(), V.dist.end());
            V.dist.resize(it - V.dist.begin());
        }

        for (int v=0; v<(int)vertices.size(); ++v) {
            int l = vertices[v].label;
            for (auto &curr: vertices[v].dist) {
                W du = curr.first;
                int u = curr.second;
                int ll = vertices[u].label;
                labels[l].S_v[u].insert(make_pair(du,v));
                labels[l].P_l[ll].insert(make_pair(du, make_pair(v, u)));
                if (v != u) labels[ll].P_l[l].insert(make_pair(du, make_pair(u, v)));
            }
        }
    }

    virtual
    void applyLabel(int v, int l) {
        vertices[v].label = l;
        for (auto &p: vertices[v].portals) {
            portals[p.second].N_l[l].insert(make_pair(p.first, v));
        }

        for (pair<W, int> &curr: vertices[v].dist) {
            W du = curr.first;
            int u = curr.second;
            int ll = vertices[u].label;

            labels[l].S_v[u].insert(make_pair(du,v));
            labels[l].P_l[ll].insert(make_pair(du, make_pair(v, u)));
            if (v != u) labels[ll].P_l[l].insert(make_pair(du, make_pair(u, v)));
        }
    }

    virtual
    void purgeLabel(int v) {
        int l = vertices[v].label;

        for (auto &p: vertices[v].portals) {
            auto it = portals[p.second].N_l.find(l);
            it->second.erase(make_pair(p.first, v));
            if (it->second.empty()) {
                portals[p.second].N_l.erase(it);
            }
        }

        for (pair<W, int> &curr: vertices[v].dist) {
            W du = curr.first;
            int u = curr.second;
            int ll = vertices[u].label;
            
            auto it1 = labels[ll].P_l.find(l);
            it1->second.erase(make_pair(du, make_pair(u, v)));
            if (it1->second.empty()) {
                labels[ll].P_l.erase(it1);
            }

            if (v != u) {
                auto it2 = labels[l].P_l.find(ll);
                it2->second.erase(make_pair(du, make_pair(v, u)));
                if (it2->second.empty()) {
                    labels[l].P_l.erase(it2);
                }
            }
            
            auto it3 = labels[l].S_v.find(u);
            it3->second.erase(make_pair(du, v));
            if (it3->second.empty()) {
                labels[l].S_v.erase(it3);
            }
        }
    }

public:
    FullFullPlanarOracle(
            int n,
            const vector< pair< int, int > >& edges, 
            const vector< W >& weights,
            const vector< int > llabels,
            W eps = 1.) : labels(n) {
        ro = max(3, min(n, (int)sqrt(n)));
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
    pair<W, int> labelToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
        for (auto &p: vertices[v].portals) {
            auto itt = portals[p.second].N_l.find(l);
            if (itt == portals[p.second].N_l.end()) continue;
            auto it = itt->second.begin();
            if (it == itt->second.end()) continue;
            result = min(result, make_pair(p.first + it->first, it->second));
        }
        auto it = labels[l].S_v.find(v);
        if (it != labels[l].S_v.end()) {
            result = min(result, *it->second.begin());
        }
        return result;
    }

    virtual
    pair<W, pair<int, int> > distanceBetweenLabels(int l1, int l2) {
        pair<W, pair<int, int> > result(infinity, make_pair(-1, -1));
        for (auto &p: portals) {
            if (p.N_l.find(l1) == p.N_l.end()) continue;
            if (p.N_l[l1].empty()) continue;
            if (p.N_l.find(l2) == p.N_l.end()) continue;
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
};

#endif
