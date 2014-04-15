#ifndef _FULL_PLANAR_ORACLE_H_
#define _FULL_PLANAR_ORACLE_H_

#include "planar_oracle.h"
#include "find_union.h"

class FullPlanarOracle : public PlanarOracle {
    
    struct Label {
        unordered_map< int, set< pair<W, int> > > S_v;
        unordered_map< int, set< pair<W, pair<int, int> > > > P_l; 
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
  
        printf("LEAF\n"); 
        for (auto i: mapping) printf("%d ", i);
        printf("\n");

        vector<W> distances;
        for (int v=0; v<(int)pg.vs().size(); ++v) {
            getDistances(pg, v, distances);
            int vv = mapping[v];
            if (vv == -1) continue;
            for (int u=0; u<(int)pg.vs().size(); ++u) {
                int uu = mapping[u];
                if (uu == -1) continue;
                vertices[vv].dist.push_back(make_pair(distances[u], uu));
            }
        }
    }

    virtual
    void processPortals(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<int>& newPortals,
            const vector<bool>& source) {
    
        printf("PORTALS:\n");
        for (int p: newPortals) printf("%d ", mapping[p]);
        printf("\n");
        for (int v: mapping) printf("%d ", v);
        printf("\n");

        vector<W> distances;
        for (int p: newPortals) {
            getDistances(pg, p, distances);
            portals.push_back(Portal());
            for (int j=0; j<(int)mapping.size(); ++j) {
                if (mapping[j] == -1) continue;
                int v = mapping[j];

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
                labels[l].S_v[u].insert(make_pair(du,v));
            }
        }

        for (auto &label: labels) {
            for (auto &S: label.S_v) {
                int v = S.first;
                int l2 = vertices[v].label;
                for (auto &u: S.second) {
                    label.P_l[l2].insert(make_pair(u.first, make_pair(u.second, v)));
                }
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
            portals[p.second].N_l[l].erase(make_pair(p.first, v));
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
    FullPlanarOracle(
            int n,
            const vector< pair< int, int > >& edges, 
            const vector< W >& weights,
            const vector< int > llabels,
            W eps) : labels(n) {
        ro = min(n, 3);
        vertices = vector<Vertex>(n);
        for (int i=0; i<n; ++i) vertices[i].label = llabels[i];
        initialize(n, edges, weights, eps);
        initializeStructures();

        for (auto &v: vertices) {
            printf("vertex\n");
            for (auto curr: v.dist) {
                printf("(%f %d) ", curr.first, curr.second);
            }
            printf("\n");
        }
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

    virtual
    pair<W, pair<int, int> > distanceBetweenLabels(int l1, int l2) {
        pair<W, pair<int, int> > result(infinity, make_pair(-1, -1));
/*
        for (auto &V: vertices) {
            printf("%d ", V.label);
        }
        printf("\n");

        for (auto &L: labels) {
            printf("Lablel\n");
            for (auto &S: L.S_v) {
                printf("S %d: ", S.first);
                for (auto curr: S.second) {
                    printf("(%.1f %d) ", curr.first, curr.second);
                }
                printf("\n");
            }
            for (auto &P: L.P_l) {
                printf("P %d: ", P.first);
                for (auto curr: P.second) {
                    printf("(%.1f (%d,%d)) ", curr.first, curr.second.first, curr.second.second);
                }
                printf("\n");
            }
        }

        for (auto &P: portals) {
            printf("Portal\n");
            for (auto &L: P.N_l) {
                printf("N %d: ", L.first);
                for (auto curr: L.second) {
                    printf("(%.1f %d) ", curr.first, curr.second);
                }
                printf("\n");
            }
        }
*/
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
};

#endif
