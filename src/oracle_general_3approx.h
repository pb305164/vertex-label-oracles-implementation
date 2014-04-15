#ifndef _ORACLE_GENERAL_3APPROX_H_
#define _ORACLE_GENERAL_3APPROX_H_

#include "oracle_general_approx.h"
#include <unordered_map>
#include <cassert>
#include <set>

using std::unordered_map;
using std::multiset;
using std::min;

class OracleGeneral3Approx : public OracleGeneralApprox {
private:    

    struct Label {
        unordered_map< int, set< pair<W, int> > > S_v;
        unordered_map< int, set< pair<W, pair<int, int> > > > P_l;
    };
    vector<Label> labels;


// Methods

    virtual
    void initializeStructures() {
        labels.resize(n);
        for (int v=0; v<n; ++v) {
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
                    label.P_l[l2].insert(make_pair(u.first, make_pair(u.second, v)));
                }
            }
        }
    }

    virtual
    void applyLabel(int v, int l) {
        vertices[v].label = l;
        
        for (auto &p: portals) {
            p.N_l[l].insert(make_pair(p.D_v[v], v));
        }
        
        for (pair<W, int> &curr: vertices[v].dist) {
            W du = curr.first;
            int u = curr.second;
            int ll = vertices[u].label;

            labels[l].S_v[u].insert(make_pair(du, v));
            labels[l].P_l[ll].insert(make_pair(du, make_pair(v, u)));
            if (v != u) labels[ll].P_l[l].insert(make_pair(du, make_pair(u, v)));
        }
    }

    virtual
    void purgeLabel(int v) {
        int l = vertices[v].label;
       
        for (auto &p: portals) {
            p.N_l[l].erase(make_pair(p.D_v[v], v));
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
    OracleGeneral3Approx(
            int n, 
            const vector< pair<int, int> > edges, 
            const vector<W> weights,
            int ro = -1)
    {
        Graph g(n, edges, weights);
        vector<int> labels;
        for (int i=0; i<n; ++i) labels.push_back(i);
        initializeWithLabels(g, labels, ro);
    }
    
    OracleGeneral3Approx(
            int n, 
            const vector< pair<int, int> > &edges, 
            const vector<W> &weights, 
            vector<int> labels,
            int ro = -1)
    {
        Graph g(n, edges, weights);
        initializeWithLabels(g, labels, ro);
    }

    virtual
    pair<W, int> distanceToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
        for (auto &p: portals) {
            auto it = p.N_l[l].begin();
            if (it == p.N_l[l].end()) continue;
            result = min(result, make_pair(p.D_v[v] + it->first, it->second));
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

// LIGHT

class OracleGeneral3ApproxLight : public OracleGeneralApproxLight {
private:    

    struct Label {
        unordered_map< int, multiset< W > > S_v;
        unordered_map< int, multiset< W > > P_l;
    };
    vector<Label> labels;


// Methods

    virtual
    void initializeStructures() {
        labels.resize(n);
        for (int v=0; v<n; ++v) {
            int l = vertices[v].label;
            for (auto curr: vertices[v].dist) {
                W du = curr.first;
                int u = curr.second;
                labels[l].S_v[u].insert(du);
            }
        }

        for (auto &label: labels) {
            for (auto &S: label.S_v) {
                int v = S.first;
                int l2 = vertices[v].label;
                for (auto &u: S.second) {
                    label.P_l[l2].insert(u);
                }
            }
        }
    }

    virtual
    void applyLabel(int v, int l) {
        vertices[v].label = l;
        
        for (auto &p: portals) {
            p.N_l[l].insert(p.D_v[v]);
        }
        
        for (pair<W, int> &curr: vertices[v].dist) {
            W du = curr.first;
            int u = curr.second;
            int ll = vertices[u].label;

            labels[l].S_v[u].insert(du);
            labels[l].P_l[ll].insert(du);
            if (v != u) labels[ll].P_l[l].insert(du);
        }
    }

    virtual
    void purgeLabel(int v) {
        int l = vertices[v].label;
       
        for (auto &p: portals) {
            p.N_l[l].erase(p.N_l[l].find(p.D_v[v]));
        }

        for (pair<W, int> &curr: vertices[v].dist) {
            W du = curr.first;
            int u = curr.second;
            int ll = vertices[u].label;
            
            auto it1 = labels[ll].P_l.find(l);
            it1->second.erase(it1->second.find(du));
            if (it1->second.empty()) {
                labels[ll].P_l.erase(it1);
            }

            if (v != u) {
                auto it2 = labels[l].P_l.find(ll);
                it2->second.erase(it2->second.find(du));
                if (it2->second.empty()) {
                    labels[l].P_l.erase(it2);
                }
            }

            auto it3 = labels[l].S_v.find(u);
            it3->second.erase(it3->second.find(du));
            if (it3->second.empty()) {
                labels[l].S_v.erase(it3);
            }
        }
    }

public:
    OracleGeneral3ApproxLight(
            int n, 
            const vector< pair<int, int> > edges, 
            const vector<W> weights,
            int ro = -1)
    {
        Graph g(n, edges, weights);
        vector<int> labels;
        for (int i=0; i<n; ++i) labels.push_back(i);
        initializeWithLabels(g, labels, ro);
    }
    
    OracleGeneral3ApproxLight(
            int n, 
            const vector< pair<int, int> > &edges, 
            const vector<W> &weights, 
            vector<int> labels,
            int ro = -1)
    {
        Graph g(n, edges, weights);
        initializeWithLabels(g, labels, ro);
    }

    virtual
    W distanceToLabel(int v, int l) {
        W result(infinity);
        for (auto &p: portals) {
            auto it = p.N_l[l].begin();
            if (it == p.N_l[l].end()) continue;
            result = min(result, p.D_v[v] + *it);
        }
        auto it = labels[l].S_v.find(v);
        if (it != labels[l].S_v.end()) {
            result = min(result, *it->second.begin());
        }
        return result;
    }

    virtual
    W distanceBetweenLabels(int l1, int l2) {
        W result(infinity);

        for (auto &p: portals) {
            if (p.N_l[l1].empty()) continue;
            if (p.N_l[l2].empty()) continue;
            auto v = *p.N_l[l1].begin();
            auto u = *p.N_l[l2].begin();
            result = min(result, v + u);
        }
        auto it = labels[l1].P_l.find(l2);
        if (it != labels[l1].P_l.end()) {
            result = min(result, *it->second.begin());
        }
        return result;
    }
};

#endif
