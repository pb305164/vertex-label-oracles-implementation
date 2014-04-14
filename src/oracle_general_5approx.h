#ifndef _ORACLE_GENERAL_5APPROX_H_
#define _ORACLE_GENERAL_5APPROX_H_

#include "precision.h"

#include <map>

using std::map;

class OracleGeneral5ApproxQuery : public OracleGeneralApprox {
private:
// Fields

  struct Label {
    map< int, set< pair<W, int> > > S_v;
  };
  vector<Label> labels;

// Methods
    virtual
    void initializeStructures() {
        labels.resize(g.n);
        for (int v=0; v<g.n; ++v) {
            int l = vertices[v].label;
            for (auto curr: vertices[v].dist) {
                W du = curr.first;
                int u = curr.second;
                labels[l].S_v[u].insert(make_pair(du, v));
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
            labels[l].S_v[u].insert(make_pair(du, v));
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
            labels[l].S_v[u].erase(make_pair(du, v));

            if (labels[l].S_v[u].empty()) {
                labels[l].S_v.erase(u);
            }
            
        }
    }

public:
    OracleGeneral5ApproxQuery(
            int n, 
            const vector< pair<int, int> > edges, 
            const vector<W> weights,
            int ro = -1)
    {
        g = Graph(n, edges, weights);
        vector<int> labels;
        for (int i=0; i<n; ++i) labels.push_back(i);
        initializeWithLabels(labels, ro);
    }
    
    OracleGeneral5ApproxQuery(
            int n, 
            const vector< pair<int, int> > &edges, 
            const vector<W> &weights, 
            vector<int> labels,
            int ro = -1)
    {
        g = Graph(n, edges, weights);
        initializeWithLabels(labels, ro);
    }

    virtual
    pair<W, int> distanceToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
            
        int p = vertices[v].p;

        auto it1 = portals[p].N_l[l].begin();
        if (it1 != portals[p].N_l[l].end())
          result = min(result, make_pair(vertices[v].pd + it1->first, it1->second));
        
        auto it2 = labels[l].S_v.find(v);
        if (it2 != labels[l].S_v.end())
          result = min(result, *it2->second.begin());
        
        return result;
    }
};

class OracleGeneral5ApproxUpdate : public OracleGeneralApprox {
private:

// Methods
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
                portals[portalIndices[u]].N_l[vertices[v].label].insert(make_pair(ud, v));
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
    void initializeStructures() {}
    
    virtual
    void applyLabel(int v, int l) { 
        vertices[v].label = l;
       
        int p = vertices[v].p; 
        portals[p].N_l[l].insert(make_pair(vertices[v].pd, v));
    }

    virtual
    void purgeLabel(int v) {
        int l = vertices[v].label;
        int p = vertices[v].p; 
        portals[p].N_l[l].erase(make_pair(vertices[v].pd, v));
    }

public:
    OracleGeneral5ApproxUpdate(
            int n, 
            const vector< pair<int, int> > edges, 
            const vector<W> weights,
            int ro = -1)
    {
        g = Graph(n, edges, weights);
        vector<int> labels;
        for (int i=0; i<n; ++i) labels.push_back(i);
        initializeWithLabels(labels, ro);
    }
    
    OracleGeneral5ApproxUpdate(
            int n, 
            const vector< pair<int, int> > &edges, 
            const vector<W> &weights, 
            vector<int> labels,
            int ro = -1)
    {
        g = Graph(n, edges, weights);
        initializeWithLabels(labels, ro);
    }

    virtual
    pair<W, int> distanceToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
            
        for (auto &curr: vertices[v].dist) {
            if (labelOf(curr.second) == l) return curr;
        }

        for (auto &p: portals) {
            auto it = p.N_l[l].begin();
            if (it != p.N_l[l].end())
              result = min(result, make_pair(p.D_v[v] + it->first, it->second));
        }
        
        return result;
    }
};

#endif
