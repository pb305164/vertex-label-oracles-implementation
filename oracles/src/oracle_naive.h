#ifndef _ORACLE_NAIVE_H_
#define _ORACLE_NAIVE_H_

#include "precision.h"
#include "graph.h"

#include <vector>
#include <utility>
#include <queue>
#include <set>

using std::set;
using std::vector;
using std::pair;
using std::priority_queue;
using std::greater;
using std::make_pair;

class OracleNaive {
    
    Graph g;
    vector<int> labels;

    void initializeWithLabels(const vector<int> &llabels) {
        labels = llabels;
    }

public:

    OracleNaive(
            int n,
            const vector< pair<int, int> > edges,
            const vector< W > weights) :
        g(n, edges, weights)
    {
        vector<int> labels;
        for (int i=0; i<n; ++i) labels.push_back(i);
        initializeWithLabels(labels);
    }

    OracleNaive(
            int n, 
            const vector< pair<int, int> > &edges, 
            const vector<W> &weights, 
            vector<int> labels) :
        g(n, edges, weights)
    {
        initializeWithLabels(labels);
    }

    void setLabel(int v, int l) {
        labels[v] = l;
    }

    int labelOf(int v) {
        return labels[v];
    }
    
    W distanceToVertex(int v, int w);
    pair<W, int> distanceToLabel(int v, int l);
    pair<W, pair<int, int> > distanceBetweenLabels(int l1, int l2);
};

W OracleNaive::distanceToVertex(int v, int w) {
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

        if (u == w) return dist[u];

        for (int i=0; i<(int)g.edges[u].size(); ++i) {
            W wd = ud + g.edges[u][i].w;
            int w = g.edges[u][i].v;

            if (wd < dist[w]) {
                dist[w] = wd;
                queue.push(make_pair(wd,w));
            }
        }
    }

    return infinity;
}

pair<W, int> OracleNaive::distanceToLabel(int v, int l) {
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

        if (labels[u] == l) {
            return curr;
        }

        for (int i=0; i<(int)g.edges[u].size(); ++i) {
            W wd = ud + g.edges[u][i].w;
            int w = g.edges[u][i].v;

            if (wd < dist[w]) {
                dist[w] = wd;
                queue.push(make_pair(wd,w));
            }
        }
    }

    return make_pair(infinity, -1);
}

pair<W, pair<int, int> > OracleNaive::distanceBetweenLabels(int l1, int l2) {
    typedef pair<W, int> QEl;
    priority_queue< QEl, vector<QEl>, greater<QEl> > queue;
    vector<W> dist(g.n, infinity);
    vector<int> source(g.n, -1);

    for (int v=0; v<(int)labels.size(); ++v) {
        if (labels[v] == l1) {
            queue.push(make_pair(0, v));
            dist[v] = 0;
            source[v] = v;
        }
    }

    while (!queue.empty()) {
        QEl curr = queue.top(); queue.pop();
        W ud = curr.first;
        int u = curr.second;
        if (ud != dist[u]) continue;

        if (labels[u] == l2) {
            return make_pair(ud, make_pair(source[u], u));
        }

        for (int i=0; i<(int)g.edges[u].size(); ++i) {
            W wd = ud + g.edges[u][i].w;
            int w = g.edges[u][i].v;

            if (wd < dist[w]) {
                dist[w] = wd;
                source[w] = source[u];
                queue.push(make_pair(wd,w));
            }
        }
    }

    return make_pair(infinity, make_pair(-1, -1));
}

class OracleNaiveSet {
    
    Graph g;
    vector<int> labels;
    vector< set<int> > labelSets;

    void initializeWithLabels(const vector<int> &llabels) {
        labels = llabels;
        labelSets.resize(g.n);
        for (int v=0; v<g.n; ++v) {
            labelSets[labels[v]].insert(v);
        }
    }

public:

    OracleNaiveSet(
            int n,
            const vector< pair<int, int> > edges,
            const vector< W > weights) :
        g(n, edges, weights)
    {
        vector<int> labels;
        for (int i=0; i<n; ++i) labels.push_back(i);
        initializeWithLabels(labels);
    }

    OracleNaiveSet(
            int n, 
            const vector< pair<int, int> > &edges, 
            const vector<W> &weights, 
            vector<int> labels) :
        g(n, edges, weights)
    {
        initializeWithLabels(labels);
    }

    void setLabel(int v, int l) {
        labelSets[labels[v]].erase(v);
        labels[v] = l;
        labelSets[labels[v]].insert(v);
    }

    int labelOf(int v) {
        return labels[v];
    }
    
    W distanceToVertex(int v, int w);
    pair<W, int> distanceToLabel(int v, int l);
    pair<W, pair<int, int> > distanceBetweenLabels(int l1, int l2);
};

W OracleNaiveSet::distanceToVertex(int v, int w) {
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

        if (u == w) return dist[u];

        for (int i=0; i<(int)g.edges[u].size(); ++i) {
            W wd = ud + g.edges[u][i].w;
            int w = g.edges[u][i].v;

            if (wd < dist[w]) {
                dist[w] = wd;
                queue.push(make_pair(wd,w));
            }
        }
    }

    return infinity;
}

pair<W, int> OracleNaiveSet::distanceToLabel(int v, int l) {
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

        if (labels[u] == l) {
            return curr;
        }

        for (int i=0; i<(int)g.edges[u].size(); ++i) {
            W wd = ud + g.edges[u][i].w;
            int w = g.edges[u][i].v;

            if (wd < dist[w]) {
                dist[w] = wd;
                queue.push(make_pair(wd,w));
            }
        }
    }

    return make_pair(infinity, -1);
}

pair<W, pair<int, int> > OracleNaiveSet::distanceBetweenLabels(int l1, int l2) {
    typedef pair<W, int> QEl;
    priority_queue< QEl, vector<QEl>, greater<QEl> > queue;
    vector<W> dist(g.n, infinity);
    vector<int> source(g.n, -1);

    for (int v: labelSets[l1]) {
        queue.push(make_pair(0, v));
        dist[v] = 0;
        source[v] = 0;
    }

    while (!queue.empty()) {
        QEl curr = queue.top(); queue.pop();
        W ud = curr.first;
        int u = curr.second;
        if (ud != dist[u]) continue;

        if (labels[u] == l2) {
            return make_pair(ud, make_pair(source[u], u));
        }

        for (int i=0; i<(int)g.edges[u].size(); ++i) {
            W wd = ud + g.edges[u][i].w;
            int w = g.edges[u][i].v;

            if (wd < dist[w]) {
                dist[w] = wd;
                source[w] = source[u];
                queue.push(make_pair(wd,w));
            }
        }
    }

    return make_pair(infinity, make_pair(-1, -1));
}

#endif
