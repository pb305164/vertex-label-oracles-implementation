#ifndef VERTEX_LABEL_ORACLES_IMPLEMENTATION_MASTER_DIJKSTRA_ORACLE_H
#define VERTEX_LABEL_ORACLES_IMPLEMENTATION_MASTER_DIJKSTRA_ORACLE_H

#include "precision.h"

#include <vector>
#include <queue>
#include <tuple>
#include <limits>

using std::vector;
using std::pair;
using std::tie;
using std::min;
using std::make_pair;
using std::greater;
using std::priority_queue;

typedef pair<W, int> QEl;

class DijkstraOracle {
private:
    int n;
    vector<vector<pair<int, W> > > edges;
    vector<vector<int> > lbl_to_ver;
    vector<int> labels;

public:
    DijkstraOracle(int nn, int m, int max_label, vector<pair<int, int>> &eedges, vector<W> &weights, vector<int> &llabels):
            n(nn), edges(nn), lbl_to_ver(max_label), labels(llabels) {
        for (int i = 0; i < (int) labels.size(); i++) {
            lbl_to_ver[labels[i]].push_back(i);
        }

        for (int i = 0; i < m; i++) {
            int s, t;
            tie(s, t) = eedges[i];
            edges[s].push_back(make_pair(t, weights[i]));
            edges[t].push_back(make_pair(s, weights[i]));
        }
    }

    W distanceToVertex(int s, int t) {
        vector<W> dist(n, infinity);
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<QEl> > queue;

        dist[s] = 0;
        queue.push(make_pair(0, s));
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;
            if (v == t) return d;
            if (d != dist[v]) continue;
            for (pair<int, W> p : edges[v]) {
                int u = p.first;
                if (dist[u] > d + p.second) {
                    dist[u] = d + p.second;
                    queue.push(make_pair(dist[u], u));
                }
            }
        }
        return -1;
    }

    pair<W, int> distanceToLabel(int s, int l) {
        vector<W> dist(n, infinity);
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<QEl> > queue;

        dist[s] = 0;
        queue.push(make_pair(0, s));
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;
            if (labels[v] == l) return curr;
            if (d != dist[v]) continue;
            for (pair<int, W> p : edges[v]) {
                int u = p.first;
                if (dist[u] > d + p.second) {
                    dist[u] = d + p.second;
                    queue.push(make_pair(dist[u], u));
                }
            }
        }
        return make_pair(-1, -1);
    }

    pair<W, pair<int, int> > distanceBetweenLabels(int l1, int l2) {
        vector<W> dist(n, infinity);
        vector<int> parent(n, -1);
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<QEl> > queue;

        for (int i: lbl_to_ver[l1]) {
            dist[i] = 0;
            parent[i] = i;
            queue.push(make_pair(0, i));
        }
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;
            if (labels[v] == l2) return make_pair(d, make_pair(parent[v], v));
            if (d != dist[v]) continue;
            for (pair<int, W> p : edges[v]) {
                int u = p.first;
                if (dist[u] > d + p.second) {
                    dist[u] = d + p.second;
                    parent[u] = parent[v];
                    queue.push(make_pair(dist[u], u));
                }
            }
        }
        return make_pair(-1, make_pair(-1, -1));
    }

    void setLabel(int v, int l) {
        labels[v] = l;
    }
};

#endif //VERTEX_LABEL_ORACLES_IMPLEMENTATION_MASTER_DIJKSTRA_ORACLE_H
