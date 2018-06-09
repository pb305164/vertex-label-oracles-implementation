#include <vector>
#include <tuple>
#include <algorithm>

#include "LCAOracle.h"

using namespace std;

LCAOracle::LCAOracle(std::vector<std::pair<int, int>> &_edges, std::vector<W> &_weights, std::vector<char> &_types, std::vector<int> &_labels, std::vector<std::pair<double, double> > &_coords): labels(_labels) {
    vector<vector<Edge>> edges(_labels.size());
    for (size_t i = 0; i < _edges.size(); i++) {
        edges[_edges[i].first].emplace_back(Edge(_types[i], _edges[i].second, _weights[i]));
        edges[_edges[i].second].emplace_back(Edge(_types[i], _edges[i].first, _weights[i]));
    }

    for (int i = 0; i < (int) labels.size(); i++) {
        if (labels[i] != 0) {
            lbl_to_ver[labels[i]].insert(i);
        }
    }

    // Znajdź otoczkę
    double min = 10000;
    size_t imin = -1;
    for (size_t i = 0; i < _coords.size(); i++) {
        if (_coords[i].second < min) {
            min = _coords[i].second;
            imin = i;
        }
    }

    vector<int> index(_coords.size(), -1);
    for (int i=0; i < (int)index.size(); i++) {
        index[i] = i;
    }

    auto cmp = [&](int a, int b) {
        if (a == (int)imin) return true;
        if (b == (int)imin) return false;

        double val = (_coords[a].first - _coords[imin].first) * (_coords[b].second - _coords[a].second) -
                     (_coords[a].second - _coords[imin].second) * (_coords[b].first - _coords[a].first);

        if (val == 0) {
            return _coords[a].second < _coords[b].second;
        }
        return val > 0;

    };
    sort(index.begin(), index.end(), cmp);

    vector<int> convex_hull;
    for (int i: index) {
        if (convex_hull.size() < 2) {
            convex_hull.push_back(i);
        } else {
            double o;
            do {
                int a = convex_hull.end()[-2], b = convex_hull.back();
                o = (_coords[b].first - _coords[a].first) * (_coords[i].second - _coords[b].second) -
                    (_coords[b].second - _coords[a].second) * (_coords[i].first - _coords[b].first);
                if (o <= 0) {
                    convex_hull.pop_back();
                }
            } while (o <= 0 && convex_hull.size() > 2);
            convex_hull.push_back(i);
        }
    }

//    for (int i: convex_hull) {
//        shortest_paths.emplace_back(LCATree(_coords.size(), i, edges));
//    }

    set<int> tree_roots;

    // Add leafs of major roads as roots
//    vector<bool> visited(_coords.size(), false);
//    for (int i=0; i<edges.size(); i++) {
//        if (!visited[i]) {
//            int major = 0;
//            for (Edge e: edges[i]) {
//                if (!e.is_local()) {
//                    major++;
//                }
//            }
//
//            if (major > 0) {
//                queue<int> bfs_queue;
//                bfs_queue.push(i);
//                while (!bfs_queue.empty()) {
//                    int v = bfs_queue.front();
//                    bfs_queue.pop();
//                    visited[v] = true;
//                    int major = 0;
//                    for (Edge e: edges[v]) {
//                        if (!e.is_local()) {
//                            major++;
//                            if (!visited[e.destination]) {
//                                bfs_queue.push(e.destination);
//                            }
//                        }
//                    }
//                    if (major == 1) {
//                        tree_roots.insert(v);
//                    }
//                }
//            } else {
//                visited[i] = true;
//            }
//        }
//    }

    for (int &i: convex_hull) {
        tree_roots.insert(i);
    }
    printf("%lu\n", tree_roots.size());
    for (int r: tree_roots) {
        shortest_paths.emplace_back(LCATree(_coords.size(), r, edges));
    }
}

W LCAOracle::distanceToVertex(int s, int t) {
    W dist = numeric_limits<W>::max();
    for (auto &tree: shortest_paths) {
        W q = tree.query(s, t);
        if (q < dist) {
            dist = q;
        }
    }
    return dist;
}

std::pair<W, int> LCAOracle::distanceToLabel(int s, int l) {
    W dist = numeric_limits<W>::max();
    int min_t=-1;
    for (int t: lbl_to_ver[l]) {
        for (auto &tree: shortest_paths) {
            W q = tree.query(s, t);
            if (q < dist) {
                dist = q;
                min_t = t;
            }
        }
    }
    return make_pair(dist, min_t);
}

std::pair<W, std::pair<int, int> > LCAOracle::distanceBetweenLabels(int l1, int l2) {
    W dist = numeric_limits<W>::max();
    int min_s=-1, min_t=-1;
    for (int s: lbl_to_ver[l1]) {
        for (int t: lbl_to_ver[l2]) {
            for (auto &tree: shortest_paths) {
                W q = tree.query(s, t);
                if (q < dist) {
                    dist = q;
                    min_s = s;
                    min_t = t;
                }
            }
        }
    }
    return make_pair(dist, make_pair(min_s, min_t));
}

void LCAOracle::setLabel(int v, int l) {
    if (labels[v] != 0) {
        lbl_to_ver[labels[v]].erase(v);
    }
    labels[v] = l;
    if (l != 0) {
        lbl_to_ver[l].insert(v);
    }
}

int LCAOracle::labelOf(int v) {
    return labels[v];
}
