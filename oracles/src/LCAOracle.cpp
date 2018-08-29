#include <vector>
#include <tuple>
#include <algorithm>
#include <cmath>

#include "LCAOracle.h"

#define PI 3.14159265

using namespace std;

// calculate distance from line crosing points (x1, y1) (x2, y2) from point (x0, y0)
double point_line_dist(double x0, double y0, double x1, double y1, double x2, double y2) {
    double num = abs((y2 - y1)*x0 - (x2 - x1)*y0 + x2*y1 - y2*x1);
    double dem = sqrt((y2 - y1)*(y2 - y1) + (x2 - x1) * (x2 - x1));
    return num / dem;
}

LCAOracle::LCAOracle(std::vector<std::pair<int, int>> &_edges, std::vector<W> &_weights, std::vector<char> &_types, std::vector<int> &_labels,
                     std::vector<std::pair<double, double> > &_coords, int _number_of_trees): labels(_labels) {
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

    // Find convex hull
    double min = 10000, avg_x = 0, avg_y = 0;
    size_t imin = -1;
    for (size_t i = 0; i < _coords.size(); i++) {
        if (_coords[i].second < min) {
            min = _coords[i].second;
            imin = i;
        }
        avg_x += _coords[i].first;
        avg_y += _coords[i].second;
    }
    avg_x /= _coords.size();
    avg_y /= _coords.size();

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

    // Add convex hull to LCA roots
//    for (int &i: convex_hull) {
//        tree_roots.insert(i);
//    }
//    printf("%lu\n", tree_roots.size());


    // Select N points ass roots of lca trees, selected points are near convex hull and N directions
    for (int i=0; i < _number_of_trees; i++) {
        double direction = (i*2*PI)/(double)_number_of_trees;
        double max_x = avg_x + cos(direction)*1000;
        double max_y = avg_y + sin(direction)*1000;
        int root = -1;
        double root_dist = 1000000;
        for (size_t j=0; j<_coords.size(); j++) {
            if (_coords[j].first != avg_x || _coords[j].second != avg_y) {
                double angle = atan2(_coords[j].first - avg_x, _coords[j].second - avg_y);
                // Only check points which are more or less in given direction
                if (angle < 0) angle += 2*PI;
                if (abs(angle - direction) < 0.5) {
                    double direction_distance = point_line_dist(_coords[j].first, _coords[j].second, avg_x, avg_y, max_x, max_y);
                    double hull_distance = 1000000;
                    for (size_t k=0; k+1<convex_hull.size(); k++) {
                        double hdist = point_line_dist(
                                _coords[j].first, _coords[j].second, _coords[convex_hull[k]].first, _coords[convex_hull[k]].second,
                                _coords[convex_hull[k+1]].first, _coords[convex_hull[k+1]].second);
                        if (hdist < hull_distance) {
                            hull_distance = hdist;
                        }
                    }
                    if (root_dist > direction_distance + hull_distance*2) {
                        root = j;
                        root_dist = direction_distance + hull_distance*2;
                    }
                }
            }
        }
        tree_roots.insert(root);
    }


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
