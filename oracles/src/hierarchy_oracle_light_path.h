#ifndef VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_LIGHT_PATH_H
#define VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_LIGHT_PATH_H

#include <algorithm>

#include "hierarchy_oracle_base.h"

// Light oracle that also can calculate paths from source to destination
class HierarchyOracleLightPath : public HierarchyOracleBase {
private:
    map<int, unordered_map<int, float>> portal_distances; // Distances from portal to vertices from incident neighborhoods
    map<int, unordered_map<int, pair<float, vector<int>>>> portal_portal_distances; // Distances from portal to vertices from incident neighborhoods

    unordered_map<int, Portal> portals; // Used for vertex-label queries

    W verify_path(std::vector<int> &path) {
        int prev = -1;
        W distance = 0;
        for (auto x: path) {
            if (prev != -1) {
                bool has = false;
                for (auto p: edges[prev]) {
                    if (p.destination == (int)x) {
                        has = true;
                        distance += p.weight;
                    }
                }
                assert(has);
            }
            prev = x;
        }
        return distance;
    }

    // Calculate distances from portal p to incident neighborhoods and portals incident to those neighborhoods
    void calc_portal_distances(unordered_map<int, pair<W, int>> &distances, int p, vector<bool> &is_portal, map<int, set<int>> &portal_to_neigh, set<int> need_visit) {
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<> > queue;

        distances[p] = make_pair(0, p);
        queue.push(make_pair(0, p));
        // Calculate untill all needed distances are calculated
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;
            if (d != distances[v].first) continue;
            need_visit.erase(v);
            for (Edge e : edges[v]) {
                int u = e.destination;
                if ((need_visit.size() > 0 || portal_to_neigh[p].count(neighborhood[e.destination]) > 0) && (distances.count(u) == 0 || distances[u].first > d + e.weight)) {
                    distances[u] = make_pair(d + e.weight, v);
                    queue.push(make_pair(distances[u].first, u));
                }
            }
        }
    }

    // Calculate distance from s to t going thought portals, return last portal in path
    int portal_dijkstra(int s, int t, float &dist, unordered_map<int, int> &parent) {
        unordered_map<int, W> distances;
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<> > queue;
        int ret=-1;

        for (int p: neigh_to_portals[neighborhood[s]]) {
            assert(portal_distances[p].count(s) > 0);
            distances[p] = portal_distances[p][s];
            parent[p] = p;
            queue.push(make_pair(distances[p], p));
        }
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;
            assert(portal_distances.count(v) > 0);

            // If current distance > already found, end searching
            if (d > dist) {
                return ret;
            }
            if (d != distances[v]) continue;

            // Check if portal can reach t
            if (portal_distances[v].count(t)>0 && dist > d + portal_distances[v][t]) {
                dist = d + portal_distances[v][t];
                ret = v;
            }

            // Check all other portals that portal v can possibly reach
            for (auto &e: portal_portal_distances[v]) {
                if (distances.count(e.first) == 0 || distances[e.first] > d + e.second.first) {
                    distances[e.first] = d + e.second.first;
                    parent[e.first] = v;
                    queue.push(make_pair(distances[e.first], e.first));
                }
            }
        }
        return ret;
    }

    // Calculate distance from s to t going thought portals, return last portal in path
    int portal_dijkstra_label(int s, int l, float &dist, unordered_map<int, int> &parent, int &end) {
        unordered_map<int, W> distances;
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<> > queue;
        int ret=-1;

        for (int p: neigh_to_portals[neighborhood[s]]) {
            assert(portal_distances[p].count(s) > 0);
            distances[p] = portal_distances[p][s];
            parent[p] = p;
            queue.push(make_pair(distances[p], p));
        }
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;

            // If current distance > already found, end searching
            if (d > dist) {
                return ret;
            }
            if (d != distances[v]) continue;

            // Check if portal can reach label l
            if (portals[v].N.count(l)>0 && dist > d + portals[v].N[l].top()) {
                dist = d + portals[v].N[l].top();
                end = portals[v].N[l].top_ver();
                ret = v;
            }

            // Check all other portals that portal v can possibly reach
            for (auto &e: portal_portal_distances[v]) {
                if (distances.count(e.first) == 0 || distances[e.first] > d + e.second.first) {
                    distances[e.first] = d + e.second.first;
                    parent[e.first] = v;
                    queue.push(make_pair(distances[e.first], e.first));
                }
            }
        }
        return ret;
    }

    // Calculate local distances (if t != -1 calculate until distance to t found otherwise find all local distances)
    float calc_local_distances(unordered_map<int, int> &parent, int s, int t = -1) {
        unordered_map<int, W> distances;
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<> > queue;

        int source_neigh = neighborhood[s];
        distances[s] = 0;
        queue.push(make_pair(0, s));
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;
            // Stop if vertex t found
            if (v == t) {
                return d;
            }
            if (d != distances[v]) continue;
            for (Edge e : edges[v]) {
                int u = e.destination;
                if (neighborhood[u] == source_neigh && (distances.count(u) == 0 || distances[u] > d + e.weight)) {
                    distances[u] = d + e.weight;
                    parent[u] = v;
                    queue.push(make_pair(distances[u], u));
                }
            }
        }
        return infinity;
    }

    // Find shortest path to vertex by setting parent map
    void find_path_to_vertex(unordered_map<int, int> &parent, int s, int t) {
        unordered_map<int, W> distances;
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<> > queue;

        distances[s] = 0;
        queue.push(make_pair(0, s));
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;
            // Stop if vertex t found
            if (v == t) {
                return;
            }
            if (d != distances[v]) continue;
            for (Edge e : edges[v]) {
                int u = e.destination;
                if (distances.count(u) == 0 || distances[u] > d + e.weight) {
                    distances[u] = d + e.weight;
                    parent[u] = v;
                    queue.push(make_pair(distances[u], u));
                }
            }
        }
    }

    // Calculate local distance to label, return distance and vertex with that label
    pair<W, int> find_local_distance_to_label(unordered_map<int, int> &parent, int s, int l) {
        unordered_map<int, W> distances;
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<> > queue;

        int source_neigh = neighborhood[s];
        distances[s] = 0;
        queue.push(make_pair(0, s));
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;
            // Stop label l found
            if (label[v] == l) return make_pair(d, v);
            if (d != distances[v]) continue;
            for (Edge e : edges[v]) {
                int u = e.destination;
                if (neighborhood[u] == source_neigh && (distances.count(u) == 0 || distances[u] > d + e.weight)) {
                    distances[u] = d + e.weight;
                    parent[u] = v;
                    queue.push(make_pair(distances[u], u));
                }
            }
        }
        return make_pair(infinity, -1);
    }

    void get_path(vector<int> &path, unordered_map<int ,int> &parent, int s, int t) {
        assert(parent.count(t) > 0);
        int p = parent[t];
        while (p != s) {
            assert(parent.count(p) > 0);
            path.push_back(p);
            p = parent[p];
        }
        path.push_back(p);
    }

    // Construct shortest path
    void construct_path(W dist_local, W dist_portal, vector<int> &path, int s, int t, unordered_map<int, int> &parent_local, unordered_map<int, int> &parent_portal, int last_portal) {
        if (dist_local <= dist_portal) {
            get_path(path, parent_local, s, t);
        } else {

            parent_local.clear();
            if (last_portal != t) {
                find_path_to_vertex(parent_local, t, last_portal);
                get_path(path, parent_local, t, last_portal);
                path.push_back(last_portal);
                reverse(path.begin(), path.end());
            } else {
                path.push_back(last_portal);
            }

            assert(last_portal == path.front());

            int prev_pp = last_portal;
            for (int pp = parent_portal[last_portal]; prev_pp != pp; pp = parent_portal[pp]) {
                if (prev_pp < pp) {
                    path.insert(path.end(), portal_portal_distances[prev_pp][pp].second.begin()+1, portal_portal_distances[prev_pp][pp].second.end());
                    path.push_back(pp);
                } else {
                    vector<int> tmp_path = portal_portal_distances[pp][prev_pp].second;
                    reverse(tmp_path.begin(), tmp_path.end());
                    path.insert(path.end(), tmp_path.begin(), tmp_path.end());
                }
                prev_pp = pp;
            }

            if (path.back() != s) {
                parent_local.clear();
                find_path_to_vertex(parent_local, s, path.back());
                vector<int> tmp_path;
                get_path(path, parent_local, s, path.back());
                path.insert(path.end(), tmp_path.begin(), tmp_path.end());
            }
        }
        reverse(path.begin(), path.end());
        assert(path.size() >= 0);
    }

public:
    HierarchyOracleLightPath(vector <pair<int, int>> &_edges, vector <W> &_weights, vector<int> &_labels,
                         vector<char> &_types, size_t _max_neigh_size = 0, W min_portal_dist = -1) : HierarchyOracleBase(_edges, _weights, _labels, _types)
    {
        vector<bool> is_portal(nodes_count, false);

        basic_construct(_edges, _weights, _types, is_portal, _max_neigh_size);
        if (min_portal_dist > 0) merge_close_portals(is_portal, min_portal_dist);

        map<int, set<int>> portal_to_neigh;
        for (auto &neigh: neigh_to_portals) {
            for (int portal: neigh.second) {
                portal_to_neigh[portal].insert(neigh.first);
            }
        }

        // Process Portals (Calculate distances, populate N heaps)
        for (size_t i = 0; i < nodes_count; i++) {
            if (is_portal[i]) {
                unordered_map<int, pair<W, int>> distances;
                set<int> need_visit;
                // Select portals that p need to have distances to
                for (int neigh: portal_to_neigh[i]) {
                    for (int portal: neigh_to_portals[neigh]) {
                        need_visit.insert(portal);
                    }
                }

                calc_portal_distances(distances, i, is_portal, portal_to_neigh, need_visit);

                // Group distances in portals by label (construct N-stacks)
                for (auto &d: distances) {
                    if (need_visit.count(d.first) > 0 || neigh_to_portals[neighborhood[d.first]].count(i) > 0) {
                        portal_distances[i][d.first] = d.second.first;
                        if (label[d.first] != 0) {
                            portals[i].N[label[d.first]].insert(portals[i].ver_pos, d.first, d.second.first);
                        }
                        if (is_portal[d.first]) {
                            portal_portal_distances[i][d.first].first = d.second.first;
                            // Remember only paths from smaller ids to higher ids
                            if ((int) i < d.first) {
                                // Construct path from i to d
                                vector<int> path;
                                int parent = d.second.second;
                                while (parent != (int) i) {
                                    path.push_back(parent);
                                    assert(distances.count(parent) > 0);
                                    parent = distances[parent].second;
                                }
                                path.push_back(i);
                                reverse(path.begin(), path.end());
                                portal_portal_distances[i][d.first].second.swap(path);
                            }
                        }
                    }
                }
                assert(portal_portal_distances[i].size() > 1);
            }
        }
    }


    W distanceToVertex(int s, int t) {
        if (s == t) return 0;
        unordered_map<int, int> parent_local, parent_portal;
        vector<int> path;

        W dist_local = infinity, dist_portal = infinity;
        if (neighborhood[s] == neighborhood[t]) {
            dist_local = calc_local_distances(parent_local, s, t);
            dist_portal = dist_local;
        }
        int last_portal = portal_dijkstra(s, t, dist_portal, parent_portal);

        construct_path(dist_local, dist_portal, path, s, t, parent_local, parent_portal, last_portal);
        W got = verify_path(path);
        if (dist_local < dist_portal) {
            return dist_local;
        }
        return got;

    }

    pair<W, int> distanceToLabel(int s, int l) {
        if (label[s] == l) return make_pair(0, s);
        unordered_map<int, int> parent_local, parent_portal;
        vector<int> path;
        W dist_local = infinity, dist_portal;
        int end = -1;
        std::tie(dist_local, end) = find_local_distance_to_label(parent_local, s, l);
        dist_portal = dist_local;

        int last_portal = portal_dijkstra_label(s, l, dist_portal, parent_portal, end);

        construct_path(dist_local, dist_portal, path, s, end, parent_local, parent_portal, last_portal);
        W got = verify_path(path);
        if (dist_local < dist_portal) {
            return make_pair(dist_local, end);
        }
        return make_pair(got, end);
    }

    void setLabel(int v, int l) {
        if (label[v] != 0) {
            int v_lbl = label[v];

            // Remove vertex from N sets
            for (int p: neigh_to_portals[neighborhood[v]]) {
                portals[p].N[v_lbl].erase(portals[p].ver_pos, v);
            }
        }

        if (l != 0) {
            label[v] = l;

            // Add vertex to N sets
            for (int p: neigh_to_portals[neighborhood[v]]) {
                assert(portal_distances[p].count(v) > 0);
                portals[p].N[l].insert(portals[p].ver_pos, v, portal_distances[p][v]);
            }
        }
    }

    int labelOf(int v) {
        return label[v];
    }
};



#endif //VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_LIGHT_PATH_H
