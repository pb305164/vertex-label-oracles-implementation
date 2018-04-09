#ifndef VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_LIGHT_H
#define VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_LIGHT_H

#include "hierarchy_oracle_base.h"

// Light oracle only vertex-vertex and vertex-label
class HierarchyOracleLight : public HierarchyOracleBase {
private:
    map<int, unordered_map<int, float>> portal_distances; // Distances from portal to vertices from incident neighborhoods
    map<int, unordered_map<int, float>> portal_portal_distances; // Distances from portal to vertices from incident neighborhoods

    unordered_map<int, Portal> portals; // Used for vertex-label queries

    // Calculate portal distances to incident neighborhoods and portals incident to those neighborhoods
    void calc_portal_distances(unordered_map<int, W> &distances, int p, vector<bool> &is_portal, map<int, set<int>> &portal_to_neigh) {
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<> > queue;
        set<int> need_visit;

        // Select portals that p need to have distances to
        for (int neigh: portal_to_neigh[p]) {
            for (int portal: neigh_to_portals[neigh]) {
                need_visit.insert(portal);
            }
        }
        set<int> hmm = need_visit;

        distances[p] = 0;
        queue.push(make_pair(0, p));
        // Calculate untill all needed distances are calculated
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;
            if (d != distances[v]) continue;
            need_visit.erase(v);
            for (Edge e : edges[v]) {
                int u = e.destination;
                if ((need_visit.size() > 0 || portal_to_neigh[p].count(neighborhood[e.destination]) > 0) && (distances.count(u) == 0 || distances[u] > d + e.weight)) {
                    distances[u] = d + e.weight;
                    queue.push(make_pair(distances[u], u));
                }
            }
        }

        // Remove not needed distances
        for (unordered_map<int, W>::iterator it=distances.begin(); it != distances.end();) {
            if (hmm.count(it->first) == 0 && neigh_to_portals[neighborhood[it->first]].count(p) == 0) {
                it = distances.erase(it);
            } else {
                it++;
            }
        }
    }

    // Calculate distance from s to t going thought portals
    void portal_dijkstra(int s, int t, float &dist) {
        unordered_map<int, W> distances;
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<> > queue;

        for (int p: neigh_to_portals[neighborhood[s]]) {
            assert(portal_distances[p].count(s) > 0);
            distances[p] = portal_distances[p][s];
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
                return;
            }
            if (d != distances[v]) continue;

            // Check if portal can reach t
            if (portal_distances[v].count(t)>0 && dist > d + portal_distances[v][t]) {
                dist = d + portal_distances[v][t];
            }

            // Check all other portals that portal v can possibly reach
            for (auto &e: portal_portal_distances[v]) {
                if (distances.count(e.first) == 0 || distances[e.first] > d + e.second) {
                    distances[e.first] = d + e.second;
                    queue.push(make_pair(distances[e.first], e.first));
                }
            }
        }
        assert(dist != infinity);
    }

    // Calculate distance from s to t going thought portals
    void portal_dijkstra_label(int s, int l, float &dist, int &end) {
        unordered_map<int, W> distances;
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<> > queue;

        for (int p: neigh_to_portals[neighborhood[s]]) {
            assert(portal_distances[p].count(s) > 0);
            distances[p] = portal_distances[p][s];
            queue.push(make_pair(distances[p], p));
        }
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;

            // If current distance > already found, end searching
            if (d > dist) {
                return;
            }
            if (d != distances[v]) continue;

            // Check if portal can reach label l
            if (portals[v].N.count(l)>0 && dist > d + portals[v].N[l].top()) {
                dist = d + portals[v].N[l].top();
                end = portals[v].N[l].top_ver();
            }

            // Check all other portals that portal v can possibly reach
            for (auto &e: portal_portal_distances[v]) {
                if (distances.count(e.first) == 0 || distances[e.first] > d + e.second) {
                    distances[e.first] = d + e.second;
                    queue.push(make_pair(distances[e.first], e.first));
                }
            }
        }
    }

    // Calculate local distance to label, return distance and vertex with that label
    pair<W, int> find_local_distance_to_label(unordered_map<int, W> &distances, int s, int l) {
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
                    queue.push(make_pair(distances[u], u));
                }
            }
        }
        return make_pair(infinity, -1);
    }

public:
    HierarchyOracleLight(vector <pair<int, int>> &_edges, vector <W> &_weights, vector<int> &_labels,
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
        float prog = 0;
        for (size_t i = 0; i < nodes_count; i++) {
            if (floor(((float) (i) / (float) (nodes_count) * 20.0)) > prog) {
                prog++;
                printfd("PROGRES: %f %%\n", prog * 5.0);
            }

            if (is_portal[i]) {
                calc_portal_distances(portal_distances[i], i, is_portal, portal_to_neigh);

                // Group distances in portals by label (construct N-stacks)
                for (auto &p: portal_distances[i]) {
                    if (label[p.first] != 0) {
                        portals[i].N[label[p.first]].insert(portals[i].ver_pos, p.first, p.second);
                    }
                    if (is_portal[p.first]) {
                        portal_portal_distances[i][p.first] = p.second;
                    }
                }
                assert(portal_portal_distances[i].size() > 1);
            }
        }
    }


    W distanceToVertex(int s, int t) {
        if (s == t) return 0;
        W dist = infinity;
        if (neighborhood[s] == neighborhood[t]) {
            unordered_map<int, W> distances;
            calc_local_distances(distances, s, t); // TODO fix ?
            dist = distances[t];
        }
        portal_dijkstra(s, t, dist);

        assert(dist != infinity);
        return dist;
    }

    pair<W, int> distanceToLabel(int s, int l) {
        if (label[s] == l) return make_pair(0, s);
        W dist = infinity;
        int end = -1;
        unordered_map<int, W> distances;
        std::tie(dist, end) = find_local_distance_to_label(distances, s, l);


        portal_dijkstra_label(s, l, dist, end);

        assert(dist != infinity);
        return make_pair(dist, end);
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



#endif //VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_LIGHT_H
