#ifndef VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_H
#define VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_H

#include "hierarchy_oracle_base.h"

// Full oracle
class HierarchyOracle : public HierarchyOracleBase {
private:

    map<int, vector<W>> portal_distances;       // Distances from portal to all other vertices
    unordered_map<int, Portal> portals;         // Used for vertex-label queries

    // sets S as defined in paper, for each vertex i and label l sets_S[i][l] -> heap containing distances from i to local vertices with label l
    vector<unordered_map<int, Heap<ScapegoatMap<int, int>>>> sets_S;
    vector<ScapegoatMap<int, int>> ver_pos_S; // maps containing positions of local vertices in heaps S for each local vertex of vertex i

    // sets P as defined in paper, for each label l1 and label l2 sets_P[l1][l2] -> heap containing min distances from vertices with label l1 to local vertices with label l2
    unordered_map<int, unordered_map<int, ScapegoatMap<pair<W, int>, int>>> sets_P;

    // Calculate local distances and populate S and P sets for vertex v
    void setup_local_distances_sets(int v, unordered_map<int, W> &distances) {
        calc_local_distances(distances, v);

        // Group distances by labels (construct S and P stacks)
        for (auto &p: distances) {
            if (label[p.first] != 0) {
                sets_S[v][label[p.first]].insert(ver_pos_S[v], p.first, p.second);
            }
        }
        if (label[v] != 0) {
            for (auto &s: sets_S[v]) {
                if (s.first != label[v]) {
                    sets_P[label[v]][s.first][make_pair(s.second.top(), v)] = s.second.top_ver();
                }
            }
        }
    }

    void purgeLabel(int v) {
        if (label[v] != 0) {
            int v_lbl = label[v];

            // Remove vertex from N sets
            for (auto &p: portals) {
                p.second.N[v_lbl].erase(p.second.ver_pos, v);
            }

            // Remove vertex v from S sets of neighbors
            for (int u: components[neighborhood[v]]) {
                if (u != v) {
                    if (sets_S[u][v_lbl].top_ver() == v) {
                        // Remove from P sets
                        sets_P[label[u]][v_lbl].erase(make_pair(sets_S[u][v_lbl].top(), u));
                        sets_S[u][v_lbl].erase(ver_pos_S[u], v);
                        if (sets_S[u][v_lbl].size() > 0) {
                            sets_P[label[u]][v_lbl][make_pair(sets_S[u][v_lbl].top(), u)] = sets_S[u][v_lbl].top_ver();
                        }
                    } else {
                        sets_S[u][v_lbl].erase(ver_pos_S[u], v);
                    }
                    if (sets_S[u][v_lbl].size() == 0) {
                        sets_S[u].erase(v_lbl);
                        if (sets_P[label[u]][v_lbl].size() == 0) {
                            sets_P[label[u]].erase(v_lbl);
                        }
                    }
                }
            }

            // Remove self from P sets
            for (auto &s: sets_S[v]) {
                if (s.first != v_lbl) {
                    sets_P[v_lbl][s.first].erase(make_pair(s.second.top(), v));
                    if (sets_P[v_lbl][s.first].size() == 0) {
                        sets_P[v_lbl].erase(s.first);
                    }
                }
            }
            // Remove self from own set
            sets_S[v][v_lbl].erase(ver_pos_S[v], v);
        }
        label[v] = 0;
    }

    void applyLabel(int v, int new_lbl) {
        assert(label[v] == 0);
        if (new_lbl != 0) {
            label[v] = new_lbl;

            // Add vertex to N sets
            for (auto &p: portals) {
                p.second.N[new_lbl].insert(p.second.ver_pos, v, portal_distances[p.first][v]);
            }

            unordered_map<int, W> distances;
            calc_local_distances(distances, v);

            sets_S[v][new_lbl].insert(ver_pos_S[v], v, 0);

            // Add vertex v to S sets of neighbors, update P sets if needed
            for (int u: components[neighborhood[v]]) {
                if (u != v) {
                    // Update P sets if new distance is smaller then previous top
                    if (label[u] != 0 && sets_S[u][new_lbl].top() > distances[u]) {
                        // Remove top from P sets
                        if (sets_S[u][new_lbl].size() > 0) {
                            sets_P[label[u]][new_lbl].erase(make_pair(sets_S[u][new_lbl].top(), u));
                        }
                        sets_P[label[u]][new_lbl][make_pair(distances[u], u)] = v;
                    }
                    sets_S[u][new_lbl].insert(ver_pos_S[u], v, distances[u]);
                }
            }

            // Add self to P sets
            for (auto &s: sets_S[v]) {
                if (s.first != label[v]) {
                    sets_P[label[v]][s.first][make_pair(s.second.top(), v)] = s.second.top_ver();
                }
            }
        }
    }


public:
    HierarchyOracle(vector<pair<int, int>>& _edges, vector<W>& _weights, vector<int>& _labels, vector<char> &_types, size_t _max_neigh_size = 0, W _min_portal_dist = -1):
            HierarchyOracleBase(_edges, _weights, _labels, _types), sets_S(_labels.size()), ver_pos_S(_labels.size())
    {
        vector<bool> is_portal(nodes_count, false);

        basic_construct(_edges, _weights, _types, is_portal, _max_neigh_size);
        if (_min_portal_dist == -1) _min_portal_dist = 15;
        if (_min_portal_dist > 0) merge_close_portals(is_portal, _min_portal_dist);

        // Process Portals (Calculate distances, populate S, N, P heaps)
        for (size_t i=0; i<nodes_count; i++) {
            if (is_portal[i]) {

                vector<W> distances(nodes_count, infinity);
                calc_all_distances((int)i, distances);

                // Group distances in portals by label (construct N-stacks)
                for (size_t j=0; j<nodes_count; j++) {
                    if (label[j] != 0) {
                        portals[i].N[label[j]].insert(portals[i].ver_pos, (int)j, distances[j]);
                    }
                }

                // Remember distances for vertex-vertex queries
                portal_distances[i].swap(distances);
            }

            // Calculate local (in neighborhood) distances
            unordered_map<int ,W> distances;
            setup_local_distances_sets(i, distances);
        }
    }


    W distanceToVertex(int s, int t) {
        if (s == t) return 0;
        W dist = infinity;
        if (neighborhood[s] == neighborhood[t]) {
            unordered_map<int, W> distances;
            calc_local_distances(distances, s, t);
            dist = distances[t];
        }
        if (neigh_to_portals[neighborhood[s]].size() > neigh_to_portals[neighborhood[t]].size()) swap(s, t);
        for (int p: neigh_to_portals[neighborhood[s]]) {
            assert(portal_distances.count(p) > 0);
            if (dist > portal_distances[p][s] + portal_distances[p][t]) {
                dist = portal_distances[p][s] + portal_distances[p][t];
            }
        }
        assert(dist != infinity);
        return dist;
    }

    pair<W, int> distanceToLabel(int s, int l) {
        W dist = infinity;
        int end=-1;
        for (int p: neigh_to_portals[neighborhood[s]]) {
            if (dist > portal_distances[p][s] + portals[p].N[l].top()) {
                dist = portal_distances[p][s] + portals[p].N[l].top();
                end = portals[p].N[l].top_ver();
            }
        }
        if (dist > sets_S[s][l].top()) {
            dist = sets_S[s][l].top();
            end = sets_S[s][l].top_ver();
        }
        return make_pair(dist, end);
    }

    pair<W, pair<int, int>> distanceBetweenLabels(int l1, int l2) {
        W dist = infinity;
        int start=-1, end=-1;
        for (auto &p: portals) {
                if (dist > p.second.N[l1].top() + p.second.N[l2].top()) {
                    dist = p.second.N[l1].top() + p.second.N[l2].top();
                    start = p.second.N[l1].top_ver();
                    end = p.second.N[l2].top_ver();
                }
        }
        if (sets_P[l1].count(l2) > 0) {
            auto top = sets_P[l1][l2].top();
            if (dist > top.first.first) {
                dist = top.first.first;
                start = top.first.second;
                end = top.second;
            }
        }
        return make_pair(dist, make_pair(start, end));
    }

    void setLabel(int v, int l) {
        purgeLabel(v);
        applyLabel(v, l);
    }

    int labelOf(int v) {
        return label[v];
    }
};

#endif //VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_H
