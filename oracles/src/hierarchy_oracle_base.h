#ifndef VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_BASE_H
#define VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_BASE_H

#include <vector>
#include <queue>
#include <map>
#include <set>
#include <unordered_map>
#include <cmath>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>


#include "precision.h"
#include "find_union.h"
#include "heap.h"

#define printfd(...) \
            do { if (DEBUG>0) fprintf(stderr, ##__VA_ARGS__); } while (0)
#define printfdd(...) \
            do { if (DEBUG>1) fprintf(stderr, ##__VA_ARGS__); } while (0)
#define DEBUG 0


using namespace std;


struct Edge {
    char type;
    int destination;
    W weight;

    Edge(char _type, int _destination, W _weight): type(_type), destination(_destination), weight(_weight) {}

    // True if edge is considered local, type > 9
    bool is_local() {
        return type > 9;
    }
};



struct Portal {
    unordered_map<int, Heap<ScapegoatMap<int, int>>> N;
    ScapegoatMap<int, int> ver_pos;
};


class HierarchyOracleBase {
protected:

    size_t nodes_count;
    vector<int> label;
    vector<int> neighborhood;
    vector<vector<Edge>> edges;
    vector<vector<int>> components; // Neighborhood id -> vertices in this neighborhood

    map<int, set<int>> neigh_to_portals;

    // Initally splits graph into neighborhoods by major roads, returns number of neighborhoods
    size_t split_by_major_roads() {
        FindUnion funion((int) nodes_count);
        map<int, int> component_size;

        // Find neighborhoods for nodes that ARE incident to major roads
        for (size_t i = 0; i < edges.size(); i++) {
            for (Edge &e : edges[i]) {
                if (!e.is_local()) { // Merge nodes connected by MAJOR roads
                    funion.unionn(i, e.destination);
                }
            }
        }
        for (size_t i = 0; i < nodes_count; i++) component_size[funion.find(i)]++;
        for (size_t i = 0; i < nodes_count; i++) {
            // Assign neighborhood by consistent components (if component size if greater then single node)
            // Nodes which are incident to both local and major roads are assigned to major neighborhoods
            if (component_size[funion.find(i)] > 1) neighborhood[i] = funion.find(i);
        }

        // Find neighborhoods for nodes that ARE NOT incident to major roads
        funion = FindUnion((int) nodes_count);
        for (size_t i = 0; i < edges.size(); i++) {
            if (neighborhood[i] == -1) {
                for (Edge &e : edges[i]) {
                    if (neighborhood[e.destination] == -1) {
                        funion.unionn((int) i, e.destination);
                    }
                }
            }
        }
        for (size_t i = 0; i < nodes_count; i++) component_size[funion.find(i)]++;
        for (size_t i = 0; i < nodes_count; i++) {
            // Assign neighborhood
            if (neighborhood[i] == -1) neighborhood[i] = funion.find(i);
        }

        map<int, int> remap_ids;
        for (size_t i = 0; i < nodes_count; i++) {
            assert(neighborhood[i] != -1); // All nodes should have neighborhood assigned
            if (remap_ids.find(neighborhood[i]) == remap_ids.end()) {
                int new_id = (int) remap_ids.size();
                remap_ids[neighborhood[i]] = new_id;
            }
            neighborhood[i] = remap_ids[neighborhood[i]];
        }

        return remap_ids.size();
    }


    // Return true if vertices in separator divides component into balanced parts
    bool separates(map<int, int> &id_map, vector<int> &comp, vector<bool> &separator, size_t max_part_limit) {
        vector<bool> visited(comp.size(), false);
        queue<int> bfs_q;
        size_t max_part_size = 0;

        for (size_t i = 0; i < comp.size(); i++) {
            if (!separator[i] && !visited[i]) {
                size_t count = 0;
                visited[i] = true;
                bfs_q.push(comp[i]);
                while (!bfs_q.empty()) {
                    int v = bfs_q.front();
                    bfs_q.pop();
                    if (!separator[id_map.at(v)]) {
                        count++;
                        for (Edge &e: edges[v]) {
                            if (id_map.count(e.destination) > 0 && !visited[id_map.at(e.destination)]) {
                                visited[id_map.at(e.destination)] = true;
                                bfs_q.push(e.destination);
                            }
                        }
                    }
                }
                if (max_part_size < count) max_part_size = count;
            }
        }

        // Maximal consistent component cannot contain more then 3/4 of all nodes
        return max_part_size < max_part_limit;
    }


    // Return 2 vertices from component that are far away from each other
    tuple<int, int> get_distant_vertices(map<int, int> &id_map, vector<int> &comp) {
        int start, end = comp[0], cur_dist = -1, prev_dist = -10;

        while (cur_dist != prev_dist) {
            start = end;
            prev_dist = cur_dist;

            vector<bool> visited(comp.size(), false);
            queue<int> bfs_q;
            int next_inc = start;

            cur_dist = 0;
            visited[id_map.at(start)] = true;
            bfs_q.push(start);

            while (!bfs_q.empty()) {
                int v = bfs_q.front();
                bfs_q.pop();
                if (v == next_inc) {
                    cur_dist++;
                    next_inc = -1;
                }
                end = v;
                for (Edge &e: edges[v]) {
                    if (id_map.find(e.destination) != id_map.end() && !visited[id_map.at(e.destination)]) {
                        visited[id_map.at(e.destination)] = true;
                        bfs_q.push(e.destination);
                        if (next_inc == -1) next_inc = e.destination;
                    }
                }
            }
            assert(start != end);
        }
        return make_tuple(start, end);
    }

    // Return 2 vertices from separator that are far away from each other
    tuple<int, int> get_distant_separator_vertices(map<int, int> &id_map, vector<int> &comp, vector<bool> &separator) {
        int start, end = -1, cur_dist = -1, prev_dist = -10;

        for (int i = 0; end == -1; i++) {
            if (separator[i]) end = comp[i];
        }

        while (cur_dist != prev_dist) {
            start = end;
            prev_dist = cur_dist;

            vector<bool> visited(comp.size(), false);
            queue<int> bfs_q;
            int next_inc = start;

            int dist = 0;
            visited[id_map.at(start)] = true;
            bfs_q.push(start);

            while (!bfs_q.empty()) {
                int v = bfs_q.front();
                bfs_q.pop();
                if (v == next_inc) {
                    dist++;
                    next_inc = -1;
                }
                if (separator[id_map.at(v)]) {
                    end = v;
                    cur_dist = dist;
                }
                for (Edge &e: edges[v]) {
                    if (id_map.find(e.destination) != id_map.end() && !visited[id_map.at(e.destination)]) {
                        visited[id_map.at(e.destination)] = true;
                        bfs_q.push(e.destination);
                        if (next_inc == -1) next_inc = e.destination;
                    }
                }
            }
            assert(start != end);
        }
        return make_tuple(start, end);
    }

    // Select nodes that could maybe create better separator (all nodes that are closer then 5 hops from border nodes)
    void select_close_to_separator(map<int, int> &id_map, vector<int> &comp, vector<bool> &separator,
                                   vector<bool> &maybe_separator) {
        vector<bool> visited(comp.size(), false);
        queue<int> bfs_q;

        for (size_t i = 0; i < separator.size(); i++) {
            if (separator[i]) maybe_separator[i] = true;
            bfs_q.push(comp[i]);
        }

        int dist = -1, next_inc = bfs_q.front();
        for (size_t i = 0; i < comp.size(); i++) visited[i] = false;
        while (!bfs_q.empty()) {
            int v = bfs_q.front();
            bfs_q.pop();
            if (v == next_inc) {
                dist++;
                next_inc = -1;
            }
            maybe_separator[id_map.at(v)] = true;
            if (dist < 5) {
                for (Edge &e: edges[v]) {
                    if (id_map.find(e.destination) != id_map.end() && !visited[id_map.at(e.destination)]) {
                        visited[id_map.at(e.destination)] = true;
                        bfs_q.push(e.destination);
                        if (next_inc == -1) next_inc = e.destination;
                    }
                }
            }
        }
    }

    // Find shortest path from start using vertices from maybe_separtor
    void
    find_shortest_paths(map<int, int> &id_map, vector<int> &comp, vector<bool> &maybe_separator, vector<int> &parent,
                        int start) {
        vector<bool> visited(comp.size(), false);
        for (size_t i = 0; i < parent.size(); i++) parent[i] = -1;
        queue<int> bfs_q;
        visited[id_map.at(start)] = true;
        bfs_q.push(start);
        while (!bfs_q.empty()) {
            int v = bfs_q.front();
            bfs_q.pop();
            if (maybe_separator[id_map.at(v)]) {
                for (Edge &e: edges[v]) {
                    if (id_map.find(e.destination) != id_map.end() && !visited[id_map.at(e.destination)]) {
                        visited[id_map.at(e.destination)] = true;
                        bfs_q.push(e.destination);
                        parent[id_map.at(e.destination)] = v;
                    }
                }
            }
        }
    }

    // Assign vertices of composition to balanced sides, based on to which vertex is closer
    void assign_side(map<int, int> &id_map, vector<int> &comp, vector<bool> &side, int start, int end) {
        vector<bool> visited(comp.size(), false);
        queue<int> bfs_start, bfs_end;
        int start_size = 0, end_size = 0;
        bfs_start.push(start);
        bfs_end.push(end);
        while (!bfs_start.empty() || !bfs_end.empty()) {
            bool start_turn = (start_size < end_size && !bfs_start.empty()) || bfs_end.empty();
            int v;
            if (start_turn) {
                v = bfs_start.front();
                bfs_start.pop();
                side[id_map.at(v)] = true;
                if (!visited[id_map.at(v)]) {
                    start_size++;
                    side.at(id_map.at(v)) = true;
                }
            } else {
                v = bfs_end.front();
                bfs_end.pop();
                if (!visited[id_map.at(v)]) {
                    end_size++;
                }
            }
            if (!visited[id_map.at(v)]) {
                visited[id_map.at(v)] = true;
                for (Edge &e: edges[v]) {
                    if (id_map.find(e.destination) != id_map.end() && !visited[id_map.at(e.destination)]) {
                        if (start_turn) {
                            bfs_start.push(e.destination);
                        } else {
                            bfs_end.push(e.destination);
                        }
                    }
                }
            }
        }
    }

    void check_neigh() {
        size_t size=0;
        for (size_t i = 0; i < components.size(); i++) {
            for (int v: components[i]) {
                assert(neighborhood[v] == (int)i);
                size++;
            }
        }

        assert(size == nodes_count);

        // Check neighborhoods are connected
        FindUnion funion((int)nodes_count);
        map<int, int> ntou;
        for (size_t i=0; i<nodes_count; i++) {
            for (Edge &e: edges[i]) {
                if (neighborhood[i] == neighborhood[e.destination]) funion.unionn(i, e.destination);
            }
        }

        for (size_t i=0; i<nodes_count; i++) {
            ntou[neighborhood[i]] = funion.find(i);
        }

        for (size_t i = 0; i < components.size(); i++) {
            for (int v: components[i]) {
                assert(funion.find(v) == ntou[neighborhood[v]]);
            }
        }
    }

    // Splits neighborhood into at least 2 pieces of similar size
    void split_neigh(size_t neigh_id) {
        // Shortcut
        vector<int> &comp = components[neigh_id];

        // Map node id to comp offset
        map<int, int> id_map;
        for (int i = 0; i < (int) comp.size(); i++) {
            id_map[comp[i]] = i;
        }

        // Get vertices from component that are distant from each other (counting number of edges not weights)
        int start, end;
        tie(start, end) = get_distant_vertices(id_map, comp);

        // Assign each vertex to start or end composition depending on which of the vertex is closer
        vector<bool> sides(comp.size(), false);
        assign_side(id_map, comp, sides, start, end);

        // Select border vertices, as vertices of separator
        vector<bool> separator(comp.size(), false);
        size_t sep_size = 0, side1_size = 0, side2_size = 0;
        for (int i: comp) {
            if (sides[id_map.at(i)]) {
                side1_size++;
                bool end_neigh = false;
                for (Edge &e: edges[i]) {
                    if (id_map.find(e.destination) != id_map.end() &&
                        sides[id_map.at(e.destination)] == false)
                        end_neigh = true;
                }
                if (end_neigh) {
                    separator[id_map.at(i)] = true;
                    sep_size++;
                }
            } else side2_size++;
        }

        printfdd("div on side1: %lu    side2: %lu   size: %lu\n", side1_size, side2_size, comp.size());

        assert(sep_size > 0);

        // Try to optimize separator size
        if (sep_size > 1) {
            // Select current separator vertices that are far away from each other
            tie(start, end) = get_distant_separator_vertices(id_map, comp, separator);

            // Select nodes that could maybe create better separator (all nodes that are closer then 5 hops from border nodes)
            vector<bool> maybe_separator(comp.size(), false);
            select_close_to_separator(id_map, comp, separator, maybe_separator);

            // Find shortest path from start to end
            vector<int> parent(comp.size(), -1);
            find_shortest_paths(id_map, comp, maybe_separator, parent, start);

            // Construct new separator as shortest path from end to start
            vector<bool> new_sep(comp.size(), false);
            size_t new_sep_size = 0;
            for (int i = end; i != -1; i = parent[id_map.at(i)]) {
                new_sep[id_map.at(i)] = true;
                new_sep_size++;
            }

            assert(new_sep[id_map[start]] == true && new_sep[id_map[end]] == true);

            printfdd("path size %lu\n", new_sep_size);

            size_t max_part_limit = comp.size() - (size_t) (min(side1_size, side2_size) * 0.7);

            // If it is proper separator try remove unnecessary nodes
            if (separates(id_map, comp, new_sep, max_part_limit)) {
                for (size_t i = 0; i < new_sep.size(); i++) {
                    if (new_sep[i] == true) {
                        // Try to remove it
                        new_sep[i] = false;
                        new_sep_size--;
                        if (!separates(id_map, comp, new_sep, max_part_limit)) {
                            new_sep[i] = true;
                            new_sep_size++;
                        }
                    }
                }
            }

            assert(new_sep_size > 0);

            printfdd("sep diff old: %lu   new: %lu\n", sep_size, new_sep_size);
            if (new_sep_size < sep_size) {
                for (size_t i = 0; i < separator.size(); i++) {
                    separator[i] = new_sep[i];
                }
            }
        }
//        check_neigh();

        // Create new components, and update neighborhood assignments
        assert(id_map.size() == comp.size());
        vector<bool> visited(comp.size(), false);
        queue<int> bfs_q;
        for (size_t i = 0; i < components[neigh_id].size(); i++) {
            if (!visited[i]) {
                vector<int> new_comp;
                visited[i] = true;
                bfs_q.push(components[neigh_id][i]);
                while (!bfs_q.empty()) {
                    int v = bfs_q.front();
                    bfs_q.pop();
                    new_comp.push_back(v);
                    if (!separator[id_map.at(v)]) {
                        for (Edge &e: edges[v]) {
                            if (id_map.find(e.destination) != id_map.end() && !visited[id_map.at(e.destination)]) {
                                visited[id_map.at(e.destination)] = true;
                                bfs_q.push(e.destination);
                            }
                        }
                    }
                }

                size_t new_neigh_id = components.size();
                for (int v: new_comp) {
                    neighborhood[v] = (int) new_neigh_id;
                }
                components.push_back(new_comp);
            }

        }
        // Clear old component
        components[neigh_id].clear();
//        check_neigh();
    }


    // Merge small neighborhood into smallest incident neighborhood
    void merge_small_neighborhood(int neigh_id) {
        size_t min_size = nodes_count;
        int new_neigh = -1;
        for (int v: components[neigh_id]) {
            for (Edge &e: edges[v]) {
                if (neighborhood[e.destination] != neigh_id &&
                    components[neighborhood[e.destination]].size() < min_size) {
                    min_size = components[neighborhood[e.destination]].size();
                    new_neigh = neighborhood[e.destination];
                }
            }
        }
        assert(new_neigh != -1);
        for (int v: components[neigh_id]) {
            components[new_neigh].push_back(v);
            neighborhood[v] = new_neigh;
        }
        // Empty old cell and ignore it
        components[neigh_id].clear();
//        check_neigh();
    }

    // Assigns vertices to neighborhoods and, selects portals
    void assign_neighborhood(vector<bool> &is_portal, size_t max_neigh_size) {

        size_t neigh_count = split_by_major_roads();

        components.resize(neigh_count);
        for (size_t i = 0; i < nodes_count; i++) {
            components[neighborhood[i]].push_back(i);
        }
//        check_neigh();

        // Fix neighborhood sizes, if to small merge with other neighborhood, if to large split into parts
        queue<size_t> fix_queue;
        for (size_t i = 0; i < components.size(); i++) {
            // If component to large add to so it will be split
            if (components[i].size() > max_neigh_size) fix_queue.push(i);

            // If component to small merge it before splitting large neighborhoods
            if (components[i].size() < max_neigh_size / 8) {
                merge_small_neighborhood(i);
            }
        }

        while (!fix_queue.empty()) {
            size_t top = fix_queue.front();
            fix_queue.pop();
            if (components[top].size() > max_neigh_size) {
                size_t old_size = components.size();
                // If neighborhood is to large, split it
                split_neigh(top);

                // New components are added to the back, need to check if they need fixing
                for (size_t i = old_size; i < components.size(); i++) {
                    if (components[i].size() > max_neigh_size) fix_queue.push(i);
                    if (components[i].size() < max_neigh_size / 8) fix_queue.push(i);
                }
            } else if (components[top].size() < max_neigh_size /
                                                8) { // Check needed -> some other small neighborhood could have been merged into this one making another merging
                merge_small_neighborhood(top);
            }
        }

        // Move all not empty cells to the front of components
        vector<vector<int>>::iterator it=components.begin(), itt=++components.begin();
        while(itt != components.end()) {
            if (itt->empty() || itt <= it) itt++;
            else if (!it->empty()) it++;
            else {
                it->swap(*itt);
            }
        }
        it++; // it now points into first empty cell
        size_t size=0;

        // Check
        for (vector<vector<int>>::iterator ittt = components.begin(); ittt != it; ittt++) {
            assert(!ittt->empty());
            size++;
        }
        for (vector<vector<int>>::iterator ittt = it; ittt != components.end(); ittt++) {
            assert(ittt->empty());
        }
        components.resize(size); // Remove all empty cells
        components.shrink_to_fit();

        // Remap neighborhood
        for (size_t i=0; i<components.size(); i++) {
            for (int &v: components[i]) {
                neighborhood[v] = i;
            }
        }

        int portal_count = 0;

        // Assign portals
        for (size_t i = 0; i < nodes_count; i++) {
            int min_n = (int) nodes_count, max_n = -1;
            for (Edge &e: edges[i]) {
                if (neighborhood[e.destination] < min_n) min_n = neighborhood[e.destination];
                if (neighborhood[e.destination] > max_n) max_n = neighborhood[e.destination];
            }
            if (min_n != max_n && neighborhood[i] == min_n) {
                is_portal[i] = true;
                portal_count++;
            } else is_portal[i] = false;
        }

        size_t max_size = 0, min_size = nodes_count;
        for (auto &v: components) {
            if (max_size < v.size()) max_size = v.size();
            if (!v.empty() && min_size > v.size()) min_size = v.size();
        }
        printfd("neigh size  max: %lu  min: %lu\nportals:  %d\n", max_size, min_size, portal_count);
//        check_neigh();
    }

    void calc_all_distances(int s, vector<W> &distances) {
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<>> queue;

        distances[s] = 0;
        queue.push(make_pair(0, s));
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;
            if (d != distances[v]) continue;
            for (Edge e : edges[v]) {
                int u = e.destination;
                if (distances[u] > d + e.weight) {
                    distances[u] = d + e.weight;
                    queue.push(make_pair(distances[u], u));
                }
            }
        }
    }

    vector<int> get_near_portals(int s, vector<bool> &is_portal, W max_dist) {
        vector<int> near_portals;
        unordered_map<int, W> distances;
        distances[s] = 0;
        typedef pair<W, int> QEl;
        priority_queue<QEl, vector<QEl>, greater<>> queue;

        queue.push(make_pair(0, s));
        while (!queue.empty()) {
            QEl curr = queue.top();
            queue.pop();
            int v = curr.second;
            W d = curr.first;
            if (d != distances[v]) continue;
            for (Edge e : edges[v]) {
                int u = e.destination;
                if ((d + e.weight < max_dist) && (distances.count(u) == 0 || distances[u] > d + e.weight)) {
                    distances[u] = d + e.weight;
                    queue.push(make_pair(distances[u], u));
                }
            }
        }
        for (auto &near: distances) {
            if (is_portal[near.first]) near_portals.push_back(near.first);
        }
        return std::move(near_portals);
    }

    // Calculate local distances (if t != -1 calculate until distance to t found otherwise find all local distances)
    void calc_local_distances(unordered_map<int, W> &distances, int s, int t = -1) {
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
                return;
            }
            if (d != distances[v]) continue;
            for (Edge e : edges[v]) {
                int u = e.destination;
                if (neighborhood[u] == source_neigh && (distances.count(u) == 0 || distances[u] > d + e.weight)) {
                    distances[u] = d + e.weight;
                    queue.push(make_pair(distances[u], u));
                }
            }
        }
    }

    void merge_close_portals(vector<bool> &is_portal, W min_dist) {
        // Merge portals that are close to each other
        {
            map<int, set<int>> portal_to_neigh;
            for (auto &neigh: neigh_to_portals) {
                for (int portal: neigh.second) {
                    portal_to_neigh[portal].insert(neigh.first);
                }
            }
            for (size_t i = 0; i < nodes_count; i++) {
                if (is_portal[i]) {
                    int new_p = -1, max_deg = -1;
                    vector<int> near_portals = get_near_portals((int) i, is_portal, min_dist);
                    // Select near portal with highest degree as new merged portal
                    for (int j: near_portals) {
                        if (max_deg < (int) edges[j].size()) {
                            max_deg = (int) edges[j].size();
                            new_p = j;
                        }
                    }
                    // Remove other portals, update neigh_to_portals accordingly
                    for (int j: near_portals) {
                        if (j != new_p) {
                            is_portal[j] = false;
                            for (int n: portal_to_neigh[j]) {
                                neigh_to_portals[n].erase(j);
                                neigh_to_portals[n].insert(new_p);
                                portal_to_neigh[new_p].insert(n);
                            }
                            portal_to_neigh.erase(j);
                        }
                    }
                }
            }
        }

        int ile = 0;
        for (size_t i = 0; i < nodes_count; i++) if (is_portal[i]) ile++;
        printfd("portals after merge: %d\n", ile);
    }

    void basic_construct(vector <pair<int, int>> &_edges, vector <W> &_weights,
                         vector<char> &_types,vector<bool> &is_portal, size_t _max_neigh_size) {
        if (_max_neigh_size == 0) _max_neigh_size = 2 * (size_t) ceil(sqrt(nodes_count));

        printfd("nodes: %lu  edges: %lu\n", nodes_count, _edges.size());
        for (size_t i = 0; i < _edges.size(); i++) {
            edges[_edges[i].first].emplace_back(Edge(_types[i], _edges[i].second, _weights[i]));
            edges[_edges[i].second].emplace_back(Edge(_types[i], _edges[i].first, _weights[i]));
        }

        // Assign neighborhoods to vertices by consistent components
        assign_neighborhood(is_portal, _max_neigh_size);

        // Assign portals to neighborhoods
        for (size_t i = 0; i < nodes_count; i++) {
            if (is_portal[i]) {
                neigh_to_portals[neighborhood[i]].insert(i);
                for (auto &e: edges[i]) {
                    neigh_to_portals[neighborhood[e.destination]].insert(i);
                }
            }
        }

        for (size_t i = 0; i < components.size(); i++) {
            assert(neigh_to_portals.count(i));
        }
    }

    HierarchyOracleBase(vector<pair<int, int>>& _edges, vector<W>& _weights, vector<int>& _labels, vector<char> &_types):
            nodes_count(_labels.size()), label(_labels), neighborhood(_labels.size(), -1), edges(_labels.size()) {}
};

#endif //VERTEX_LABEL_ORACLES_HIERARCHY_ORACLE_BASE_H
