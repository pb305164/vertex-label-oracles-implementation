#include <set>
#include <vector>
#include <algorithm>
#include "common.h"
#include "KDTree.h"
#include "pugixml.hpp"

using namespace std;
using mpfr::mpreal;

const vector<string> highway_filter = {
        "motorway",
        "motorway_link",
        "trunk",
        "trunk_link",
        "primary",
        "primary_link",
        "secondary",
        "secondary_link",
        "tertiary",
        "tertiary_link",
        "unclassified",
        "residential",
        "service",
        "living_street",
};

const vector<string> label_filter = {
        "shop",
        "amenity",
        "tourism",
        "sport",
        "leisure",
        "historic",
        "emergency",
        "public_transport",
};


// Construct query used to select all roads from osm
string get_road_query() {
    string road_query = "/osm/way[tag[@k='highway' and (";
    road_query += "@v='"+highway_filter.front()+"' ";
    for (auto it = highway_filter.begin()+1; it != highway_filter.end(); ++it) {
        road_query += "or @v='" + *it + "' ";
    }
    road_query += ")]]";
    return road_query;
}


// Construct query used to select all nodes with labels from osm
string get_label_query() {
    // Search for all nodes with tag defined in label_filter
    string label_query = "/osm/node[tag[";
    label_query += "@k='"+label_filter.front()+"' ";
    for (auto it = label_filter.begin()+1; it != label_filter.end(); ++it) {
        label_query += "or @k='" + *it + "' ";
    }
    label_query += "]]";

    // Search not only for nodes but also for ways
    label_query +=  "|/osm/way[tag["; // And all ways with tag defined in label_filter
    label_query += "@k='"+label_filter.front()+"' ";
    for (auto it = label_filter.begin()+1; it != label_filter.end(); ++it) {
        label_query += "or @k='" + *it + "' ";
    }
    label_query += "]]";
    return label_query;
}


// Cache all nodes that will be added to graph from osm file for fast access later
// This function also merges different osm nodes that have the same coordinates into the same graph node
void cache_nodes(pugi::xml_document &doc,
                 unordered_mapB<int, Node> &nodes,
                 unordered_mapB<llu, int> &osm_id_to_new_id)
{
    set<llu> nodes_to_cache;
    // Get all nodes ids used in roads
    string query = get_road_query();
    pugi::xpath_node_set xpath = doc.select_nodes(query.c_str());

    for (pugi::xpath_node_set::const_iterator it = xpath.begin(); it != xpath.end(); ++it) {
        pugi::xml_node road = it->node();
        for (pugi::xml_node node = road.child("nd"); node; node = node.next_sibling("nd")) {
            nodes_to_cache.insert(stoull(node.attribute("ref").value()));
        }
    }

    // Get all nodes ids with labels
    query = get_label_query();
    xpath = doc.select_nodes(query.c_str());

    for (pugi::xpath_node_set::const_iterator it = xpath.begin(); it != xpath.end(); ++it) {
        pugi::xml_node poi = it->node();

        // If poi is defined by osm node cache this node
        if (strcmp(poi.name(), "node") == 0) {
            nodes_to_cache.insert(stoull(poi.attribute("id").value()));
        } else {
            // If poi is defined by osm way cache first node of way as poi
            pugi::xml_node node = poi.child("nd");
            nodes_to_cache.insert(stoull(node.attribute("ref").value()));
        }
    }

    // Cache selected nodes
    set<Node> node_set;
    for (pugi::xml_node node = doc.child("osm").child("node"); node; node = node.next_sibling("node")) {
        llu osm_id = stoull(node.attribute("id").value());
        if (nodes_to_cache.find(osm_id) != nodes_to_cache.end()) {
            mpreal lat = node.attribute("lat").value();
            mpreal lon = node.attribute("lon").value();

            Node n = Node(lat, lon, (int) node_set.size(), -1, osm_id);
            auto it = node_set.find(n);
            if (it != node_set.end()) {
                // Merging osm nodes which are at the same place into the same graph node (same new id)
                n = *it;
            } else {
                node_set.insert(n);
                nodes[n.id] = n;
            }
            osm_id_to_new_id[osm_id] = n.id;
        }
    }
}


// Translate osm node id to graph node id
int get_new_id(unordered_mapB<llu, int> &osm_id_to_new_id, llu osm_id) {
    unordered_mapB<llu, int>::iterator it = osm_id_to_new_id.find(osm_id);
    assert(it != osm_id_to_new_id.end() && "All nodes should be cached");
    return it->second;
}


struct FindUnion {
    vector<int> p;

    FindUnion(int n) : p(n) {
        for (int i=0; i<n; ++i) p[i] = i;
    }

    int find(int v) {
        if (p[v] == v) return v;
        return p[v] = find(p[v]);
    }

    void unionn(int v, int u) {
        p[find(u)] = find(v);
    }

};


//Parse max_speed string and change speed unit to m/s, max_speed format: number or number space unit
float parse_max_speed(string s) {
    float max_speed;
    // Check if unit is specified
    if (s.find(" ") != string::npos) {
        // Read speed
        max_speed = stof(s.substr(0, s.find(" ")));

        // Check speed unit and convert to m/s
        string unit = s.substr(s.find(" ")+1, s.size());
        if (unit == "mph") max_speed *= 0.44704;
        else if (unit == "knots") max_speed *= 0.514444;
        else max_speed *= 0.277778; // km/h
    } else {
        if (s == "none") max_speed = 55.5556; // If there is no speed limit assume 200km/h
        else if (s == "walk") max_speed = FOOT_SPEED;
        else if (s == "signals") max_speed = 0; // Max speed is defined by dynamic signals, assume default speed depending on road type
        else {
            try {
                max_speed = stof(s)*(float)0.277778; // By default speed is given in km/h
            } catch (invalid_argument &e) { // Speed also can be specified as nations standard speed in urban area (eg "DE:urban")
                max_speed = 0;              // in that case just assume default speed depending on road type
            }
        }
    }
    return max_speed;
}


// Return default speed for given road type in m/s
float get_default_max_speed(char type) {
    if (type == 0) return 38.8889; // motorway 140 km/h
    if (type == 2) return 33.3333; // trunk  120 km/h
    if (type == 4 || type == 6 || type == 8) return 25; // primary, secondary and tertiary  90 km/h
    if (type == 13) return 8.33333; // living_street 30 km/h
    if (type == 12) return 5.55556; //sevice 20 km/h
    return 13.8889; // all other (links and residential)  50km/h
}


// Parse all roads from osm file and fill graph nodes and edges
void read_graph_edges(pugi::xml_document &doc,
                      mpreal &max_max_speed,
                      unordered_mapB<int, Node> &nodes,
                      unordered_mapB<int, Node> &all_nodes,
                      unordered_mapB<int, set<Edge> > &edges,
                      unordered_mapB<llu, int> &osm_id_to_new_id)
{
    string road_query = get_road_query();

    // Select all roads
    pugi::xpath_node_set roads = doc.select_nodes(road_query.c_str());

    // Iterate though roads, add nodes and edges to graph
    for (pugi::xpath_node_set::const_iterator it = roads.begin(); it != roads.end(); ++it) {
        // Read road attributes (Road is described as ordered list of nodes so it is represented as multiple edges in graph)
        pugi::xml_node road = it->node();
        float max_speed = 0;
        char type = (char)highway_filter.size();
        for (pugi::xml_node node = road.child("tag"); node; node = node.next_sibling("tag")) {
            if (strcmp(node.attribute("k").value(), "maxspeed") == 0) max_speed = parse_max_speed(node.attribute("v").value());
            if (strcmp(node.attribute("k").value(), "highway") == 0) type = (char)(find(highway_filter.begin(), highway_filter.end(), node.attribute("v").value()) - highway_filter.begin());
            if (max_speed > max_max_speed) max_max_speed = max_speed;
        }
        if (max_speed == 0) max_speed = get_default_max_speed(type);

        // Add edges which make up the road
        int prev_node = -1;
        for (pugi::xml_node node = road.child("nd"); node; node = node.next_sibling("nd")) {
            int cur_node = get_new_id(osm_id_to_new_id, stoull(node.attribute("ref").value()));
            if (nodes.find(cur_node) == nodes.end()) nodes[cur_node] = all_nodes[cur_node];
            if (prev_node != -1 && prev_node != cur_node) {
                mpreal dist = calc_distance(all_nodes[prev_node].lat, all_nodes[prev_node].lon, all_nodes[cur_node].lat, all_nodes[cur_node].lon);
                edges[prev_node].insert(Edge(prev_node, cur_node, max_speed, type, dist));
                edges[cur_node].insert(Edge(cur_node, prev_node, max_speed, type, dist));
            }
            prev_node = cur_node;
        }
    }
}


// Remove all other components then the biggest one thus making graph connected
void filter_max_component(unordered_mapB<int, Node> &nodes, unordered_mapB<int, set<Edge> > &edges) {
    int max_node_id = -1;
    for (auto &p: nodes) {
        if (p.first > max_node_id) max_node_id = p.first;
    }

    // Find biggest consistent component of the graph
    FindUnion f(max_node_id+1);
    for (pair<int, set<Edge>> p : edges) {
        for (Edge e : p.second) {
            f.unionn(e.source, e.dest);
        }
    }
    map<int ,int> comp_size;
    int max_size=f.find(0);
    for (int i=0; i<max_node_id; i++) {
        comp_size[f.find(i)]++;
        if (comp_size[max_size] < comp_size[f.find(i)]) max_size = f.find(i);
    }

    // Remove nodes and edges from other components from the graph
    for (int i=0; i<max_node_id; i++) {
        if (f.find(i) != max_size) {
            edges.erase(i);
            nodes.erase(i);
        }
    }
}


// Find osm nodes which represent points of interest and add them to the graph with proper labels
// Labeled nodes are connected by an edge to the nearest other node (distance = straight line, max_speed = FOOT_SPEED)
void add_labels(pugi::xml_document &doc,
                unordered_mapB<int, Node> &all_nodes,
                unordered_mapB<int, Node> &nodes,
                unordered_mapB<int, set<Edge> > &edges,
                unordered_mapB<llu, int> &osm_id_to_new_id)
{
    // Create KDTree for finding nearest node
    vector<Node> vnodes;
    for (auto node : nodes) {
        vnodes.push_back(node.second);
    }
    KDTree kdtree = KDTree::create(vnodes);

    string label_query = get_label_query();

    // Go through all points of interests in osm file, Add new node and connect it to nearest road node or edit existing node
    unordered_mapB<string, int> labels;
    pugi::xpath_node_set pois = doc.select_nodes(label_query.c_str());
    for (pugi::xpath_node_set::const_iterator it=pois.begin(); it!=pois.end(); ++it) {
        // Read attributes of the point of interest
        pugi::xml_node poi = it->node();
        mpreal poi_lat = 0, poi_lon = 0;
        llu osm_id = stoull(poi.attribute("id").value());
        int poi_id;

        // If poi is defined by osm node
        if (strcmp(poi.name(), "node") == 0) {
            poi_id = get_new_id(osm_id_to_new_id, osm_id);
            poi_lat = poi.attribute("lat").value();
            poi_lon = poi.attribute("lon").value();
            assert(poi_lat == all_nodes[poi_id].lat);
            assert(poi_lon == all_nodes[poi_id].lon);
        } else {
            // If poi is defined by osm way select first node of way as poi
            pugi::xml_node node = poi.child("nd");
            poi_id = get_new_id(osm_id_to_new_id, stoull(node.attribute("ref").value()));
            poi_lat = all_nodes[poi_id].lat;
            poi_lon = all_nodes[poi_id].lon;
        }
        pugi::xml_node tag = poi.child("tag");

        // Find tag (way/node can contain multiple matching tags first one is selected)
        while (tag && find(label_filter.begin(), label_filter.end(), tag.attribute("k").value()) == label_filter.end()) {
            tag = tag.next_sibling("tag");
        }
        assert(tag && "Tag should be found");

        // Add it to the graph
        if (tag) {
            string slbl = tag.attribute("k").value() + str(" ") + tag.attribute("v").value();
            int lbl;
            if (labels.find(slbl) != labels.end()) {
                lbl = labels[slbl];
            } else {
                lbl = (int)labels.size();
                labels[slbl] = lbl;
            }

            // Add new node and connect it to nearest if new
            pair<Node, mpreal> near = kdtree.find_nearest(poi_lat, poi_lon);
            if (nodes.find(poi_id) == nodes.end()) {
                nodes[poi_id] = Node(poi_lat, poi_lon, poi_id, lbl, osm_id);
                edges[poi_id].insert(Edge(poi_id, near.first.id, FOOT_SPEED, (char)highway_filter.size(), near.second));
                edges[near.first.id].insert(Edge(near.first.id, poi_id, FOOT_SPEED, (char)highway_filter.size(), near.second));
            } else {
                // Edit if node already exists and doesn't have label assigned
                if (nodes[poi_id].label == -1) {
                    nodes[poi_id].label = lbl;
                }
            }
        }
    }
}


// Remove nodes of degree = 2 and merge incidental edges, don't create loops
void merge_edges(unordered_mapB<int, Node> &nodes, unordered_mapB<int, set<Edge> > &edges) {
    for (unordered_mapB<int, set<Edge> >::iterator it = edges.begin(); it != edges.end(); ) {
        if (it->second.size() == 2 && nodes[it->first].label == -1) {
            Edge e1 = *(it->second.begin()), e2 = *(++(it->second.begin()));
            if (edges[e1.dest].find(Edge(e1.dest, e2.dest)) == edges[e1.dest].end() && edges[e2.dest].find(Edge(e2.dest, e1.dest)) == edges[e2.dest].end() && e1.type == e2.type) {
                mpreal new_dist = e1.distance + e2.distance;
                float new_max_speed = (float)((new_dist*e1.max_speed*e2.max_speed)/(e2.max_speed*e1.distance + e1.max_speed*e2.distance));
                set<Edge> *s;

                Edge e(e1.dest, it->first);
                s = &edges[e1.dest];
                s->erase(s->find(e));
                s->insert(Edge(e1.dest, e2.dest, new_max_speed, e1.type, new_dist));

                e.source = e2.dest;
                s = &edges[e2.dest];
                s->erase(s->find(e));
                s->insert(Edge(e2.dest, e1.dest, new_max_speed, e1.type, new_dist));

                nodes.erase(it->first);
                it = edges.erase(it);
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }
}


// Remap ids of nodes so they are continuous starting at 0, [0, 1, ..., nodes.size()-1]
void remap_ids(unordered_mapB<int, Node> &nodes, unordered_mapB<int, set<Edge> > &edges) {
    vector<Node> vnodes;
    vector<Edge> vedges;
    set<int> labels;
    labels.insert(-1);
    unordered_mapB<int, int> new_ids, new_lables;
    new_lables[-1] = -1;
    for (auto node : nodes) {
        if (labels.find(node.second.label) == labels.end()) {
            new_lables[node.second.label] = (int)labels.size() - 1;
            labels.insert(node.second.label);
        }
        int new_id = (int)new_ids.size();
        new_ids[node.second.id] = new_id;
        node.second.id = new_id;
        vnodes.push_back(node.second);
    }
    for (auto &edge_set : edges) {
        for (auto edge : edge_set.second) {
            edge.dest = new_ids[edge.dest];
            edge.source = new_ids[edge.source];
            vedges.push_back(edge);
        }
    }
    nodes.clear();
    edges.clear();
    for (auto node : vnodes) {
        node.label = new_lables[node.label];
        nodes[node.id] = node;
    }
    for (auto edge : vedges) {
        edges[edge.source].insert(edge);
    }
}


// Create graph from osm file
tuple<unordered_mapB<int, Node>, unordered_mapB<int, set<Edge>>, mpreal> read_graph(
        char* file_name,
        bool _merge_edges=false,
        bool _add_labels=true)
{
    unordered_mapB<llu, int> osm_id_to_new_id;
    unordered_mapB<int, Node> all_nodes; // All osm nodes

    unordered_mapB<int, Node> nodes; // Graph nodes
    unordered_mapB<int, set<Edge> > edges; // Graph edges
    mpreal max_max_speed; // maximum of maximal allowed speeds on roads
    pugi::xml_document doc;

    pugi::xml_parse_result result = doc.load_file(file_name);

    // Check if parsing succeeded
    if (result) {
    } else {
        fprintf(stderr, "XML %s parsed with errors: %s\n", file_name, result.description());
        exit(1);
    }

    cache_nodes(doc, all_nodes, osm_id_to_new_id);
    read_graph_edges(doc, max_max_speed, nodes, all_nodes, edges, osm_id_to_new_id);
    if (_add_labels) add_labels(doc, all_nodes, nodes, edges, osm_id_to_new_id);
    filter_max_component(nodes, edges);
    if (_merge_edges) merge_edges(nodes, edges);
    remap_ids(nodes, edges);
    return make_tuple(nodes, edges, max_max_speed);
}
