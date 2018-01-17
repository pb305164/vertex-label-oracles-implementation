#include <set>
#include <vector>
#include "common.h"
#include "KDTree.h"
#include "pugixml.hpp"

void cache_nodes(pugi::xml_document &doc,
                 std::set<Node> &node_set,
                 unordered_map<int, Node> &all_nodes,
                 unordered_map <llu, int> &osm_id_to_new_id,
                 bool normalize=false)
{
    mpfr::mpreal max_lat = std::numeric_limits<mpfr::mpreal>::min(), max_lon = std::numeric_limits<mpfr::mpreal>::min();
    mpfr::mpreal min_lat = std::numeric_limits<mpfr::mpreal>::max(), min_lon = std::numeric_limits<mpfr::mpreal>::max();
    if (normalize) {
        for (pugi::xml_node node = doc.child("osm").child("node"); node; node = node.next_sibling("node")) {
            mpfr::mpreal lat = node.attribute("lat").value();
            mpfr::mpreal lon = node.attribute("lon").value();
            if (lat > max_lat) max_lat = lat;
            if (lon > max_lon) max_lon = lon;
            if (lat < min_lat) min_lat = lat;
            if (lon < min_lon) min_lon = lon;
        }
//        printfd("MAX LAT %lf  LON %lf     MIN  LAT %lf  LON %lf\n", max_lat.toDouble(), max_lon.toDouble(), min_lat.toDouble(), min_lon.toDouble());
    }
    for (pugi::xml_node node = doc.child("osm").child("node"); node; node = node.next_sibling("node")) {
        llu osm_id = std::stoull(node.attribute("id").value());
        mpfr::mpreal lat = node.attribute("lat").value();
        mpfr::mpreal lon = node.attribute("lon").value();
        if (normalize) {
            lat = ((lat - min_lat) / (max_lat - min_lat)) * 10;
            lon = ((lon - min_lon) / (max_lon - min_lon)) * 10;
        }

        Node n = Node(node_set.size(), osm_id, lat, lon);
        std::set<Node>::iterator it = node_set.find(n);
        if (it != node_set.end()) {
            n = *it;
        } else {
            node_set.insert(n);
            all_nodes[n.id] = n;
        }
        osm_id_to_new_id[osm_id] = n.id;
    }
}

int get_new_id(unordered_map<llu, int> &osm_id_to_new_id, llu osm_id) {
    unordered_map<llu, int>::iterator it = osm_id_to_new_id.find(osm_id);
//    assert(it != osm_id_to_new_id.end() && "All nodes should be cached");
    return it->second;
}

struct FindUnion {
    std::vector<int> p;

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

void read_graph_edges(pugi::xml_document &doc,
                      std::vector<std::string> &highway_filter,
                      mpfr::mpreal &max_max_speed,
                      unordered_map<int, Node> &nodes,
                      unordered_map<int, Node> &all_nodes,
                      unordered_map<int, std::set<Edge> > &edges,
                      unordered_map<llu, int> &osm_id_to_new_id)
{
    // Select roads
    std::string road_query = "/osm/way[tag[@k='highway' and (";
    road_query += "@v='"+highway_filter.front()+"' ";
    for (std::vector<std::string>::iterator it = highway_filter.begin()+1; it != highway_filter.end(); ++it) {
        road_query += "or @v='" + *it + "' ";
    }
    road_query += ")]]";
    pugi::xpath_node_set roads = doc.select_nodes(road_query.c_str());
    int max_node_id=-1;

    for (pugi::xpath_node_set::const_iterator it=roads.begin(); it!=roads.end(); ++it) {
        pugi::xml_node road = it->node();
        float max_speed = FOOT_SPEED;
        char type = (char)highway_filter.size();
        for (pugi::xml_node node = road.child("tag"); node; node = node.next_sibling("tag")) {
            if (strcmp(node.attribute("k").value(), "maxspeed") == 0) max_speed = std::stof(node.attribute("v").value());
            if (strcmp(node.attribute("k").value(), "highway") == 0) type = (char)(std::find(highway_filter.begin(), highway_filter.end(), node.attribute("v").value()) - highway_filter.begin());
            if (max_speed > max_max_speed) max_max_speed = max_speed;
        }
        int prev_node = -1;
        for (pugi::xml_node node = road.child("nd"); node; node = node.next_sibling("nd")) {
            int cur_node = get_new_id(osm_id_to_new_id, std::stoull(node.attribute("ref").value()));
            if (cur_node > max_node_id) max_node_id = cur_node;
            if (nodes.find(cur_node) == nodes.end()) nodes[cur_node] = all_nodes[cur_node];
            if (prev_node != -1 && prev_node != cur_node) {
                mpfr::mpreal dist = calc_distance(all_nodes[prev_node].lat, all_nodes[prev_node].lon, all_nodes[cur_node].lat, all_nodes[cur_node].lon);
                edges[prev_node].insert(Edge(prev_node, cur_node, max_speed, type, dist));
                edges[cur_node].insert(Edge(cur_node, prev_node, max_speed, type, dist));
            }
            prev_node = cur_node;
        }
    }
    max_node_id++;
    FindUnion f(max_node_id);
    for (std::pair<int, std::set<Edge>> p : edges) {
        for (Edge e : p.second) {
            f.unionn(e.source, e.dest);
        }
    }
    std::map<int ,int> comp_size;
    int max_size=f.find(0);
    for (int i=0; i<max_node_id; i++) {
        comp_size[f.find(i)]++;
        if (comp_size[max_size] < comp_size[f.find(i)]) max_size = f.find(i);
    }

    for (int i=0; i<max_node_id; i++) {
        if (f.find(i) != max_size) {
            edges.erase(i);
            nodes.erase(i);
        }
    }
}

void add_labels(pugi::xml_document &doc,
                unordered_map<int, Node> &nodes,
                unordered_map<int, std::set<Edge> > &edges,
                unordered_map<llu, int> &osm_id_to_new_id)
{
    std::vector<Node> vnodes;
    for (auto node : nodes) {
        vnodes.push_back(node.second);
    }
    KDTree kdtree = KDTree::create(vnodes);
    unordered_map<std::string, int> labels;
    pugi::xpath_node_set pois = doc.select_nodes("/osm/node[tag[@k='shop' or @k='amenity']]");
    for (pugi::xpath_node_set::const_iterator it=pois.begin(); it!=pois.end(); ++it) {
        pugi::xml_node poi = it->node();
        llu osm_id = std::stoull(poi.attribute("id").value());
        int poi_id = get_new_id(osm_id_to_new_id, osm_id);
        mpfr::mpreal poi_lat = poi.attribute("lat").value();
        mpfr::mpreal poi_lon = poi.attribute("lon").value();
        pugi::xml_node tag = poi.child("tag");

        while (tag && strcmp(tag.attribute("k").value(), "shop") != 0 && strcmp(tag.attribute("k").value(), "amenity") != 0) {
            tag = tag.next_sibling("tag");
        }
        assert(tag && "Tag should be found");

        if (tag) {
            std::string slbl = tag.attribute("v").value();
            int lbl;
            if (labels.find(slbl) != labels.end()) {
                lbl = labels[slbl];
            } else {
                lbl = (int)labels.size();
                labels[slbl] = lbl;
            }

            std::pair<Node, mpfr::mpreal> near = kdtree.find_nearest(poi_lat, poi_lon);
            if (nodes.find(poi_id) == nodes.end()) {
                nodes[poi_id] = Node(poi_id, osm_id, poi_lat, poi_lon, lbl);
                edges[poi_id].insert(Edge(poi_id, near.first.id, FOOT_SPEED, 20, near.second));
                edges[near.first.id].insert(Edge(near.first.id, poi_id, FOOT_SPEED, 20, near.second));
            } else {
                nodes[poi_id].label = lbl;
            }
        }
    }
}

void merge_edges(unordered_map<int, Node> &nodes, unordered_map<int, std::set<Edge> > &edges) {
    for (unordered_map<int, std::set<Edge> >::iterator it = edges.begin(); it != edges.end(); ) {
        if (it->second.size() == 2 && nodes[it->first].label == -1) {
            Edge e1 = *(it->second.begin()), e2 = *(++(it->second.begin()));
            if (edges[e1.dest].find(Edge(e1.dest, e2.dest)) == edges[e1.dest].end() && edges[e2.dest].find(Edge(e2.dest, e1.dest)) == edges[e2.dest].end() && e1.type == e2.type) {
                mpfr::mpreal new_dist = e1.distance + e2.distance;
                float new_max_speed = (float)((new_dist*e1.max_speed*e2.max_speed)/(e2.max_speed*e1.distance + e1.max_speed*e2.distance));
                std::set<Edge> *s;

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

void remap_ids(unordered_map<int, Node> &nodes, unordered_map<int, std::set<Edge> > &edges) {
    std::vector<Node> vnodes;
    std::vector<Edge> vedges;
    unordered_map<int, int> new_ids;
    for (auto node : nodes) {
        int new_id = new_ids.size();
        new_ids[node.second.id] = new_id;
        node.second.id = new_id;
        vnodes.push_back(node.second);
    }
    for (auto edge_set : edges) {
        for (auto edge : edge_set.second) {
            edge.dest = new_ids[edge.dest];
            edge.source = new_ids[edge.source];
            vedges.push_back(edge);
        }
    }
    nodes.clear();
    edges.clear();
    for (auto node : vnodes) {
        nodes[node.id] = node;
    }
    for (auto edge : vedges) {
        edges[edge.source].insert(edge);
    }
}

void check_edges(unordered_map<int, std::set<Edge> > &edges) {
    for (auto p: edges) {
        for (Edge e: p.second) {
            assert(e.source != e.dest);
        }
    }
}

std::pair<unordered_map<int, Node>, unordered_map<int, std::set<Edge> > > read_graph(
        char* file_name,
        std::vector<std::string> &highway_filter,
        mpfr::mpreal &max_max_speed,
        bool bmerge_edges=true,
        bool badd_labels=true) {
    std::set<Node> node_set;
    unordered_map<llu, int> osm_id_to_new_id;
    unordered_map<int, Node> all_nodes;

    unordered_map<int, Node> nodes;
    unordered_map<int, std::set<Edge> > edges;
    const mpfr::mpreal MAX_DOUBLE = std::numeric_limits<mpfr::mpreal>::max();
    pugi::xml_document doc;

    pugi::xml_parse_result result = doc.load_file(file_name);

    // Check if parsing succeeded
    if (result) {
    } else {
        fprintf(stderr, "XML %s parsed with errors: %s\n", file_name, result.description());
        exit(1);
    }

//    printfd("Caching nodes\n");
    cache_nodes(doc, node_set, all_nodes, osm_id_to_new_id);
//    printfd("Reading grapg\n");
    read_graph_edges(doc, highway_filter, max_max_speed, nodes, all_nodes, edges, osm_id_to_new_id);
    check_edges(edges);
//    printfd("Merging edges\n");
    if (bmerge_edges) merge_edges(nodes, edges);
    check_edges(edges);
//    printfd("Adding labels\n");
    if (badd_labels) add_labels(doc, nodes, edges, osm_id_to_new_id);
    check_edges(edges);
    remap_ids(nodes, edges);
    check_edges(edges);
    return std::make_pair(nodes, edges);
}
