#include <set>

#include "mpreal.h"
#include "common.h"
#include "make_planar.h"

#define NOT_CROSSING Point(MAX_DOUBLE, MAX_DOUBLE)

using namespace std;
using mpfr::mpreal;


struct Point {
    Point (): x(0), y(0) {}
    Point (mpreal _x, mpreal _y) {
        x = mpreal_round(_x);
        y = mpreal_round(_y);
    }
    mpreal x;
    mpreal y;

    bool operator <(const Point p) const {
        if (x != p.x) return x < p.x;
        return y < p.y;
    }

    bool operator ==(const Point p) const {
        return x == p.x && y == p.y;
    }

    bool operator !=(const Point p) const {
        return x != p.x || y != p.y;
    }
};


// Edges are represented as straight lines
struct Line {
    Line (Point _start, Point _end, int _start_id, int _end_id, int _line_id):
            start(_start), end(_end), start_id(_start_id), end_id(_end_id), line_id(_line_id) {}

    Point start;
    Point end;
    int start_id;
    int end_id;
    int line_id;

    mpreal angle() const {
        if (start.x  == end.x) {
            if (start.y < end.y) {
                return numeric_limits<mpreal>::infinity();
            } else {
                return -numeric_limits<mpreal>::infinity();
            }
        }
        return (end.y - start.y) / (end.x - start.x);
    }

    // Comparator used for sweeping algorithm
    bool operator < (const Line &l) const {
        mpreal a1 = this->angle();
        mpreal a2 = l.angle();

        if (start == l.start) {
            if (a1 == a2) {
                if (end.x != l.end.x) return end.x < l.end.x;
                return end.y < l.end.y;
            }
            return a1 < a2;
        }
        if (start.x == l.start.x) {
            return start.y < l.start.y;
        }
        if (start.x > l.start.x) {
            mpreal cmp_y = l.start.y + (start.x - l.start.x)*a2;
            if (cmp_y != start.y) {
                return start.y < cmp_y;
            }
        } else {
            mpreal cmp_y = start.y + (l.start.x - start.x)*a1;
            if (cmp_y != l.start.y) {
                return cmp_y < l.start.y;
            }
        }
        return a1 < a2;
    }
};


// Events that happen in the same coordinates
struct Event {
    set<Line> starts;  // Lines that start
    set<int> crosses;  // Lines that cross
    set<int> ends;     // Lines that end
};


// When lines/edges cross add new node as intermediate node in both of them
void edges_remove_old_add_new(unordered_mapB<int, set<Edge> > &edges, Node &new_node, Line &l) {
    if (l.start_id != new_node.id && l.end_id != new_node.id) {
        Edge old = *edges[l.start_id].find(Edge(l.start_id, l.end_id));
        mpreal ratio;

        if (new_node.lat != l.start.x) {
            ratio = (new_node.lat - l.start.x)/(l.end.x - l.start.x);
        } else {
            ratio = (new_node.lon - l.start.y)/(l.end.y - l.start.y);
        }
        assert(ratio > 0 && ratio < 1);

        //Remove edge from start to end
        edges[l.start_id].erase(old);
        edges[l.start_id].insert(Edge(l.start_id, new_node.id, old.max_speed, old.type, old.distance*ratio));
        edges[new_node.id].insert(Edge(new_node.id, l.start_id, old.max_speed, old.type, old.distance*ratio));

        //Remove edge from end to start
        old = *edges[l.end_id].find(Edge(l.end_id, l.start_id));
        edges[l.end_id].erase(old);
        edges[l.end_id].insert(Edge(l.end_id, new_node.id, old.max_speed, old.type, old.distance*(1-ratio)));
        edges[new_node.id].insert(Edge(new_node.id, l.end_id, old.max_speed, old.type, old.distance*(1-ratio)));
    }
}


// Return if lines intersect
bool lines_intersect(const Line &l1, const Line &l2) {
    mpreal c1 = ((l2.start.x-l1.start.x)*(l1.end.y-l1.start.y) - (l2.start.y-l1.start.y)*(l1.end.x-l1.start.x))
                * ((l2.end.x-l1.start.x)*(l1.end.y-l1.start.y) - (l2.end.y-l1.start.y)*(l1.end.x-l1.start.x));
    mpreal c2 = ((l1.start.x-l2.start.x)*(l2.end.y-l2.start.y) - (l1.start.y-l2.start.y)*(l2.end.x-l2.start.x))
                * ((l1.end.x-l2.start.x)*(l2.end.y-l2.start.y) - (l1.end.y-l2.start.y)*(l2.end.x-l2.start.x));
    return c1 <= 0 && c2 <= 0 && (c1 < 0 || c2 < 0);
}


// Return point of intersection or NOT_CROSSING
Point get_cross(const Line &l1, const Line &l2) {
    if (lines_intersect(l1, l2)) {
        mpreal deno = ((l1.start.x-l1.end.x)*(l2.start.y-l2.end.y)) - ((l1.start.y-l1.end.y)*(l2.start.x-l2.end.x));
        mpreal x = ((((l1.start.x*l1.end.y)-(l1.start.y*l1.end.x))*(l2.start.x-l2.end.x)) - ((l1.start.x-l1.end.x)*((l2.start.x*l2.end.y)-(l2.start.y*l2.end.x)))) / deno;
        mpreal y = ((((l1.start.x*l1.end.y)-(l1.start.y*l1.end.x))*(l2.start.y-l2.end.y)) - ((l1.start.y-l1.end.y)*((l2.start.x*l2.end.y)-(l2.start.y*l2.end.x)))) / deno;
        return Point(x, y);
    }
    return NOT_CROSSING;
}


// Check if neighboring lines intersect, (line given by iterator and previous one)
void check_add_cross(set<Line> &lines, map<Point, Event> &events, set<Line>::iterator it, set<pair<int, int> > &crosses) {
    if (it != lines.end()) {
        Line l = *it;
        if (it != lines.begin()) {
            it--;
            Point cross = get_cross(l, *it);
            if (cross != NOT_CROSSING) {
                pair<int, int> cr(min(l.line_id, it->line_id), max(l.line_id, it->line_id));
                if (crosses.find(cr) == crosses.end()) {
                    crosses.insert(cr);
                    if (cross != l.start && cross != l.end) {
                        events[cross].crosses.insert(l.line_id);
                    }
                    if (cross != it->start && cross != it->end) {
                        events[cross].crosses.insert(it->line_id);
                    }
                }
            }
        }
    }
}


// Takes graph and sub graph induced by edges in kuratowski, makes changes to orginal graph so sub graph becomes planar
void make_planar(unordered_mapB<int, Node> &nodes, unordered_mapB<int, set<Edge> > &edges, vector<pair<int, int> > &kuratowski) {
    int edge_count=0;
    map<Point, Event> events;  // Map of events at given coordinates (lines start, lines cross, lines end)
    set<Node> node_set;        // Set of all nodes, used to check if point of crossing is already occupied by a node
    set<Line> lines;           // Set of lines used for sweeping
    unordered_mapB<int, set<Line>::iterator> id_to_line; // Map from line_id to line iterator in lines set

    for (auto node : nodes) {
        node_set.insert(node.second);
    }
    for (auto kur : kuratowski) {
        set<Line>::iterator lit;
        set<Event>::iterator eit;
        set<int>::iterator iit;
        int start_id = kur.first, end_id = kur.second;
        Point start(nodes[start_id].lat, nodes[start_id].lon), end(nodes[end_id].lat, nodes[end_id].lon);
        if (end < start) {
            swap(start, end);
            swap(start_id, end_id);
        }

        Line l(start, end, start_id, end_id, edge_count++);
        events[start].starts.insert(l);
        events[end].ends.insert(l.line_id);
    }

    set<pair<int, int> > crosses;

    while (!events.empty()) {
        Event e = events.begin()->second;
        Point coords = events.begin()->first;

        for (int line_id : e.ends) {
            Line l = *(id_to_line[line_id]);
            set<Line>::iterator it;
            it = id_to_line[l.line_id];
            it = lines.erase(it);
            id_to_line.erase(l.line_id);
            check_add_cross(lines, events, it, crosses);
        }

        vector<Line> vcross;
        Node new_node(coords.x, coords.y);
        if (!e.crosses.empty()) {
            set<Node>::iterator sit;
            sit = node_set.find(new_node);
            // Check if new node already exists
            if (sit != node_set.end()) {
                new_node = *sit;
            } else {
                // Add new node if doesn't exist
                new_node.id = (int)nodes.size();
                nodes[new_node.id] = new_node;
                node_set.insert(new_node);
            }
        }

        // Remove lines that cross and modify edges in graph
        for (int line_id : e.crosses) {
            set<Line>::iterator it = id_to_line[line_id];
            Line l = *it;

            lines.erase(it);
            id_to_line.erase(l.line_id);
            edges_remove_old_add_new(edges, new_node, l);

            // Update line start coordinates and id
            l.start.x = new_node.lat;
            l.start.y = new_node.lon;
            l.start_id = new_node.id;
            vcross.push_back(l);
        }

        // Add removed lines with starting coordinates as current point
        for (Line l : vcross) {
            set<Line>::iterator it;
            bool tmp;
            tie(it, tmp) = lines.insert(l);
            id_to_line[l.line_id] = it;
            check_add_cross(lines, events, it, crosses);
            check_add_cross(lines, events, ++it, crosses);
        }

        // Add lines that start in current point
        for (Line l : e.starts) {
            bool tmp;
            set<Line>::iterator it;
            tie(it, tmp) = lines.insert(l);

            id_to_line[l.line_id] = it;
            check_add_cross(lines, events, it, crosses);
            check_add_cross(lines, events, ++it, crosses);
        }
//        assert(coords == events.begin()->first);
        events.erase(events.begin());
    }
}