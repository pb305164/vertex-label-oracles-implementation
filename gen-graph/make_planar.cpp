#include <set>

#include "mpreal.h"
#include "common.h"
#include "make_planar.h"

#define NOT_CROSSING Point(MAX_DOUBLE, MAX_DOUBLE)

struct Point {
    Point (): x(0), y(0) {}
    Point (mpfr::mpreal _x, mpfr::mpreal _y) {
        x = mpreal_round(_x);
        y = mpreal_round(_y);
    }
    mpfr::mpreal x;
    mpfr::mpreal y;

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

mpfr::mpreal cord_to_a(mpfr::mpreal x1, mpfr::mpreal y1, mpfr::mpreal x2, mpfr::mpreal y2) {
    if (x1  == x2) {
        if (y1 < y2) {
            return std::numeric_limits<mpfr::mpreal>::infinity();
        } else {
            return -std::numeric_limits<mpfr::mpreal>::infinity();
        }
    }
    return (y2 - y1) / (x2 - x1);
}

struct Line {
    Line (Point _start, Point _end, int _start_id, int _end_id, int _line_id):
            start(_start), end(_end), start_id(_start_id), end_id(_end_id), line_id(_line_id) {}

    Point start;
    Point end;
    int start_id;
    int end_id;
    int line_id;

    bool operator < (const Line &l) const { // TODO jeśli x !=  to czy jest po prawej od kreski / ten sam x1 -> y1 decyduje / te same x1 y1 a decyduje !
        mpfr::mpreal a1 = cord_to_a(start.x, start.y, end.x, end.y);
        mpfr::mpreal a2 = cord_to_a(l.start.x, l.start.y, l.end.x, l.end.y);

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
            mpfr::mpreal cmp_y = l.start.y + (start.x - l.start.x)*a2;
            if (cmp_y != start.y) {
                return start.y < cmp_y;
            }
        } else {
            mpfr::mpreal cmp_y = start.y + (l.start.x - start.x)*a1;
            if (cmp_y != l.start.y) {
                return cmp_y < l.start.y;
            }
        }
        return a1 < a2;
    }
};

struct Event {
    Point cords;
    mutable std::set<Line> starts;
    mutable std::set<int> crosses;
    mutable std::set<int> ends;

    Event(mpfr::mpreal x, mpfr::mpreal y): cords(x, y), starts(), crosses(), ends() {}
    Event(Point _cords): cords(_cords), starts(), crosses(), ends() {}

    bool operator <(const Event &e) const {
        return cords < e.cords;
    }

    bool operator ==(const Event &e) const {
        return cords == e.cords;
    }
};


void edges_remove_old_add_new(unordered_map<int, std::set<Edge> > &edges, Node &new_node, Line &l) {
    if (l.start_id != new_node.id && l.end_id != new_node.id) {
        std::set<Edge>::iterator old = edges[l.start_id].find(Edge(l.start_id, l.end_id));
        mpfr::mpreal dist = old->distance, ratio;
        mpfr::mpreal prev_start_x = l.start.x;
        mpfr::mpreal prev_start_y = l.start.y;
        float max_speed = old->max_speed;
        char type = old->type;
        l.start.x = new_node.lat;
        l.start.y = new_node.lon;
        if (l.start.x != prev_start_x) {
            ratio = (l.start.x - prev_start_x)/(l.end.x - prev_start_x);
        } else {
            ratio = abs(l.start.y - std::min(prev_start_y, l.end.y))/abs(prev_start_y - l.end.y);
        }
        //Remove edge from start to end
        edges[l.start_id].erase(old);
        edges[l.start_id].insert(Edge(l.start_id, new_node.id, max_speed, type, dist*ratio));
        edges[new_node.id].insert(Edge(new_node.id, l.start_id, max_speed, type, dist*ratio));

        //Remove edge from end to start
        old = edges[l.end_id].find(Edge(l.end_id, l.start_id));
        edges[l.end_id].erase(old);
        edges[l.end_id].insert(Edge(l.end_id, new_node.id, max_speed, type, dist*(1-ratio)));
        edges[new_node.id].insert(Edge(new_node.id, l.end_id, max_speed, type, dist*(1-ratio)));

        l.start_id = new_node.id;
    }
}

bool lines_intersect(const Line &l1, const Line &l2) {
    mpfr::mpreal c1 = ((l2.start.x-l1.start.x)*(l1.end.y-l1.start.y) - (l2.start.y-l1.start.y)*(l1.end.x-l1.start.x))
                * ((l2.end.x-l1.start.x)*(l1.end.y-l1.start.y) - (l2.end.y-l1.start.y)*(l1.end.x-l1.start.x));
    mpfr::mpreal c2 = ((l1.start.x-l2.start.x)*(l2.end.y-l2.start.y) - (l1.start.y-l2.start.y)*(l2.end.x-l2.start.x))
                * ((l1.end.x-l2.start.x)*(l2.end.y-l2.start.y) - (l1.end.y-l2.start.y)*(l2.end.x-l2.start.x));
    return c1 <= 0 && c2 <= 0 && (c1 < 0 || c2 < 0);
}

Point get_cross(const Line &l1, const Line &l2) {
    if (lines_intersect(l1, l2)) {
        mpfr::mpreal deno = ((l1.start.x-l1.end.x)*(l2.start.y-l2.end.y)) - ((l1.start.y-l1.end.y)*(l2.start.x-l2.end.x));
        mpfr::mpreal x = ((((l1.start.x*l1.end.y)-(l1.start.y*l1.end.x))*(l2.start.x-l2.end.x)) - ((l1.start.x-l1.end.x)*((l2.start.x*l2.end.y)-(l2.start.y*l2.end.x)))) / deno;
        mpfr::mpreal y = ((((l1.start.x*l1.end.y)-(l1.start.y*l1.end.x))*(l2.start.y-l2.end.y)) - ((l1.start.y-l1.end.y)*((l2.start.x*l2.end.y)-(l2.start.y*l2.end.x)))) / deno;
        return Point(x, y);
    }
    return NOT_CROSSING;
}

void check_add_cross(std::set<Line> &lines, std::set<Event> &events, std::set<Line>::iterator it, std::set<std::pair<int, int> > &crosses) {
    if (it != lines.end()) {
        Line l = *it;
        if (it != lines.begin()) {
            it--;
            Point cross = get_cross(l, *it);
            if (cross != NOT_CROSSING) {
                std::pair<int, int> cr(std::min(l.line_id, it->line_id), std::max(l.line_id, it->line_id));
                if (crosses.find(cr) == crosses.end()) {
                    crosses.insert(cr);
                    Event e(cross), b = *(events.begin());
                    std::set<Event>::iterator sit = events.find(e);
                    if (sit != events.end()) {
                        if (cross != l.start && cross != l.end) {
                            sit->crosses.insert(l.line_id);
                        }
                        if (cross != it->start && cross != it->end) {
                            sit->crosses.insert(it->line_id);
                        }
                    } else {
                        if (cross != l.start && cross != l.end) {
                            e.crosses.insert(l.line_id);
                        }
                        if (cross != it->start && cross != it->end) {
                            e.crosses.insert(it->line_id);
                        }
                        events.insert(e);
                    }
                }
            }
        }
    }
}

//int ile=0;
void make_planar(unordered_map<int, Node> &nodes, unordered_map<int, std::set<Edge> > &edges, std::vector<std::pair<int, int> > &kuratowski) {
    int edge_count=0;
    std::set<Event> events;
    std::set<Node> node_set;
    std::set<Line> lines;
    unordered_map<int, std::set<Line>::iterator> id_to_line;
    mpfr::mpreal min = std::numeric_limits<mpfr::mpreal>::max();
    for (auto node : nodes) {
        node_set.insert(node.second);
        if (node.second.lat < min) min = node.second.lat;
    }
    for (auto kur : kuratowski) {
        std::set<Line>::iterator lit;
        std::set<Event>::iterator eit;
        std::set<int>::iterator iit;
        int start_id = kur.first, end_id = kur.second;
        Point start(nodes[start_id].lat, nodes[start_id].lon), end(nodes[end_id].lat, nodes[end_id].lon);
        if (end < start) {
            int tmp = start_id;
            start_id = end_id;
            end_id = tmp;
            Point ptmp = start;
            start = end;
            end = ptmp;
        }
        Event es(start);
        Line l(start, end, start_id, end_id, edge_count++);
        std::set<Event>::iterator it = events.find(es);
        if (it != events.end()) {
            (it->starts).insert(l);
        } else {
            es.starts.insert(l);
            events.insert(es);
        }
        Event et = Event(end);
        it = events.find(et);
        if (it != events.end()) {
            it->ends.insert(l.line_id);
        } else {
            et.ends.insert(l.line_id);
            events.insert(et);
        }
    }

    std::set<std::pair<int, int> > crosses;

    while (!events.empty()) {
        Event e = *(events.begin());

        for (int line_id : e.ends) {
            Line l = *(id_to_line[line_id]);
            std::set<Line>::iterator it;
            it = id_to_line[l.line_id];
            it = lines.erase(it);
            id_to_line.erase(l.line_id);
            check_add_cross(lines, events, it, crosses);
        }

        std::vector<Line> vcross;
        Node new_node(-1, -1, e.cords.x, e.cords.y);
        if (e.crosses.size() > 0) {
            std::set<Node>::iterator sit;
            sit = node_set.find(new_node);
            if (sit != node_set.end()) {
                new_node = *sit;
            } else {
//                ile++;
                new_node.id = nodes.size();
                nodes[new_node.id] = new_node;
                node_set.insert(new_node);
            }
        }
        for (int line_id : e.crosses) {
            std::set<Line>::iterator it = id_to_line[line_id];
            Line l = *it;

            lines.erase(it);
            id_to_line.erase(l.line_id);
            edges_remove_old_add_new(edges, new_node, l);
            vcross.push_back(l);
        }
        for (Line l : vcross) {
            std::set<Line>::iterator it;
            bool tmp;
            tie(it, tmp) = lines.insert(l);
            id_to_line[l.line_id] = it;
            check_add_cross(lines, events, it, crosses);
            check_add_cross(lines, events, ++it, crosses);
        }

        for (Line l : e.starts) {
            bool tmp;
            std::set<Line>::iterator it;
            std::tie(it, tmp) = lines.insert(l);

            id_to_line[l.line_id] = it;
            check_add_cross(lines, events, it, crosses);
            check_add_cross(lines, events, ++it, crosses);
        }

        events.erase(events.begin());
    }
//    printfd(stderr, "Dodałem %d nodow\n", ile - prev_ile);
}