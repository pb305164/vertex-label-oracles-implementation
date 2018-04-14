#ifndef VERTEX_LABEL_ORACLES_IMPLEMENTATION_MASTER_ASTAR_ORACLE_H
#define VERTEX_LABEL_ORACLES_IMPLEMENTATION_MASTER_ASTAR_ORACLE_H

const double EARTH_RADIUS=6378137.0;
const double INVERSE_FLATTENING=298.257223563;

#include "precision.h"

//#include <GeographicLib/Geodesic.hpp>
#include <vector>
#include <queue>

typedef pair<W, int> QEl;

// This function converts decimal degrees to radians
double deg2rad(double deg) {
    return (deg * M_PI / 180);
}

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
    return (rad * 180 / M_PI);
}

/**
 * Returns the distance between two points on the Earth.
 * Direct translation from http://en.wikipedia.org/wiki/Haversine_formula
 * @param lat1d Latitude of the first point in degrees
 * @param lon1d Longitude of the first point in degrees
 * @param lat2d Latitude of the second point in degrees
 * @param lon2d Longitude of the second point in degrees
 * @return The distance between the two points in kilometers
 */
double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) {
    double lat1r, lon1r, lat2r, lon2r, u, v;
    lat1r = deg2rad(lat1d);
    lon1r = deg2rad(lon1d);
    lat2r = deg2rad(lat2d);
    lon2r = deg2rad(lon2d);
    u = sin((lat2r - lat1r)/2);
    v = sin((lon2r - lon1r)/2);
    return 2.0 * EARTH_RADIUS * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}


class AstarOracle{
private:
    int n;
    vector<vector<pair<int, W> > > edges;
    unordered_map<int, set<int>> lbl_to_ver;
    vector<pair<W, W> > coordinates;
    vector<int> labels;
    W max_speed;
//    GeographicLib::Geodesic geodesic;

    W calc_dist(int v, int u) {
//        double dist;
//        geodesic.Inverse((double)coordinates[v].first, (double)coordinates[v].second, (double)coordinates[u].first, (double)coordinates[u].second, dist);
//        printf("%lf %lf\n", dist, distanceEarth((double)coordinates[v].first, (double)coordinates[v].second, (double)coordinates[u].first, (double)coordinates[u].second));
        return distanceEarth((double)coordinates[v].first, (double)coordinates[v].second, (double)coordinates[u].first, (double)coordinates[u].second)/max_speed;
    }

public:
    AstarOracle(int nn, int m, W mmax_speed, vector<pair<int, int>>& eedges, vector<W>& weights, vector<int>& llabels, vector<pair<W, W>>& cords):
            n(nn), edges(nn), lbl_to_ver(), coordinates(cords), labels(llabels), max_speed(mmax_speed) //, geodesic(EARTH_RADIUS, 1/INVERSE_FLATTENING)

    {
        for (int i = 0; i < (int) labels.size(); i++) {
            if (labels[i] != 0) {
                lbl_to_ver[labels[i]].insert(i);
            }
        }

        for(int i=0; i<m; i++) {
            int s, t;
            tie(s, t) = eedges[i];
            edges[s].push_back(make_pair(t, weights[i]));
            edges[t].push_back(make_pair(s, weights[i]));
        }
    }

    W distanceToVertex(int s, int t) {
        vector<bool> closed(n, false);
        vector<W> dist(n, infinity), h(n);
        priority_queue< QEl, vector<QEl>, greater<QEl> > open;
        for (int i=0; i<n; i++) h[i] = 0 + calc_dist(i, t);
        dist[s] = 0;
        open.push(make_pair(0, s));
        while (!open.empty()) {
            QEl curr = open.top();
            open.pop();
            int v = curr.second;
            W d = dist[v];
            if (v == t) return dist[v];
            if (closed[v]) continue;
            closed[v] = true;
            for (pair<int, W> p : edges[v]) {
                int u = p.first;
                if (dist[u] > d + p.second) {
                    dist[u] = d + p.second;
                    closed[u] = false;
                    open.push(make_pair(dist[u] + h[u], u));
                }
            }
        }
        return -1;
    };

    pair<W, int> distanceToLabel(int s, int l) {
        vector<bool> closed(n, false);
        vector<W> dist(n, infinity), h(n, 0);
        priority_queue< QEl, vector<QEl>, greater<QEl> > open;
        vector<int> parent(n, -1);

        for (int i: lbl_to_ver[l]) {
            dist[i] = 0;
            parent[i] = i;
            open.push(make_pair(0, i));
        }
        while (!open.empty()) {
            QEl curr = open.top();
            open.pop();
            int v = curr.second;
            W d = dist[v];
            if (v == s) return make_pair(d, parent[v]);
            if (closed[v]) continue;
            closed[v] = true;
            for (pair<int, W> p : edges[v]) {
                int u = p.first;
                if (dist[u] > d + p.second) {
                    dist[u] = d + p.second;
                    parent[u] = parent[v];
                    closed[u] = false;
                    if (h[u] == infinity) h[u] = calc_dist(u, s);
                    open.push(make_pair(dist[u] + h[u], u));
                }
            }
        }
        return make_pair(-1, -1);
    };

    pair<W, pair<int,int> > distanceBetweenLabels(int l1, int l2) {

        vector<bool> closed(n, false);
        vector<W> dist(n, infinity), h(n, infinity);
        priority_queue< QEl, vector<QEl>, greater<QEl> > open;
        vector<int> parent(n, -1);

        for (int i: lbl_to_ver[l1]) {
            dist[i] = 0;
            parent[i] = i;
            open.push(make_pair(0, i));
        }

        while (!open.empty()) {
            QEl curr = open.top();
            open.pop();
            int v = curr.second;
            W d = dist[v];
            if (labels[v] == l2) return make_pair(d, make_pair(parent[v], v));
            if (closed[v]) continue;
            closed[v] = true;
            for (pair<int, W> p : edges[v]) {
                int u = p.first;
                if (dist[u] > d + p.second) {
                    dist[u] = d + p.second;
                    parent[u] = parent[v];
                    closed[u] = false;
                    if (h[u] == infinity) {
                        for (int i:lbl_to_ver[l2]) {
                            h[u] = min(h[u], calc_dist(u, i));
                        }
                    }
                    open.push(make_pair(dist[u] + h[u], u));
                }
            }
        }
        return make_pair(-1, make_pair(-1, -1));
    };

    void setLabel(int v, int l) {
        if (labels[v] != 0) {
            lbl_to_ver[labels[v]].erase(v);
        }
        labels[v] = l;
        if (l != 0) {
            lbl_to_ver[l].insert(v);
        }
    }

    int labelOf(int v) {
        return labels[v];
    }
};

#endif //VERTEX_LABEL_ORACLES_IMPLEMENTATION_MASTER_ASTAR_ORACLE_H
