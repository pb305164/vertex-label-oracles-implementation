#ifndef VERTEX_LABEL_ORACLES_IMPLEMENTATION_MASTER_READ_MOJE_H
#define VERTEX_LABEL_ORACLES_IMPLEMENTATION_MASTER_READ_MOJE_H


#include "precision.h"

#include <cstdio>
#include <cassert>
#include <vector>

using std::vector;
using std::pair;
using std::make_pair;

void read_graph(FILE *pfile, int &n, int &m, int &max_label, W &max_speed, vector<pair<int, int>> &edges,
                vector<char> &edge_types, vector<W> &max_speeds, vector<W> &distances, vector<int> &labels, vector<pair<W, W> > &coords) {
    fscanf(pfile, "%d %d %f", &n, &m, &max_speed);
    max_label=0;
    edges.clear();
    distances.clear();
    max_speeds.clear();
    edge_types.clear();
    labels.clear();
    coords.clear();
    for (int i=0; i<n; i++) {
        int k;
        W lat, lon;
        fscanf(pfile, "%d %f %f", &k, &lat, &lon);
        if (k > max_label) max_label = k;
        labels.push_back(k);
        coords.push_back(make_pair(lat, lon));
    }
    max_label++;

    for (int i=0; i<m; i++) {
        int s, t, type;
        W dist, speed;
        fscanf(pfile, "%d %d %d %f %f", &s, &t, &type, &speed, &dist);
        assert(s != t);
        edges.push_back(make_pair(s, t));
        edge_types.push_back((char)type);
        max_speeds.push_back(speed);
        distances.push_back(dist);
    }
}


#endif //VERTEX_LABEL_ORACLES_IMPLEMENTATION_MASTER_READ_MOJE_H
