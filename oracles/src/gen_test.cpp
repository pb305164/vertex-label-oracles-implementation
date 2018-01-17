#include <cstdio>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <queue>
#include <tuple>
#include <map>
#include <random>
#include <functional>

#include "read_graph.h"
#include <set>

using namespace std;

typedef pair<W, int> QEl;
typedef priority_queue<QEl, vector<QEl>, greater<QEl>> PQ;

vector<vector<pair<W, int>>> edges;
vector<int> labels;
int n, max_label;


void print_random(int short_size, int med_size, int long_size, vector<pair<W, pair<int, int> > > &all_dist) {

    // procenty decydujące które odległości są długie / średnie / krótkie
    const float long_per_start = 0.95, long_per_end = 1, med_per_start = 0.6, med_per_end = 0.8, short_per_start = 0.25, short_per_end = 0.5;

    const int long_start  = all_dist.size()*long_per_start-1,  long_end =  all_dist.size()*long_per_end-1,
              med_start   = all_dist.size()*med_per_start-1,   med_end =   all_dist.size()*med_per_end-1,
              short_start = all_dist.size()*short_per_start-1, short_end = all_dist.size()*short_per_end-1;

    auto cmp = [](pair<W, pair<int, int> > &a, pair<W, pair<int, int> > &b) { return a.first < b.first; };
    sort(all_dist.begin(), all_dist.end(), cmp);

    printf("%d %d %d\n", short_size, med_size, long_size);
    auto rd = default_random_engine();
    auto rng = uniform_int_distribution<int>(short_start, short_end);
    for (int i=0; i<short_size; i++) {
        auto e = all_dist[rng(rd)];
        printf("%d %d %f\n", e.second.first, e.second.second, e.first);

    }

    rng = uniform_int_distribution<int>(med_start, med_end);
    for (int i=0; i<med_size; i++) {
        auto e = all_dist[rng(rd)];
        printf("%d %d %f\n", e.second.first, e.second.second, e.first);
    }

    rng = uniform_int_distribution<int>(long_start, long_end);
    for (int i=0; i<long_size; i++) {
        auto e = all_dist[rng(rd)];
        printf("%d %d %f\n", e.second.first, e.second.second, e.first);
    }
}


void dijkstra(PQ &queue, vector<W> &dist)
{
    while (!queue.empty()) {
        pair<W, int> curr = queue.top();
        queue.pop();
        int v = curr.second;
        W d = curr.first;
        if (d != dist[v]) continue;
        for (pair<W, int> p : edges[v]) {
            int u = p.second;
            if (dist[u] > d + p.first) {
                dist[u] = d + p.first;
                queue.push(make_pair(dist[u], u));
            }
        }
    }
}


void print_vertex_vertex_queries(int VER_LOOP=1000, int short_size = 50000, int med_size = 10000, int long_size = 2000)
{
    vector<W> dist(n);
    vector<pair<W, pair<int, int> > > all_dist;
    set<int> used;

    assert(VER_LOOP < n);

    auto ver_rng = bind(uniform_int_distribution<int>(0, n-1), default_random_engine());
    for (int i=0; i<VER_LOOP; i++) {
        PQ queue;
        for (int j=0; j<n; j++) dist[j] = infinity;
        int v = ver_rng();
        while (used.find(v) != used.end()) v = ver_rng();
        used.insert(v);
        dist[v] = 0;
        queue.push(make_pair(0, v));
        dijkstra(queue, dist);

        for (int j=0; j<n; j++) if (dist[j] != 0) all_dist.push_back(make_pair(dist[j], make_pair(v, j)));
    }

    print_random(short_size, med_size, long_size, all_dist);
}


void print_vertex_label_queries(int short_size = 50000, int med_size = 10000, int long_size = 2000)
{
    vector<W> dist(n);
    vector<pair<W, pair<int, int> > > all_dist;

    for (int i=0; i<max_label; i++) {
        PQ queue;

        for (int j=0; j<n; j++) {
            if (labels[j] != i) dist[j] = infinity;
            else {
                dist[j] = 0;
                queue.push(make_pair(0, j));
            }
        }
        dijkstra(queue, dist);

        for (int j=0; j<n; j++) if (dist[j] != 0) all_dist.push_back(make_pair(dist[j], make_pair(i, j)));
    }

    print_random(short_size, med_size, long_size, all_dist);
}


void print_label_label_queries(int short_size = 50000, int med_size = 10000, int long_size = 2000)
{
    vector<W> dist(n);
    W lbl_dist[max_label];
    vector<pair<W, pair<int, int> > > all_dist;

    for (int i=0; i<max_label; i++) {
        PQ queue;

        for (int j=0; j<n; j++) {
            if (labels[j] != i) dist[j] = infinity;
            else {
                dist[j] = 0;
                queue.push(make_pair(0, j));
            }
        }
        dijkstra(queue, dist);
        for (int j=0; j<max_label; j++) lbl_dist[j] = infinity;
        for (int j=0; j<n; j++) if (dist[j] < lbl_dist[labels[j]]) lbl_dist[labels[j]] = dist[j];
        for (int j=0; j<max_label; j++) if (lbl_dist[j] != 0) all_dist.push_back(make_pair(lbl_dist[j], make_pair(i, j)));
    }

    print_random(short_size, med_size, long_size, all_dist);
}

int main(int argc, char* argv[])
{
    vector<pair<int, int>> eedges;
    vector<char> types;
    vector<W> max_speeds;
    vector<W> distances;
    vector<pair<W, W> > cords;
    W max_speed;
    int m;
    pair<W, W> min(infinity, infinity);

    if (argc != 2) {
        fprintf(stderr, "Usage ./gen-test <generated-graph>\n");
        return 1;
    }
    
    FILE *pfile = fopen(argv[1], "r");
    if (pfile == nullptr) {
        fprintf(stderr, "ERROR while opening test file\n");
        return 1;
    }
    read_graph(pfile, n, m, max_label, max_speed, eedges, types, max_speeds, distances, labels, cords);
    fclose(pfile);

    edges.resize(n);

//    edges by:

//     DISTANCE
//    for (size_t i=0; i<eedges.size(); i++) {
//        edges[eedges[i].first].push_back(make_pair(distances[i], eedges[i].second));
//        edges[eedges[i].second].push_back(make_pair(distances[i], eedges[i].first));
//    }

//    TIME
    for (size_t i=0; i<eedges.size(); i++) {
        edges[eedges[i].first].push_back(make_pair(distances[i]/max_speeds[i], eedges[i].second));
        edges[eedges[i].second].push_back(make_pair(distances[i]/max_speeds[i], eedges[i].first));
    }

    eedges.clear();
    eedges.shrink_to_fit();

    print_vertex_vertex_queries(1000, 1000, 500, 200);
    print_vertex_label_queries(1000, 500, 200);
    print_label_label_queries(1000, 500, 200);
}
