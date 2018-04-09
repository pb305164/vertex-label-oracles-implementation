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
#include <cstring>

using namespace std;

typedef pair<W, int> QEl;
typedef priority_queue<QEl, vector<QEl>, greater<QEl>> PQ;

vector<char> types;
vector<W> max_speeds;
vector<W> distances;
vector<pair<W, W> > cords;
vector<int> labels;
vector<vector<pair<W, int>>> edges;
int n, m, max_label;
W max_speed;


// Default settings
char *graph_path = nullptr;
bool weights_as_distance = false;
int sample_count=100, sample_size=10000, allowed_queries=111, test_type=0;
float option_start=-1, option_end=-1;


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


void calc_some_vertex_vertex_queries(vector<pair<W, pair<int, int>>> &some_vv_dist) {
    vector<W> dist(n);
    set<int> used;
    int VER_LOOP = max_label*10; // TODO check if not to much

    assert(VER_LOOP < n);

    auto ver_rng = bind(uniform_int_distribution<int>(0, n-1), default_random_engine());
    auto add_rng = bind(uniform_int_distribution<int>(0, 9), default_random_engine());
    for (int i=0; i<VER_LOOP; i++) {
        PQ queue;
        for (int j=0; j<n; j++) dist[j] = infinity;
        int v = ver_rng();
        while (used.find(v) != used.end()) v = ver_rng();
        used.insert(v);
        dist[v] = 0;
        queue.push(make_pair(0, v));
        dijkstra(queue, dist);

        for (int j=0; j<n; j++) if (dist[j] != 0 && add_rng() == 0) some_vv_dist.push_back(make_pair(dist[j], make_pair(v, j)));
    }
}


void calc_all_vertex_label(vector<pair<W, pair<int, int>>> &all_dist) {
    vector<W> dist(n);

    for (int i=1; i<max_label; i++) {
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
}


void calc_all_label_label_dist(vector<pair<W, pair<int, int>>> &all_dist) {
    vector<W> dist(n);
    W lbl_dist[max_label];

    for (int i=1; i<max_label; i++) {
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
}

void print_help() {
    fprintf(stderr,
            "Usage: ./new_gen_test [options] generated_graph\n\n"
                    "Options:\n"
                    "  -h           : Print this help message\n\n"
                    "  -D           : Change edge weights to distance instead of time\n\n"
                    "  -S  int      : Number of samples in test (default 100)\n\n"
                    "  -SS int      : Number of queries in sample (default 10000)\n\n"
                    "  -Q  mask     : Allowed distance query types, depending on 1 or 0 type is allowed or not:\n"
                    "           default -> 111: all queries allowed\n"
                    "                      100: only vertex-vertex\n"
                    "                      010: only vertex-label\n"
                    "                      001: only label-label\n\n"
                    "  -T  int      : Type of test to generate:\n"
                    "           default -> 0: distance queries only\n"
                    "                      1: distance queries vs setting labels\n"
                    "                      2: Add new labels close to each other, then remove them\n"
                    "                      3: Start with graph without labels, add them gradually\n\n"
                    "  -CS float    : Start value of constraints on test\n\n"
                    "  -CE float    : End value of constraints on test\n\n"

    );

}

void parse_program_options(int argc, char* argv[]) {
    for (int i=1; i<argc;) {
        if (strcmp("-h", argv[i]) == 0) {
            print_help();
            exit(0);
        } else if (strcmp("-D", argv[i]) == 0) {
            weights_as_distance = true;
        } else if (strcmp("-S", argv[i]) == 0) {
            i++;
            int v=-1;
            if (i >= argc) {
                fprintf(stderr, "Error while parsing -S option, no value\n\n");
                print_help();
                exit(1);
            }
            if (sscanf(argv[i], "%d", &v) && v >= 0) {
                sample_count = v;
            } else {
                fprintf(stderr, "Error while parsing -S option, invalid value\n\n");
                print_help();
                exit(1);
            }
        } else if (strcmp("-SS", argv[i]) == 0) {
            i++;
            int v=-1;
            if (i >= argc) {
                fprintf(stderr, "Error while parsing -SS option, no value\n\n");
                print_help();
                exit(1);
            }
            if (sscanf(argv[i], "%d", &v) && v >= 0) {
                sample_size = v;
            } else {
                fprintf(stderr, "Error while parsing -SS option, invalid value\n\n");
                print_help();
                exit(1);
            }
        }
        else if (strcmp("-Q", argv[i]) == 0) {
            i++;
            int v=-1;
            if (i >= argc) {
                fprintf(stderr, "Error while parsing -Q option, no value\n\n");
                print_help();
                exit(1);
            }
            if (sscanf(argv[i], "%d", &v) && v >= 0 && v/100 <= 1 && (v/10)%10 <= 1 && v%10 <= 1) {
                allowed_queries = v;
            } else {
                fprintf(stderr, "Error while parsing -Q option, invalid value\n\n");
                print_help();
                exit(1);
            }
        } else if (strcmp("-T", argv[i]) == 0) {
            i++;
            int v=-1;
            if (i >= argc) {
                fprintf(stderr, "Error while parsing -T option, no value\n\n");
                print_help();
                exit(1);
            }
            if (sscanf(argv[i], "%d", &v) && v >= 0 && v < 4) {
                test_type = v;
            } else {
                fprintf(stderr, "Error while parsing -T option, invalid value\n\n");
                print_help();
                exit(1);
            }
        } else if (strcmp("-CS", argv[i]) == 0) {
            i++;
            if (i >= argc) {
                fprintf(stderr, "Error while parsing -CS option, no value\n\n");
                print_help();
                exit(1);
            }
            float v=-1;
            if (sscanf(argv[i], "%f", &v) && v >= 0) {
                option_start = v;
            } else {
                fprintf(stderr, "Error while parsing -CS option, invalid value\n\n");
                print_help();
                exit(1);
            }
        } else if (strcmp("-CE", argv[i]) == 0) {
            i++;
            float v=-1;
            if (i >= argc) {
                fprintf(stderr, "Error while parsing -CE option, no value\n\n");
                print_help();
                exit(1);
            }
            if (sscanf(argv[i], "%f", &v) && v >= 0) {
                option_end = v;
            } else {
                fprintf(stderr, "Error while parsing -CE option, invalid value\n\n");
                print_help();
                exit(1);
            }
        } else {
            if (graph_path == nullptr) {
                graph_path = argv[i];
            } else {
                fprintf(stderr, "Error while parsing arguments %d\n", i);
                exit(1);
            }
        }
        i++;
    }
    if (graph_path == nullptr) {
        fprintf(stderr, "Error no path to graph specified\n\n");
        print_help();
        exit(1);
    }
}

void generate_query_test() {
    vector<pair<W, pair<int, int>>> vv_queries, vl_queries, ll_queries;
    if (option_start == -1) option_start = 0;
    if (option_end == -1) option_end = 1;

    // Calculate queries
    if (allowed_queries%10 == 1) {
        calc_some_vertex_vertex_queries(vv_queries);
    }
    if ((allowed_queries/10)%10 == 1) {
        calc_all_vertex_label(vl_queries);
    }
    if (allowed_queries/100 == 1) {
        calc_all_label_label_dist(ll_queries);
    }

    // Sort by distance
    auto cmp = [](pair<W, pair<int, int> > &a, pair<W, pair<int, int> > &b) { return a.first < b.first; };
    sort(vv_queries.begin(), vv_queries.end(), cmp);
    sort(vl_queries.begin(), vl_queries.end(), cmp);
    sort(ll_queries.begin(), ll_queries.end(), cmp);

    // Setup rng
    size_t queries_size = vv_queries.size() + vl_queries.size() + ll_queries.size();
    auto rd = default_random_engine();
    auto vec_rng = uniform_int_distribution<int>(0, queries_size-1);

    for (int i=0; i<sample_count; i++) {

        // Setup query randomizers for different query types.
        // Option start and option end define percentile range, samples start from shortest queries, end longest
        float start_percentile = option_start + ((float)i/(float)sample_count)*(option_end-option_start);
        float end_percentile = option_start + ((float)(i+1)/(float)sample_count)*(option_end-option_start);
        auto vv_rng = uniform_int_distribution<int>(vv_queries.size()*start_percentile , vv_queries.size()*end_percentile - 1);
        auto vl_rng = uniform_int_distribution<int>(vl_queries.size()*start_percentile , vl_queries.size()*end_percentile - 1);
        auto ll_rng = uniform_int_distribution<int>(ll_queries.size()*start_percentile , ll_queries.size()*end_percentile - 1);

        for (int j=0; j<sample_size; j++) {
            // Select vector to random query from
            int r = vec_rng(rd);
            if (r < (int)vv_queries.size()) {
                int q = vv_rng(rd);
                // Query type (0: vertex-vertx), start vertex, end vertex, answer
                printf("0 %d %d %f\n", vv_queries[q].second.first, vv_queries[q].second.second, vv_queries[q].first);
            } else if (r < (int)vv_queries.size() + (int)vl_queries.size()){
                int q = vl_rng(rd);
                // Query type (0: vertex-vertx), start vertex, end vertex, answer
                printf("1 %d %d %f\n", vl_queries[q].second.second, vl_queries[q].second.first, vl_queries[q].first);
            } else {
                int q = ll_rng(rd);
                // Query type (0: vertex-vertx), start vertex, end vertex, answer
                printf("2 %d %d %f\n", ll_queries[q].second.first, ll_queries[q].second.second, ll_queries[q].first);
            }
        }
    }
}

void generate_distance_vs_set_label_test() {
    // TODO
}

void generate_new_label_test() {
    // TODO
}

void generate_build_labels_test() {
    // TODO
}


int main(int argc, char* argv[])
{
    parse_program_options(argc, argv);
//    printf("sc: %d   ss: %d   alowed: %d   type: %d   WasDist: %d  optionSt: %f  optionEn: %f   path: %s\n",
//    sample_count, sample_size, allowed_queries, test_type, weights_as_distance, option_start, option_end, graph_path);
//    return 0;

    FILE *pfile;
    pfile = fopen(graph_path, "r");

    if (pfile == nullptr) {
        fprintf(stderr, "ERROR while opening test file\n");
        return 1;
    }

    // Read graph
    {
        vector<pair<int, int>> eedges;
        read_graph(pfile, n, m, max_label, max_speed, eedges, types, max_speeds, distances, labels, cords);
        fclose(pfile);

        edges.resize(n);

        if (weights_as_distance) {
            // Weights as distance
            for (size_t i = 0; i < eedges.size(); i++) {
                edges[eedges[i].first].push_back(make_pair(distances[i], eedges[i].second));
                edges[eedges[i].second].push_back(make_pair(distances[i], eedges[i].first));
            }
        } else {
            // Weights as time
            for (size_t i = 0; i < eedges.size(); i++) {
                edges[eedges[i].first].push_back(make_pair(distances[i] / max_speeds[i], eedges[i].second));
                edges[eedges[i].second].push_back(make_pair(distances[i] / max_speeds[i], eedges[i].first));
            }
        }
    }

    // Print sample count, sample size, type, allowed queries
    printf("%d %d %d %d %d\n", sample_count, sample_size, test_type, allowed_queries, weights_as_distance);
    switch (test_type) {
        case 0: {
            // Generate standard, distance queries only test
            generate_query_test();
            break;
        }
        case 1: {
            // Generate test with changing ratio distance queries to set label queries
            generate_distance_vs_set_label_test();
            break;
        }
        case 2: {
            // Generate test with setting new label in one place then removing it
            generate_new_label_test();
            break;
        }
        case 3: {
            // Generate add labels to clean nodes test
            generate_build_labels_test();
            break;
        }
    }
}
