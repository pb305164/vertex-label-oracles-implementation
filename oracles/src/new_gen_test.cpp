#include <cstdio>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <queue>
#include <tuple>
#include <map>
#include <unordered_map>
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
vector<pair<W, W> > coords;
vector<int> labels;
vector<vector<pair<W, int>>> edges;
int n, m, max_label;
W max_speed;


// Default settings
char *graph_path = nullptr;
bool weights_as_distance = false;
int sample_count=100, sample_size=10000, allowed_queries=111, test_type=0;
float option_start=-1, option_end=-1;


bool allowed_vv() {
    return allowed_queries/100 > 0;
}

bool allowed_vl() {
    return (allowed_queries/10)%10 > 0;
}

bool allowed_ll() {
    return allowed_queries%10 > 0;
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

vector<pair<W, pair<int, int>>> get_queries_to_vertices(int new_lbl, unordered_map<int, set<int>> &lbl_to_ver) {
    vector<W> dist(n, infinity);
    PQ queue;
    for (auto v: lbl_to_ver[new_lbl]) {
        dist[v] = 0;
        queue.push(make_pair(0, v));
    }
    dijkstra(queue, dist);

    vector<pair<W, pair<int, int>>> queries;
    for (int i=0; i<n; i++) {
        if (dist[i] != 0 && dist[i] != infinity) {
            queries.push_back(make_pair(dist[i], make_pair(i, new_lbl)));
        }
    }
    return queries;
}

vector<pair<W, pair<int, int>>> get_queries_to_labels(int new_lbl, unordered_map<int, set<int>> &lbl_to_ver) {
    vector<W> dist(n, infinity);
    vector<W> lbl_dist(new_lbl+1, infinity);

    PQ queue;
    for (auto v: lbl_to_ver[new_lbl]) {
        dist[v] = 0;
        queue.push(make_pair(0, v));
    }
    dijkstra(queue, dist);

    vector<pair<W, pair<int, int>>> queries;
    for (int i=0; i<n; i++) {
        if (dist[i] != 0 && dist[i] != infinity && lbl_dist[labels[i]] > dist[i]) {
            lbl_dist[labels[i]] = dist[i];
        }
    }
    for (int i=0; i<new_lbl; i++) {
        if (lbl_dist[i] != 0 && lbl_dist[i] != infinity) queries.push_back(make_pair(lbl_dist[i], make_pair(i, new_lbl)));
    }
    return queries;
}

void calc_some_vertex_vertex_queries(vector<pair<W, pair<int, int>>> &some_vv_dist) {
    set<int> used;
    int VER_LOOP = max_label*10; // TODO check if not to much

    assert(VER_LOOP < n);

    auto ver_rng = bind(uniform_int_distribution<int>(0, n-1), default_random_engine());
    auto add_rng = bind(uniform_int_distribution<int>(0, 9), default_random_engine());
    for (int i=0; i<VER_LOOP; i++) {
        int v = ver_rng();
        while (used.find(v) != used.end()) v = ver_rng();
        used.insert(v);
        vector<W> dist(n, infinity);
        PQ queue;
        dist[v] = 0;
        queue.push(make_pair(0, v));
        dijkstra(queue, dist);

        for (int j=0; j<n; j++) if (dist[j] != 0 && add_rng() == 0) some_vv_dist.push_back(make_pair(dist[j], make_pair(v, j)));
    }

    // Sort by distance
    auto cmp = [](pair<W, pair<int, int> > &a, pair<W, pair<int, int> > &b) { return a.first < b.first; };
    sort(some_vv_dist.begin(), some_vv_dist.end(), cmp);
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

        for (int j=0; j<n; j++) if (dist[j] != 0 && dist[j] != infinity) all_dist.push_back(make_pair(dist[j], make_pair(j, i)));
    }

    // Sort by distance
    auto cmp = [](pair<W, pair<int, int> > &a, pair<W, pair<int, int> > &b) { return a.first < b.first; };
    sort(all_dist.begin(), all_dist.end(), cmp);
}


void calc_all_label_label_dist(vector<pair<W, pair<int, int>>> &all_dist) {
    vector<W> dist(n);
    unordered_map<int, W> lbl_dist;

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
        for (int j=0; j<n; j++) if (labels[j] != 0 && (lbl_dist.count(labels[j]) == 0 || dist[j] < lbl_dist[labels[j]])) lbl_dist[labels[j]] = dist[j];
        for (auto &p: lbl_dist) {
            all_dist.push_back(make_pair(p.second, make_pair(i, p.first)));
        }
    }

    // Sort by distance
    auto cmp = [](pair<W, pair<int, int> > &a, pair<W, pair<int, int> > &b) { return a.first < b.first; };
    sort(all_dist.begin(), all_dist.end(), cmp);
}


// Split queries in dist, into bucket_count buckets based on distance
void split_by_distance(vector<pair<W, pair<int, int>>> queries, vector<vector<pair<W, pair<int, int>>>> &buckets, float start_percentile = 0, float end_percentile = 1) {
    assert(queries.size()>100);
    W min_dist = (queries.back().first-queries.front().first)*start_percentile + queries.front().first;
    W max_dist = queries.back().first*end_percentile + (W)0.1;
    for (auto &q: queries) {
        if (q.first > min_dist && q.first < max_dist) {
            buckets[floor(((q.first - min_dist) * buckets.size()) / (max_dist-min_dist))].push_back(q);
        }
    }

    // TODO hmmm
    // If bucket is empty -> copy half or queries from next / prev not empty bucket
    for (size_t i=0; i<buckets.size(); i++) {
        if (buckets[i].size() == 0) {
            int neigh_b = (int)i;

            // From next
            while (neigh_b < (int)buckets.size() && buckets[neigh_b].size() == 0) {
                neigh_b++;
            }
            if (neigh_b < (int)buckets.size()) {
                for (int j = (int)buckets[neigh_b].size() - 1; j >= (int)buckets[neigh_b].size() / 2; j--) {
                    buckets[i].push_back(buckets[neigh_b][j]);
                }
            }


            // From prev
            neigh_b = i;
            while (neigh_b >= 0 && buckets[neigh_b].size() == 0) {
                neigh_b--;
            }
            if (neigh_b >= 0) {
                for (int j = (int)buckets[neigh_b].size() - 1; j >= (int)buckets[neigh_b].size() / 2; j--) {
                    buckets[i].push_back(buckets[neigh_b][j]);
                }
            }

            assert(buckets[i].size() > 0);
        }
    }
    for (auto &bck: buckets) {
        random_shuffle(bck.begin(), bck.end());
    }
}


float get_distance_to_label(int start, int l) {
    vector<W> dist(n, infinity);
    PQ queue;

    dist[start] = 0;
    queue.push(make_pair(0, start));
    while (!queue.empty()) {
        pair<W, int> curr = queue.top();
        queue.pop();
        int v = curr.second;
        W d = curr.first;
        if (labels[v] == l) return d;
        if (d != dist[v]) continue;
        for (pair<W, int> p : edges[v]) {
            int u = p.second;
            if (dist[u] > d + p.first) {
                dist[u] = d + p.first;
                queue.push(make_pair(dist[u], u));
            }
        }
    }
    return infinity;
}

float get_distance_between_labels(int l1, int l2, unordered_map<int, set<int>> &lbl_to_ver) {
    vector<W> dist(n, infinity);
    PQ queue;

    for (int i: lbl_to_ver[l1]) {
        dist[i] = 0;
        queue.push(make_pair(0, i));
    }
    while (!queue.empty()) {
        pair<W, int> curr = queue.top();
        queue.pop();
        int v = curr.second;
        W d = curr.first;
        if (labels[v] == l2) return d;
        if (d != dist[v]) continue;
        for (pair<W, int> p : edges[v]) {
            int u = p.second;
            if (dist[u] > d + p.first) {
                dist[u] = d + p.first;
                queue.push(make_pair(dist[u], u));
            }
        }
    }
    return infinity;
}

int get_border_vertex() {
    int lv = -1;
    W lx = 10000;
    for (int i=0; i<n; i++) {
        if (coords[i].first < lx && labels[i] == 0) {
            lx = coords[i].first;
            lv = i;
        }
    }
    return lv;
}

void get_sorted_by_distance_from_vertex(vector<int> &vertices, int v) {
    vector<W> dist(n, infinity);
    dist[v] = 0;
    int count=0;
    PQ queue;
    queue.push(make_pair(0, v));
    while (!queue.empty()) {
        pair<W, int> curr = queue.top();
        queue.pop();
        int v = curr.second;
        W d = curr.first;
        if (d != dist[v]) continue;
        vertices[count] = v;
        count++;
        for (pair<W, int> p : edges[v]) {
            int u = p.second;
            if (dist[u] > d + p.first) {
                dist[u] = d + p.first;
                queue.push(make_pair(dist[u], u));
            }
        }
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
            if (sscanf(argv[i], "%d", &v) && v >= 1 && v/100 <= 1 && (v/10)%10 <= 1 && v%10 <= 1) {
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
            if (sscanf(argv[i], "%f", &v) && v >= 0 && v <= 1) {
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
            if (sscanf(argv[i], "%f", &v) && v >= 0 && v <= 1) {
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

    vector<vector<pair<W, pair<int, int>>>> vv_buckets(sample_count), vl_buckets(sample_count), ll_buckets(sample_count);

    // Calculate queries
    if (allowed_vv()) {
        calc_some_vertex_vertex_queries(vv_queries);
        split_by_distance(vv_queries, vv_buckets, option_start, option_end);
    }
    if (allowed_vl()) {
        calc_all_vertex_label(vl_queries);
        split_by_distance(vl_queries, vl_buckets, option_start, option_end);
    }
    if (allowed_ll()) {
        calc_all_label_label_dist(ll_queries);
        split_by_distance(ll_queries, ll_buckets, option_start, option_end);
    }

    // Setup rng
//    size_t queries_size = vv_queries.size() + vl_queries.size() + ll_queries.size();
    auto rd = default_random_engine();
//    auto vec_rng = uniform_int_distribution<int>(0, queries_size-1);

    for (int s=0; s<sample_count; s++) {
        int i=s;

        // Not by buckets
//        // Setup query randomizers for different query types.
//        // Option start and option end define percentile range, samples start from shortest queries, end longest
//        float start_percentile = option_start + ((float)i/(float)sample_count)*(option_end-option_start);
//        float end_percentile = option_start + ((float)(i+1)/(float)sample_count)*(option_end-option_start);
//        auto vv_rng = uniform_int_distribution<int>(vv_queries.size()*start_percentile , vv_queries.size()*end_percentile - 1);
//        auto vl_rng = uniform_int_distribution<int>(vl_queries.size()*start_percentile , vl_queries.size()*end_percentile - 1);
//        auto ll_rng = uniform_int_distribution<int>(ll_queries.size()*start_percentile , ll_queries.size()*end_percentile - 1);

        // By buckets
        int vv_index = 0;
        int vl_index = 0;
        int ll_index = 0;

        // TODO hmm
//        int hmm=0;
//        while (vv_buckets[i].size() + vl_buckets[i].size() + ll_buckets[i].size() == 0) {
//            if (s+hmm >= 0 && s+hmm<sample_count) i = s+hmm;
//            if (hmm>0) hmm = -hmm;
//            else hmm = -hmm + 1;
//        }
        assert(i>=0 && i<sample_count && vv_buckets[i].size() + vl_buckets[i].size() + ll_buckets[i].size() > 0);
        auto vec_rng = uniform_int_distribution<int>(0, vv_buckets[i].size() + vl_buckets[i].size() + ll_buckets[i].size() - 1);

        // Not by buckets
//        for (int j=0; j<sample_size; j++) {
//            // Select vector to random query from
//            int r = vec_rng(rd);
//            if (r < (int)vv_queries.size()) {
//                int q = vv_rng(rd);
//                printf("0 %d %d %f\n", vv_queries[q].second.first, vv_queries[q].second.second, vv_queries[q].first);
//            } else if (r < (int)vv_queries.size() + (int)vl_queries.size()){
//                int q = vl_rng(rd);
//                printf("1 %d %d %f\n", vl_queries[q].second.second, vl_queries[q].second.first, vl_queries[q].first);
//            } else {
//                int q = ll_rng(rd);
//                printf("2 %d %d %f\n", ll_queries[q].second.first, ll_queries[q].second.second, ll_queries[q].first);
//            }
//        }

        // By buckets
        for (int j=0; j<sample_size; j++) {
            // Select vector to random query from
            int r = vec_rng(rd);
            if (r < (int)vv_buckets[i].size()) {
                // Query type (0: vertex-vertx), start vertex, end vertex, answer
                printf("0 %d %d %f\n", vv_buckets[i][vv_index].second.first, vv_buckets[i][vv_index].second.second, vv_buckets[i][vv_index].first);
                if (++vv_index >= (int)vv_buckets[i].size()) vv_index = 0;
            } else if (r < (int)vv_buckets[i].size() + (int)vl_buckets[i].size()){
                // Query type (1: vertex-label), start vertex, end label, answer
                printf("1 %d %d %f\n", vl_buckets[i][vl_index].second.first, vl_buckets[i][vl_index].second.second, vl_buckets[i][vl_index].first);
                if (++vl_index >= (int)vl_buckets[i].size()) vl_index = 0;
            } else {
                // Query type (2: label-label), start label, end label, answer
                printf("2 %d %d %f\n", ll_buckets[i][ll_index].second.first, ll_buckets[i][ll_index].second.second, ll_buckets[i][ll_index].first);
                if (++ll_index >= (int)ll_buckets[i].size()) ll_index = 0;
            }
        }
    }
}

void generate_distance_vs_set_label_test() {
    vector<pair<W, pair<int, int>>> vv_queries;
    unordered_map<int, set<int>> lbl_to_ver;
    if (option_start == -1) option_start = 0;
    if (option_end == -1) option_end = 1;

    // Calculate queries
    if (allowed_vv()) {
        calc_some_vertex_vertex_queries(vv_queries);
    }

    for (int i=0; i < (int)labels.size(); i++) {
        if (labels[i] > 0) {
            lbl_to_ver[labels[i]].insert(i);
        }
    }

    auto rd = default_random_engine();
    auto vv_query_rng = uniform_int_distribution<int>(0, (int)vv_queries.size()-1);
    auto distance_query_type_rng = uniform_int_distribution<int>(0, 2);
    auto vertex_rng = uniform_int_distribution<int>(0, n-1);
    auto label_rng = uniform_int_distribution<int>(1, (int)1.2*max_label-2);

    for (int i=0; i<sample_count; i++) {

        // Setup query randomizer for query type
        float setLabel_probability = option_start + ((float)i/(float)(sample_count-1))*(option_end-option_start);
        auto query_type_rng = uniform_real_distribution<float>(0, 1); // TODO maybe other than linear change in probability

        for (int j=0; j<sample_size; j++) {
            // Select vector to random query from
            float q = query_type_rng(rd);
            if (q < setLabel_probability) {
                // Generate set label query
                int l = label_rng(rd), v = vertex_rng(rd);
                if (labels[v] != 0) {
                    lbl_to_ver[labels[v]].erase(v);
                }
                if (l != 0) {
                    lbl_to_ver[l].insert(v);
                }
                labels[v] = l;
                printf("3 %d %d 0\n", v, l);

            } else {
                // Generate distance query;
                int query_type = distance_query_type_rng(rd);
                while (!((query_type == 0 && allowed_vv()) || (query_type == 1 && allowed_vl()) || (query_type == 2 && allowed_ll()))) {
                    query_type = distance_query_type_rng(rd);
                }
                if (query_type == 0) {
                    int vvq = vv_query_rng(rd);
                    // Query type (0: vertex-vertx), start vertex, end vertex, answer
                    printf("0 %d %d %f\n", vv_queries[vvq].second.first, vv_queries[vvq].second.second, vv_queries[vvq].first);
                } else if (query_type == 1){
                    int l = label_rng(rd), v = vertex_rng(rd);
                    while (lbl_to_ver.count(l) == 0 || lbl_to_ver[l].size() == 0) l = label_rng(rd);
                    // Query type (1: vertex-label), start vertex, end label, answer
                    printf("1 %d %d %f\n", v, l, get_distance_to_label(v, l));
                } else {
                    int l1 = label_rng(rd), l2 = label_rng(rd);
                    while (lbl_to_ver.count(l1) == 0 || lbl_to_ver[l1].size() == 0) l1 = label_rng(rd);
                    while (lbl_to_ver.count(l2) == 0 || lbl_to_ver[l2].size() == 0) l2 = label_rng(rd);
                    // Query type (2: label-label), start label, end label, answer
                    printf("2 %d %d %f\n", l1, l2, get_distance_between_labels(l1, l2, lbl_to_ver));
                }
            }
        }
    }
}

void generate_new_label_test() {
    vector<pair<W, pair<int, int>>> queries_v, queries_l;
    vector<int> closest(n);
    unordered_map<int, set<int>> lbl_to_ver;

    for (int i=0; i < (int)labels.size(); i++) {
        if (labels[i] > 0) {
            lbl_to_ver[labels[i]].insert(i);
        }
    }

    // Get vertices most on both left and rigth
    int lv = get_border_vertex(), new_lbl = ++max_label;

    // Get list of vertices sorted by distance from border vertices
    get_sorted_by_distance_from_vertex(closest, lv);

    auto rd = default_random_engine();
    auto bucket_rng = uniform_int_distribution<int>(0, 9);
    auto query_type_rng = uniform_int_distribution<int>(1, 2);

    int query_index_v[10], query_index_l[10];
    for (int i=0; i<10; i++) {
        query_index_v[i] = 0;
        query_index_l[i] = 0;
    }

    int ver_count = 0;

    for (int i=0; i<sample_count; i++) {

        float labeled_size = ((float)(i+1)/(float)sample_count)*(option_end)*n;
        int j=0;
        while (j<sample_size && ver_count < (int)ceil(labeled_size)) {
            // Generate set label query
            int v = closest[ver_count];
            ver_count++;
            if (labels[v] != 0) {
                lbl_to_ver[labels[v]].erase(v);
            }
            lbl_to_ver[new_lbl].insert(v);
            labels[v] = new_lbl;
            printf("3 %d %d 0\n", v, new_lbl);
            j++;
        }

        vector<vector<pair<W, pair<int, int>>>> buckets_v(10), buckets_l(10);

        // Get distances from new label to vertices
        if (allowed_vl()) {
            split_by_distance(get_queries_to_vertices(new_lbl, lbl_to_ver), buckets_v);
        }

        // Get distances from new label to labels
        if (allowed_ll()) {
            split_by_distance(get_queries_to_labels(new_lbl, lbl_to_ver), buckets_l);
        }

        while (j<sample_size) {
            int b = bucket_rng(rd);
            int qt = query_type_rng(rd);
            while (!((qt == 1 && allowed_vl()) || (qt == 2 && allowed_ll()))) {
                qt = query_type_rng(rd);
            }
            // Generate distance query;
            if (qt == 1) {
                auto q = buckets_v[b][query_index_v[b]];
                if (++query_index_v[b] >= (int)buckets_v[b].size()) query_index_v[b] = 0;
                // Query type (1: vertex-label), start label, end label, answer
                printf("1 %d %d %f\n", q.second.first, q.second.second, q.first);
            } else {
                auto q = buckets_l[b][query_index_l[b]];
                if (++query_index_l[b] >= (int)buckets_l[b].size()) query_index_l[b] = 0;

                // Query type (2: label-label), start label, end label, answer
                printf("2 %d %d %f\n", q.second.first, q.second.second, q.first);
            }
            j++;
        }
    }
}

void generate_build_labels_test(vector<pair<int, int>> &set_labels) {
    vector<pair<W, pair<int, int>>> vv_queries;
    vector<vector<pair<W, pair<int, int>>>> vv_buckets(10);
    unordered_map<int, set<int>> lbl_to_ver;

    option_start = 0;
    option_end = 1;

    // Calculate queries
    if (allowed_vv()) {
        calc_some_vertex_vertex_queries(vv_queries);
        split_by_distance(vv_queries, vv_buckets, option_start, option_end);
    }

    auto query_type_rng = uniform_int_distribution<int>(0, 2);
    auto rd = default_random_engine();
    auto bucket_rng = uniform_int_distribution<int>(0, 9);

    int query_index_vv[10], query_index_vl[10], query_index_ll[10];
    for (int i=0; i<10; i++) {
        query_index_vv[i] = 0;
    }

    int lbl_count=0;
    for (int i=0; i<sample_count; i++) {
        float to_label = ((float)(i+1)/(float)sample_count)*set_labels.size();
        int j=0;
        while (j<sample_size && lbl_count < (int)ceil(to_label) && lbl_count < (int)set_labels.size()) {
            // Generate set label query
            int v = set_labels[lbl_count].first;
            int l = set_labels[lbl_count].second;

            lbl_count++;
            assert(labels[v] == 0);
            lbl_to_ver[l].insert(v);
            labels[v] = l;
            printf("3 %d %d 0\n", v, l);
            j++;
        }

        // Calculate distances to labels after changes
        vector<pair<W, pair<int, int>>> vl_queries, ll_queries;
        vector<vector<pair<W, pair<int, int>>>> vl_buckets(10), ll_buckets(10);

        if (allowed_vl()) {
            calc_all_vertex_label(vl_queries);
            split_by_distance(vl_queries, vl_buckets, option_start, option_end);
        }
        if (allowed_ll()) {
            calc_all_label_label_dist(ll_queries);
            split_by_distance(ll_queries, ll_buckets, option_start, option_end);
        }
        for (int i=0; i<10; i++) {
            query_index_vl[i] = 0;
            query_index_ll[i] = 0;
        }

        int b = bucket_rng(rd);

        while (j<sample_size) {
            int qt = query_type_rng(rd);
            while (!((qt == 0 && allowed_vv()) || (qt == 1 && allowed_vl()) || (qt == 2 && allowed_ll()))) {
                qt = query_type_rng(rd);
            }

            if (qt == 0) {
                // Query type (0: vertex-vertx), start vertex, end vertex, answer
                auto q = vv_buckets[b][query_index_vv[b]];
                printf("0 %d %d %f\n", q.second.first, q.second.second, q.first);
                if (++query_index_vv[b] >= (int)vv_buckets[b].size()) query_index_vv[b] = 0;
            } else if (qt == 1){
                auto q = vl_buckets[b][query_index_vl[b]];
                assert(abs(get_distance_to_label(q.second.second, q.second.first) - q.first) < 0.1);
                // Query type (1: vertex-label), start vertex, end label, answer
                printf("1 %d %d %f\n", q.second.first, q.second.second, q.first);
                if (++query_index_vl[b] >= (int)vl_buckets[b].size()) query_index_vl[b] = 0;
            } else {
                auto q = ll_buckets[b][query_index_ll[b]];
                assert(abs(get_distance_between_labels(q.second.second, q.second.first, lbl_to_ver) - q.first) < 0.1);
                // Query type (2: label-label), start label, end label, answer
                printf("2 %d %d %f\n", q.second.first, q.second.second, q.first);
                if (++query_index_ll[b] >= (int)ll_buckets[b].size()) query_index_ll[b] = 0;
            }
            j++;
        }
    }
}


int main(int argc, char* argv[])
{
    parse_program_options(argc, argv);

    if (option_end != -1 && option_start > option_end) {
        fprintf(stderr, "Error start constraint must be lower than end constraint");
    }

    FILE *pfile;
    pfile = fopen(graph_path, "r");

    if (pfile == nullptr) {
        fprintf(stderr, "ERROR while opening test file\n");
        return 1;
    }

    // Read graph
    {
        vector<pair<int, int>> eedges;
        read_graph(pfile, n, m, max_label, max_speed, eedges, types, max_speeds, distances, labels, coords);
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

    switch (test_type) {
        case 0: {
            // Generate standard, distance queries only test
            printf("%d %d %d %d %d\n", sample_count, sample_size, test_type, allowed_queries, weights_as_distance);
            generate_query_test();
            break;
        }
        case 1: {
            // Generate test with changing ratio distance queries to set label queries
            printf("%d %d %d %d %d\n", sample_count, sample_size, test_type, allowed_queries, weights_as_distance);
            generate_distance_vs_set_label_test();
            break;
        }
        case 2: {
            // Generate test with setting new label in one place then removing it
            if (option_end > 0.1) {
                fprintf(stderr, "Error end constraint cannot by higher than 0.1 for test type 2\n");
                exit(1);
            }
            if (!allowed_vl() && !allowed_ll()) {
                fprintf(stderr, "Error vertex-label or label-label queries must by allowed for test type 2\n");
                exit(1);
            }

            if (option_end == -1) option_end = 0.1;
            sample_size += ceil((option_end*n)/sample_count);

            printf("%d %d %d %d %d\n", sample_count, sample_size, test_type, allowed_queries, weights_as_distance);
            generate_new_label_test();
            break;
        }
        case 3: {
            // Generate test starting with clear graph, then add labels to it
            vector<pair<int, int>> set_labels;
            for (int i=0; i<n; i++) {
                if (labels[i] > 0) {
                    set_labels.emplace_back(i, labels[i]);
                    labels[i] = 0;
                }
            }
            random_shuffle(set_labels.begin(), set_labels.end());
            sample_size += ceil(set_labels.size()/sample_count);

            printf("%d %d %d %d %d\n", sample_count, sample_size, test_type, allowed_queries, weights_as_distance);
            generate_build_labels_test(set_labels);
            break;
        }
    }
}
