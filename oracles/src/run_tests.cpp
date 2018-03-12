#include "oracle_naive.h"
#include "oracle_general_3approx.h"
#include "oracle_general_5approx.h"
#include "full_planar_oracle.h"
#include "find_union.h"

#include "read_graph.h"
#include "dijkstra_oracle.h"
#include "astar_oracle.h"
#include "osrm_oracle.h"

#include <chrono>

using std::map;


template <class T>
tuple<W, W, int, std::chrono::duration<double, std::milli> >  test_distanceToVertex(T &oracle, vector<pair<W, pair<int, int> > > &args)
{
    std::chrono::duration<double, std::milli> t_sum = std::chrono::milliseconds::zero();
    W result;
    W d_sum=0, d_res=0;
    int suc=0;
    for (auto p: args) {
        auto t1 = std::chrono::steady_clock::now();
        result = oracle.distanceToVertex(p.second.second, p.second.first);
        auto t2 = std::chrono::steady_clock::now();
        if (result != -1) {
            d_res += result;
            d_sum += p.first;
            t_sum += t2 - t1;
            suc++;
        }
    }
    return make_tuple(d_sum, d_res, suc, t_sum);
}


template <class T>
tuple<W, W, int, std::chrono::duration<double, std::milli> > test_distanceToLabel(T &oracle, vector<pair<W, pair<int, int> > > &args)
{
    std::chrono::duration<double, std::milli> t_sum = std::chrono::milliseconds::zero();
    pair<W, int> result;
    W d_sum=0, d_res=0;
    int suc=0;
    for (auto p: args) {
        auto t1 = std::chrono::steady_clock::now();
        result = oracle.distanceToLabel(p.second.second, p.second.first);
        auto t2 = std::chrono::steady_clock::now();
        if (result.first != -1) {
            d_res += result.first;
            d_sum += p.first;
            t_sum += t2 - t1;
            suc++;
        }
    }
    return make_tuple(d_sum, d_res, suc, t_sum);
}


template <class T>
tuple<W, W, int, std::chrono::duration<double, std::milli> > test_distanceBetweenLabels(T &oracle, vector<pair<W, pair<int, int> > > &args)
{
    std::chrono::duration<double, std::milli> t_sum = std::chrono::milliseconds::zero();
    pair<W, pair<int, int> > result;
    W d_sum=0, d_res=0;
    int suc=0;
    for (auto p: args) {
        auto t1 = std::chrono::steady_clock::now();
        result = oracle.distanceBetweenLabels(p.second.second, p.second.first);
        auto t2 = std::chrono::steady_clock::now();
        if (result.first != -1) {
            d_res += result.first;
            d_sum += p.first;
            t_sum += t2 - t1;
            suc++;
        }
    }
    return make_tuple(d_sum, d_res, suc, t_sum);
}


// Run tests on test function
template <class T>
std::chrono::duration<double, std::milli> run_tests(
        T &oracle, vector<pair<W, pair<int, int> > > &queries,
        tuple<W, W, int, std::chrono::duration<double, std::milli> > (test_func)(T&, vector<pair<W, pair<int, int> > >&))
{
    W d_exp, d_got;
    std::chrono::duration<double, std::milli> sum;
    int suc;
    tie(d_exp, d_got, suc, sum) = test_func(oracle, queries);
    printf("Czasy testu suma: %lf  śr: %lf  poprawnie %d/%lu\n", sum.count(), sum.count()/suc, suc, queries.size());
    printf("Wyniki nadwyżka: %f  śr nad: %f  proc nad: %f  śr dł: %f\n\n", d_got-d_exp, (d_got-d_exp)/suc, d_got/d_exp, d_exp/suc);
    return sum;
}


// Run vertex-vertex tests
template <class T>
void run_all_vv_tests(T &oracle, vector<pair<W, pair<int, int> > > tests[3][3])
{
    std::chrono::duration<double, std::milli> sum;

    sum = run_tests(oracle, tests[0][0], test_distanceToVertex);
    sum += run_tests(oracle, tests[0][1], test_distanceToVertex);
    sum += run_tests(oracle, tests[0][2], test_distanceToVertex);
    printf("SUMA CZASU dla distanceToVertex  %lf\n\n", sum.count());
}


// Run vertex-label tests
template <class T>
void run_all_vl_tests(T &oracle, vector<pair<W, pair<int, int> > > tests[3][3])
{
    std::chrono::duration<double, std::milli> sum;

    sum = run_tests(oracle, tests[1][0], test_distanceToLabel);
    sum += run_tests(oracle, tests[1][1], test_distanceToLabel);
    sum += run_tests(oracle, tests[1][2], test_distanceToLabel);
    printf("SUMA CZASU distanceToLabel  %lf\n\n", sum.count());
}


// Run label-label tests
template <class T>
void run_all_ll_tests(T &oracle, vector<pair<W, pair<int, int> > > tests[3][3])
{
    std::chrono::duration<double, std::milli> sum;

    sum =  run_tests(oracle, tests[2][0], test_distanceBetweenLabels);
    sum += run_tests(oracle, tests[2][1], test_distanceBetweenLabels);
    sum += run_tests(oracle, tests[2][2], test_distanceBetweenLabels);
    printf("SUMA CZASU distanceBetweenLabels %lf\n\n\n\n", sum.count());
}


// Run all types of tests (vertex-vertex, vertex-label, label-label)
template <class T>
void run_all_tests(T &oracle, vector<pair<W, pair<int, int> > > tests[3][3])
{
    run_all_vv_tests(oracle, tests);
    run_all_vl_tests(oracle, tests);
    run_all_ll_tests(oracle, tests);
}


int main(int argc, char* argv[]) {
    vector<pair<int, int>> edges;
    vector<char> types;
    vector<W> max_speeds;
    vector<W> distances;
    vector<int> labels;
    W max_speed;
    vector<pair<W,W> > coords;
    vector<pair<W, pair<int, int> > > tests[3][3];
    int n, m, max_label;
    W EPS = 1.;

    if (argc != 4 && argc != 5) {
        fprintf(stderr, "Usage: %s osrm-file generated-graph generated-test [(int)epsilon for planar oracle (default = 1)]>\n", argv[0]);
        return 1;
    }
    // Eps in planar oracles
    if (argc == 5) EPS = stof(argv[4]);

    // Read graph
    FILE *pfile = fopen(argv[2], "r");
    if (pfile == nullptr) {
        fprintf(stderr, "ERROR while opening graph file\n");
        return 1;
    }
    read_graph(pfile, n, m, max_label, max_speed, edges, types, max_speeds, distances, labels, coords);
    fclose(pfile);

    // Change distance to time (TODO add as program option)
    for (int i=0; i<m; i++) {
        distances[i] = distances[i] / max_speeds[i];
    }

    // Read tests
    pfile = fopen(argv[3], "r");
    if (pfile == nullptr) {
        fprintf(stderr, "ERROR while opening test file\n");
        fclose(pfile);
        return 1;
    }

    // Read 3 types of queries, vertex-vertex, vertex-label, label-label
    for (int i=0; i<3; i++) {
        int long_size, med_size, short_size, a, b;
        W d;

        // Short distance queries
        fscanf(pfile, "%d %d %d", &short_size, &med_size, &long_size);
        for (int j = 0; j < short_size; j++) {
            fscanf(pfile, "%d %d %f", &a, &b, &d);
            tests[i][0].emplace_back(make_pair(d, make_pair(a, b)));
        }

        // Medium distance queries
        for (int j = 0; j < med_size; j++) {
            fscanf(pfile, "%d %d %f", &a, &b, &d);
            tests[i][1].emplace_back(make_pair(d, make_pair(a, b)));
        }

        // Long distance queries
        for (int j = 0; j < long_size; j++) {
            fscanf(pfile, "%d %d %f", &a, &b, &d);
            tests[i][2].emplace_back(make_pair(d, make_pair(a, b)));
        }
    }
    fclose(pfile);

// TEST CONNECTED
//    FindUnion f(n);
//    for (int i=0; i<m; i++) {
//        f.unionn(edges[i].first, edges[i].second);
//    }
//    map<int ,int> hmm;
//    for (int i=0; i<n; i++) hmm[f.find(i)]++;
//
//    for (pair<int, int> e : hmm) {
//        printf("%d  %d\n", e.first, e.second);
//    }


    std::chrono::duration<double, std::milli> build_time;
    printf("FULL PLANAR\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        FullPlanarOracle oracle(n, edges, distances, labels, EPS);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs\n", build_time.count()/1000);
        run_all_vl_tests(oracle, tests);
        printf("\n\n");
    }

    printf("FULL FULL PLANAR\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        FullFullPlanarOracle oracle(n, edges, distances, labels, EPS);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs\n", build_time.count()/1000);
        run_all_ll_tests(oracle, tests);
        printf("\n\n");
    }

    printf("OSRM Oracle\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        OsrmOracle oracle(argv[1], max_label, coords, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs\n", build_time.count()/1000);
        run_all_tests(oracle, tests);
        printf("\n\n");
    }

    printf("5 APPROX QUERY\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        OracleGeneral5ApproxQuery oracle(n, edges, distances, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs\n", build_time.count()/1000);
        run_all_vl_tests(oracle, tests);
        printf("\n\n");
    }

    printf("5 APPROX UPDATE\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        OracleGeneral5ApproxUpdate oracle(n, edges, distances, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs\n", build_time.count()/1000);
        run_all_vl_tests(oracle, tests);
        printf("\n\n");
    }

    printf("3 APPROX LIGHT\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        OracleGeneral3ApproxLight oracle(n, edges, distances, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs\n", build_time.count()/1000);
        run_all_vl_tests(oracle, tests);
        run_all_ll_tests(oracle, tests);
        printf("\n\n");
    }

    printf("3 APPROX\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        OracleGeneral3Approx oracle(n, edges, distances, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs\n", build_time.count()/1000);
        run_all_vl_tests(oracle, tests);
        run_all_ll_tests(oracle, tests);
        printf("\n\n");
    }

    printf("ASTAR\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        AstarOracle oracle(n, m, max_label, max_speed, edges, distances, labels, coords);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs\n", build_time.count()/1000);
        run_all_tests(oracle, tests);
        printf("\n\n");
    }

    printf("DIJKSTRA\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        DijkstraOracle oracle(n, m, max_label, edges, distances, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs\n", build_time.count()/1000);
        run_all_tests(oracle, tests);
        printf("\n\n");
    }
}
