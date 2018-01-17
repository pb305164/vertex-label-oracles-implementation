#include "oracle_naive.h"
#include "oracle_general_3approx.h"
#include "oracle_general_5approx.h"
#include "full_planar_oracle.h"
#include "find_union.h"

#include "read_graph.h"
#include "dijkstra_oracle.h"
#include "astar_oracle.h"
#include "osrm_oracle.h"

#include <cstdio>
#include <cassert>
#include <chrono>
#include <map>
#include <vector>
#include <ratio>


using std::map;

template <class T>
tuple<W, W, int, std::chrono::duration<double, std::milli> >  test_distanceToVertex(T &oracle, vector<pair<W, pair<int, int> > > &args)
{
    std::chrono::duration<double, std::milli> t_sum = std::chrono::milliseconds::zero();
//    pair<W, int> result;
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

template <class T>
std::chrono::duration<double, std::milli> run_tests(T &oracle, vector<pair<W, pair<int, int> > > &zapytania,
                                                    tuple<W, W, int, std::chrono::duration<double, std::milli> > (test_func)(T&, vector<pair<W, pair<int, int> > >&))
{
    W d_exp, d_got;
    std::chrono::duration<double, std::milli> sum;
    int suc;
    tie(d_exp, d_got, suc, sum) = test_func(oracle, zapytania);
    printf("Czasy testu suma: %lf  śr: %lf  poprawnie %d/%lu\n", sum.count(), sum.count()/suc, suc, zapytania.size());
    printf("Wyniki nadwyżka: %f  śr nad: %f  proc nad: %f  śr dł: %f\n\n", d_got-d_exp, (d_got-d_exp)/suc, d_got/d_exp, d_exp/suc);
    return sum;
}

// vertex-vertex
template <class T>
void run_all_vv_tests(T &oracle, vector<pair<W, pair<int, int> > > tests[3][3])
{
    std::chrono::duration<double, std::milli> sum;

    sum = run_tests(oracle, tests[0][0], test_distanceToVertex);
    sum += run_tests(oracle, tests[0][1], test_distanceToVertex);
    sum += run_tests(oracle, tests[0][2], test_distanceToVertex);
    printf("SUMA CZASU dla distanceToVertex  %lf\n\n", sum.count());
}

// vertex-label
template <class T>
void run_all_vl_tests(T &oracle, vector<pair<W, pair<int, int> > > tests[3][3])
{
    std::chrono::duration<double, std::milli> sum;

    sum = run_tests(oracle, tests[1][0], test_distanceToLabel);
    sum += run_tests(oracle, tests[1][1], test_distanceToLabel);
    sum += run_tests(oracle, tests[1][2], test_distanceToLabel);
    printf("SUMA CZASU distanceToLabel  %lf\n\n", sum.count());
}


// label-label
template <class T>
void run_all_ll_tests(T &oracle, vector<pair<W, pair<int, int> > > tests[3][3])
{
    std::chrono::duration<double, std::milli> sum;

    sum =  run_tests(oracle, tests[2][0], test_distanceBetweenLabels);
    sum += run_tests(oracle, tests[2][1], test_distanceBetweenLabels);
    sum += run_tests(oracle, tests[2][2], test_distanceBetweenLabels);
    printf("SUMA CZASU distanceBetweenLabels %lf\n\n\n\n", sum.count());
}



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

    if (argc != 4) {
        fprintf(stderr, "Usage: %s <osrm-file> <generated-graph> <generated-test>\n", argv[0]);
        return 1;
    }

    // READ GRAPH
    FILE *pfile = fopen(argv[2], "r");
    if (pfile == nullptr) {
        fprintf(stderr, "ERROR while opening graph file\n");
        return 1;
    }
    read_graph(pfile, n, m, max_label, max_speed, edges, types, max_speeds, distances, labels, coords);
    fclose(pfile);

    // CHANGE DISTANCE TO TIME (TODO prog flag)
    for (int i=0; i<m; i++) {
        distances[i] = distances[i] / max_speeds[i];
    }

    // READ TEST
    pfile = fopen(argv[3], "r");
    if (pfile == nullptr) {
        fprintf(stderr, "ERROR while opening test file\n");
        fclose(pfile);
        return 1;
    }
    for (int i=0; i<3; i++) {
        int long_size, med_size, short_size, a, b;
        W d;

        fscanf(pfile, "%d %d %d", &short_size, &med_size, &long_size);
        for (int j = 0; j < short_size; j++) {
            fscanf(pfile, "%d %d %f", &a, &b, &d);
            tests[i][0].emplace_back(make_pair(d, make_pair(a, b)));
        }

        for (int j = 0; j < med_size; j++) {
            fscanf(pfile, "%d %d %f", &a, &b, &d);
            tests[i][1].emplace_back(make_pair(d, make_pair(a, b)));
        }

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


    printf("FULL PLANAR\n");
    {
        FullPlanarOracle oracle(n, edges, distances, labels);
        run_all_vl_tests(oracle, tests);
        printf("\n\n");
    }

    printf("FULL FULL PLANAR\n");
    {
        FullFullPlanarOracle oracle(n, edges, distances, labels);
        run_all_ll_tests(oracle, tests);
        printf("\n\n");
    }

    printf("OSRM Oracle\n");
    {
        OsrmOracle oracle(argv[1], max_label, coords, labels);
        run_all_tests(oracle, tests);
        printf("\n\n");
    }

    printf("5 APPROX QUERY\n");
    {
        OracleGeneral5ApproxQuery oracle(n, edges, distances, labels);
        run_all_vl_tests(oracle, tests);
        printf("\n\n");
    }

    printf("5 APPROX UPDATE\n");
    {
        OracleGeneral5ApproxUpdate oracle(n, edges, distances, labels);
        run_all_vl_tests(oracle, tests);
        printf("\n\n");
    }

    printf("3 APPROX LIGHT\n");
    {
        OracleGeneral3ApproxLight oracle(n, edges, distances, labels);
        run_all_vl_tests(oracle, tests);
        run_all_ll_tests(oracle, tests);
        printf("\n\n");
    }

    printf("3 APPROX\n");
    {
        OracleGeneral3Approx oracle(n, edges, distances, labels);
        run_all_vl_tests(oracle, tests);
        run_all_ll_tests(oracle, tests);
        printf("\n\n");
    }

    printf("ASTAR\n");
    {
        AstarOracle oracle(n, m, max_label, max_speed, edges, distances, labels, coords);
        run_all_tests(oracle, tests);
        printf("\n\n");
    }

    printf("DIJKSTRA\n");
    {
        DijkstraOracle oracle(n, m, max_label, edges, distances, labels);
        run_all_tests(oracle, tests);
        printf("\n\n");
    }

}
