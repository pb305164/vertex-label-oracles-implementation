#include "oracle_naive.h"
#include "oracle_general_3approx.h"
#include "oracle_general_5approx.h"
#include "full_planar_oracle.h"
#include "find_union.h"

#include "read_graph.h"
#include "dijkstra_oracle.h"
#include "astar_oracle.h"
#include "osrm_oracle.h"
#include "hierarchy_oracle.h"
#include "hierarchy_oracle_light.h"
#include "hierarchy_oracle_light_path.h"

#include <chrono>


using std::map;


template <class T>
tuple<W, W, float, float, int, std::chrono::duration<double, std::milli> >  test_distanceToVertex(T &oracle, vector<pair<W, pair<int, int> > > &args)
{
    std::chrono::duration<double, std::milli> t_sum = std::chrono::milliseconds::zero();
    W result;
    W d_sum=0, d_res=0; float d_ratio=0, max_ratio=0;
    int suc=0;
    for (auto p: args) {
        auto t1 = std::chrono::steady_clock::now();
        result = oracle.distanceToVertex(p.second.second, p.second.first);
        auto t2 = std::chrono::steady_clock::now();
        if (result != -1 && result != infinity) {
            if(p.first==0) p.first=0.0000001;
            d_ratio += result/p.first;
            max_ratio = max(max_ratio, result/p.first);
            d_res += result;
            d_sum += p.first;
            t_sum += t2 - t1;
            suc++;
        }
    }
    return make_tuple(d_sum, d_res, d_ratio, max_ratio, suc, t_sum);
}


template <class T>
tuple<W, W,float, float, int, std::chrono::duration<double, std::milli> > test_distanceToLabel(T &oracle, vector<pair<W, pair<int, int> > > &args)
{
    std::chrono::duration<double, std::milli> t_sum = std::chrono::milliseconds::zero();
    pair<W, int> result;
    W d_sum=0, d_res=0; float d_ratio=0, max_ratio=0;
    int suc=0;
    for (auto p: args) {
        auto t1 = std::chrono::steady_clock::now();
        result = oracle.distanceToLabel(p.second.second, p.second.first);
        auto t2 = std::chrono::steady_clock::now();
        if (result.first != -1) {
            if(p.first==0) p.first=0.0000001;
            d_ratio += result.first/p.first;
            max_ratio = max(max_ratio, result.first/p.first);
            d_res += result.first;
            d_sum += p.first;
            t_sum += t2 - t1;
            suc++;
        }
    }
    return make_tuple(d_sum, d_res, d_ratio, max_ratio, suc, t_sum);
}


template <class T>
tuple<W, W, float, float, int, std::chrono::duration<double, std::milli> > test_distanceBetweenLabels(T &oracle, vector<pair<W, pair<int, int> > > &args)
{
    std::chrono::duration<double, std::milli> t_sum = std::chrono::milliseconds::zero();
    pair<W, pair<int, int> > result;
    W d_sum=0, d_res=0;
    float d_ratio=0, max_ratio=0;
    int suc=0;
    for (auto p: args) {
        auto t1 = std::chrono::steady_clock::now();
        result = oracle.distanceBetweenLabels(p.second.second, p.second.first);
        auto t2 = std::chrono::steady_clock::now();
        if (result.first != -1) {
            if(p.first==0) p.first=0.0000001;
            d_ratio += result.first/p.first;
            max_ratio = max(max_ratio, result.first/p.first);
            d_res += result.first;
            d_sum += p.first;
            t_sum += t2 - t1;
            suc++;
        }
    }
    return make_tuple(d_sum, d_res, d_ratio , max_ratio , suc, t_sum);
}


// Run tests on test function
template <class T>
std::chrono::duration<double, std::milli> run_tests(
        T &oracle, vector<pair<W, pair<int, int> > > &queries,
        tuple<W, W, float, float, int, std::chrono::duration<double, std::milli> > (test_func)(T&, vector<pair<W, pair<int, int> > >&))
{
    W d_exp, d_got; float d_ratio, max_ratio;
    std::chrono::duration<double, std::milli> sum;
    int suc;
    tie(d_exp, d_got, d_ratio, max_ratio, suc, sum) = test_func(oracle, queries);
    printf("Czasy testu suma: %lf  śr: %lf  poprawnie %d/%lu\n", sum.count(), sum.count()/suc, suc, queries.size());
    printf("Wyniki nadwyżka: %f  śr nad: %f  proc nad: %f  śr dł: %f\n\n", d_got-d_exp, (d_got-d_exp)/suc, d_got/d_exp, d_exp/suc);
    printf("Avg ratio: %f  Max ratio: %f\n\n", d_ratio/suc, max_ratio);

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

std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

long get_mem_size() {
    char cmd[100];
    sprintf(cmd, "pmap %d | tail -n 1 | awk -v N=2 '{print $N}' | rev | cut -c 2- | rev", ::getpid());
    return stol(exec(cmd));
}


int main(int argc, char* argv[]) {
    vector<pair<int, int>> edges;
    vector<char> types;
    vector<W> max_speeds;
    vector<W> distances;
    vector<int> labels;
    W max_speed;
    vector<pair<W, W>> coords;
    vector<pair<W, pair<int, int> > > tests[3][3];
    int n, m, max_label;

    W EPS = 1.;


    if (argc != 5 && argc != 6) {
        fprintf(stderr, "Usage: %s algorithm-number osrm-file generated-graph generated-test [(int)epsilon for planar oracle (default = 1)]>\n", argv[0]);
        return 1;
    }

    int alg_num=atoi(argv[1]);
    argc--;
    argv++;

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

    //WYPISZ ROZMIAR GRAFU I ORIENTACYJNĄ INFORMACJĘ DOT. ETYKIET
    printf("Nodes: %lu, Edges: %lu\n", labels.size(), edges.size());
    int zeros=0; vector<int> nozero;
    for(int i=0; i<labels.size(); ++i) {
        if(labels[i]==0) ++zeros;
        else nozero.push_back(labels[i]);
    }
    printf("Labelled zero: %d\n",zeros);
    sort(nozero.begin(),nozero.end());
    int minlab=labels.size()+1;
    int maxlab=0;
    if(nozero.size()==0) printf("No other labels.\n");
    else {
        int curlab=1;
        for(int i=1; i < nozero.size(); ++i) {
            if(nozero[i]==nozero[i-1]) curlab++;
            else {
               if(minlab > curlab) minlab = curlab;
               if(maxlab < curlab) maxlab = curlab;
               curlab=1;
               }
           }
        if(minlab > curlab) minlab = curlab;
        if(maxlab < curlab) maxlab = curlab;
        }

    printf("Max label size: %d, Min label size: %d\n",maxlab,minlab);

    // Change distance to time (TODO add as program option)
    for (int i = 0; i < m; i++) {
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
    for (int i = 0; i < 3; i++) {
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


    //long mem_begin = get_mem_size();
    //printf("pamięć przed: %ldK\n", get_mem_size());

    std::chrono::duration<double, std::milli> build_time;

    switch (alg_num) {
    case 1:
        printf("SIMPLE PLANAR, EPS=1\n");
        {
            auto t1 = std::chrono::steady_clock::now();
            DynamicSimplePlanarOracle oracle(n, edges, distances, labels, EPS);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            printf("Czas budowy: %lfs\n", build_time.count()/1000);
            run_all_vv_tests(oracle, tests);
            run_all_vl_tests(oracle, tests);
            printf("\n\n");
        }
        break;
    case 2:
        printf("SIMPLE PLANAR, EPS=4\n");
        {
            EPS=4.;
            auto t1 = std::chrono::steady_clock::now();
            DynamicSimplePlanarOracle oracle(n, edges, distances, labels, EPS);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            printf("Czas budowy: %lfs\n", build_time.count()/1000);
            run_all_vv_tests(oracle, tests);
            run_all_vl_tests(oracle, tests);
            printf("\n\n");
        }
        break;
    case 3:
        printf("SIMPLE PLANAR, EPS=8\n");
        {
            EPS=8.;
            auto t1 = std::chrono::steady_clock::now();
            DynamicSimplePlanarOracle oracle(n, edges, distances, labels, EPS);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            printf("Czas budowy: %lfs\n", build_time.count()/1000);
            run_all_vv_tests(oracle, tests);
            run_all_vl_tests(oracle, tests);
            printf("\n\n");
        }
        break;
    case 4:
        printf("PLANAR, EPS=1\n");
        {
            EPS=1.;
            auto t1 = std::chrono::steady_clock::now();
            DynamicPlanarOracle oracle(n, edges, distances, labels, EPS);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            printf("Czas budowy: %lfs\n", build_time.count()/1000);
            run_all_vv_tests(oracle,tests);
            run_all_vl_tests(oracle, tests);
            printf("\n\n");
        }
        break;
    case 5:
        printf("PLANAR, EPS=4\n");
        {
            EPS=4.;
            auto t1 = std::chrono::steady_clock::now();
            DynamicPlanarOracle oracle(n, edges, distances, labels, EPS);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            printf("Czas budowy: %lfs\n", build_time.count()/1000);
            run_all_vv_tests(oracle, tests);
            run_all_vl_tests(oracle, tests);
            printf("\n\n");
        }
        break;
    case 6:
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
        break;
    case 7:
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
        break;
    case 8:
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
        break;
    case 9:
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
        break;
    case 10:
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
        break;
    case 11:
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
        break;
    case 12:
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
        break;
    case 13:
        printf("HIERARCHY\n");
        {
            auto t1 = std::chrono::steady_clock::now();
            HierarchyOracle oracle(edges, distances, labels, types);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            printf("Czas budowy: %lfs\n", build_time.count() / 1000);
            run_all_tests(oracle, tests);
     //       printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
            printf("\n\n");
        }
        break;
    case 14:
        printf("HIERARCHY LIGHT\n");
        {
            auto t1 = std::chrono::steady_clock::now();
            HierarchyOracleLight oracle(edges, distances, labels, types);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            printf("Czas budowy: %lfs\n", build_time.count() / 1000);
            run_all_vv_tests(oracle, tests);
            run_all_vl_tests(oracle, tests);
    //        printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
            printf("\n\n");
        }
        break;
    case 15:
        printf("HIERARCHY LIGHT Path\n");
        {
            auto t1 = std::chrono::steady_clock::now();
            HierarchyOracleLightPath oracle(edges, distances, labels, types);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            printf("Czas budowy: %lfs\n", build_time.count() / 1000);
            run_all_vv_tests(oracle, tests);
            run_all_vl_tests(oracle, tests);
     //       printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
            printf("\n\n");
        }
        break;
    default:
        break;
    }

    printf("pamięć pomiędzy: %ldK\n", get_mem_size());

    printf("HIERARCHY LIGHT\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        HierarchyOracleLight oracle(edges, distances, labels, types);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs   pamięć: %ldK\n", build_time.count() / 1000, get_mem_size() - mem_begin);
        run_all_vv_tests(oracle, tests);
        run_all_vl_tests(oracle, tests);
        printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
        printf("\n\n");
    }

   printf("pamięć pomiędzy: %ldK\n", get_mem_size());

    printf("HIERARCHY LIGHT Path\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        HierarchyOracleLightPath oracle(edges, distances, labels, types);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs   pamięć: %ldK\n", build_time.count() / 1000, get_mem_size() - mem_begin);
        run_all_vv_tests(oracle, tests);
        run_all_vl_tests(oracle, tests);
        printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
        printf("\n\n");
    }

    printf("pamięć pomiędzy: %ldK\n", get_mem_size());

    printf("FULL PLANAR\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        FullPlanarOracle oracle(n, edges, distances, labels, EPS);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs   pamięć: %ldK\n", build_time.count() / 1000, get_mem_size() - mem_begin);
        run_all_vl_tests(oracle, tests);
        printf("\n\n");
    }

    printf("pamięć pomiędzy: %ldK\n", get_mem_size());

    printf("FULL FULL PLANAR\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        FullFullPlanarOracle oracle(n, edges, distances, labels, EPS);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs   pamięć: %ldK\n", build_time.count() / 1000, get_mem_size() - mem_begin);
        run_all_ll_tests(oracle, tests);
        printf("\n\n");
    }

    printf("pamięć pomiędzy: %ldK\n", get_mem_size());

    printf("OSRM Oracle\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        OsrmOracle oracle(argv[1], max_label, coords, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs   pamięć: %ldK\n", build_time.count() / 1000, get_mem_size() - mem_begin);
        run_all_tests(oracle, tests);
        printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
        printf("\n\n");
    }

    printf("pamięć pomiędzy: %ldK\n", get_mem_size());

    printf("5 APPROX QUERY\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        OracleGeneral5ApproxQuery oracle(n, edges, distances, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs   pamięć: %ldK\n", build_time.count() / 1000, get_mem_size() - mem_begin);
        run_all_vl_tests(oracle, tests);
        printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
        printf("\n\n");
    }

    printf("pamięć pomiędzy: %ldK\n", get_mem_size());

    printf("5 APPROX UPDATE\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        OracleGeneral5ApproxUpdate oracle(n, edges, distances, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs   pamięć: %ldK\n", build_time.count() / 1000, get_mem_size() - mem_begin);
        run_all_vl_tests(oracle, tests);
        printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
        printf("\n\n");
    }

    printf("pamięć pomiędzy: %ldK\n", get_mem_size());

    printf("3 APPROX LIGHT\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        OracleGeneral3ApproxLight oracle(n, edges, distances, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs   pamięć: %ldK\n", build_time.count() / 1000, get_mem_size() - mem_begin);
        run_all_vl_tests(oracle, tests);
        run_all_ll_tests(oracle, tests);
         printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
        printf("\n\n");
    }

    printf("pamięć pomiędzy: %ldK\n", get_mem_size());

    printf("3 APPROX\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        OracleGeneral3Approx oracle(n, edges, distances, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs   pamięć: %ldK\n", build_time.count() / 1000, get_mem_size() - mem_begin);
        run_all_vl_tests(oracle, tests);
        run_all_ll_tests(oracle, tests);
        printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
        printf("\n\n");
    }

    printf("pamięć pomiędzy: %ldK\n", get_mem_size());

    printf("ASTAR\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        AstarOracle oracle(n, m, max_label, max_speed, edges, distances, labels, coords);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs   pamięć: %ldK\n", build_time.count() / 1000, get_mem_size() - mem_begin);
        run_all_tests(oracle, tests);
        printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
        printf("\n\n");
    }

    printf("pamięć pomiędzy: %ldK\n", get_mem_size());

    printf("DIJKSTRA\n");
    {
        auto t1 = std::chrono::steady_clock::now();
        DijkstraOracle oracle(n, m, max_label, edges, distances, labels);
        auto t2 = std::chrono::steady_clock::now();
        build_time = t2 - t1;
        printf("Czas budowy: %lfs   pamięć: %ldK\n", build_time.count() / 1000, get_mem_size() - mem_begin);
        run_all_tests(oracle, tests);
        printf("pamięć po testach: %ldK\n", get_mem_size() - mem_begin);
        printf("\n\n");
    }
    printf("pamięć: %ldK\n", get_mem_size());

}
