#include "oracle_naive.h"
#include "oracle_general_3approx.h"
#include "oracle_general_5approx.h"
#include "full_planar_oracle.h"

#include "read_graph.h"
#include "dijkstra_oracle.h"
#include "astar_oracle.h"
#include "osrm_oracle.h"
#include "hierarchy_oracle.h"
#include "hierarchy_oracle_light.h"
#include "hierarchy_oracle_light_path.h"

#include <chrono>


using namespace std;

// Exec given command
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


// Try to estimate memory usage
long get_mem_usage() {
    char cmd[100];
    sprintf(cmd, "pmap %d | tail -n 1 | awk -v N=2 '{print $N}' | rev | cut -c 2- | rev", ::getpid());
    return stol(exec(cmd));
}

float EPS = 1.0, min_port_dist = -1.0;
int RO = -1, oracle_type = -1;
char *osrm_path = nullptr, *graph_path = nullptr;
vector<string> test_paths;
int n, m, max_label, sample_count, sample_size, test_type, allowed_queries, weights_as_distance;


void print_help() {
    fprintf(stderr,
            "Usage: ./new_run_test [options] -O oracle_id osrm_file generated_graph generated_test [more generated_tests]\n\n"
                    "Options:\n"
                    "  -h           : Print this help message\n\n"
                    "  -O   int     : Select oracle to test (Required):\n"
                    "                       0: FullPlanar\n"
                    "                       1: FullFullPlanar\n"
                    "                       2: StaticPlanar\n"
                    "                       3: StaticLLPlanar\n"
                    "                       4: 5AproxQuery\n"
                    "                       5: 5AproxUpdate\n"
                    "                       6: 3AproxLight\n"
                    "                       7: 3Aprox\n"
                    "                       8: Hierarchy\n"
                    "                       9: HierarchyLight\n"
                    "                      10: HierarchyLightPath\n"
                    "                      11: Osrm\n"
                    "                      12: Dijkstra\n"
                    "                      13: Astar\n"

                    "  -EPS float   : Epsilon value for planar oracles (default = 1.0)\n\n"
                    "  -MPD float   : Minimal distances between portals for hierarchal oracles (default 0 for light, 15 for full)\n\n"
                    "  -RO  int     : Ro value for aprox oracles (default -1)\n\n"
    );

}


void parse_program_options(int argc, char* argv[]) {
    for (int i=1; i<argc;) {
        if (strcmp("-h", argv[i]) == 0) {
            print_help();
            exit(0);
        }  if (strcmp("-O", argv[i]) == 0) {
            i++;
            int v=-1;
            if (i >= argc) {
                fprintf(stderr, "Error while parsing -O option, no value\n\n");
                print_help();
                exit(1);
            }
            if (sscanf(argv[i], "%d", &v) && v >= 0) {
                oracle_type = v;
            } else {
                fprintf(stderr, "Error while parsing -O option, invalid value\n\n");
                print_help();
                exit(1);
            }
        } else if (strcmp("-EPS", argv[i]) == 0) {
            i++;
            if (i >= argc) {
                fprintf(stderr, "Error while parsing -EPS option, no value\n\n");
                print_help();
                exit(1);
            }
            float v=-1;
            if (sscanf(argv[i], "%f", &v) && v >= 0) {
                EPS = v;
            } else {
                fprintf(stderr, "Error while parsing -EPS option, invalid value\n\n");
                print_help();
                exit(1);
            }
        } else if (strcmp("-MPD", argv[i]) == 0) {
            i++;
            float v=-1;
            if (i >= argc) {
                fprintf(stderr, "Error while parsing -MPD option, no value\n\n");
                print_help();
                exit(1);
            }
            if (sscanf(argv[i], "%f", &v) && v >= 0) {
                min_port_dist = v;
            } else {
                fprintf(stderr, "Error while parsing -MPD option, invalid value\n\n");
                print_help();
                exit(1);
            }
        } else if (strcmp("-RO", argv[i]) == 0) {
            i++;
            int v=-1;
            if (i >= argc) {
                fprintf(stderr, "Error while parsing -RO option, no value\n\n");
                print_help();
                exit(1);
            }
            if (sscanf(argv[i], "%d", &v) && v >= 0) {
                RO = v;
            } else {
                fprintf(stderr, "Error while parsing -RO option, invalid value\n\n");
                print_help();
                exit(1);
            }
        } else {
            if (osrm_path == nullptr) {
                osrm_path = argv[i];
            } else if (graph_path == nullptr) {
                graph_path = argv[i];
            } else {
                test_paths.push_back(string(argv[i]));
            }
        }
        i++;
    }
    if (test_paths.size() == 0) {
        fprintf(stderr, "Error no tests to run specified\n\n");
        print_help();
        exit(1);
    }

    if (oracle_type == -1) {
        fprintf(stderr, "Error no oracle specified\n\n");
        print_help();
        exit(1);
    }
}

template <class T>
void run_query0(FILE *out_file, T &oracle, tuple<int, int, int, float> &q) {
    auto t1 = std::chrono::steady_clock::now();
    float d = oracle.distanceToVertex(get<1>(q), get<2>(q));
    auto t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> t = t2 -t1;
    fprintf(out_file, "%d %f %f %lf\n", 0, d, get<3>(q), t.count());
}

template <class T>
void run_query1(FILE *out_file, T &oracle, tuple<int, int, int, float> &q) {
    auto t1 = std::chrono::steady_clock::now();
    pair<W, int> d = oracle.distanceToLabel(get<1>(q), get<2>(q));
    auto t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> t = t2 -t1;
    fprintf(out_file, "%d %f %f %lf\n", 1, d.first, get<3>(q), t.count());
}

template <class T>
void run_query2(FILE *out_file, T &oracle, tuple<int, int, int, float> &q) {
    auto t1 = std::chrono::steady_clock::now();
    pair<W, pair<int, int>> d = oracle.distanceBetweenLabels(get<1>(q), get<2>(q));
    auto t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> t = t2 -t1;
    fprintf(out_file, "%d %f %f %lf\n", 2, d.first, get<3>(q), t.count());
}

template <class T>
void run_query3(FILE *out_file, T &oracle, tuple<int, int, int, float> &q) {
    auto t1 = std::chrono::steady_clock::now();
    oracle.setLabel(get<1>(q), get<2>(q));
    auto t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> t = t2 -t1;
    fprintf(out_file, "%d %f %f %lf\n", 3, .0, .0, t.count());
}

template <class T>
void run_all_queries(FILE *out_file, T &oracle, vector<tuple<int, int, int, float>> &queries)
{
    size_t count=0;
    for (auto &q: queries) {
        // For each query there are 5 numbers printed out: type of query, oracle answer, solution, time in miliseconds, estimated memory usage
        if (get<0>(q) == 0) {
            run_query0(out_file, oracle, q);
        } else if (get<0>(q) == 1) {
            run_query1(out_file, oracle, q);
        } else if (get<0>(q) == 2) {
            run_query2(out_file, oracle, q);
        } else if (get<0>(q) == 3) {
            run_query3(out_file, oracle, q);
        } else {
            fprintf(stderr, "Error, bad query type ?\n");
            exit(1);
        }
        count++;
        if (count%sample_size==0) fprintf(out_file, "%ld\n", get_mem_usage());
    }
}

template <class T>
void run_VL_SL_queries(FILE *out_file, T &oracle, vector<tuple<int, int, int, float>> &queries)
{
    // Check proper test
    for (auto &q: queries) {
        if (get<0>(q) == 0 || get<0>(q) == 2) {
            fprintf(stderr, "Error cannot run this test for this oracle (oracle doesnt support queries from the test)\n");
            exit(1);
        }
    }
    size_t count=0;
    for (auto &q: queries) {
        // For each query there are 5 numbers printed out: type of query, oracle answer, solution, time in miliseconds, estimated memory usage
        if (get<0>(q) == 1) {
            run_query1(out_file, oracle, q);
        } else if (get<0>(q) == 3) {
            run_query3(out_file, oracle, q);
        } else {
            fprintf(stderr, "Error, bad query type ?\n");
            exit(1);
        }
        count++;
        if (count%sample_size==0) fprintf(out_file, "%ld\n", get_mem_usage());
    }
}

template <class T>
void run_VV_VL_queries(FILE *out_file, T &oracle, vector<tuple<int, int, int, float>> &queries)
{
    // Check proper test
    for (auto &q: queries) {
        if (get<0>(q) > 1) {
            fprintf(stderr, "Error cannot run this test for this oracle (oracle doesnt support queries from the test)\n");
            exit(1);
        }
    }
    size_t count=0;
    for (auto &q: queries) {
        // For each query there are 5 numbers printed out: type of query, oracle answer, solution, time in miliseconds, estimated memory usage
        if (get<0>(q) == 0) {
            run_query1(out_file, oracle, q);
        } else if (get<0>(q) == 1) {
            run_query3(out_file, oracle, q);
        } else {
            fprintf(stderr, "Error, bad query type ?\n");
            exit(1);
        }
        count++;
        if (count%sample_size==0) fprintf(out_file, "%ld\n", get_mem_usage());
    }
}

template <class T>
void run_LL_SL_queries(FILE *out_file, T &oracle, vector<tuple<int, int, int, float>> &queries)
{
    // Check proper test
    for (auto &q: queries) {
        if (get<0>(q) < 2) {
            fprintf(stderr, "Error cannot run this test for this oracle (oracle doesnt support queries from the test)\n");
            exit(1);
        }
    }
    size_t count=0;
    for (auto &q: queries) {
        // For each query there are 5 numbers printed out: type of query, oracle answer, solution, time in miliseconds, estimated memory usage
        if (get<0>(q) == 2) {
            run_query2(out_file, oracle, q);
        } else if (get<0>(q) == 3) {
            run_query3(out_file, oracle, q);
        } else {
            fprintf(stderr, "Error, bad query type ?\n");
            exit(1);
        }
        count++;
        if (count%sample_size==0) fprintf(out_file, "%ld\n", get_mem_usage());
    }
}

template <class T>
void run_VV_VL_SL_queries(FILE *out_file, T &oracle, vector<tuple<int, int, int, float>> &queries)
{
    // Check proper test
    for (auto &q: queries) {
        if (get<0>(q) == 2) {
            fprintf(stderr, "Error cannot run this test for this oracle (oracle doesnt support queries from the test)\n");
            exit(1);
        }
    }
    size_t count=0;
    for (auto &q: queries) {
        // For each query there are 5 numbers printed out: type of query, oracle answer, solution, time in miliseconds, estimated memory usage
        if (get<0>(q) == 0) {
            run_query0(out_file, oracle, q);
        } else if (get<0>(q) == 1) {
            run_query1(out_file, oracle, q);
        } else if (get<0>(q) == 3) {
            run_query3(out_file, oracle, q);
        } else {
            fprintf(stderr, "Error, bad query type ?\n");
            exit(1);
        }
        count++;
        if (count%sample_size==0) fprintf(out_file, "%ld\n", get_mem_usage());
    }
}

template <class T>
void run_VL_LL_SL_queries(FILE *out_file, T &oracle, vector<tuple<int, int, int, float>> &queries)
{
    // Check proper test
    for (auto &q: queries) {
        if (get<0>(q) == 0) {
            fprintf(stderr, "Error cannot run this test for this oracle (oracle doesnt support queries from the test)\n");
            exit(1);
        }
    }
    size_t count=0;
    for (auto &q: queries) {
        // For each query there are 5 numbers printed out: type of query, oracle answer, solution, time in miliseconds, estimated memory usage
        if (get<0>(q) == 1) {
            run_query1(out_file, oracle, q);
        } else if (get<0>(q) == 2) {
            run_query2(out_file, oracle, q);
        } else if (get<0>(q) == 3) {
            run_query3(out_file, oracle, q);
        } else {
            fprintf(stderr, "Error, bad query type ?\n");
            exit(1);
        }
        count++;
        if (count%sample_size==0) fprintf(out_file, "%ld\n", get_mem_usage());
    }
}

template <class T>
void run_VV_VL_LL_queries(FILE *out_file, T &oracle, vector<tuple<int, int, int, float>> &queries)
{
    // Check proper test
    for (auto &q: queries) {
        if (get<0>(q) == 3) {
            fprintf(stderr, "Error cannot run this test for this oracle (oracle doesnt support queries from the test)\n");
            exit(1);
        }
    }
    size_t count=0;
    for (auto &q: queries) {
        // For each query there are 5 numbers printed out: type of query, oracle answer, solution, time in miliseconds, estimated memory usage
        if (get<0>(q) == 0) {
            run_query0(out_file, oracle, q);
        } else if (get<0>(q) == 1) {
            run_query1(out_file, oracle, q);
        } else if (get<0>(q) == 2) {
            run_query2(out_file, oracle, q);
        } else {
            fprintf(stderr, "Error, bad query type ?\n");
            exit(1);
        }
        count++;
        if (count%sample_size==0) fprintf(out_file, "%ld\n", get_mem_usage());
    }
}


vector<W> max_speeds;
vector<W> distances;
vector<int> labels;
vector<tuple<int, int, int, float>> queries;


// Check if all tests have the same weights (distance / time)
void check_weight_format() {
    FILE *pfile = fopen(test_paths[0].c_str(), "r");
    if (pfile == nullptr) {
        fprintf(stderr, "ERROR while opening test file nr %d\n", 0);
        fclose(pfile);
        exit(1);
    }
    fscanf(pfile, "%d %d %d %d %d", &sample_count, &sample_size, &test_type, &allowed_queries, &weights_as_distance);
    fclose(pfile);

    int weight_type = weights_as_distance;
    for (size_t i=1; i < test_paths.size(); i++) {
        FILE *pfile = fopen(test_paths[i].c_str(), "r");
        if (pfile == nullptr) {
            fprintf(stderr, "ERROR while opening test file nr %lu\n", i);
            fclose(pfile);
            exit(1);
        }
        fscanf(pfile, "%d %d %d %d %d", &sample_count, &sample_size, &test_type, &allowed_queries, &weights_as_distance);
        fclose(pfile);
        if (weights_as_distance != weight_type) {
            fprintf(stderr, "All tests should have the same weights on edges, cannot run multiple tests when some use time, other distance as weight\n");
            exit(1);
        }
    }
}


FILE *prep_test(size_t test_id) {
    // Read next test
    FILE *pfile = fopen(test_paths[test_id].c_str(), "r");
    if (pfile == nullptr) {
        fprintf(stderr, "ERROR while opening test file nr %lu\n", test_id);
        fclose(pfile);
        exit(1);
    }
    queries.clear();
    fscanf(pfile, "%d %d %d %d %d", &sample_count, &sample_size, &test_type, &allowed_queries, &weights_as_distance);
    for (int i=0; i<sample_count*sample_size; i++) {
        int query_type, a, b;
        float d;
        fscanf(pfile, "%d %d %d %f", &query_type, &a, &b, &d);
        queries.emplace_back(query_type, a, b, d);
    }
    fclose(pfile);

    string out_path(test_paths[test_id]);
    out_path.insert(out_path.find_last_of('/')+1, "OUT_"+to_string(oracle_type)+"_");
    FILE *out_file = fopen(out_path.c_str(), "w");
    if (out_file == nullptr) {
        fprintf(stderr, "ERROR while opening test output file for test nr %lu\n", test_id);
        exit(1);
    }
    return out_file;
}

template <class T>
void prep_oracle(T &oracle) {
    // If test type == 3 clear labels
    if (test_type == 3) {
        for (int i = 0; i < n; i++) {
            oracle.setLabel(i, 0);
        }
    } else {
        for (size_t i=0; i<labels.size(); i++) {
            oracle.setLabel(i, labels[i]);
        }
    }
}


int main(int argc, char* argv[]) {
    vector<pair<int, int>> edges;
    vector<char> types;
    W max_speed;
    vector<pair<W, W>> coords;

    parse_program_options(argc, argv);
//    printf("oracle_type: %d   EPS: %f   MPD: %f   RO: %d  osm_path: %s   graph_path: %s  test_path: %s\n",
//           oracle_type, EPS, min_port_dist, RO, osrm_path, graph_path, test_path);
//    return 0;

    // Read graph
    FILE *pfile = fopen(graph_path, "r");
    if (pfile == nullptr) {
        fprintf(stderr, "ERROR while opening graph file\n");
        return 1;
    }
    read_graph(pfile, n, m, max_label, max_speed, edges, types, max_speeds, distances, labels, coords);
    fclose(pfile);

    check_weight_format();

    // Change distance to time is needed
    if (weights_as_distance == 0) {
        for (int i = 0; i < m; i++) {
            distances[i] = distances[i] / max_speeds[i];
        }
    }

    std::chrono::duration<double, std::milli> build_time;
    switch (oracle_type) {
        case 0: {
            auto t1 = std::chrono::steady_clock::now();
            FullPlanarOracle oracle(n, edges, distances, labels, EPS);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                prep_oracle(oracle);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_VL_SL_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }

        case 1: {
            auto t1 = std::chrono::steady_clock::now();
            FullFullPlanarOracle oracle(n, edges, distances, labels, EPS);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                prep_oracle(oracle);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_LL_SL_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }

        case 2: {
            // TODO uncomment after merge
//            auto t1 = std::chrono::steady_clock::now();
//            StaticPlanarOracle oracle(n, edges, distances, labels, EPS);
//            auto t2 = std::chrono::steady_clock::now();
//            build_time = t2 - t1;
//            for (size_t i = 0; i < test_paths.size(); i++) {
//                FILE *out_file = prep_test(i);
//                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
//                run_VV_VL_queries(out_fileoracle, queries);
//                fclose(out_file);
//            }
            break;
        }

        case 3: {
            // TODO uncomment after merge
//            auto t1 = std::chrono::steady_clock::now();
//            StaticLLPlanarOracle oracle(n, edges, distances, labels, EPS);
//            auto t2 = std::chrono::steady_clock::now();
//            build_time = t2 - t1;
//            for (size_t i = 0; i < test_paths.size(); i++) {
//                FILE *out_file = prep_test(i);
//                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
//                run_VV_VL_LL_queries(out_file, oracle, queries);
//                fclose(out_file);
//            }
            break;
        }

        case 4: {
            auto t1 = std::chrono::steady_clock::now();
            OracleGeneral5ApproxQuery oracle(n, edges, distances, labels, RO);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                prep_oracle(oracle);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_VL_SL_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }

        case 5: {
            auto t1 = std::chrono::steady_clock::now();
            OracleGeneral5ApproxUpdate oracle(n, edges, distances, labels, RO);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                prep_oracle(oracle);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_VL_SL_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }

        case 6: {
            auto t1 = std::chrono::steady_clock::now();
            OracleGeneral3ApproxLight oracle(n, edges, distances, labels, RO);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                prep_oracle(oracle);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_VL_LL_SL_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }

        case 7: {
            auto t1 = std::chrono::steady_clock::now();
            OracleGeneral3Approx oracle(n, edges, distances, labels, RO);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                prep_oracle(oracle);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_VL_LL_SL_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }

        case 8: {
            auto t1 = std::chrono::steady_clock::now();
            HierarchyOracle oracle(edges, distances, labels, types, 0, min_port_dist);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                prep_oracle(oracle);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_all_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }

        case 9: {
            auto t1 = std::chrono::steady_clock::now();
            HierarchyOracleLight oracle(edges, distances, labels, types, 0, min_port_dist);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                prep_oracle(oracle);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_VV_VL_SL_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }

        case 10: {
            auto t1 = std::chrono::steady_clock::now();
            HierarchyOracleLightPath oracle(edges, distances, labels, types, 0, min_port_dist);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                prep_oracle(oracle);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_VV_VL_SL_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }

        case 11: {
            auto t1 = std::chrono::steady_clock::now();
            OsrmOracle oracle(osrm_path, max_label, coords, labels);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_VV_VL_LL_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }

        case 12: {
            auto t1 = std::chrono::steady_clock::now();
            DijkstraOracle oracle(n, m, edges, distances, labels);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                prep_oracle(oracle);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_all_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }

        case 13: {
            auto t1 = std::chrono::steady_clock::now();
            AstarOracle oracle(n, m, max_speed, edges, distances, labels, coords);
            auto t2 = std::chrono::steady_clock::now();
            build_time = t2 - t1;
            for (size_t i = 0; i < test_paths.size(); i++) {
                FILE *out_file = prep_test(i);
                prep_oracle(oracle);
                fprintf(out_file, "%d %d %lf %ld\n", sample_count, sample_size, build_time.count(), get_mem_usage());
                run_all_queries(out_file, oracle, queries);
                fclose(out_file);
            }
            break;
        }
    }
}
