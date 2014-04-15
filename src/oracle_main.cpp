#include "oracle_naive.h"
#include "oracle_general_3approx.h"
#include "oracle_general_5approx.h"
#include "oracle_tester.h"

#include <cstdio>
#include <cassert>

time_t start, stop;
void startTime() {
    start = clock();
}

float stopTime() {
    stop = clock();
    return (float)(stop - start) / CLOCKS_PER_SEC;
}

void stopTimePrint() {
    stop = clock();
    printf("%.5f\n", (float)(stop - start) / CLOCKS_PER_SEC);
}

template <class O>
inline
double timeProportionTest(O &oracle, int m, const vector<int> &type, const vector< pair<int, int> > &query) {
    startTime();
    for (int i=0; i<m; ++i) {
        switch (type[i]) {
            case 0: oracle.setLabel(query[i].first, query[i].second); break;
            case 1: oracle.distanceToLabel(query[i].first, query[i].second); break;
        }
    }
    return stopTime();
}
    
const int K = 10;
const int T = 20;
const int M = 100;

template <class O>
void performVertexToLabelProportionTest(int n, const vector< pair<int, int> > &edges, const vector<W> &weights) {


    srand(-1);

    vector<int> labels(n);
    for (int i=0; i<n; ++i) labels[i] = rand() % n;
//    for (int i=0; i<n; ++i) labels[i] = i / K;
//    vector<int> labelsCopy(labels);
//    int last = 0;

    fprintf(stderr, "Constructing...\n");
    fflush(stderr);
    O oracle(n, edges, weights, labels);
    fprintf(stderr, " - done\n");

    for (int t=0; t<=T; ++t) {
        fprintf(stderr, "t: %d\n", t);
        vector< int > type(M);
        vector< pair<int, int> > query(M);

        vector< int > typeCycle;
        for (int i=0; i<T; ++i) {
            if (i < t) {
                typeCycle.push_back(0);
            } else {
                typeCycle.push_back(1);
            }
        }
        int tc = 0;
        for (int i=0; i<M; ++i) {
            type[i] = typeCycle[tc++];
            if (tc == T) tc = 0;


            switch (type[i]) {
                case 0: query[i] = make_pair(rand()%n, rand()%n); break;
                case 1: query[i] = make_pair(rand()%n, rand()%n); break;
            }
/*
            int v;
            switch (type[i]) {
                case 0: 
                    v = rand()%n;
                    query[i] = make_pair(last, labelsCopy[v]); 
                    labelsCopy[last] = labelsCopy[v];
                    last = v;
                    break;
                case 1: 
                    query[i] = make_pair(labelsCopy[rand()%n], labelsCopy[rand()%n]); 
                    break;
            }
*/
        }

        printf("%.8f ", timeProportionTest(oracle, M, type, query)); 
    }

    printf("\n");
}

void printLabels() {
    for (int t=0; t<=T; ++t) {
        printf("%d:%d ", t, T-t);
    }
    printf("\n");
}

void performVertexToLabelProportionTestAll(int n, const vector< pair<int, int> > &edges, const vector<W> &weights) {
    printLabels();
    performVertexToLabelProportionTest<OracleNaive>(n, edges, weights);
    performVertexToLabelProportionTest<OracleGeneral3Approx>(n, edges, weights);
    performVertexToLabelProportionTest<OracleGeneral5ApproxUpdate>(n, edges, weights);
    performVertexToLabelProportionTest<OracleGeneral5ApproxQuery>(n, edges, weights);
}

int main() {
    int n;
    vector< pair<int, int> > edges;
    vector< W > weights;

    vector< int > labels;
    vector< pair< int, int > > updates, queries;


    OracleTester::generateGraph(80000, 640000, 200, n, edges, weights);
    //OracleTester::readUnweightedGraphFromInput(n, edges, weights);

    fprintf(stderr, "Read!\n");
    fflush(stderr);

    {
        performVertexToLabelProportionTestAll(n, edges, weights);
    }


// Correctness test
/*
    {
        const int K = 50000;
        int T = 10;
        OracleTester::selectQueries(n, 4, K, labels, updates);
        OracleGeneral3Approx oracle3(n, edges, weights, labels);
        OracleGeneral5ApproxQuery oracle5q(n, edges, weights, labels);
        OracleGeneral5ApproxUpdate oracle5u(n, edges, weights, labels);
        OracleNaive oraclen(n, edges, weights, labels);

        for (int i=0; i<(int)updates.size(); ++i) {
            printf("%d\n", i);
            
            pair<int, int> update = updates[i];
            oraclen.setLabel(update.first, update.second);
            oracle3.setLabel(update.first, update.second);
            oracle5q.setLabel(update.first, update.second);
            oracle5u.setLabel(update.first, update.second);

            for (int t=0; t<T; ++t) {
            int u = rand()%n;
            int v = rand()%n;

            auto exact = oraclen.distanceToLabel(u, oraclen.labelOf(v));
            auto approx3 = oracle3.distanceToLabel(u, oracle3.labelOf(v));
            auto approx5q = oracle5q.distanceToLabel(u, oracle5q.labelOf(v));
            auto approx5u = oracle5u.distanceToLabel(u, oracle5u.labelOf(v));

            assert(exact.first <= approx3.first);
            assert(exact.first <= approx5q.first);
            assert(exact.first <= approx5u.first);
            assert(exact.first * 3 >= approx3.first);
            assert(exact.first * 5 >= approx5q.first);
            assert(exact.first * 5 >= approx5u.first);

            }

            for (int t=0; t<T; ++t) {
            int u = rand()%n;
                int v = rand()%n;

                auto exact = oraclen.distanceBetweenLabels(oraclen.labelOf(u), oraclen.labelOf(v));
                auto approx3 = oracle3.distanceBetweenLabels(oracle3.labelOf(u), oracle3.labelOf(v));

                assert(exact.first <= approx3.first);
                assert(exact.first * 3 >= approx3.first);
            }
        }
    }
*/

    // Time test
/*
    {
        OracleTester::selectQueries(n, 4, K, labels, updates);
        printf("Comparing:\n");
        printf(" - Dijkstra oracle\n");
        printf(" - 3 approximate oracle\n");
        printf(" - 5 approximate oracle with constant query time\n");
        printf(" - 5 approximate oracle with constant update time\n");
        printf("\n");

        printf(" -- INITIALIZATION --\n");
        printf("Oracle Naive\n");
        startTime();
        OracleNaive oraclen(n, edges, weights, labels);
        stopTimePrint();

        printf("Oracle 3-approx\n");
        startTime();
        OracleGeneral3Approx oracle3(n, edges, weights, labels);
        stopTimePrint();

        printf("Oracle 5-approx query\n");
        startTime();
        OracleGeneral5ApproxQuery oracle5q(n, edges, weights, labels);
        stopTimePrint();

        printf("Oracle 5-approx update\n");
        startTime();
        OracleGeneral5ApproxUpdate oracle5u(n, edges, weights, labels);
        stopTimePrint();

        printf("\n -- UPDATE --\n");
        printf("Oracle Naive\n");
        startTime();
        for (auto &u: updates)
            oraclen.setLabel(u.first, u.second);
        stopTimePrint();

        printf("Oracle 3-approx\n");
        startTime();
        for (auto &u: updates)
            oracle3.setLabel(u.first, u.second);
        stopTimePrint();

        printf("Oracle 5-approx query\n");
        startTime();
        for (auto &u: updates)
            oracle5q.setLabel(u.first, u.second);
        stopTimePrint();

        printf("Oracle 5-approx update\n");
        startTime();
        for (auto &u: updates)
            oracle5u.setLabel(u.first, u.second);
        stopTimePrint();

        updates.clear();
        for (int i=0; i<K; ++i)
            updates.push_back(make_pair(rand()%n, oraclen.labelOf(rand()%n)));

        printf("\n -- QUERY1 --\n");
        printf("Oracle Naive\n");
        startTime();
        for (auto &u: updates)
            oraclen.distanceToLabel(u.first, u.second);
        stopTimePrint();

        printf("Oracle 3-approx\n");
        startTime();
        for (auto &u: updates)
            oracle3.distanceToLabel(u.first, u.second);
        stopTimePrint();

        printf("Oracle 5-approx query\n");
        startTime();
        for (auto &u: updates)
            oracle5q.distanceToLabel(u.first, u.second);
        stopTimePrint();

        printf("Oracle 5-approx update\n");
        startTime();
        for (auto &u: updates)
            oracle5u.distanceToLabel(u.first, u.second);
        stopTimePrint();

        updates.clear();
        for (int i=0; i<K; ++i)
            updates.push_back(make_pair(oraclen.labelOf(rand()%n), oraclen.labelOf(rand()%n)));

        printf("\n -- QUERY2 --\n");
        printf("Oracle Naive\n");
        startTime();
        for (auto &u: updates)
            oraclen.distanceToLabel(u.first, u.second);
        stopTimePrint();

        printf("Oracle 3-approx\n");
        startTime();
        for (auto &u: updates)
            oracle3.distanceToLabel(u.first, u.second);
        stopTimePrint();

    }
*/
    return 0;
}
