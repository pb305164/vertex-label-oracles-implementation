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

void stopTime() {
  stop = clock();
  printf("%.3f sec\n", (float)(stop - start) / CLOCKS_PER_SEC);
}

int main() {
    int n;
    vector< pair<int, int> > edges;
    vector< W > weights;

    vector< int > labels;
    vector< pair< int, int > > updates, queries;

    const int K = 10000;

    OracleTester::generateGraph(1000, 4000, 200, n, edges, weights);
    OracleTester::selectQueries(n, 4, K, labels, updates);

    int T = 10;

// Correctness test
/*
    {
        OracleGeneral3Approx oracle3(n, edges, weights, labels);
        OracleGeneral5Approx oracle5(n, edges, weights, labels);
        OracleNaive oraclen(n, edges, weights, labels);

        for (int i=0; i<(int)updates.size(); ++i) {
            printf("%d\n", i);
            
            pair<int, int> update = updates[i];
            oraclen.setLabel(update.first, update.second);
            oracle3.setLabel(update.first, update.second);
            oracle5.setLabel(update.first, update.second);

            for (int t=0; t<T; ++t) {
                int u = rand()%n;
                int v = rand()%n;
            
                auto exact = oraclen.distanceToLabel(u, oraclen.labelOf(v));
                auto approx3 = oracle3.distanceToLabel(u, oracle3.labelOf(v));
                auto approx5 = oracle5.distanceToLabel(u, oracle5.labelOf(v));

                assert(exact.first <= approx3.first);
                assert(exact.first <= approx5.first);
                assert(exact.first * 3 >= approx3.first);
                assert(exact.first * 5 >= approx5.first);
            
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

    {
      printf(" -- INITIALIZATION --\n");
      printf("Oracle Naive\n");
      startTime();
      OracleNaive oraclen(n, edges, weights, labels);
      stopTime();

      printf("Oracle 3-approx\n");
      startTime();
      OracleGeneral3Approx oracle3(n, edges, weights, labels);
      stopTime();
      
      printf("Oracle 5-approx\n");
      startTime();
      OracleGeneral5Approx oracle5(n, edges, weights, labels);
      stopTime();
      
      printf("\n -- UPDATE --\n");
      printf("Oracle Naive\n");
      startTime();
      for (auto &u: updates)
        oraclen.setLabel(u.first, u.second);
      stopTime();

      printf("Oracle 3-approx\n");
      startTime();
      for (auto &u: updates)
        oracle3.setLabel(u.first, u.second);
      stopTime();
      
      printf("Oracle 5-approx\n");
      startTime();
      for (auto &u: updates)
        oracle5.setLabel(u.first, u.second);
      stopTime();

      updates.clear();
      for (int i=0; i<K; ++i)
        updates.push_back(make_pair(rand()%n, oraclen.labelOf(rand()%n)));

      printf("\n -- QUERY1 --\n");
      printf("Oracle Naive\n");
      startTime();
      for (auto &u: updates)
        oraclen.distanceToLabel(u.first, u.second);
      stopTime();

      printf("Oracle 3-approx\n");
      startTime();
      for (auto &u: updates)
        oracle3.distanceToLabel(u.first, u.second);
      stopTime();
      
      printf("Oracle 5-approx\n");
      startTime();
      for (auto &u: updates)
        oracle5.distanceToLabel(u.first, u.second);
      stopTime();

      updates.clear();
      for (int i=0; i<K; ++i)
        updates.push_back(make_pair(oraclen.labelOf(rand()%n), oraclen.labelOf(rand()%n)));

      printf("\n -- QUERY2 --\n");
      printf("Oracle Naive\n");
      startTime();
      for (auto &u: updates)
        oraclen.distanceToLabel(u.first, u.second);
      stopTime();

      printf("Oracle 3-approx\n");
      startTime();
      for (auto &u: updates)
        oracle3.distanceToLabel(u.first, u.second);
      stopTime();

    }

    return 0;
}
