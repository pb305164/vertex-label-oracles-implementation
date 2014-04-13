#include "oracle_naive.h"
#include "oracle_general.h"
#include "oracle_tester.h"

#include <cstdio>
#include <cassert>

int main() {
    int n;
    vector< pair<int, int> > edges;
    vector< W > weights;

    vector< int > labels;
    vector< pair< int, int > > updates;

    OracleTester::generateGraph(1000, 4000, 200, n, edges, weights);
    OracleTester::selectQueries(n, 4, 10000, labels, updates);

    W cappr = 3;
    int T = 10;

// Correctness test

    {
        OracleGeneral oracle(n, edges, weights, labels);
        OracleNaive oraclen(n, edges, weights, labels);

        for (int i=0; i<(int)updates.size(); ++i) {
            printf("%d\n", i);
            
            pair<int, int> update = updates[i];
            oraclen.setLabel(update.first, update.second);
            oracle.setLabel(update.first, update.second);

//            oracle.print();

            for (int t=0; t<T; ++t) {
                int u = rand()%n;
                int v = rand()%n;
            
                auto exact = oraclen.distanceToLabel(u, oraclen.labelOf(v));
                auto approx = oracle.distanceToLabel(u, oracle.labelOf(v));

                assert(exact.first <= approx.first);
                assert(exact.first * cappr >= approx.first);
            
            }
        }
    }


    return 0;
}
