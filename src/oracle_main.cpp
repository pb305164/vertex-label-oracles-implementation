#include "oracle_naive.h"
#include "oracle_general_3approx.h"
#include "oracle_general_5approx.h"
#include "full_planar_oracle.h"
#include "oracle_tester.h"

#include <cstdio>
#include <cassert>
#include <chrono>

using namespace std::chrono;

high_resolution_clock::time_point start, stop;
void startTime() {
    start = high_resolution_clock::now();
}

float stopTime() {
    stop = high_resolution_clock::now();
    return duration_cast< duration<float> >(stop - start).count();
}

void stopTimePrint() {
    stop = high_resolution_clock::now();
    printf("%.12f\n", duration_cast< duration<float> >(stop - start).count());
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
const int M = 10000;

template <class O>
void performVertexToLabelProportionTest(int n, const vector< pair<int, int> > &edges, const vector<W> &weights, float frac = 1.) {

    srand(-1);

    vector<int> labels(n);
// random
    //    for (int i=0; i<n; ++i) labels[i] = rand() % n;
// groups of K
    for (int i=0; i<n; ++i) labels[i] = i / K;
    vector<int> labelsCopy(labels);
    int last = 0;

    fprintf(stderr, "Constructing...\n");
    fflush(stderr);
    O oracle(n, edges, weights, labels);
    fprintf(stderr, " - done\n");

    for (int t=0; t<=T; ++t) {
        fprintf(stderr, "t: %d\n", t);
        vector< int > type(M);
        vector< pair<int, int> > query(M);

        int a = pow(n, 0.5+(float)(t-T/2)/T/frac);
        int b = pow(n, 0.5+(float)(T/2-t)/T/frac);
        vector<int> typeCycle(a+b);
        for (int i=0; i<a; ++i) typeCycle[i] = 0;
        for (int i=a; i<a+b; ++i) typeCycle[i] = 1;
        random_shuffle(typeCycle.begin(), typeCycle.end());

        int tc = 0;
        for (int i=0; i<M; ++i) {
            type[i] = typeCycle[tc++];
            if (tc == a+b) tc = 0;
// random
/*
            switch (type[i]) {
                case 0: query[i] = make_pair(rand()%n, rand()%n); break;
                case 1: query[i] = make_pair(rand()%n, rand()%n); break;
            }
*/
// groups of K
            int v;
            switch (type[i]) {
                case 0: 
                    v = rand()%n;
                    query[i] = make_pair(last, labelsCopy[v]); 
                    labelsCopy[last] = labelsCopy[v];
                    last = v;
                    break;
                case 1: 
                    query[i] = make_pair(rand()%n, labelsCopy[rand()%n]); 
                    break;
            }

        }

        printf("%.12f ", timeProportionTest(oracle, M, type, query)); 
    }

    printf("\n");
}

void printProportionLabels(int n, float frac = 1.) {
    for (int t=0; t<=T; ++t) {
        printf("%.1f ", (float)(t*2-T)/T/frac);
    }
    printf("\n");
}

template <class O>
void performLabelToLabelGroupTest(int n, const vector< pair<int, int> > &edges, const vector<W> &weights, string filename) {

    srand(-1);
    int gc = 0;

    vector<int> labels(n);
    vector< vector<int> > labelCandidates(n);
    
    vector< vector<int> > groups;
    OracleTester::readGroupsFromFile(filename, groups);
    fprintf(stderr, "groups! %d\n", (int)groups.size());

    gc = 0;
    for (vector<int> &g: groups) {
        for (int v: g) {
            labelCandidates[v].push_back(gc);       
        }
        ++gc;
    }
    for (vector<int> &c: labelCandidates) {
        if (c.empty()) {
            groups.push_back(vector<int>(1, gc));
            c.push_back(gc++);
        }
    }

// static
/*
    {
        gc = 0;
        unordered_map<int, int> gMap;
        for (int v=0; v<n; ++v) {
            int l = labelCandidates[v][ rand() % labelCandidates[v].size()];
            if (gMap.find(l) == gMap.end()) gMap[l] = gc++;
            labels[v] = gMap[l];
        }
    }
*/
// dynamic 

    {
        fprintf(stderr, "%d <= %d ?\n", gc, n);
        assert(gc <= n);
        vector<int> order(gc);
        for (int i=0; i<gc; ++i) order[i] = i;
        random_shuffle(order.begin(), order.end());
        for (int o: order) {
            for (int v: labelCandidates[o]) {
                labels[v] = o;
            }
        }
    }

    vector< pair<int, int> > query(M);
    for (auto &q: query) {
        q = make_pair(rand() % gc, rand() % gc);
    }
    vector<int> checkTime(T+1);
    for (int i=0; i<(int)checkTime.size(); ++i) {
        checkTime[i] = M / T * i;
    }

    startTime();
    
    fprintf(stderr, "Constructing...\n");
    fflush(stderr);
    O oracle(n, edges, weights, labels);
    fprintf(stderr, " - done\n");

    
    int q = 0;
    for (int t=0; t<=T; ++t) {
        fprintf(stderr, "tt: %d %d\n", t, checkTime[t]);

        while (q != checkTime[t]) {
// static
//            oracle.distanceBetweenLabels(query[q].first, query[q].second);
// dynamic 

            int g1 = query[q].first;
            for (int v: groups[g1]) {
                if (oracle.labelOf(v) != g1) {
                    oracle.setLabel(v, g1);
                }
            }
            int g2 = query[q].second;
            for (int v: groups[g2]) {
                if (oracle.labelOf(v) != g2) {
                    oracle.setLabel(v, g2);
                }
            }
            oracle.distanceBetweenLabels(g1, g2);

            ++q;
        }
        printf("%.12f ", stopTime());
    }
    printf("\n");
}

void printGroupLabels(int n) {
    for (int t=0; t<=T; ++t) {
        printf("%d ", M / T * t);
    }
    printf("\n");
}

void performVertexToLabelProportionTestAll(int n, const vector< pair<int, int> > &edges, const vector<W> &weights, float frac = 1.) {
    printProportionLabels(n, frac);
    performVertexToLabelProportionTest<OracleNaive>(n, edges, weights, frac);
    performVertexToLabelProportionTest<OracleGeneral3Approx>(n, edges, weights, frac);
    performVertexToLabelProportionTest<OracleGeneral5ApproxUpdate>(n, edges, weights, frac);
    performVertexToLabelProportionTest<OracleGeneral5ApproxQuery>(n, edges, weights, frac);
    performVertexToLabelProportionTest<FullPlanarOracle>(n, edges, weights, frac);
    performVertexToLabelProportionTest<FullPlanarOracle2>(n, edges, weights, frac);
}

void performLabelToLabelGroupTestAll(int n, const vector< pair<int, int> > &edges, const vector<W> &weights, string filename) {
    printGroupLabels(n);
    performLabelToLabelGroupTest<OracleNaiveSet>(n, edges, weights, filename);
    performLabelToLabelGroupTest<OracleGeneral3Approx>(n, edges, weights, filename);
}

int main() {
    int n;
    vector< pair<int, int> > edges;
    vector< W > weights;

    vector< int > labels;
    vector< pair< int, int > > updates, queries;

//    OracleTester::generateGraph(2000, 8000, 200, n, edges, weights);
    OracleTester::readGraphFromInput(n, edges, weights);

    fprintf(stderr, "Read %d %d!\n", n, (int)edges.size());
    fflush(stderr);
/*
    {
        performLabelToLabelGroupTestAll(n, edges, weights, "../dblp-g.in");
    }
*/

    {
        performVertexToLabelProportionTestAll(n, edges, weights, 2.);
    }


// Correctness test
/*
    {
        const int K = 500;
        int T = 10;
        OracleTester::selectQueries(n, 4, K, labels, updates);

        OracleGeneral3Approx oracle3(n, edges, weights, labels);
        OracleGeneral5ApproxQuery oracle5q(n, edges, weights, labels);
        OracleGeneral5ApproxUpdate oracle5u(n, edges, weights, labels);
        OracleNaive oraclen(n, edges, weights, labels);
        FullPlanarOracle oraclep(n, edges, weights, labels, 0.5);
	
	double oracle3Err = 0, oracle5qErr = 0, oracle5uErr = 0, oraclepErr = 0;
	double oracle3LLErr = 0;

        for (int i=0; i<(int)updates.size(); ++i) {
            printf("%d\n", i);
            
            pair<int, int> update = updates[i];
            oraclen.setLabel(update.first, update.second);
            oracle3.setLabel(update.first, update.second);
            oracle5q.setLabel(update.first, update.second);
            oracle5u.setLabel(update.first, update.second);
            oraclep.setLabel(update.first, update.second);

            for (int t=0; t<T; ++t) {
                int u = rand()%n;
                int v = rand()%n;

                auto exact = oraclen.distanceToLabel(u, oraclen.labelOf(v));
                auto approx3 = oracle3.distanceToLabel(u, oracle3.labelOf(v));
                auto approx5q = oracle5q.distanceToLabel(u, oracle5q.labelOf(v));
                auto approx5u = oracle5u.distanceToLabel(u, oracle5u.labelOf(v));
                auto approxp = oraclep.distanceToLabel(u, oraclep.labelOf(v));

                assert(exact.first <= approx3.first);
                assert(exact.first <= approx5q.first);
                assert(exact.first <= approx5u.first);
                assert(exact.first <= approxp.first);
                assert(exact.first * 3 >= approx3.first);
                assert(exact.first * 5 >= approx5q.first);
                assert(exact.first * 5 >= approx5u.first);
                assert(exact.first * 1.5 >= approxp.first);
                
		if (exact.first == 0) {
			oracle3Err += 1;
			oracle5qErr += 1;
			oracle5uErr += 1;
			oraclepErr += 1;
		} else {
			oracle3Err += approx3.first / exact.first;
			oracle5qErr += approx5q.first / exact.first;
			oracle5uErr += approx5u.first / exact.first;
			oraclepErr += approxp.first / exact.first;
		}
            }

            for (int t=0; t<T; ++t) {
            int u = rand()%n;
                int v = rand()%n;

                auto exact = oraclen.distanceBetweenLabels(oraclen.labelOf(u), oraclen.labelOf(v));
                auto approx3 = oracle3.distanceBetweenLabels(oracle3.labelOf(u), oracle3.labelOf(v));

                assert(exact.first <= approx3.first);
                assert(exact.first * 3 >= approx3.first);

		if(exact.first == 0) {
			oracle3LLErr += 1;
		} else {
			oracle3LLErr += approx3.first / exact.first;
		}
            }
        }

	cout << "approx 3 " << oracle3Err / (K*T) << endl;
	cout << "approx u 5 " << oracle5uErr / (K*T) << endl;
	cout << "approx q 5 " << oracle5qErr / (K*T) << endl;
	cout << "approx p 1.5 " << oraclepErr / (K*T) << endl;
	
	cout << "approx 3 l-l " << oracle3LLErr / (K*T) << endl;
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
