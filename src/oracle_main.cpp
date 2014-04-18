#include "oracle_naive.h"
#include "oracle_general_3approx.h"
#include "oracle_general_5approx.h"
#include "full_planar_oracle.h"
#include "oracle_tester.h"

#include <cstdio>
#include <cassert>
#include <chrono>
#include <string>
using std::string;

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
            case 1: oracle.distanceToLabel(query[i].first, oracle.labelOf(query[i].second)); break;
        }
    }
    return stopTime();
}

template <class O>
inline
double timeLabelProportionTest(O &oracle, int m, const vector<int> &type, const vector< pair<int, int> > &query) {
    startTime();
    for (int i=0; i<m; ++i) {
        switch (type[i]) {
            case 0: oracle.setLabel(query[i].first, query[i].second); break;
            case 1: oracle.distanceBetweenLabels(oracle.labelOf(query[i].first), oracle.labelOf(query[i].second)); break;
        }
    }
    return stopTime();
}
    
const int K = 100;
const int T = 20;
const int M = 10000;

template <class O>
void performVertexToLabelProportionTest(int n, const vector< pair<int, int> > &edges, const vector<W> &weights, float frac = 1.) {

    srand(-1);

    vector<int> labels(n);
// random
    for (int i=0; i<n; ++i) labels[i] = rand() % n;
// groups of K
/*
    for (int i=0; i<n; ++i) labels[i] = i / K;
    vector<int> labelsCopy(labels);
    int last = 0;
*/
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

            switch (type[i]) {
                case 0: query[i] = make_pair(rand()%n, rand()%n); break;
                case 1: query[i] = make_pair(rand()%n, rand()%n); break;
            }
/*
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
*/
        }

        printf("%.12f ", timeProportionTest(oracle, M, type, query)); 
    }

    printf("\n");
}

template <class O>
void performLabelToLabelProportionTest(int n, const vector< pair<int, int> > &edges, const vector<W> &weights, float frac = 1.) {

    srand(-1);

    vector<int> labels(n);
// random
    for (int i=0; i<n; ++i) labels[i] = rand() % n;
// groups of K
/*
    for (int i=0; i<n; ++i) labels[i] = i / K;
    vector<int> labelsCopy(labels);
    int last = 0;
*/
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

            switch (type[i]) {
                case 0: query[i] = make_pair(rand()%n, rand()%n); break;
                case 1: query[i] = make_pair(rand()%n, rand()%n); break;
            }
/*
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
*/
        }

        printf("%.12f ", timeLabelProportionTest(oracle, M, type, query)); 
    }

    printf("\n");
}

template <class O>
void performVertexToLabelGroupTest(int n, const vector< pair<int, int > > &edges, const vector<W> &weights, float frac, string filename) {
    
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
    
    for (int v=0; v<n; ++v) {
        labels[v] = labelCandidates[v][ rand() % labelCandidates[v].size()];
    }
    
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

            int v = rand()%n;

            switch (type[i]) {
                case 0: query[i] = make_pair(v, labelCandidates[v][ rand() % labelCandidates[v].size()]); break;
                case 1: query[i] = make_pair(rand()%n, rand()%n); break;
            }
        }

        printf("%.12f ", timeProportionTest(oracle, M, type, query)); 
    }

    printf("\n");
}

template <class O>
void performLabelToLabelGroupTest(int n, const vector< pair<int, int > > &edges, const vector<W> &weights, float frac, string filename) {
    
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
    
    for (int v=0; v<n; ++v) {
        labels[v] = labelCandidates[v][ rand() % labelCandidates[v].size()];
    }
    
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

            int v = rand()%n;

            switch (type[i]) {
                case 0: query[i] = make_pair(v, labelCandidates[v][ rand() % labelCandidates[v].size()]); break;
                case 1: query[i] = make_pair(rand()%n, rand()%n); break;
            }
        }

        printf("%.12f ", timeLabelProportionTest(oracle, M, type, query)); 
    }

    printf("\n");
}


void printProportionLabels(int n, float frac = 1.) {
    for (int t=0; t<=T; ++t) {
        printf("%.1f ", (float)(t*2-T)/T/frac);
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
}

void performLabelToLabelProportionTestAll(int n, const vector< pair<int, int> > &edges, const vector<W> &weights, float frac = 1.) {
    printProportionLabels(n, frac);
    performLabelToLabelProportionTest<OracleNaiveSet>(n, edges, weights, frac);
    performLabelToLabelProportionTest<OracleGeneral3Approx>(n, edges, weights, frac);
    performLabelToLabelProportionTest<FullFullPlanarOracle>(n, edges, weights, frac);
}

void performVertexToLabelGroupTestAll(int n, const vector< pair<int, int> > &edges, const vector<W> &weights, float frac, string filename) {
    printProportionLabels(n, frac);
    performVertexToLabelGroupTest<OracleNaive>(n, edges, weights, frac, filename);
    performVertexToLabelGroupTest<OracleGeneral3Approx>(n, edges, weights, frac, filename);
    performVertexToLabelGroupTest<OracleGeneral5ApproxUpdate>(n, edges, weights, frac, filename);
    performVertexToLabelGroupTest<OracleGeneral5ApproxQuery>(n, edges, weights, frac, filename);
}

void performLabelToLabelGroupTestAll(int n, const vector< pair<int, int> > &edges, const vector<W> &weights, float frac, string filename) {
    printProportionLabels(n, frac);
    performLabelToLabelGroupTest<OracleNaiveSet>(n, edges, weights, frac, filename);
    performLabelToLabelGroupTest<OracleGeneral3Approx>(n, edges, weights, frac, filename);
}

template <int epsn>
void performPlanarErrorTest(int n, const vector< pair<int, int> > &edges, const vector<W> &weights) {
    srand(-1);

    float eps = (float)epsn/100;
    vector<int> labels(n);
// random
    for (int i=0; i<n; ++i) labels[i] = rand() % n;
// groups of K
/*
    for (int i=0; i<n; ++i) labels[i] = i / K;
    vector<int> labelsCopy(labels);
    int last = 0;
*/
    fprintf(stderr, "Constructing...\n");
    fflush(stderr);
    OracleNaive oraclen(n, edges, weights, labels);
    FullPlanarOracle oracle(n, edges, weights, labels, eps);
    fprintf(stderr, " - done\n");

    printf("%.2f ", 1+eps);

    vector< pair<int, int> > query(M);
    vector< float > result(M);
    vector< float > approx(M);

    for (int i=0; i<M; ++i) {
        query[i] = make_pair(rand()%n, oraclen.labelOf(rand()%n));
        result[i] = oraclen.distanceToLabel(query[i].first, query[i].second).first;
    }

    startTime();

    for (int i=0; i<M; ++i) {
        approx[i] = oracle.distanceToLabel(query[i].first, query[i].second).first;
    }

    printf("%.12f ", stopTime());

    float err = 0;
    for (int i=0; i<M; ++i) {
        if (result[i] == 0) {
            err += 1;
        } else {
            err += approx[i] / result[i];
        }
    }

    printf("%.12f\n", err/M);
}

void performPlanarErrorTestAll(int n, const vector< pair<int, int> > &edges, const vector<W> &weights) {
    performPlanarErrorTest<20>(n, edges, weights);
    performPlanarErrorTest<30>(n, edges, weights);
    performPlanarErrorTest<40>(n, edges, weights);
    performPlanarErrorTest<50>(n, edges, weights);
    performPlanarErrorTest<60>(n, edges, weights);
    performPlanarErrorTest<70>(n, edges, weights);
    performPlanarErrorTest<80>(n, edges, weights);
    performPlanarErrorTest<90>(n, edges, weights);
    performPlanarErrorTest<100>(n, edges, weights);
    performPlanarErrorTest<110>(n, edges, weights);
    performPlanarErrorTest<120>(n, edges, weights);
    performPlanarErrorTest<130>(n, edges, weights);
    performPlanarErrorTest<140>(n, edges, weights);
    performPlanarErrorTest<150>(n, edges, weights);
    performPlanarErrorTest<160>(n, edges, weights);
    performPlanarErrorTest<170>(n, edges, weights);
    performPlanarErrorTest<180>(n, edges, weights);
    performPlanarErrorTest<190>(n, edges, weights);
    performPlanarErrorTest<200>(n, edges, weights);
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
        performVertexToLabelGroupTestAll(n, edges, weights, 2., "../amazon-g.in");
    }
*/
    /*
    {
        performLabelToLabelGroupTestAll(n, edges, weights, 2., "../dblp-g.in");
    }
*/

    {
        performVertexToLabelProportionTestAll(n, edges, weights, 2.);
    }

/*
    {
        performLabelToLabelProportionTestAll(n, edges, weights, 2.);
    }
*/
/*
    {
        performPlanarErrorTestAll(n, edges, weights);
    }
*/
// Correctness test
/*
    {
        srand(24);
        const int K = 500;
        int T = 10;
//        OracleTester::selectQueries(n, 4, K, labels, updates);
        labels.resize(n);
        for (int i=0; i<n; ++i) labels[i] = i;
        updates.resize(K);
        for (auto &u: updates) {
            u = make_pair(rand()%n, rand()%n);
        }

        OracleGeneral3Approx oracle3(n, edges, weights, labels);
        OracleGeneral5ApproxQuery oracle5q(n, edges, weights, labels);
        OracleGeneral5ApproxUpdate oracle5u(n, edges, weights, labels);
        OracleNaive oraclen(n, edges, weights, labels);
        FullPlanarOracle oraclep(n, edges, weights, labels, 0.5);
        FullFullPlanarOracle oraclepp(n, edges, weights, labels, 0.5);
	
	double oracle3Err = 0, oracle5qErr = 0, oracle5uErr = 0, oraclepErr = 0;
	double oracle3LLErr = 0, oraclepLLErr = 0;

        for (int i=0; i<(int)updates.size(); ++i) {
            printf("%d\n", i);
            
            pair<int, int> update = updates[i];
            oraclen.setLabel(update.first, update.second);
            oracle3.setLabel(update.first, update.second);
            oracle5q.setLabel(update.first, update.second);
            oracle5u.setLabel(update.first, update.second);
            oraclep.setLabel(update.first, update.second);
            oraclepp.setLabel(update.first, update.second);

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
                auto approxp = oraclepp.distanceBetweenLabels(oraclepp.labelOf(u), oraclepp.labelOf(v));

                assert(exact.first <= approx3.first);
                assert(exact.first <= approxp.first);
                assert(exact.first * 3 >= approx3.first);
                assert(exact.first * 1.5 >= approxp.first);

		if(exact.first == 0) {
			oracle3LLErr += 1;
			oraclepLLErr += 1;
		} else {
			oracle3LLErr += approx3.first / exact.first;
			oraclepLLErr += approxp.first / exact.first;
		}
            }
        }

	cout << "approx 3 " << oracle3Err / (K*T) << endl;
	cout << "approx u 5 " << oracle5uErr / (K*T) << endl;
	cout << "approx q 5 " << oracle5qErr / (K*T) << endl;
	cout << "approx p 1.5 " << oraclepErr / (K*T) << endl;
	
	cout << "approx 3 l-l " << oracle3LLErr / (K*T) << endl;
	cout << "approx p l-l " << oraclepLLErr / (K*T) << endl;
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
