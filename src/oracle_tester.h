#ifndef _ORACLE_TESTER_H_
#define _ORACLE_TESTER_H_

#include "precision.h"
#include "find_union.h"

#include <vector>
#include <utility>
#include <set>

using std::pair;
using std::vector;
using std::set;
using std::make_pair;

class OracleTester {
public:
    static
    void readGraphFromInput(int &n, vector< pair<int, int> > &edges, vector< W > &weights) {
        scanf("%d", &n);
        int u, v;
        W w;
        while (scanf("%d %d %f", &u, &v, &w)) {
            edges.push_back(make_pair(u, v));
            weights.push_back(w);
        }
    }

    static
    void generateGraph(int nn, int m, int w, int& n, vector< pair<int, int> > &edges, vector< W > &weights) {
        n = nn;
        int counter = n;
        FindUnion fu(n);
        set< pair<int, int> > e;

        for (int i=0; i<m-counter+1; ++i) {
            int u = rand()%n;
            int v = rand()%n;
            if (u > v) swap(u,v);
            if (u == v) continue;
            if (e.find(make_pair(u,v)) != e.end()) continue;
            e.insert(make_pair(u,v));
            if (fu.find(u) == fu.find(v)) continue;
            fu.unionn(u,v);
            counter--;
        }

        while (counter > 1) {
            int u = rand()%n;
            int v = rand()%n;
            if (u > v) swap(u,v);
            if (u == v) continue;
            if (e.find(make_pair(u,v)) != e.end()) continue;
            if (fu.find(u) == fu.find(v)) continue;
            e.insert(make_pair(u,v));
            fu.unionn(u,v);
            counter--;
        }

        for (pair<int, int> p: e) {
            edges.push_back(p);
            weights.push_back(rand()%w+1);
        }
    }

    static
    void selectQueries(int n, int k, int m, vector<int> &labels, vector< pair<int, int> > &updates) {
        vector<int> curr(n);
        for (int i=0; i<n; ++i) curr[i] = i/k;
        random_shuffle(curr.begin(), curr.end());
        labels = curr;
        
        int v = rand()%n;
        for (int i=0; i<m; ++i) {
            int u = rand()%n;
            updates.push_back(make_pair(v, curr[u]));
            curr[v] = curr[u];
            v = u;
        }
    }

};

#endif
