#include "oracle_general.h"

#include <cstdio>

int main() {
    int n;
    vector< pair<int, int> > edges;
    vector< W > weights;

    scanf("%d", &n);
    int u, v;
    W w;
    while (scanf("%d %d %f", &u, &v, &w) != EOF) {
        edges.push_back(make_pair(u,v));
        weights.push_back(w);
    }

    OracleGeneral oracle(n, edges, weights);

    for (int i=0; i<n; ++i) {
        for (int j=0; j<n; ++j) {
            pair<W, pair<int, int> > ans = oracle.distanceBetweenLabels(i, j);
            printf("%d %d -> %f : %d %d\n", i, j, ans.first, ans.second.first, ans.second.second);
        }
    }

    return 0;
}
