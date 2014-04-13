#ifndef _FIND_UNION_H_
#define _FIND_UNION_H_

#include <vector>
using std::vector;

struct FindUnion {
    vector<int> p;
    
    FindUnion(int n) : p(n) {
        for (int i=0; i<n; ++i) p[i] = i;
    }

    int find(int v) {
        if (p[v] == v) return v;
        return p[v] = find(p[v]);
    }

    void unionn(int v, int u) {
        p[find(u)] = find(v);
    }

};

#endif
