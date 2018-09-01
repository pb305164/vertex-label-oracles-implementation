#include <tuple>
#include <queue>
#include <cassert>
#include "LCATree.h"

using namespace std;


void dijkstra(int n, vector<W> &dist, vector<int> &parent, vector<vector<Edge>> &edges, int s) {
    typedef pair<W, int> QEl;
    priority_queue<QEl, vector<QEl>, greater<QEl> > queue;

    dist[s] = 0;
    queue.push(make_pair(0, s));
    while (!queue.empty()) {
        QEl curr = queue.top();
        queue.pop();
        int v = curr.second;
        W d = curr.first;
        if (d != dist[v]) continue;
        for (Edge e : edges[v]) {
            int u = e.destination;
            if (dist[u] > d + e.weight) {
                dist[u] = d + e.weight;
                parent[u] = v;
                queue.push(make_pair(dist[u], u));
            }
        }
    }
}

void euler_tour(vector<int> &e, vector<int> &l, vector<int> &r, int v, vector<vector<int>> &children, int lvl) {
    if (r[v] == -1) {
        r[v] = (int)e.size();
    }
    e.push_back(v);
    l.push_back(lvl);
    for (int c: children[v]) {
        euler_tour(e, l, r, c, children, lvl+1);
        e.push_back(v);
        l.push_back(lvl);
    }
}

void preprocess_RMQ(vector<vector<int>> &st, vector<int> &l)
{
    int log=0;
    for (int tmp=1; tmp<(int)l.size(); tmp*=2) {
        log++;
    }
    vector<vector<int>> stl(l.size(), vector<int>(log, -1));
    st.resize(l.size(), vector<int>(log, -1));

    //initialize M for the intervals with length 1
    for (int i = 0; i < (int)l.size(); i++) {
        st[i][0] = i;
        stl[i][0] = l[i];
    }
    //compute values from smaller to bigger intervals
    for (int j = 1; j<log; j++) {
        for (int i=0; i + (1 << j) < (int)l.size(); i++) {
            if (stl[i][j - 1] < stl[i + (1 << (j - 1))][j - 1]) {
                stl[i][j] = stl[i][j - 1];
                st[i][j] = st[i][j - 1];
            } else {
                stl[i][j] = stl[i + (1 << (j - 1))][j - 1];
                st[i][j] = st[i + (1 << (j - 1))][j - 1];
            }
        }
    }
}

LCATree::LCATree(int _size, int root, vector<vector<Edge>> &edges): distance(_size, infinity), parent(_size, -1), r(_size, -1) {
    vector<std::vector<int>> children(_size);

    dijkstra(_size, distance, parent, edges, root);

    for (int i=0; i<_size; i++) {
        if (i != root) {
            assert(parent[i] != -1);
            children[parent[i]].push_back(i);
        }
    }

    euler_tour(e, l, r, root, children, 0);
    int max_r = -1;
    for (int i: r) {
        if (i > max_r) max_r = i;
    }
    ++max_r;
    e.erase(e.begin()+max_r, e.end());
    l.erase(l.begin()+max_r, l.end());
    preprocess_RMQ(st, l);
}

W LCATree::query(int s, int t) {
    if (s == t) {
        return 0;
    }
    int a = min(r[s], r[t]), b = max(r[s], r[t]), log=0, p=1;
    while (2*p<b-a) {
        p*=2;
        log++;
    }
    if (l[st[a][log]] < l[st[b-p][log]]) {
        return distance[s] + distance[t] - 2*distance[e[st[a][log]]];
    }
    return distance[s] + distance[t] - 2*distance[e[st[b-p][log]]];
}

vector<int> LCATree::find_path(int s, int t) {
    vector<int> path, rev_path;
    if (s == t) {
        path.push_back(s);
        return path;
    }
    int a = min(r[s], r[t]), b = max(r[s], r[t]), log=0, p=1;
    while (2*p<b-a) {
        p*=2;
        log++;
    }
    int i = s, lca;
    if (l[st[a][log]] < l[st[b-p][log]]) {
        lca = e[st[a][log]];
    } else {
        lca = e[st[b-p][log]];
    }
    while (i != lca) {
        path.push_back(i);
        i = parent[i];
    }
    path.push_back(lca);
    i = t;
    while (i != lca) {
        rev_path.push_back(i);
        i = parent[i];
    }
    path.insert(path.end(), rev_path.rbegin(), rev_path.rend());
    return path;
}