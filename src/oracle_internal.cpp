#include "oracle_internal.h"
#include "planar.h"

#include <cassert>
#include <queue>
#include <cstdio>

using std::printf;
using std::priority_queue;
using std::pair;
using std::make_pair;
using std::greater;
using std::min;
using std::max;

void
getDistances(
        const PlanarGraph& g,
        int u,
        vector<W> &dist) {
    
    dist = vector<W>(g.vs().size(), infinity);
    typedef pair<W, int> QEl;
    priority_queue< QEl, vector<QEl>, greater<QEl> > queue;
   
    dist[u] = 0;
    queue.push(make_pair(0, u));
    while (!queue.empty()) {
        QEl curr = queue.top(); queue.pop();
        int u = curr.second;
        W d = curr.first;
        if (d != dist[u]) continue;
        for (int e: g.vs()[u].edges) {
            int v = g.opp(u, e);
            if (dist[v] > d + g.es()[e].w) {
                dist[v] = d + g.es()[e].w;
                queue.push(make_pair(dist[v], v));
            }
        }
    }

}

pair<W, W>
getStretch(
        const PlanarGraph& g) { 
    W minD = infinity, maxD = 0;
    vector<W> distance;
    getDistances(g, 0, distance);
    for (int i=1; i<(int)distance.size(); ++i) {
        maxD = max(maxD, distance[i]);
    }
    maxD *= 2;
    for (int i=0; i<(int)g.es().size(); ++i) {
        minD = min(minD, g.es()[i].w);
    }

    return make_pair(minD, maxD);
}
    
void
extractSubgraph(
        const PlanarGraph& g,
        const vector<int>& parent,
        vector<int>& selection,
        PlanarGraph& subg,
        vector<int>& subparent,
        vector<int>& mapping,
// three vectors fiiled with -1's
        vector<int>& vInd, 
        vector<int>& eInd) {
    
    typedef PlanarGraph::Vertex Vertex;

    subg = PlanarGraph(1);
    mapping.clear();
    mapping.push_back(-1);

    for (int v: vInd) assert(v == -1);

    for (int v: selection) {
        vInd[v] = subg.vs().size();
        mapping.push_back(v);
        subg.vs().push_back(Vertex());
    }

    subparent = vector<int>(subg.vs().size(), -1);
    
    for (int vv: selection) {
        for (int e: g.vs()[vv].edges) {
            if (eInd[e] != -1) continue;

            int u = g.es()[e].u;
            int v = g.es()[e].v;
            bool su = vInd[u] != -1;
            bool sv = vInd[v] != -1;
            if (su && sv) {
                if (e == parent[u]) {
                    subparent[vInd[u]] = subg.es().size();
                }
                if (e == parent[v]) {
                    subparent[vInd[v]] = subg.es().size();
                }
                eInd[e] = subg.es().size();
                subg.add_edge(vInd[u], vInd[v], g.es()[e].w);
            } else if((e == parent[u]) && su) {
                subparent[vInd[u]] = subg.es().size();
                eInd[e] = subg.es().size();
                subg.add_edge(0, vInd[u], infinity);
            } else if ((e == parent[v]) && sv) {
                subparent[vInd[v]] = subg.es().size();
                eInd[e] = subg.es().size();
                subg.add_edge(0, vInd[v], infinity);
            }

        }
    }
/*
    for (int vv: selection) {
        if (g.vs()[vv].edges.empty()) continue;

        int lastE = -1;
        int e = g.vs()[vv].edges[0], f = e;
        do {
           if (eInd[e] != -1) {
                if (lastE != -1) subg.eNext(vInd[vv], lastE) = eInd[e];
                lastE = eInd[e];
           }
           e = g.eNext(vv,e);
        } while (e != f);
        do {
           if (eInd[e] != -1) {
                if (lastE != -1) subg.eNext(vInd[vv], lastE) = eInd[e];
                lastE = eInd[e];
           }
           e = g.eNext(vv,e);
        } while (e != f);
    }
    

    vector<int> ePrev(subg.es().size(), -1);
    for (int e: subg.vs()[0].edges) {
        subg.eNext(0, e) = -1;

        int f = e, u = subg.opp(0, e);
        while (u != 0) {
            f = subg.eNext(u, f);
            u = subg.opp(u, f);
        }
        ePrev[e] = f;
    }

    int lastE = 0;
    for (int e: subg.vs()[0].edges) {
        int f = e;
        while ((subg.eNext(0, f) == -1)) {
            subg.eNext(0, f) = lastE;
            lastE = f;
            f = ePrev[f];
        }
    }
    if (!subg.vs()[0].edges.empty()) subg.eNext(0, subg.vs()[0].edges[0]) = lastE;
*/

    if (subg.vs()[0].edges.empty() && subg.vs().size()>1) {
        subparent[vInd[0]] = subg.es().size();
        subg.add_edge(0, vInd[0], infinity);
    }
/*        
        subg.eNext(0, subparent[vInd[0]]) = subparent[vInd[0]];

        subg.eNext(vInd[0], subparent[vInd[0]]) = subg.eNext(vInd[0], subg.vs()[vInd[0]].edges[0]);
        subg.eNext(vInd[0], subg.vs()[vInd[0]].edges[0]) = subparent[vInd[0]];
    }
*/
    for (int v: selection) {
        vInd[v] = -1;
    }
    for (int v: selection) {
        for (int e: g.vs()[v].edges) {
            eInd[e] = -1;
        }
    }

    return;
}

static void printEmbedded(PlanarGraph& pg) {
    for (int v=0; v<(int)pg.vs().size(); ++v) {
        printf("Vertex %d:\n", v);
        if (pg.vs()[v].edges.empty()) continue;
        int e_end = pg.vs()[v].edges[0], e = e_end;
        do {
            printf("%d - %d\n", pg.es()[e].u, pg.es()[e].v);
            e = pg.eNext(v, e);
        } while (e != e_end);
    }
    return;
}

void
subdivide(
        PlanarGraph g,
        const vector<int>& parent,
        vector< PlanarGraph >& subgs,
        vector< vector<int> >& mappings,
        vector< vector<int> >& parents,
        vector< vector< pair<int, int > > >& paths) {
    if (g.vs().size() <= 3) return;

    triangulate(g);
    int eC = 0;
    vector< vector< pair< int, int > > > eNum(g.es().size());

    if (g.vs()[0].edges.empty()) return;
    vector<int> eque;
    int v = 0, f = g.vs()[0].edges[0], ff, e = ff = g.eNext(v,f);
    g.eNext(0,f) = -1;

    while (e != -1) {
        if (e == parent[v]) {
            v = g.opp(v,e); ++eC; eque.push_back(e);
            e = g.eNext(v,e);
        } else {
            int u = g.opp(v, e);
            if (e == parent[u]) {
                v = u; ++eC; eque.push_back(e);
                e = g.eNext(v,e);
            } else {
                eNum[e].push_back(make_pair(v, eC));
                e = g.eNext(v,e);
            }
        }
    }
    g.eNext(0,f) = ff;

    int x[3], xe[3], xl[3], xr[3];
    x[0] = 0;
    xe[0] = g.vs()[0].edges[0];

//    printEmbedded(g);

    bool stop = false;   
    while (!stop) {
        stop = true;

        x[1] = g.opp(x[0], xe[0]);
        xe[1] = g.eNext(x[1], xe[0]);
        x[2] = g.opp(x[1], xe[1]);
        xe[2] = g.eNext(x[2], xe[1]);
        assert(x[0] == g.opp(x[2], xe[2]));

        for (int i=0; i<3; ++i) {
            int a = x[i], b = x[(i+1)%3];
            int e = xe[i];
            int n;

            if (eNum[e].empty()) {
                xr[i] = -1;
                xl[i] = -1;
                continue;
            } else if (eNum[e][0].first == a && eNum[e][1].first == b) {
                xr[i] = eNum[e][1].second;
                xl[i] = eNum[e][0].second;
                n = xr[i] - xl[i];
            } else if (eNum[e][0].first == b && eNum[e][1].first == a) {
                xr[i] = eNum[e][0].second;
                xl[i] = eNum[e][1].second;
                n = xr[i] - xl[i];
            } else {
                assert(false);
            }
            if (n <= 0) n += (g.vs().size()-1) * 2;
            if (n >  ((int)g.vs().size()-1)*2 / 2 ) { // log_{2}
                stop = false;
                xe[0] = xe[i];
                x[0] = x[(i+1)%3];
                break;
            }
        }
    }

    vector<int> vsplit(g.vs().size(), -1);
    for (int i=0; i<3; ++i) {
        if (xl[i] == -1) continue;
        
        bool xxor = xl[i] >= xr[i];
        for (int j=0; j<(int)eque.size(); ++j) {
            if (((xl[i] <= j) && (j < xr[i])) ^ xxor) {
                vsplit[g.es()[eque[j]].u] = i;
                vsplit[g.es()[eque[j]].v] = i;
            }
        }
    }

    for (int i=0; i<3; ++i) {
        vector< pair< int, int > > path;
        int v = x[i];
        while (v != 0) {
            vsplit[v] = -1;
            path.push_back(make_pair(v, parent[v]));
            v = g.opp(v, parent[v]);
        }
        path.push_back(make_pair(0, -1));
        paths.push_back(path);
    }
    vsplit[0] = -1;

    vector< int > vInd(g.vs().size(), -1);
    vector< int > eInd(g.es().size(), -1);
    for (int i=0; i<3; ++i) {
        if (xl[i] == -1) continue;
        
        vector<int> selection;
        for (int j=0; j<(int)g.vs().size(); ++j) {
            if (vsplit[j] == i) selection.push_back(j);
        }

        PlanarGraph tmpSubg;
        vector<int> tmpMapping, tmpParent;
    
        extractSubgraph(
            g, parent, selection,
            tmpSubg, tmpParent, tmpMapping,
            vInd, eInd);
        
        subgs.push_back(tmpSubg);
        mappings.push_back(tmpMapping);
        parents.push_back(tmpParent);

    } 
}
