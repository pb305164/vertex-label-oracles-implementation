#include "precision.h"
#include "planar_oracle.h"

#include <algorithm>
#include <queue>
#include <iostream>
using namespace std;
using std::min;
using std::max;
using std::sort;
using std::priority_queue;
using std::greater;
using std::make_pair;

void
PlanarOracle::initialize(
        int n,
        const vector< pair< int, int > >& edge, 
        const vector< W >& weight,
        W eps) {
    PlanarGraph graph(n);

    for (int i=0; i<(int)edge.size(); ++i) {
        graph.add_edge(edge[i].first, edge[i].second, weight[i]);
    }
    embed(graph);

    pair<W, W> stretch(getStretch(graph));
    W alpha = min(stretch.second,max((W)60,stretch.first));
    //W alpha=stretch.first;
    cout << "Stretch = " << stretch.first << " ... " << stretch.second << std::endl;
    int rounds=0;
    while (alpha <= stretch.second) {
       rounds++;
//	cerr << alpha << endl;

        vector< vector<int> > tmpParents, tmpMappings;
        vector< vector<bool> > tmpSources;
        vector< PlanarGraph > tmpSubgs;

        getAlphaFamily(graph, alpha, tmpSubgs, tmpMappings, tmpParents, tmpSources);

        assert(tmpSources.size() == tmpMappings.size());
       
        for (int i=0; i<(int)tmpSubgs.size(); ++i) {

            for (int j=1; j<(int)tmpMappings[i].size(); ++j) {
                assert(tmpMappings[i][j] != -1);
            }

            subdivideRecursively(
                    tmpSubgs[i],
                    alpha,
                    eps,
                    tmpMappings[i],
                    tmpParents[i],
                    tmpSources[i]);
        }

        alpha *= 2;
    }
    cout << "log(Stretch) = " << rounds << endl;
}

void
PlanarOracle::getLayers(
        vector<W> dist,
        W alpha,
        vector<int>& layerNum) {
    layerNum = vector<int>(dist.size());
    for (int i=0; i<(int)dist.size(); ++i) {
        layerNum[i] = (int)(dist[i] / alpha);
    }
}

void
PlanarOracle::getAlphaFamily(
        const PlanarGraph& g, 
        W alpha, 
        vector< PlanarGraph >& subgs,
        vector< vector<int> >& mappings,
        vector< vector<int> >& parents,
        vector< vector<bool> >& sources) {

    vector<int> parent(g.vs().size(), -1);

// almost getDistances
// however, we calculate parent table
    vector<W> dist(g.vs().size(), infinity); 
    typedef pair<W, int> QEl;
    priority_queue< QEl, vector<QEl>, greater<QEl> > queue;

    dist[0] = 0;
    queue.push(make_pair(0, 0));
    while (!queue.empty()) {
        QEl curr = queue.top(); queue.pop();
        int u = curr.second;
        W d = curr.first;
        if (d != dist[u]) continue;
        for (int e: g.vs()[u].edges) {
            int v = g.opp(u, e);
            if (dist[v] > d + g.es()[e].w) {
                dist[v] = d + g.es()[e].w;
                parent[v] = e;
                queue.push(make_pair(dist[v], v));
            }
        }
    }

    vector<int> layerNum;
    getLayers(dist, alpha, layerNum);
    
    int maxLayer = 0;
    for (int i=0; i<(int)g.vs().size(); ++i) {
        maxLayer = max(maxLayer, layerNum[i]);
    }
    vector< vector<int> > layers(maxLayer+1);
    for (int i=0; i<(int)g.vs().size(); ++i) {
        if (i) {
            assert(parent[i] != -1);
            assert(layerNum[g.opp(i, parent[i])] <= layerNum[i]);
        }
        layers[layerNum[i]].push_back(i);
    }

    //vector< pair< PlanarGraph, vector<int> > > result;
    vector< int > vInd(g.vs().size(), -1);
    vector< int > eInd(g.es().size(), -1);
   
    for (int l = 0; l <= maxLayer; ++l) {
        int la = l-1, lb = l+1; 
        
        vector<int> selection;
        for (int i=max(0, la); i<=min(maxLayer, lb); ++i) {
            selection.insert(selection.end(), layers[i].begin(), layers[i].end());
        }

        PlanarGraph subg;
        vector<int> mapping, subparent;

        extractSubgraph(
            g, parent, selection,
            subg, subparent, mapping,
            vInd, eInd);

        assert(isPlanar(subg));
        
        vector<bool> source(mapping.size(), false);
        for (int i=0; i<(int)mapping.size(); ++i) {
            if (mapping[i] == -1) continue;
            if (layerNum[mapping[i]] == l)
                source[i] = true;
        }

        subgs.push_back(subg);
        mappings.push_back(mapping);
        parents.push_back(subparent);
        sources.push_back(source);
    }
}

void
PlanarOracle::selectPathPortals(
        const PlanarGraph& g, 
        W alpha, 
        W eps,
        vector< vector< pair<int, int> > > paths,
        vector<int>& portal) {

    assert(paths.size() <= 3);
    portal.clear();    
/*
    for (int j=0; j<(int)paths.size(); ++j) {
        pair<int, int> prevV(-1, -1);
        W dist = infinity;

        for (int k=paths[j].size()-2; k>=0; --k) {
            int v = paths[j][k].first;
            W w;
            if (k != 0) {
                w = g.es()[paths[j][k-1].second].w;
            } else {
                w = g.es()[paths[j][paths.size()-2].second].w;
            }

            if (dist + w > alpha*eps/2) {
                portal.push_back(v);
                dist = 0;
            }

            dist += w;
        }
    }
*/
    for (int j=0; j<(int)paths.size(); ++j) {
//        pair<int, int> prevV(-1, -1);
        W dist = infinity;
        for (int k=0; k<(int)paths[j].size()-1; ++k) {
            int v = paths[j][k].first;
            W w = g.es()[paths[j][k].second].w;

            if (dist + w > alpha*eps/2) {
                portal.push_back(v);
                dist = 0;
            }

            dist += w;
        }
    }

    sort(portal.begin(), portal.end());
    auto it = unique(portal.begin(), portal.end());
    portal.resize(std::distance(portal.begin(), it));

    assert((int)portal.size() <= 3 * 3 * (int)(1/eps + 1) * 4);
}

void
PlanarOracle::subdivideRecursively(
        const PlanarGraph& pg,
        W alpha,
        W eps,
        const vector<int>& mapping,
        const vector<int>& parents,
        const vector<bool>& sources) {

    if ((int)pg.vs().size() <= ro) {
        processLeaf(pg, mapping, sources);
        return;
    }

    vector< vector<int> > tmpParents, tmpMappings;
    vector< PlanarGraph > tmpSubgs;
    vector< vector< pair<int, int> > > tmpPaths;
    vector<int> newPortals;
    
    subdivide(pg, parents, tmpSubgs, tmpMappings, 
            tmpParents, tmpPaths);

// Sciezki
    selectPathPortals(pg, alpha, eps, tmpPaths, newPortals);
    processPortals(pg, mapping, newPortals, sources);

// Rekursja
    for (int j=0; j<(int)tmpSubgs.size(); ++j) {
        vector<bool> tmpSources;
        for (int k=0; k<(int)tmpMappings[j].size(); ++k) {
            if (tmpMappings[j][k] == -1) {
                tmpSources.push_back(false);
                continue;
            }
            tmpSources.push_back(sources[tmpMappings[j][k]]);
            tmpMappings[j][k] = mapping[tmpMappings[j][k]];
        }
        
        subdivideRecursively(
                tmpSubgs[j],
                alpha,
                eps,
                tmpMappings[j],
                tmpParents[j],
                tmpSources);
    }
}
