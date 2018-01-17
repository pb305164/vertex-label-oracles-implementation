#ifndef _ORACLE_INTERNAL_H_
#define _ORACLE_INTERNAL_H_

#include "planar.h"
#include <utility>

using std::pair;

void
getDistances(
        const PlanarGraph& g,
        int u,
        vector<W> &distances);

pair<W, W>
getStretch(const PlanarGraph& g);

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
        vector<int>& eInd);

void
subdivide(
        PlanarGraph g,
        const vector<int>& parent,
        vector< PlanarGraph >& subgs,
        vector< vector<int> >& mappings,
        vector< vector<int> >& parents,
        vector< vector< pair<int, int > > >& paths);

#endif
