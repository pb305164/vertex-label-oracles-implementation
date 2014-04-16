#ifndef _ORACLE_H_
#define _ORACLE_H_

#include "planar.h"
#include "oracle_internal.h"

#include <map>
#include <unordered_map>
#include <set>
#include <cmath>
using std::map;
using std::unordered_map;
using std::set;
using std::min;
using std::max;
using std::sqrt;

class PlanarOracle {
protected:
    //! Ro parameter specifying size of leafs of recursive subdivision
    int ro;

    //! Process a planar graph - a leaf of recursive subdivision
    virtual
    void processLeaf(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<bool>& source) = 0;

    //! Process portals - selected vertices during a step of subdivision of a planar graph
    virtual
    void processPortals(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<int>& portal,
            const vector<bool>& source) = 0;

    //! Builds an oracle
    void initialize(
            int n,
            const vector< pair< int, int > >& edges, 
            const vector< W >& weights,
            W eps);

    //! Split set of vertices into layers 
    virtual
    void getLayers(
            vector<W> dist,
            W alpha,
            vector<int>& layerNum);

    //! Creates a set of planar graphs based on set of layers
    void getAlphaFamily(
            const PlanarGraph& g, 
            W alpha, 
            vector< PlanarGraph >& subgs,
            vector< vector<int> >& mappings,
            vector< vector<int> >& parents,
            vector< vector<bool> >& sources);

    //! Chooses a set of portals on given paths splitting planar graph
    void selectPathPortals(
            const PlanarGraph& g, 
            W alpha, 
            W eps,
            vector< vector< pair<int, int> > > paths,
            vector<int>& portal);

    //! Performs a recursive subdivision
    void subdivideRecursively(
            const PlanarGraph& pg,
            W alpha,
            W eps,
            const vector<int>& mapping,
            const vector<int>& parents,
            const vector<bool>& sources
    );

public:
    PlanarOracle() {}
    PlanarOracle(
            int n,
            const vector< pair< int, int > >& edges, 
            const vector< W >& weights,
            W eps) {
        initialize(n, edges, weights, eps);
    }
};

#endif
