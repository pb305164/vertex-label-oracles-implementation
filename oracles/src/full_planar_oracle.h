#ifndef _FULL_PLANAR_ORACLE_H_
#define _FULL_PLANAR_ORACLE_H_

#include "planar_oracle.h"
#include "find_union.h"

#include <iostream>
using namespace std;

#define NDEBUG 1

const int forb_lab=0;

// mozliwie najprostszy kopiec
template<class Key>
class Heap {
public:
    vector<Key> h;

    Heap() {
        h.push_back(Key());
    }
    int push(Key k) {
        h.push_back(k);
        int iter=h.size()-1;
        while(iter > 1 && h[iter] < h[iter/2]) {
            swap(h[iter],h[iter/2]);
            iter /= 2;
        }
        return iter;
    }

    // pos >= 1, indeks w tablicy h to jednoczesnie pozycja na kopcu
    void remove(int pos) {
        if(h.size() < 2) return;
        if(h.size()==2) {
            h.pop_back();
            return;
        }
        // w gore
        while(pos > 1) {
            h[pos]=h[pos/2];
            pos /= 2;
        }
        // podmiana
        Key temp=h[h.size()-1];
        h.pop_back();
        // w dol
        int next=2*pos;
        if(next+1 < h.size() && h[next] > h[next+1]) ++next;
        while(next < h.size() && h[next] < temp ) {
            h[pos]=h[next];
            pos=next;
            next = 2*pos;
            if(next+1 < h.size() && h[next] > h[next+1]) ++next;
        }
        h[pos]=temp;

    }

    Key top() {
        return h[1];
    }

    int size() {
        return h.size()-1;
    }
};

class FullPlanarOracle : public PlanarOracle {
    
    struct Vertex {
        int label;
        vector< pair<W, int> > portals, rportals;
        vector< pair<W, int> > rdist;
    };
    vector< Vertex > vertices;
    
    struct Label {
        unordered_map< int, set< pair<W, int> > > S_v;
    }; 
    vector< Label > labels;

    struct Portal {
        unordered_map<int, set< pair<W, int> > > N_l;
        Portal() {}
    };
    vector< Portal > portals;
    
    virtual
    void processLeaf(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<bool>& source) {
  
        vector<W> distances;
        for (int v=0; v<(int)pg.vs().size(); ++v) {
            int vv = mapping[v];
            if (vv == -1) continue;
            if (!source[v]) continue;

            getDistances(pg, v, distances);

            for (int u=0; u<(int)pg.vs().size(); ++u) {
                int uu = mapping[u];
                if (uu == -1) continue;
                if (distances[u] == infinity) continue;
                vertices[uu].rdist.push_back(make_pair(distances[u], vv));
            }
        }
    }

    virtual
    void processPortals(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<int>& newPortals,
            const vector<bool>& source) {
    
        vector<W> distances;
        for (int p: newPortals) {
            getDistances(pg, p, distances);
            portals.push_back(Portal());
            for (int j=0; j<(int)mapping.size(); ++j) {
                int v = mapping[j];
                if (v == -1) continue;
                if (distances[j] == infinity) continue;

                portals.back().N_l[ vertices[v].label ].insert(make_pair(distances[j], v));
                vertices[v].rportals.push_back(make_pair(distances[j], portals.size()-1));
                if (source[j]) {
                    vertices[v].portals.push_back(make_pair(distances[j], portals.size()-1));
                }
            }
        }
    }

    virtual
    void initializeStructures() {
        for (auto &V: vertices) {
            sort(V.rdist.begin(), V.rdist.end());
            auto it = unique(V.rdist.begin(), V.rdist.end());
            V.rdist.resize(it - V.rdist.begin());
        }

        for (int v=0; v<(int)vertices.size(); ++v) {
            int l = vertices[v].label;
            for (auto &curr: vertices[v].rdist) {
                W du = curr.first;
                int u = curr.second;
                labels[l].S_v[u].insert(make_pair(du,v));
            }
        }
    }

    virtual
    void applyLabel(int v, int l) {
        vertices[v].label = l;
        for (auto &p: vertices[v].rportals) {
            portals[p.second].N_l[l].insert(make_pair(p.first, v));
        }

        for (pair<W, int> &curr: vertices[v].rdist) {
            W du = curr.first;
            int u = curr.second;
            labels[l].S_v[u].insert(make_pair(du,v));
        }
    }

    virtual
    void purgeLabel(int v) {
        int l = vertices[v].label;

        for (auto &p: vertices[v].rportals) {
            auto it = portals[p.second].N_l.find(l);
            it->second.erase(make_pair(p.first, v));
            if (it->second.empty()) {
                portals[p.second].N_l.erase(it);
            }
        }

        for (pair<W, int> &curr: vertices[v].rdist) {
            W du = curr.first;
            int u = curr.second;
            auto it3 = labels[l].S_v.find(u);
            it3->second.erase(make_pair(du, v));
            if (it3->second.empty()) {
                labels[l].S_v.erase(it3);
            }
        }
    }

public:
    FullPlanarOracle(
            int n,
            const vector< pair< int, int > >& edges, 
            const vector< W >& weights,
            const vector< int > llabels,
            W eps = 1.) : labels(n) {
        ro = min(n, 3);
        vertices = vector<Vertex>(n);
        for (int i=0; i<n; ++i) vertices[i].label = llabels[i];
        initialize(n, edges, weights, eps);
        initializeStructures();

	long long sum = 0;
        for (auto &v: vertices) {
		sum += (int)v.portals.size();
        }
    cout << "Number of portals: " << (int)portals.size() << endl;
    cout << "Avr. portals per vertex= " << sum << " / "
             << (int)vertices.size() << " = " << (float)sum/vertices.size() << endl;
    //cerr << sum << " / " << (int)portals.size() << " = " << (float)sum/portals.size() << endl;
    //cerr << sum << " / " << (int)vertices.size() << " = " << (float)sum/vertices.size() << endl;
    }

    virtual
    int labelOf(int v) {
        return vertices[v].label;
    }

    virtual
    void setLabel(int v, int l) {
        purgeLabel(v);
        applyLabel(v, l);
    }

    virtual
    pair<W, int> distanceToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
        for (auto &p: vertices[v].portals) {
            auto it = portals[p.second].N_l[l].begin();
            if (it == portals[p.second].N_l[l].end()) continue;
            result = min(result, make_pair(p.first + it->first, it->second));
        }
        auto it = labels[l].S_v.find(v);
        if (it != labels[l].S_v.end()) {
            result = min(result, *it->second.begin());
        }
        return result;
    }
};


// aktualnie używana w testach
// wersja ta rozróżnia portale i query-portale,
// przez co kod sie komplikuje i jest wolniejsza od wersji simple...
class DynamicPlanarOracle : public PlanarOracle {

    struct Vertex {
        int label;
        int portal_pos;
        vector< pair<int,W> > portals, rportals;
        vector< int > parents, rparents;
        vector< pair< int, pair<W,int> > > dist;
    };
    vector< Vertex > vertices;

    struct Portal {
        unordered_map<int, set< pair<W, int> > > N_l;
        Portal() {}
    };
    vector< Portal > portals;
    
    virtual
    void processLeaf(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<bool>& source) {
        vector<W> distances;
        vector<int> parents;
        for (int v=0; v<(int)pg.vs().size(); ++v) {
            int vv = mapping[v];
            if (vv == -1) continue;

            getDistParents(pg, v, distances,parents);

            for (int u=0; u<(int)pg.vs().size(); ++u) {
                int uu = mapping[u];
                if (uu == -1) continue;
                if (distances[u] == infinity) continue;
                if(u==v) continue;

                auto it=lower_bound(vertices[uu].dist.begin(),vertices[uu].dist.end(),make_pair(vv, make_pair((W)-1,-2)) );
                if(it==vertices[uu].dist.end() || it->first != vv )
                    vertices[uu].dist.insert(it,make_pair(vv,make_pair(distances[u],mapping[parents[u]])));
                else
                    if((it->second).first > distances[u]) {
                        (it->second).first=distances[u];
                        (it->second).second=mapping[parents[u]];
                    }
            }
        }
    }

    virtual
    void processPortals(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<int>& newPortals,
            const vector<bool>& source) {
    
        vector<W> distances; vector<int> parents;
        int pos;
        for (int p: newPortals) {
            getDistParents(pg, p, distances, parents);

            if( mapping[p]==-1 || vertices[mapping[p]].portal_pos==-1 ) {
               portals.push_back(Portal());
               pos=portals.size()-1;
               if(mapping[p]!=-1) vertices[mapping[p]].portal_pos=pos;
            }
            else pos=vertices[mapping[p]].portal_pos;

            for (int j=0; j<(int)mapping.size(); ++j) {
                int v = mapping[j];
                if (v == -1) continue;
                if (distances[j] == infinity) continue;

                int par=-1;
                if(j!=p) {
                    assert(parents[j]!=-1);
                    par=mapping[parents[j]];
                }

                auto it1=lower_bound( vertices[v].portals.begin(),
                                      vertices[v].portals.end(),
                                      make_pair(pos,(W)-1));
                auto it2=lower_bound(vertices[v].rportals.begin(),
                                     vertices[v].rportals.end(),
                                     make_pair(pos,(W)-1));
                bool is_portal=false, is_rportal=false;
                if(it1 != vertices[v].portals.end() && it1->first == pos) is_portal=true;
                if(it2 != vertices[v].rportals.end() && it2->first == pos) is_rportal=true;
                assert(!(is_portal && is_rportal));


                if( (! is_portal) && (! is_rportal) ) {
                    if( source[j] ) {
                        auto itp=vertices[v].parents.begin()+(it1-vertices[v].portals.begin());
                        vertices[v].portals.insert(it1,make_pair(pos,distances[j]));
                        vertices[v].parents.insert(itp, par);
                    }
                    else {
                        auto itp=vertices[v].rparents.begin()+(it2-vertices[v].rportals.begin());
                        vertices[v].rportals.insert(it2,make_pair(pos,distances[j]));
                        vertices[v].rparents.insert(itp, par);
                    }
                    continue;
                }
                if( is_portal ) {
                    if(distances[j] < it1->second) {
                        it1->second=distances[j];
                        vertices[v].parents[it1-vertices[v].portals.begin()]=par;
                    }
                    continue;
                }
                if( is_rportal ) {
                    if(distances[j] < it2->second) {
                        it2->second=distances[j];
                        vertices[v].rparents[it2-vertices[v].rportals.begin()]=par;
                    }
                    if(source [j]) {
                        auto itp=vertices[v].parents.begin()+(it1-vertices[v].portals.begin());
                        vertices[v].portals.insert(it1,make_pair(it2->first,it2->second));
                        auto ite=vertices[v].rparents.begin()+(it2-vertices[v].rportals.begin());
                        vertices[v].parents.insert(itp, *ite);
                        vertices[v].rportals.erase(it2);
                        vertices[v].rparents.erase(ite);
                    }
                    continue;
                }
            }
        }
    }

    virtual
    void applyLabel(int v, int l) {
        vertices[v].label = l;
        for (auto &p: vertices[v].rportals) {
            portals[p.first].N_l[l].insert(make_pair(p.second, v));
        }
        for (auto &p: vertices[v].portals) {
            portals[p.first].N_l[l].insert(make_pair(p.second, v));
        }
    }

    virtual
    void purgeLabel(int v) {
        int l = vertices[v].label;

        for (auto &p: vertices[v].rportals) {
            auto it = portals[p.first].N_l.find(l);
            if( it != portals[p.first].N_l.end() ) {
                it->second.erase(make_pair(p.second, v));
                if (it->second.empty()) {
                    portals[p.first].N_l.erase(it);
                }
            }
        }

        for (auto &p: vertices[v].portals) {
            auto it = portals[p.first].N_l.find(l);
            if(it !=portals[p.first].N_l.end()) {
                it->second.erase(make_pair(p.second, v));
                if (it->second.empty()) {
                    portals[p.first].N_l.erase(it);
                }
            }
        }
    }

public:
    DynamicPlanarOracle(
            int n,
            const vector< pair< int, int > >& edges, 
            const vector< W >& weights,
            const vector< int > llabels,
            W eps = 1.) {

        ro = min(n, 3);
        vertices = vector<Vertex>(n);
        for (int i=0; i<n; ++i) vertices[i].portal_pos=-1;

        initialize(n, edges, weights, eps);

        for(int i=0; i<n; ++i)
            if(llabels[i]!=forb_lab) applyLabel(i,llabels[i]);
    //long long sump = 0, sumrp=0, sumdist=0;
    //    for (auto &v: vertices) {
    //    sump += (int)v.portals.size();
    //    sumrp +=(int)v.rportals.size();
    //    sumdist+=(int)v.dist.size();
    //    }
    //cout << "Number of portals: " << (int)portals.size() << endl;
    //cout << "Avr. portals per vertex= " << sump << " / "
    //         << (int)vertices.size() << " = " << (float)sump/vertices.size() << endl;
    //cout << "Avr. r-portals per vertex= " << sumrp << " / "
    //         << (int)vertices.size() << " = " << (float)sumrp/vertices.size() << endl;
    //cout << "Avr. piece dists per vertex= " << sumdist << " / "
    //         << (int)vertices.size() << " = " << (float)sumdist/vertices.size() << endl;

    }

    virtual
    int labelOf(int v) {
        return vertices[v].label;
    }

    virtual
    void setLabel(int v, int l) {
        purgeLabel(v);
        if(l!=forb_lab) applyLabel(v, l);
    }

    virtual
    W distanceToVertex1(int v, int w) {
        if(v==w) return 0;
        int i=0, j=0;
        W result=infinity;

        while( i < vertices[v].portals.size() && j < vertices[w].portals.size() ) {
            if( vertices[v].portals[i].first == vertices[w].portals[j].first ) {
                result=min(result, vertices[v].portals[i].second+vertices[w].portals[j].second);
                ++i; ++j;
            }
            else {
                if(vertices[v].portals[i].first < vertices[w].portals[j].first) ++i;
                else ++j;
            }
        }

        while( i < vertices[v].rportals.size() && j < vertices[w].portals.size() ) {
            if( vertices[v].rportals[i].first == vertices[w].portals[j].first ) {
                result=min(result, vertices[v].rportals[i].second+vertices[w].portals[j].second);
                ++i; ++j;
            }
            else {
                if(vertices[v].rportals[i].first < vertices[w].portals[j].first) ++i;
                else ++j;
            }
        }


        auto it=lower_bound(vertices[w].dist.begin(),vertices[w].dist.end(),make_pair(v,make_pair((W)-1,-2)));
        if(it!=vertices[w].dist.end() && it->first==v) {
            if( result > (it->second).first ) {
                result = (it->second).first;
            }
        }

        return result;
    }

    virtual
    W distanceToVertex(int v, int w) {
        if(v==w) return 0;
        vector<int> spv, spw;
        int i=0, j=0, common_portal=-1, pos_portal_v, pos_portal_w;
        W result=infinity;

        while( i < vertices[v].portals.size() && j < vertices[w].portals.size() ) {
            if( vertices[v].portals[i].first == vertices[w].portals[j].first ) {
                W dist=vertices[v].portals[i].second+vertices[w].portals[j].second;
                if(result > dist) {
                    result=dist;
                    pos_portal_v=i;
                    pos_portal_w=j;
                    common_portal=vertices[v].portals[i].first;
                }
                ++i; ++j;
            }
            else {
                if(vertices[v].portals[i].first < vertices[w].portals[j].first) ++i;
                else ++j;
            }
        }

        bool vr=false;
        while( i < vertices[v].rportals.size() && j < vertices[w].portals.size() ) {
            if( vertices[v].rportals[i].first == vertices[w].portals[j].first ) {
                W dist=vertices[v].rportals[i].second+vertices[w].portals[j].second;
                if(result > dist) {
                    result=dist;
                    pos_portal_v=i;
                    pos_portal_w=j;
                    common_portal=vertices[v].rportals[i].first;
                    vr=true;
                }
                ++i; ++j;
            }
            else {
                if(vertices[v].rportals[i].first < vertices[w].portals[j].first) ++i;
                else ++j;
            }
        }

        bool piece_d=false;
        auto it=lower_bound(vertices[w].dist.begin(),vertices[w].dist.end(),make_pair(v,make_pair((W)-1,-2)));
        if(it!=vertices[w].dist.end() && it->first==v) {
            if( result > (it->second).first ) {
                result = (it->second).first;
                piece_d=true;
                int x=(it->second).second;
                while(x != v) {
                    auto itn=lower_bound(vertices[x].dist.begin(),vertices[x].dist.end(),make_pair(v,make_pair((W)-1,-2)));
                    assert(itn!=vertices[x].dist.end() && itn->first==v);
                    x=(itn->second).second;
                }
            }
        }

        if( !piece_d && common_portal!=-1 ) {
            spv.push_back(v);
            int x = vr ? vertices[v].rparents[pos_portal_v] : vertices[v].parents[pos_portal_v];
      //      int count=0;
            while( x != -1 ) {
      //          assert(++count < vertices.size()+2);
                spv.push_back(x);
                auto it1=lower_bound(vertices[x].portals.begin(),
                                    vertices[x].portals.end(),
                                    make_pair(common_portal,(W)-1));
                bool por=(it1 != vertices[x].portals.end() && it1->first == common_portal);
                vector< pair<int,W> >::iterator it2;
                bool rpor=false;
                if(! por) {
                    it2=lower_bound(vertices[x].rportals.begin(),
                                         vertices[x].rportals.end(),
                                         make_pair(common_portal,(W)-1));
                    rpor = (it2 != vertices[x].rportals.end() && it2->first == common_portal);
                }
                assert(por || rpor);
                x = rpor ? vertices[x].rparents[it2-vertices[x].rportals.begin()]
                         : vertices[x].parents[it1-vertices[x].portals.begin()];
            }

            spw.push_back(w); x = vertices[w].parents[pos_portal_w];
        //    count = 0;
            while( x!= -1 ) {
        //        assert(++count < vertices.size()+2);
                spw.push_back(x);
                auto it1=lower_bound(vertices[x].portals.begin(),
                                    vertices[x].portals.end(),
                                    make_pair(common_portal,(W)-1));
                bool por=(it1 != vertices[x].portals.end() && it1->first == common_portal);
                vector< pair<int,W> >::iterator it2;
                bool rpor=false;
                if(! por) {
                    it2=lower_bound(vertices[x].rportals.begin(),
                                         vertices[x].rportals.end(),
                                         make_pair(common_portal,(W)-1));
                    rpor = (it2 != vertices[x].rportals.end() && it2->first == common_portal);
                }
                assert(por || rpor);
                x = rpor ? vertices[x].rparents[it2-vertices[x].rportals.begin()]
                         : vertices[x].parents[it1-vertices[x].portals.begin()];
            }

            //reverse(spw.begin(),spw.end());
            //spv.pop_back();
            //spv.insert(spv.end(),spw.begin(),spw.end());
        }
        return result;
    }

    virtual
    pair<W, int> distanceToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
        if(l==0) return result;
        for (auto &p: vertices[v].portals) {
            if( portals[p.first].N_l.find(l)==portals[p.first].N_l.end() ) continue;
            auto it = portals[p.first].N_l[l].begin();
            assert (it != portals[p.first].N_l[l].end());
            result = min(result, make_pair(p.second + it->first, it->second));
        }
        return result;
    }
};


// obecnie uzywana wyrocznia
// obsluguje vertex-vertex i vertex-label
// porzadnie wydebugowana
// z opcją znajdywania ścieżek (funkcja distanceToVertex) lub tylko odległości (funkcja distanceToVertex1)
class DynamicSimplePlanarOracle : public PlanarOracle {

    struct Vertex {
        int label;
        int portal_pos;   // pozycja wierzcholka na globalnej liscie portals, jesli jest on portalem, wpp -1
        vector< pair<int,W> > portals;  // portale wierzcholka: pozycja na glob. liscie portals i odleglosc
        vector< int > parents;   // kazdemu portalowi wierzcholka odpowiada rodzic na sciezce do tego portala
        vector< pair< int, pair<W,int> > > dist;
        // lista odl. wewnatrz kawalka: identyfikator wierzch, odl. i rodzic, posortowana po id
    };
    vector< Vertex > vertices; // lista wierzcholkow

    struct Portal {
        unordered_map<int, set< pair<W, int> > > N_l; // mapa etykieta -> kopiec z wierzch. o tej etykiecie
        Portal() {}
    };
    vector< Portal > portals; // globalna lista portali

    //flagi
    bool noProcessLeaf;   // bez odleglosci wewn. kawalka

    virtual
    void processLeaf(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<bool>& source) {
        if(!noProcessLeaf) {
            vector<W> distances;
            vector<int> parents;
            for (int v=0; v<(int)pg.vs().size(); ++v) {
                int vv = mapping[v];   // odzyskanie globalnego indeksu wierzcholka v z kawalka pg
                if (vv == -1) continue;   // czy to sie w ogole zdarza ??

                // Dijkstra z v po pg
                getDistParents(pg, v, distances,parents);

                for (int u=0; u<(int)pg.vs().size(); ++u) {
                    int uu = mapping[u];
                    if (uu == -1) continue;
                    if (distances[u] == infinity) continue;
                    if(u==v) continue;

                    // szukamy czy vv jest uwzgledniony w liscie dist wierzch. uu
                    auto it=lower_bound(
                            vertices[uu].dist.begin(),
                            vertices[uu].dist.end(),
                            make_pair(vv, make_pair((W)-1,-2))
                            );

                    // jesli nie jest, to go wstawiamy w odpowiednie miejsce na liscie dist
                    if(it==vertices[uu].dist.end() || it->first != vv )
                        vertices[uu].dist.insert(
                                it,
                                make_pair(
                                    vv,
                                    make_pair(
                                        distances[u],
                                        mapping[parents[u]]
                                    )
                                )
                            );
                    else
                    // wpp. poprawiamy odleglosc i rodzica
                        if((it->second).first > distances[u]) {
                            (it->second).first=distances[u];
                            (it->second).second=mapping[parents[u]];
                        }
                }
            }

        }

    }

    virtual
    void processPortals(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<int>& newPortals,
            const vector<bool>& source) {

        vector<W> distances; vector<int> parents;
        int pos;
        for (int p: newPortals) {
            getDistParents(pg, p, distances, parents);

            if( mapping[p]==-1 || vertices[mapping[p]].portal_pos==-1 ) {
               portals.push_back(Portal());
               pos=portals.size()-1;
               if(mapping[p]!=-1) vertices[mapping[p]].portal_pos=pos;
            }
            else pos=vertices[mapping[p]].portal_pos;

            for (int j=0; j<(int)mapping.size(); ++j) {
                int v = mapping[j];
                if (v == -1) continue;
                if (distances[j] == infinity) continue;

                int par=-1;
                if(j!=p) {
                    //assert(parents[j]!=-1);
                    par=mapping[parents[j]];
                }

                auto it1=lower_bound( vertices[v].portals.begin(),
                                      vertices[v].portals.end(),
                                      make_pair(pos,(W)-1));
                bool is_portal=false;
                if(it1 != vertices[v].portals.end() && it1->first == pos) is_portal=true;

                if( ! is_portal ) {
                   auto itp=vertices[v].parents.begin()+(it1-vertices[v].portals.begin());
                   vertices[v].portals.insert(it1,make_pair(pos,distances[j]));
                   vertices[v].parents.insert(itp, par);
                }
                else
                    if(distances[j] < it1->second) {
                        it1->second=distances[j];
                        vertices[v].parents[it1-vertices[v].portals.begin()]=par;
                    }

            }
        }
    }

    virtual
    void applyLabel(int v, int l) {
        vertices[v].label = l;
        for (auto &p: vertices[v].portals) {
            portals[p.first].N_l[l].insert(make_pair(p.second, v));
        }
    }

    virtual
    void purgeLabel(int v) {
        int l = vertices[v].label;

        for (auto &p: vertices[v].portals) {
            auto it = portals[p.first].N_l.find(l);
            if(it!=portals[p.first].N_l.end()) {
                it->second.erase(make_pair(p.second, v));
                if (it->second.empty()) {
                    portals[p.first].N_l.erase(it);
                }
            }
        }
    }

public:
    DynamicSimplePlanarOracle(
            int n,
            const vector< pair< int, int > >& edges,
            const vector< W >& weights,
            const vector< int > llabels,
            W eps = 1.,
            bool noLeaf = false,
            int jump = 2) {

        noProcessLeaf = noLeaf;
        ro = min(n, 3);
        vertices = vector<Vertex>(n);
        for (int i=0; i<n; ++i) vertices[i].portal_pos = -1;

        initialize(n, edges, weights, eps, jump);

        for(int i=0; i<n; ++i)
            if(llabels[i]!=forb_lab) applyLabel(i,llabels[i]);
    //long long sump = 0, sumdist=0;//, sumrp=0;
    //    for (auto &v: vertices) {
    //    sump += (int)v.portals.size();
    //    //sumrp +=(int)v.rportals.size();
    //    sumdist+=(int)v.dist.size();
    //    }
    //cout << "Number of portals: " << (int)portals.size() << endl;
    //cout << "Avr. portals per vertex= " << sump << " / "
    //         << (int)vertices.size() << " = " << (float)sump/vertices.size() << endl;
    //cout << "Avr. piece dists per vertex= " << sumdist << " / "
    //         << (int)vertices.size() << " = " << (float)sumdist/vertices.size() << endl;

    //cout << "Avr. r-portals per vertex= " << sumrp << " / "
    //         << (int)vertices.size() << " = " << (float)sumrp/vertices.size() << endl;

    //cout << "Num of times portal no ver: " << no_ver_por << ", num no mapping: " << no_mapping << endl;
    //cerr << sum << " / " << (int)portals.size() << " = " << (float)sum/portals.size() << endl;
    //cerr << sum << " / " << (int)vertices.size() << " = " << (float)sum/vertices.size() << endl;
    }

    virtual
    int labelOf(int v) {
        return vertices[v].label;
    }

    virtual
    void setLabel(int v, int l) {
        purgeLabel(v);
        if(l!=forb_lab) applyLabel(v, l);
    }

    virtual
    W distanceToVertex1(int v, int w) {
        int i=0, j=0;
        W result=infinity;

        while( i < vertices[v].portals.size() && j < vertices[w].portals.size() ) {
            if( vertices[v].portals[i].first == vertices[w].portals[j].first ) {
                result=min(result, vertices[v].portals[i].second+vertices[w].portals[j].second);
                ++i; ++j;
            }
            else {
                if(vertices[v].portals[i].first < vertices[w].portals[j].first) ++i;
                else ++j;
            }
        }

        auto it=lower_bound(vertices[w].dist.begin(),vertices[w].dist.end(),make_pair(v,make_pair((W)-1,-2)));
        if(it!=vertices[w].dist.end() && it->first==v) {
            if( result > (it->second).first ) {
                result = (it->second).first;
            }
        }

        return result;
    }

    virtual
    W distanceToVertex(int v, int w) {
        if(v==w) return 0;
        vector<int> spv, spw;
        int i=0, j=0, common_portal=-1, pos_portal_v, pos_portal_w;
        W result=infinity;

        while( i < vertices[v].portals.size() && j < vertices[w].portals.size() ) {
            if( vertices[v].portals[i].first == vertices[w].portals[j].first ) {
                W dist=vertices[v].portals[i].second+vertices[w].portals[j].second;
                if(result > dist) {
                    result=dist;
                    //pos_portal_v=i;
                    //pos_portal_w=j;
                    common_portal=vertices[v].portals[i].first;
                }
                ++i; ++j;
            }
            else {
                if(vertices[v].portals[i].first < vertices[w].portals[j].first) ++i;
                else ++j;
            }
        }

        bool piece_d=false;
        if(!noProcessLeaf) {
            auto it=lower_bound(vertices[w].dist.begin(),vertices[w].dist.end(),make_pair(v,make_pair((W)-1,-2)));
            if(it!=vertices[w].dist.end() && it->first==v) {
                if( result > (it->second).first ) {
                    result = (it->second).first;
                    piece_d=true;
                    int x=(it->second).second;
                    while(x != v) {
                        auto itn=lower_bound(vertices[x].dist.begin(),vertices[x].dist.end(),make_pair(v,make_pair((W)-1,-2)));
                        assert(itn!=vertices[x].dist.end() && itn->first==v);
                        x=(itn->second).second;
                    }
                }
            }
        }

        if(!piece_d && common_portal!=-1) {
            auto itv=lower_bound(vertices[v].portals.begin(),vertices[v].portals.end(),make_pair(common_portal,(W)-1));
            auto itw=lower_bound(vertices[v].portals.begin(),vertices[v].portals.end(),make_pair(common_portal,(W)-1));
            pos_portal_v=itv-vertices[v].portals.begin();
            pos_portal_w=itw-vertices[w].portals.begin();

            spv.push_back(v); int x = vertices[v].parents[pos_portal_v];
            //int count=0;
            int pos_por;
            while( x != -1 ) {
                //++count;
                //assert(count < vertices.size());
                auto it1=lower_bound(vertices[x].portals.begin(),
                                    vertices[x].portals.end(),
                                    make_pair(common_portal,(W)-1));
                //assert(it1 != vertices[x].portals.end() && it1->first == common_portal);
                pos_por = it1-vertices[x].portals.begin();

                spv.push_back(x);
                x = vertices[x].parents[pos_por];
            }

            //count=0;
            spv.push_back(w); x = vertices[w].parents[pos_portal_w];
            while( x!= -1 ) {
                //++count;
                //assert(count < vertices.size());
                auto it1=lower_bound(vertices[x].portals.begin(),
                                    vertices[x].portals.end(),
                                    make_pair(common_portal,(W)-1));
                //assert(it1 != vertices[x].portals.end() && it1->first == common_portal);
                pos_por = it1-vertices[x].portals.begin();

                spw.push_back(x);
                x = vertices[x].parents[pos_por];
            }

            // tu trzeba wykopac podwojna kopie wspolnego portala
            //reverse(spw.begin(),spw.end());
            //spv.pop_back();
            //spv.insert(spv.end(),spw.begin(),spw.end());
        }
        return result;
    }

    virtual
    pair<W, int> distanceToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
        if(l==forb_lab) return result;
        //int w;
        for (auto &p: vertices[v].portals) {
            if( portals[p.first].N_l.find(l)==portals[p.first].N_l.end() ) continue;
            auto it = portals[p.first].N_l[l].begin();
            assert (it != portals[p.first].N_l[l].end());
            //w=it->second;
            result = min(result, make_pair(p.second + it->first, it->second));
        }
        //if(result.first < infinity) result.first=min(result.first,distanceToVertex(v,w));
        return result;
    }
};


class StaticPlanarOracle : public PlanarOracle {

    struct Vertex {
        int label;
        int portal_pos;
        vector< pair< int, W > > portals;
    };
    vector< Vertex > vertices;

    struct Portal {
        unordered_map< int, pair<W, int> > N_l;
        Portal() {}
    };
    vector< Portal > portals;

    virtual
    void processLeaf(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<bool>& source) {}

    virtual
    void processPortals(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<int>& newPortals,
            const vector<bool>& source) {

        vector<W> distances;
        int pos;
        for (int p: newPortals) {
            getDistances(pg, p, distances);

            if( mapping[p]==-1 || vertices[mapping[p]].portal_pos==-1 ) {
               portals.push_back(Portal());
               pos=portals.size()-1;
               if(mapping[p]!=-1) vertices[mapping[p]].portal_pos=pos;
            }
            else pos=vertices[mapping[p]].portal_pos;

            for (int j=0; j<(int)mapping.size(); ++j) {
                int v = mapping[j];
                if (v == -1) continue;
                if (distances[j] == infinity) continue;
                if(portals[pos].N_l.find(vertices[v].label)==portals[pos].N_l.end())
                    portals[pos].N_l[vertices[v].label]=make_pair(distances[j],v);
                if(distances[j] < portals[pos].N_l[ vertices[v].label ].first)
                    portals[pos].N_l[ vertices[v].label ]=make_pair(distances[j],v);
                if (source[j]) {
                   vertices[v].portals.push_back(make_pair(pos,distances[j]));
                }
            }
        }
    }


public:
    StaticPlanarOracle(
            int n,
            const vector< pair< int, int > >& edges,
            const vector< W >& weights,
            const vector< int > llabels,
            W eps = 1.) {
        ro = min(n, 1);
        vertices = vector<Vertex>(n);
        for (int i=0; i<n; ++i) {
            vertices[i].label = llabels[i];
            vertices[i].portal_pos = -1;
        }
        initialize(n, edges, weights, eps);
        //scalanie portali w wierzcholkach
        for(auto &v : vertices) {
            sort(v.portals.begin(),v.portals.end());
            for( int i=1; i < v.portals.size(); ++i )
                if(v.portals[i].first==v.portals[i-1].first)
                    v.portals[i].second=v.portals[i-1].second;
            auto it = unique(v.portals.begin(), v.portals.end());
            v.portals.resize( distance(v.portals.begin(),it) );
        }
        // koniec scalania
    long long sum = 0;
        for (auto &v: vertices) {
        sum += (int)v.portals.size();
        }
    cout << "Number of portals: " << (int)portals.size() << endl;
    cout << "Avr. portals per vertex= " << sum
         << " / " << (int)vertices.size() << " = " << (float)sum/vertices.size() << endl;
    }

    virtual
    int labelOf(int v) {
        return vertices[v].label;
    }

    // UWAGA: nalezu szukac tez przez r-portale ----> blad!!
    virtual
    W distanceToVertex(int v, int w) {
        int i=0, j=0;
        W result=infinity;

        while( i < vertices[v].portals.size() && j < vertices[w].portals.size() ) {
            if( vertices[v].portals[i].first == vertices[w].portals[j].first ) {
                result=min(result, vertices[v].portals[i].second+vertices[w].portals[j].second);
                ++i; ++j;
            }
            else {
                if(vertices[v].portals[i].first < vertices[w].portals[j].first) ++i;
                else ++j;
            }
        }

        return result;
    }

    virtual
    pair<W, int> distanceToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
        for (auto &p: vertices[v].portals) {
            if( portals[p.first].N_l.find(l)!=portals[p.first].N_l.end() )
            result = min(result,
                         make_pair(portals[p.first].N_l[l].first+p.second,
                                   portals[p.first].N_l[l].second));
        }
        return result;
    }
};


class FullFullPlanarOracle : public PlanarOracle {
    
    struct Vertex {
        int label;
        vector< pair<W, int> > portals;
        vector< pair<W, int> > dist;
    };
    vector< Vertex > vertices;
    
    struct Label {
        unordered_map< int, set< pair<W, int> > > S_v;
        unordered_map< int, set< pair<W, pair<int, int> > > > P_l;
    }; 
    vector< Label > labels;

    struct Portal {
        map<int, set< pair<W, int> > > N_l;
        Portal() {}
    };
    vector< Portal > portals;
    
    virtual
    void processLeaf(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<bool>& source) {
  
        vector<W> distances;
        for (int v=0; v<(int)pg.vs().size(); ++v) {
            int vv = mapping[v];
            if (vv == -1) continue;

            getDistances(pg, v, distances);

            for (int u=0; u<(int)pg.vs().size(); ++u) {
                int uu = mapping[u];
                if (uu == -1) continue;
                if (distances[u] == infinity) continue;
                vertices[uu].dist.push_back(make_pair(distances[u], vv));
            }
        }
    }

    virtual
    void processPortals(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<int>& newPortals,
            const vector<bool>& source) {
    
        vector<W> distances;
        for (int p: newPortals) {
            getDistances(pg, p, distances);
            portals.push_back(Portal());
            for (int j=0; j<(int)mapping.size(); ++j) {
                int v = mapping[j];
                if (v == -1) continue;
                if (distances[j] == infinity) continue;

                portals.back().N_l[ vertices[v].label ].insert(make_pair(distances[j], v));
                vertices[v].portals.push_back(make_pair(distances[j], portals.size()-1));
            }
        }
    }

    virtual
    void initializeStructures() {
        for (auto &V: vertices) {
            sort(V.dist.begin(), V.dist.end());
            auto it = unique(V.dist.begin(), V.dist.end());
            V.dist.resize(it - V.dist.begin());
        }

        for (int v=0; v<(int)vertices.size(); ++v) {
            int l = vertices[v].label;
            for (auto &curr: vertices[v].dist) {
                W du = curr.first;
                int u = curr.second;
                int ll = vertices[u].label;
                labels[l].S_v[u].insert(make_pair(du,v));
                labels[l].P_l[ll].insert(make_pair(du, make_pair(v, u)));
                if (v != u) labels[ll].P_l[l].insert(make_pair(du, make_pair(u, v)));
            }
        }
    }

    virtual
    void applyLabel(int v, int l) {
        vertices[v].label = l;
        for (auto &p: vertices[v].portals) {
            portals[p.second].N_l[l].insert(make_pair(p.first, v));
        }

        for (pair<W, int> &curr: vertices[v].dist) {
            W du = curr.first;
            int u = curr.second;
            int ll = vertices[u].label;

            labels[l].S_v[u].insert(make_pair(du,v));
            labels[l].P_l[ll].insert(make_pair(du, make_pair(v, u)));
            if (v != u) labels[ll].P_l[l].insert(make_pair(du, make_pair(u, v)));
        }
    }

    virtual
    void purgeLabel(int v) {
        int l = vertices[v].label;

        for (auto &p: vertices[v].portals) {
            auto it = portals[p.second].N_l.find(l);
            it->second.erase(make_pair(p.first, v));
            if (it->second.empty()) {
                portals[p.second].N_l.erase(it);
            }
        }

        for (pair<W, int> &curr: vertices[v].dist) {
            W du = curr.first;
            int u = curr.second;
            int ll = vertices[u].label;
            
            auto it1 = labels[ll].P_l.find(l);
            it1->second.erase(make_pair(du, make_pair(u, v)));
            if (it1->second.empty()) {
                labels[ll].P_l.erase(it1);
            }

            if (v != u) {
                auto it2 = labels[l].P_l.find(ll);
                it2->second.erase(make_pair(du, make_pair(v, u)));
                if (it2->second.empty()) {
                    labels[l].P_l.erase(it2);
                }
            }
            
            auto it3 = labels[l].S_v.find(u);
            it3->second.erase(make_pair(du, v));
            if (it3->second.empty()) {
                labels[l].S_v.erase(it3);
            }
        }
    }

public:
    FullFullPlanarOracle(
            int n,
            const vector< pair< int, int > >& edges, 
            const vector< W >& weights,
            const vector< int > llabels,
            W eps = 1.) : labels(n) {
        ro = max(3, min(n, (int)sqrt(n)));
        vertices = vector<Vertex>(n);
        for (int i=0; i<n; ++i) vertices[i].label = llabels[i];
        initialize(n, edges, weights, eps);
        initializeStructures();

	long long sum = 0;
        for (auto &v: vertices) {
		sum += (int)v.portals.size();
        }

    cout << "Number of portals: " << (int)portals.size() << endl;
    cout << "Avr. portals per vertex= " << sum << " / "
             << (int)vertices.size() << " = " << (float)sum/vertices.size() << endl;
    //cerr << sum << " / " << (int)portals.size() << " = " << (float)sum/portals.size() << endl;
    //cerr << sum << " / " << (int)vertices.size() << " = " << (float)sum/vertices.size() << endl;
    }

    virtual
    int labelOf(int v) {
        return vertices[v].label;
    }

    virtual
    void setLabel(int v, int l) {
        purgeLabel(v);
        applyLabel(v, l);
    }


    virtual
    pair<W, int> labelToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
        for (auto &p: vertices[v].portals) {
            auto itt = portals[p.second].N_l.find(l);
            if (itt == portals[p.second].N_l.end()) continue;
            auto it = itt->second.begin();
            if (it == itt->second.end()) continue;
            result = min(result, make_pair(p.first + it->first, it->second));
        }
        auto it = labels[l].S_v.find(v);
        if (it != labels[l].S_v.end()) {
            result = min(result, *it->second.begin());
        }
        return result;
    }

    virtual
    pair<W, pair<int, int> > distanceBetweenLabels(int l1, int l2) {
        pair<W, pair<int, int> > result(infinity, make_pair(-1, -1));
        for (auto &p: portals) {
            if (p.N_l.find(l1) == p.N_l.end()) continue;
            if (p.N_l[l1].empty()) continue;
            if (p.N_l.find(l2) == p.N_l.end()) continue;
            if (p.N_l[l2].empty()) continue;
            auto v = *p.N_l[l1].begin();
            auto u = *p.N_l[l2].begin();
            result = min(result, make_pair(v.first + u.first, make_pair(v.second, u.second)));
        }
        auto it = labels[l1].P_l.find(l2);
        if (it != labels[l1].P_l.end()) {
            result = min(result, *it->second.begin());
        }
        return result;
    }
};

class StaticLLPlanarOracle : public PlanarOracle {

    struct Vertex {
        int label;
        int portal_pos;
        vector< pair<int, W> > portals;
        vector< pair<int, W> > dist;
    };
    vector< Vertex > vertices;

    struct Label {
        unordered_map< int, pair<W, int> > S_v;
        unordered_map< int, pair< W, pair<int, int> > > P_l;
    };
    vector< Label > labels;

    struct Portal {
        map<int, pair<W, int> > N_l;
        Portal() {}
    };
    vector< Portal > portals;

    virtual
    void processLeaf(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<bool>& source) {

        vector<W> distances;
        for (int v=0; v<(int)pg.vs().size(); ++v) {
            int vv = mapping[v];
            if (vv == -1) continue;

            getDistances(pg, v, distances);

            for (int u=0; u<(int)pg.vs().size(); ++u) {
                int uu = mapping[u];
                if (uu == -1) continue;
                if (distances[u] == infinity) continue;
                vertices[uu].dist.push_back(make_pair(vv,distances[u]));
            }
        }
    }

    virtual
    void processPortals(
            const PlanarGraph& pg,
            const vector<int>& mapping,
            const vector<int>& newPortals,
            const vector<bool>& source) {

        vector<W> distances; int pos;
        for (int p: newPortals) {
            getDistances(pg, p, distances);
            if( mapping[p]==-1 || vertices[mapping[p]].portal_pos==-1 ) {
                portals.push_back(Portal());
                pos=portals.size()-1;
                if(mapping[p]!=-1) vertices[mapping[p]].portal_pos=pos;
            }
            else pos=vertices[mapping[p]].portal_pos;

            for (int j=0; j<(int)mapping.size(); ++j) {
                int v = mapping[j];
                if (v == -1) continue;
                if (distances[j] == infinity) continue;
                if(portals[pos].N_l.find(vertices[v].label)==portals[pos].N_l.end())
                    portals[pos].N_l[vertices[v].label]=make_pair(distances[j],v);
                if(portals[pos].N_l[vertices[v].label].first > distances[j])
                    portals[pos].N_l[vertices[v].label]=make_pair(distances[j],v);
                vertices[v].portals.push_back(make_pair(pos, distances[j]));
            }
        }
    }

    virtual
    void initializeStructures() {
        for (auto &V: vertices) {
            sort(V.dist.begin(), V.dist.end());
            for( int i=1; i < V.dist.size(); ++i )
                if(V.dist[i].first==V.dist[i-1].first)
                    V.dist[i].second=V.dist[i-1].second;
            auto it = unique(V.dist.begin(), V.dist.end());
            V.dist.resize(it - V.dist.begin());
        }

        for(auto &v : vertices) {
            sort(v.portals.begin(),v.portals.end());
            for( int i=1; i < v.portals.size(); ++i )
                if(v.portals[i].first==v.portals[i-1].first)
                    v.portals[i].second=v.portals[i-1].second;
            auto it = unique(v.portals.begin(), v.portals.end());
            v.portals.resize( distance(v.portals.begin(),it) );
        }

        for (int v=0; v<(int)vertices.size(); ++v) {
            int l = vertices[v].label;
            for (auto &curr: vertices[v].dist) {
                W du = curr.second;
                int u = curr.first;
                int ll = vertices[u].label;
                if(labels[l].S_v.find(u)==labels[l].S_v.end())
                    labels[l].S_v[u]=make_pair(du,v);
                if(labels[l].S_v[u].first > du) labels[l].S_v[u]=make_pair(du,v);

                if(l < ll) {
                    if(labels[l].P_l.find(ll)==labels[l].P_l.end())
                        labels[l].P_l[ll]=make_pair(du, make_pair(v, u));
                    else
                        if(labels[l].P_l[ll].first > du) labels[l].P_l[ll]=make_pair(du, make_pair(v, u));
                }
                //if (v != u) {
                //    if(labels[ll].P_l.find(l)==labels[ll].P_l.end())
                //        labels[ll].P_l[l]=make_pair(du, make_pair(v, u));
                //    if(labels[ll].P_l[l].first > du)
                //        labels[ll].P_l[l]=make_pair(du, make_pair(v, u));
                //}
            }
        }


    }

public:
    StaticLLPlanarOracle(
            int n,
            const vector< pair< int, int > >& edges,
            const vector< W >& weights,
            const vector< int > llabels,
            W eps = 1.) : labels(n) {
        ro = max(3, min(n, (int)sqrt(n)));
        vertices = vector<Vertex>(n);
        for (int i=0; i<n; ++i) {
            vertices[i].label = llabels[i];
            vertices[i].portal_pos=-1;
        }
        initialize(n, edges, weights, eps);
        initializeStructures();

    //long long sum = 0;
    //    for (auto &v: vertices) {
    //    sum += (int)v.portals.size();
    //    }
    //cout << "Number of portals: " << (int)portals.size() << endl;
    //cout << "Avr. portals per vertex= " << sum << " / "
    //     << (int)vertices.size() << " = " << (float)sum/vertices.size() << endl;
    }

    virtual
    int labelOf(int v) {
        return vertices[v].label;
    }

    virtual
    W distanceToVertex(int v, int w) {
        int i=0, j=0;
        W result=infinity;
        while( i < vertices[v].portals.size() && j < vertices[w].portals.size() ) {
            if( vertices[v].portals[i].first == vertices[w].portals[j].first ) {
                result = min(result,vertices[v].portals[i].second+vertices[w].portals[j].second);
                ++i; ++j;
            }
            else {
                if(vertices[v].portals[i].first < vertices[w].portals[j].first) ++i;
                else ++j;
            }
        }    

        // JEszcze szukanie w kawalku ....
        auto it=lower_bound(vertices[v].dist.begin(),vertices[v].dist.end(),make_pair(w,(W)-1));
        if (it!=vertices[v].dist.end() && it->first==w) result=min(result, it->second);
        return result;
    }

    virtual
    pair<W, int> distanceToLabel(int v, int l) {
        pair<W, int> result(infinity, -1);
        for (auto &p: vertices[v].portals) {
            if(portals[p.first].N_l.find(l)==portals[p.first].N_l.end()) continue;
            result = min(result,
                         make_pair(
                             p.second+portals[p.first].N_l[l].first,
                             portals[p.first].N_l[l].second));
        }
        if (labels[l].S_v.find(v)!=labels[l].S_v.end()) {
            result = min(result,
                         make_pair(
                             labels[l].S_v[v].first,
                             labels[l].S_v[v].second));
        }
        return result;
    }

    virtual
    pair<W, pair<int, int> > distanceBetweenLabels(int l1, int l2) {
        if(l1==l2) return make_pair(0, make_pair(-1, -1));
        pair<W, pair<int, int> > result(infinity, make_pair(-1, -1));

        for (auto &p: portals) {
            if (p.N_l.find(l1)==p.N_l.end()) continue;
            if (p.N_l.find(l2)==p.N_l.end()) continue;
            result = min(result,
                         make_pair(
                             p.N_l[l1].first + p.N_l[l2].first,
                             make_pair(p.N_l[l1].second, p.N_l[l2].second)));
        }
        if(l1 > l2) swap(l1,l2);
        if( labels[l1].P_l.find(l2)!=labels[l1].P_l.end() )
            result = min(result, labels[l1].P_l[l2]);

        return result;
    }
};

#endif
