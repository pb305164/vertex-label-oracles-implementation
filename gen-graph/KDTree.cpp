#include "KDTree.h"

using namespace std;
using mpfr::mpreal;


bool lat_cmp(Node a, Node b) {
    return a.lat < b.lat;
}

bool lon_cmp(Node a, Node b) {
    return a.lon < b.lon;
}


KDTree::KDTree(vector<Node> &vnodes, int lvl) {
    level = lvl;
    if (level % 2) {
        sort(vnodes.begin(), vnodes.end(), lat_cmp);
    } else {
        sort(vnodes.begin(), vnodes.end(), lon_cmp);
    }
    if (vnodes.size() == 1) {
        node = vnodes[0];
        LTree = NULL;
        RTree = NULL;
    } else if (vnodes.size() == 2) {
        node = vnodes[0];
        LTree = NULL;
        RTree = new KDTree(vnodes[1]);
    } else {
        size_t const half_size = vnodes.size() / 2;
        vector<Node> vn1(vnodes.begin(), vnodes.begin() + half_size);
        vector<Node> vn2(vnodes.begin() + half_size + 1, vnodes.end());
        node = vnodes[half_size];
        LTree = new KDTree(vn1, level+1);
        RTree = new KDTree(vn2, level+1);
    }
}

void KDTree::find_nearest(mpreal lat, mpreal lon, mpreal &best_dist, Node &best_node) {
    mpreal my_dist = calc_distance(lat, lon, node.lat, node.lon);
    if (best_dist > my_dist) {
        best_dist = my_dist;
        best_node = node;
    }

    if ((level%2==1 && lat > node.lat) || (level%2==0 && lon > node.lon)) {
        if (RTree != NULL) {
            RTree->find_nearest(lat, lon, best_dist, best_node);
        }
        if (LTree != NULL) {
            if ((level%2==1 && calc_distance(lat, lon, node.lat, lon) < best_dist) || (level%2==0 && calc_distance(lat, lon, lat, node.lon) < best_dist )) {
                LTree->find_nearest(lat, lon, best_dist, best_node);
            }
        }
    } else {
        if (LTree != NULL) {
            LTree->find_nearest(lat, lon, best_dist, best_node);
        }
        if (RTree != NULL) {
            if ((level%2==1 && calc_distance(lat, lon, node.lat, lon) < best_dist) || (level%2==0 && calc_distance(lat, lon, lat, node.lon) < best_dist )) {
                RTree->find_nearest(lat, lon, best_dist, best_node);
            }
        }
    }
}

void KDTree::print(int indent) {
    for (int i=0; i<indent; ++i) fprintf(stderr, "  ");
    fprintf(stderr, "%lld %d %lf %lf\n", node.osm_id, node.id, node.lat.toDouble(), node.lon.toDouble());
    if (RTree != NULL) {
        for (int i=0; i<indent; ++i) fprintf(stderr, "  ");
        fprintf(stderr, "R:\n");
        RTree->print(indent+1);
    }
    if (LTree != NULL) {
        for (int i=0; i<indent; ++i) fprintf(stderr, "  ");
        fprintf(stderr, "L:\n");
        LTree->print(indent+1);
    }
}

KDTree KDTree::create(vector<Node> &vnodes) {
    assert(vnodes.size()>0);
    return KDTree(vnodes, 0);
}

KDTree::KDTree(Node n): level(0), node(n), LTree(nullptr), RTree(nullptr) {}

KDTree::~KDTree() {
    delete LTree;
    delete RTree;
}

void KDTree::print() {
    print(0);
}

pair<Node, mpreal> KDTree::find_nearest(mpreal lat, mpreal lon) {
    Node best_node = node;
    mpreal best_dist = MAX_DOUBLE;
    find_nearest(lat, lon, best_dist, best_node);
    return make_pair(best_node, best_dist);
}

