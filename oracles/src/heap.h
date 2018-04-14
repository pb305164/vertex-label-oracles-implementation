#ifndef VERTEX_LABEL_ORACLES_HEAP_H
#define VERTEX_LABEL_ORACLES_HEAP_H

#include "precision.h"

using namespace std;

template <class Key, class Val>
class ScapegoatMap {
private:
    struct MapNode {
        Key key;
        Val val;
        int left;
        int right;
        MapNode(Key _key, Val _val): key(_key), val(_val), left(-1), right(-1) {};
        MapNode(): key(-1), val(-1), left(-1), right(-1) {};
    };

    size_t _size;
    vector<MapNode> nodes;

    static int const log32(int n) {
        double const log23 = 2.4663034623764317;
        return (int)ceil(log23 * log(n));
    }

    void assign_values(vector<pair<Key, Val>> &ordered, int s, int t, int &next_pos) {
        int o = (s+t)/2, pos = next_pos;
        nodes[pos] = MapNode(ordered[o].first, ordered[o].second);
        next_pos++;
        if (o > s) {
            nodes[pos].left = next_pos;
            assign_values(ordered, s, o, next_pos);
        }
        if (t - o > 1) {
            nodes[pos].right = next_pos;
            assign_values(ordered, o+1, t, next_pos);
        }
    }

    void assign_values2(vector<pair<Key, Val>> &ordered, vector<int> &free_pos, int s, int t, int &next_pos) {
        int o = (s+t)/2, pos = free_pos[next_pos];
        nodes[pos] = MapNode(ordered[o].first, ordered[o].second);
        next_pos++;
        if (o > s) {
            nodes[pos].left = free_pos[next_pos];
            assign_values2(ordered, free_pos, s, o, next_pos);
        }
        if (t - o > 1) {
            nodes[pos].right = free_pos[next_pos];
            assign_values2(ordered, free_pos, o+1, t, next_pos);
        }
    }

    void rebalance(int pos) {
        if (pos == 0) {
            full_rebuild();
        }
        else {
            vector<pair<Key, Val>> ordered;
            vector<int> free_pos;
            inorder2(ordered, free_pos, pos);
            int next_pos = 0;
            assign_values2(ordered, free_pos, 0, (int)ordered.size(), next_pos);
        }
    }

    // Clears empty cells in vector and balances tree
    void full_rebuild() {
        vector<pair<Key, Val>> ordered = inorder();

        // TODO ? better space optimalization
        nodes.resize(_size*1.1);
        nodes.shrink_to_fit();

        if (_size > 0) {
            int next_pos=0;
            assign_values(ordered, 0, _size, next_pos);
        }
    }

    // Return position in nodes vector (create if not found)
    void find(int pos, Key k, vector<Key> &path) {
        assert(pos != -1);
        path.push_back(pos);
        while (pos != -1) {
            MapNode &n = nodes[pos];
            if (n.key == k) {
                return;
            }
            if (n.key < k) {
                pos = n.right;
            } else {
                pos = n.left;
            }
            path.push_back(pos);
        }
    }

    int findMax(int pos, int &parent) {
        if (nodes[pos].right != -1) {
            parent = pos;
            return findMax(nodes[pos].right, parent);
        }
        return pos;
    }

public:
    ScapegoatMap(): _size(0), nodes() {}

    void inorder(vector<pair<Key, Val>> &ret, int pos) {
        if (pos != -1) {
            inorder(ret, nodes[pos].left);
            ret.push_back(make_pair(nodes[pos].key, nodes[pos].val));
            inorder(ret, nodes[pos].right);
        }
    };

    void inorder2(vector<pair<Key, Val>> &ret, vector<int> &free_pos, int pos) {
        if (pos != -1) {
            free_pos.push_back(pos);
            inorder2(ret, free_pos, nodes[pos].left);
            ret.push_back(make_pair(nodes[pos].key, nodes[pos].val));
            inorder2(ret, free_pos, nodes[pos].right);
        }
    };

    vector<pair<Key, Val>> inorder() {
        vector<pair<Key, Val>> ret;
        if (_size > 0) {
            inorder(ret, 0);
        }
        assert(_size == ret.size());
        return ret;
    };

    // Remove vertex v from heap
    void erase(Key k) {
        if (_size > 0) {
            int pos=0, parent=-1;
            while (pos != -1) {
                MapNode n = nodes[pos];
                if (n.key == k) {
                    _size--;
                    if (n.left == -1 && n.right == -1) {
                        if (parent != -1) {
                            if (nodes[parent].key > k) nodes[parent].left = -1;
                            else nodes[parent].right = -1;
                        } else {
                            nodes.clear();
                        }
                    } else if (n.left == -1) {
                        nodes[pos] = nodes[n.right];
                    } else if (n.right == -1) {
                        nodes[pos] = nodes[n.left];
                    } else {
                        int max_parent = pos, max = findMax(n.left, max_parent);
                        nodes[pos] = nodes[max];
                        nodes[pos].right = n.right;
                        if (max_parent != pos) {
                            nodes[pos].left = n.left;
                            if (nodes[max].left != -1) {
                                nodes[max_parent].right = nodes[max].left;
                            } else {
                                nodes[max_parent].right = -1;
                            }
                        }
                    }
                    pos = -1;
                } else if (n.key < k) {
                    parent = pos;
                    pos = n.right;
                } else {
                    parent = pos;
                    pos = n.left;
                }

            }
        }
        if (_size < nodes.size()/2) {
            full_rebuild();
        }
    }

    // Return number of elements in map (not vector size)
    size_t size() {
        return _size;
    }

    // Return size of subtree
    size_t size(int pos) {
        if (pos == -1) {
            return 0;
        } else {
            return size(nodes[pos].left) + 1 + size(nodes[pos].right);
        }

    }

    void clear() {
        _size = 0;
        nodes.clear();
        nodes.shrink_to_fit();
    }

    size_t count(Key k) {
        if (_size > 0) {
            vector<Key> path;
            find(0, k, path);
            if (path.back() != -1) {
                return 1;
            }
        }
        return 0;
    }

    Val& operator[](Key k) {
        if (_size > 0) {
            vector<Key> path;
            find(0, k, path);
            if (path.back() != -1) {
                // Key found return value
                return nodes[path.back()].val;
            } else {
                // Add new key
                int new_ptr = (int)nodes.size();
                nodes.push_back(MapNode(k, Val()));
                _size++;
                MapNode &parent = nodes[*(path.rbegin()+1)];
                if (parent.key < k) {
                    parent.right = new_ptr;
                } else {
                    parent.left = new_ptr;
                }

                // Check if tree is unbalanced
                if ((int)path.size() > log32(_size)) {
                    // If so find scapegoat
                    auto it = path.rbegin();
                    // Size of subtree goin up using path
                    int child_size, parent, parent_size = 1;

                    // Size of parrent on path
                    do {
                        child_size = parent_size;
                        // Add other child size
                        it++;
                        assert(it != path.rend());
                        parent = *it;
                        if (nodes[parent].key < k) {
                            parent_size += size(nodes[parent].left) + 1;
                        } else {
                            parent_size += size(nodes[parent].right) + 1;
                        }
                    } while (3*child_size <= 2*parent_size);
                    rebalance(parent);
                }
                return nodes[new_ptr].val;
            }
        } else {
            _size++;
            nodes.emplace_back(k, 0);
            return nodes.front().val;
        }
    }
};

template<class T>
class Heap {
private:
    // Heap stores pairs of distance and vertex id
    vector<pair<W, int>> heap;


    // Swaps position p1 and p2 in heap and update positions accordingly
    void hswap(T &ver_to_pos, int p1, int p2) {
        assert(p1 < (int)heap.size() && p2 < (int)heap.size());
        swap(ver_to_pos[heap[p1].second], ver_to_pos[heap[p2].second]);
        swap(heap[p1], heap[p2]);
    }

    // Move given position up the heap as far as possible
    void push_top(T &ver_to_pos, int p) {
        while (heap[p] < heap[p/2] && p > 1) {
            hswap(ver_to_pos, p, p/2);
            p/=2;
        }
    }

    // Move given position down the heap as far as possible
    void push_bot(T &ver_to_pos, int p) {
        int m = p;
        do {
            p = m;
            if (p*2 < (int)heap.size() && heap[p*2] < heap[m]) {
                m = p*2;
            }
            if (p*2+1 < (int)heap.size() && heap[p*2+1] < heap[m]) {
                m = p*2+1;
            }
            hswap(ver_to_pos, p, m);
        } while (m != p);
    }

public:

    Heap(): heap(1, make_pair(-10000, -10000)) {} // Add dummy value at 0 so parent/child indexes would nicely align

    // Insert distance d for vertex v into heap and assign its position
    void insert(T &ver_to_pos, int v, W d) {
        assert(ver_to_pos.count(v) == 0); // Check if not already in heap
        ver_to_pos[v] = (int)heap.size();
        heap.emplace_back(d, v);
        push_top(ver_to_pos, (int)heap.size()-1);
    }

    // Remove vertex v from heap
    void erase(T &ver_to_pos, int v) {
        int vpos = ver_to_pos[v];
        assert(vpos > 0 && vpos < (int)heap.size());
        hswap(ver_to_pos, vpos, (int)heap.size()-1);
        ver_to_pos.erase(v);
        heap.pop_back();

        // Since vertex is removed from anywhere in the heap and is replaced by last element it could go both higher or lower
        if (vpos > 1 && heap[vpos] < heap[vpos/2]) push_top(ver_to_pos, vpos);
        else push_bot(ver_to_pos, vpos);
    }

    // Insert distance d for vertex v into heap and assign its position (special version from vector)
    void insert(vector<int> &ver_to_pos, int v, W d) {
        assert(ver_to_pos[v] == 0); // Check if not already in heap
        ver_to_pos[v] = (int)heap.size();
        heap.emplace_back(d, v);
        push_top(ver_to_pos, (int)heap.size()-1);
    }

    // Remove vertex v from heap (specific version for vector, cannot use erase)
    void erase(vector<int> &ver_to_pos, int v) {
        int vpos = ver_to_pos[v];
        hswap(ver_to_pos, vpos, (int)heap.size()-1);
        ver_to_pos[v] = 0;
        heap.pop_back();

        // Since vertex is removed from anywhere in the heap and is replaced by last element it could go both higher or lower
        if (vpos > 1 && heap[vpos] < heap[vpos/2]) push_top(ver_to_pos, vpos);
        else push_bot(ver_to_pos, vpos);

        if (heap.capacity() > 2*heap.size()) {
            heap.shrink_to_fit();
        }
    }


    // Return element at given position in heap
    pair<W, int> at(int pos) {
        assert(pos > 0 && pos < heap.size());
        return heap[pos];
    };

    // Return smallest distance in heap
    W top() {
        if (heap.size()>1) {
            return heap[1].first;
        }
        return infinity;
    }

    // Return vertex index of smallest distance in heap
    int top_ver() {
        if (heap.size()>1) {
            return heap[1].second;
        }
        return -1;
    }

    // Return number of elements in heap
    size_t size() {
        return heap.size()-1;
    }
};


#endif //VERTEX_LABEL_ORACLES_HEAP_H
