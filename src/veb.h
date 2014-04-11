#ifndef _VEB_H_
#define _VEB_H_

#include <utility>
#include <iostream>
using std::cout;
using std::endl;
using std::swap;

class VEBHeap {
    typedef unsigned int uint;
    typedef uint size_t;

    struct Node {
        uint k;
        uint min, max;
        Node *aux;
        Node *sons;

        Node () {}
        Node(uint kk) : min(1<<kk), k(kk) {
            if (kk > 1) {
                aux = new Node((k+1)/2);
                sons = new Node[1 << ((k+1)/2)];
                for (uint i=0; i<(1 << (k+1)/2); ++i) {
                    sons[i] = Node(k/2);
                }
            }
        }

        bool empty() {
            return min == (1u << k);
        }

        uint low(uint x) {
            return x & ((1<<(k/2))-1);
        }

        uint high(uint x) {
            return x >> (k/2);
        }

        void insert(uint x) {
            if (empty()) {
                min = max = x;
                return;
            }
            if (min == max) {
                if (x < min) {
                    min = x;
                } else if (x > max) {
                    max = x;
                }
                return;
            }
            
            if (k == 1) return;
            
            if (x <= min) {
                if (x == min) return;
                std::swap(x, min);
            } else if (x >= max) {
                if (x == max) return;
                std::swap(x, max);
            }

            uint h = high(x);
            uint l = low(x);
            if (sons[h].empty()) aux->insert(h);
            sons[h].insert(l);
        }

        void erase(uint x) {
            if (k == 1) {
                if (min == x) {
                    if (max == x) {
                        min = 2u;
                    } else {
                        min = max;
                    }
                } else if (max == x) {
                    max = min;
                }
                return;
            }

            if (min == x) {
                if (max == x) {
                    min = (1u << k);
                    return;
                }
                if (aux->empty()) {
                    min = max;
                    return;
                }
                uint h = aux->min;
                min = (h << (k/2)) + sons[h].min;
                x = min;
            } else if (max == x) {
                if (aux->empty()) {
                    max = min;
                    return;
                }
                uint h = aux->max;
                max = (h << (k/2)) + sons[h].max;
                x = max;
            }

            if (aux->empty()) return;
            
            uint h = high(x);
            uint l = low(x);
            
            sons[h].erase(l);
            if (sons[h].empty()) aux->erase(h);
        }

        uint succ(uint x) {
            if (x <= min) return min;
            if (x > max) return (1u << k);
            if (x == max) return max;

            uint h = high(x);
            uint l = low(x);
            if (sons[h].empty() || sons[h].max < l) {
                h = aux->succ(h);
                if (h == (1<<(k/2))) return max;
                return (h << (k/2)) + sons[h].min;
            }
            return (h << (k/2)) + sons[h].succ(l);
        }
    };

    Node* root;
    
public:
    VEBHeap() {}
    VEBHeap(uint n) {
        uint k = 1;
        while (n > 2) {
            n >>= 1;
            ++k;
        }
        root = new Node(k);
    }

    void insert(uint x) {
        root->insert(x);
    }

    void erase(uint x) {
        root->erase(x);
    }

    uint lowerBound(uint x) {
        return root->succ(x);
    }
};

#endif  
