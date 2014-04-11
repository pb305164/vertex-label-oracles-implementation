#ifndef _VEB_TEMP_H_
#define _VEB_TEMP_H_

#include <utility>
#include <iostream>
using std::cout;
using std::endl;
using std::swap;

template <unsigned int N, typename Val>
struct vebHeapTemp {
    typedef unsigned int uint;

    uint min, max;
    Val minVal, maxVal;

    vebHeapTemp<N/2, unsigned char> aux;
    vebHeapTemp<N/2, Val> sons[1<<((N+1)/2)];

    vebHeapTemp() {
        min = (1u<<N);
    }

    inline
    uint high(uint k) {
        return k >> (N/2);
    }

    inline
    uint low(uint k) {
        return k & ((1u<<(N/2))-1u);
    }
    
    inline
    bool empty() {
        return min == (1u<<N);
    }

    void insert(uint x, Val val) {
        if (empty()) {
            min = max = x;
            minVal = maxVal = val;
            return;
        }
        
        if (min == max) {
            if (x < min) {
                min = x;
                minVal = val;
                return;
            } else if (x > max) {
                max = x;
                maxVal = val;
            } else {
                minVal = maxVal = val;
            }
            return;
        }

        if (x == min) {
            minVal = val;
            return;
        }
        if (x == max) {
            maxVal = val;
            return;
        }
        
        if (x < min) {
            std::swap(x, min);
            std::swap(val, minVal);
        } else if(x > max) {
            std::swap(x, max);
            std::swap(val, maxVal);
        }
        
        aux.insert(high(x), 0);
        sons[high(x)].insert(low(x), val);
    }

    void erase(uint x) {
        if (empty()) return;

        if ((min == x) && (max == x)) {
            min = (1u<<N);
            return;
        }
        if (min == x) {
            if (aux.empty()) {
                min = max;
                minVal = maxVal;
                return;
            }
            min = sons[aux.min].min;
            minVal = sons[aux.min].minVal;
            x = min;
        } else if (max == x) {
            if (aux.empty()) {
                max = min;
                maxVal = minVal;
                return;
            }
            max = sons[aux.max].max;
            maxVal = sons[aux.max].maxVal;
            x = max;
        }
        
        if (aux.empty()) return;

        sons[high(x)].erase(low(x));
        if (sons[high(x)].empty()) aux.erase(high(x));
    }

    uint findNext(uint x) {
        if (x <= min) return min;
        if (x > max) return (1u<<N);
        if (x == max) return max;

        uint i = high(x);
        if (sons[i].empty() || sons[i].max < low(x)) {
            i = aux.findNext(i);
            if (i == 1<<(N/2)) return max;
            return (i << (N/2)) + sons[i].min;
        }
        return (i << (N/2)) + sons[i].findNext(low(x));
    }
};

template<typename Val>
struct vebHeapTemp<1, Val> {
    typedef unsigned int uint;

    uint min, max;
    Val minVal, maxVal;

    vebHeapTemp() {
        min = 2u;
    }

    inline
    bool empty() {
        return min == 2u;
    }
    
    void insert(uint x, Val val) {
        if (empty()) {
            min = max = x;
            minVal = maxVal = val;
            return;
        }
        
        if (min == max) {
            if (x < min) {
                min = x;
                minVal = val;
                return;
            } else if (x > max) {
                max = x;
                maxVal = val;
            } else {
                minVal = maxVal = val;
            }
            return;
        }

        if (x == min) {
            minVal = val;
            return;
        }
        if (x == max) {
            maxVal = val;
            return;
        }
        return;
    }
        
    void erase(uint x) {
        if ((min == x) && (max == x)) {
            min = 2u;
            return;
        }
        if (min == x) {
            min = max;
            minVal = maxVal;
            return;
        } else if (max == x) {
            max = min;
            maxVal = minVal;
            return;
        }
    }

    uint findNext(uint x) {
        if (x <= min) return min;
        if (x <= max) return max;
        return 2;
    }
};

#endif
