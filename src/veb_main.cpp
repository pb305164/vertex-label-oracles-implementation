#include "veb.h"
#include "veb_temp.h"
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <unordered_set>
#include <vector>

int main() {
    using namespace std;
    cout << fixed;

    const int T = 10;
    const int M = 14;
    const int N = 1u<<M;

//    vebHeapTemp<M, int> *myMap = new vebHeapTemp<M, int>();
    VEBHeap *mySet = new VEBHeap(N);
//    unordered_set<unsigned int> set;
    vector<int> instr;
    for (int i=0; i<N; ++i) instr.push_back(rand()%N);
    
    clock_t begin, end;

    begin = clock();
    for (int t=0; t<T; ++t) {
        for (int i=0; i<N; ++i) {
            mySet->insert(instr[i]);
        }
        for (int i=0; i<N; ++i) {
            mySet->erase(instr[i]);
        }
    }
    end = clock();
    cout << double(end - begin) / CLOCKS_PER_SEC << endl;
/*   
    begin = clock();
    for (int t=0; t<T; ++t) {
        for (int i=0; i<N; ++i) {
            myMap->insert(instr[i], 0);
        }
        for (int i=0; i<N; ++i) {
            myMap->erase(instr[i]);
        }
    }
    end = clock();
    cout << double(end - begin) / CLOCKS_PER_SEC << endl;
    
    begin = clock();
    for (int t=0; t<T; ++t) {
        for (int i=0; i<N; ++i) {
            set.insert(instr[i]);
        }
        for (int i=0; i<N; ++i) {
            set.erase(instr[i]);
        }
    }
    end = clock();
    cout << double(end - begin) / CLOCKS_PER_SEC << endl;
*/
    return 0;
}
