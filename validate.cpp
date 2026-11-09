#include <bits/stdc++.h>
using namespace std;

// Validates solution for longest path problem
// Usage: validate graph.txt solution.txt

int main(int argc, char** argv) {
    if (argc < 3) {
        cerr << "Usage: validate graph.txt solution.txt\n";
        return 1;
    }
    
    ifstream gin(argv[1]);
    int N, M;
    gin >> N >> M;
    
    map<pair<int,int>, int> edgeW;
    for (int i = 0; i < M; i++) {
        int u, v, w;
        gin >> u >> v >> w;
        edgeW[{min(u,v), max(u,v)}] = w;
    }
    
    ifstream sol(argv[2]);
    int k;
    sol >> k;
    vector<int> path(k);
    for (int i = 0; i < k; i++) sol >> path[i];
    
    bool valid = true;
    set<int> seen;
    long long score = 0;
    
    for (int i = 0; i < k; i++) {
        if (path[i] < 0 || path[i] >= N) {
            cerr << "ERROR: node " << path[i] << " out of range\n";
            valid = false;
        }
        if (seen.count(path[i])) {
            cerr << "ERROR: node " << path[i] << " repeated\n";
            valid = false;
        }
        seen.insert(path[i]);
    }
    
    for (int i = 0; i+1 < k; i++) {
        auto key = make_pair(min(path[i], path[i+1]), max(path[i], path[i+1]));
        if (!edgeW.count(key)) {
            cerr << "ERROR: edge " << path[i] << "-" << path[i+1] << " missing\n";
            valid = false;
        } else {
            score += edgeW[key];
        }
    }
    
    if (valid)
        cout << "VALID | nodes=" << k << " | score=" << score << "\n";
    else
        cout << "INVALID\n";
    
    return valid ? 0 : 1;
}
