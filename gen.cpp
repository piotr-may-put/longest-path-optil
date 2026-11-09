#include <bits/stdc++.h>
using namespace std;

// Generates test graphs for longest path problem
// Usage: gen <type> <N> <M> [seed]
// Types: random, sparse, dense, tree, grid

int main(int argc, char** argv) {
    string type = argc > 1 ? argv[1] : "random";
    int N = argc > 2 ? atoi(argv[2]) : 100;
    int M = argc > 3 ? atoi(argv[3]) : 300;
    int seed = argc > 4 ? atoi(argv[4]) : 42;
    
    mt19937 rng(seed);
    set<pair<int,int>> edges;
    vector<tuple<int,int,int>> edgeList;
    
    auto addEdge = [&](int u, int v, int w) {
        if (u > v) swap(u, v);
        if (u == v) return;
        if (edges.count({u, v})) return;
        edges.insert({u, v});
        edgeList.push_back({u, v, w});
    };
    
    if (type == "tree") {
        M = N - 1;
        for (int i = 1; i < N; i++) {
            int p = uniform_int_distribution<>(0, i-1)(rng);
            int w = uniform_int_distribution<>(1, 100)(rng);
            addEdge(p, i, w);
        }
    } else if (type == "grid") {
        int side = (int)sqrt(N);
        N = side * side;
        M = 0;
        for (int r = 0; r < side; r++) {
            for (int c = 0; c < side; c++) {
                int u = r * side + c;
                if (c + 1 < side) addEdge(u, u + 1, uniform_int_distribution<>(1,100)(rng));
                if (r + 1 < side) addEdge(u, u + side, uniform_int_distribution<>(1,100)(rng));
            }
        }
        M = edgeList.size();
    } else {
        // Ensure connected: spanning tree first
        vector<int> perm(N);
        iota(perm.begin(), perm.end(), 0);
        shuffle(perm.begin(), perm.end(), rng);
        for (int i = 1; i < N; i++) {
            int w = uniform_int_distribution<>(1, 100)(rng);
            addEdge(perm[i-1], perm[i], w);
        }
        // Add remaining random edges
        int attempts = 0;
        while ((int)edgeList.size() < M && attempts < M * 10) {
            int u = uniform_int_distribution<>(0, N-1)(rng);
            int v = uniform_int_distribution<>(0, N-1)(rng);
            int w = uniform_int_distribution<>(1, 100)(rng);
            addEdge(u, v, w);
            attempts++;
        }
        M = edgeList.size();
    }
    
    cout << N << " " << M << "\n";
    for (auto& [u, v, w] : edgeList)
        cout << u << " " << v << " " << w << "\n";
    
    return 0;
}
