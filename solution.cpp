#include <bits/stdc++.h>
using namespace std;

// Compressed Sparse Row (CSR) graph representation for fast cache-friendly traversal
static int N, M;
static int *adjStart, *adjTo, *adjW;

// Global best solution found
static vector<int> bestPath;
static long long bestScore = 0;

static mt19937 rng(42);
static chrono::steady_clock::time_point startTime;
static int TIME_LIMIT = 29500; // 29.5 seconds (leaving 0.5s margin for I/O)

inline int elapsedMs() {
    return (int)chrono::duration_cast<chrono::milliseconds>(
        chrono::steady_clock::now() - startTime).count();
}

// Find weight of edge (u, v); returns 0 if not connected
inline int getEdgeWeight(int u, int v) {
    for (int i = adjStart[u]; i < adjStart[u + 1]; i++) {
        if (adjTo[i] == v) return adjW[i];
    }
    return 0;
}

// Maintains the current path and O(1) position lookup
struct PathState {
    vector<int> path;       // Ordered list of vertices on the path
    vector<int> pos;        // pos[v] = index in path, or -1 if not on path
    long long score = 0;

    PathState(int n) : pos(n, -1) {}

    void clear() {
        for (int v : path) pos[v] = -1;
        path.clear();
        score = 0;
    }

    void init(int startNode) {
        clear();
        path.push_back(startNode);
        pos[startNode] = 0;
    }

    void pushBack(int v, int w) {
        pos[v] = (int)path.size();
        path.push_back(v);
        score += w;
    }

    void reversePath() {
        reverse(path.begin(), path.end());
        for (int i = 0; i < (int)path.size(); i++) {
            pos[path[i]] = i;
        }
    }

    void reverseSubsegment(int fromIdx, int toIdx) {
        reverse(path.begin() + fromIdx, path.begin() + toIdx + 1);
        for (int i = fromIdx; i <= toIdx; i++) {
            pos[path[i]] = i;
        }
    }
};

// Extend path from its back end greedily (preferring heavier edges)
bool extendEnd(PathState& state, bool randomize) {
    bool extended = false;
    while (true) {
        int u = state.path.back();
        
        int c1 = -1, w1 = 0;
        int c2 = -1, w2 = 0;
        int c3 = -1, w3 = 0;

        for (int i = adjStart[u]; i < adjStart[u + 1]; i++) {
            int v = adjTo[i], w = adjW[i];
            if (state.pos[v] != -1) continue; // already on path

            if (w > w1) {
                c3 = c2; w3 = w2;
                c2 = c1; w2 = w1;
                c1 = v;  w1 = w;
            } else if (w > w2) {
                c3 = c2; w3 = w2;
                c2 = v;  w2 = w;
            } else if (w > w3) {
                c3 = v;  w3 = w;
            }
        }

        if (c1 == -1) break; // no unvisited neighbors

        int chosen = c1, chosenW = w1;
        if (randomize && c2 != -1) {
            long long tot = (long long)w1 * w1 + (long long)w2 * w2;
            if (c3 != -1) tot += (long long)w3 * w3;
            long long r = rng() % tot;
            if (r < (long long)w1 * w1) {
                chosen = c1; chosenW = w1;
            } else if (r < (long long)w1 * w1 + (long long)w2 * w2) {
                chosen = c2; chosenW = w2;
            } else {
                chosen = c3; chosenW = w3;
            }
        }

        state.pushBack(chosen, chosenW);
        extended = true;
    }
    return extended;
}

// Extend from back, reverse, and extend from new back (both ends)
void extendBothEnds(PathState& state, bool randomize) {
    extendEnd(state, randomize);
    state.reversePath();
    extendEnd(state, randomize);
}

// Pósa endpoint rotation:
// If endpoint u = path.back() is connected to path[i] on the path,
// we reverse subsegment [i+1, k-1]. The old path[i+1] becomes the new endpoint!
bool tryRotation(PathState& state) {
    int k = (int)state.path.size();
    if (k < 4) return false;

    int u = state.path.back();
    
    int bestI = -1;
    int bestGain = -1e9;
    int bestNewW = 0, bestOldW = 0;

    for (int idx = adjStart[u]; idx < adjStart[u + 1]; idx++) {
        int vi = adjTo[idx];
        int i = state.pos[vi];
        if (i < 0 || i >= k - 2) continue;

        int vi_next = state.path[i + 1];
        int oldW = getEdgeWeight(vi, vi_next);
        int newW = adjW[idx];
        int gain = newW - oldW;

        // Check if the new endpoint would have unvisited neighbors
        bool hasFreeNeighbor = false;
        for (int j = adjStart[vi_next]; j < adjStart[vi_next + 1]; j++) {
            if (state.pos[adjTo[j]] == -1) {
                hasFreeNeighbor = true;
                break;
            }
        }

        if (hasFreeNeighbor) {
            // Prioritize rotations that unlock new vertices
            if (gain + 1000 > bestGain) {
                bestGain = gain + 1000;
                bestI = i;
                bestNewW = newW;
                bestOldW = oldW;
            }
        } else if (gain > bestGain && gain >= 0) {
            bestGain = gain;
            bestI = i;
            bestNewW = newW;
            bestOldW = oldW;
        }
    }

    if (bestI != -1) {
        state.score += (bestNewW - bestOldW);
        state.reverseSubsegment(bestI + 1, k - 1);
        return true;
    }

    return false;
}

// Build a path from startNode using extensions and rotations
void searchPath(int startNode, PathState& state) {
    state.init(startNode);
    extendBothEnds(state, true);

    int maxRotations = 60;
    for (int r = 0; r < maxRotations; r++) {
        bool rotated = tryRotation(state);
        if (!rotated) {
            state.reversePath();
            rotated = tryRotation(state);
        }
        if (!rotated) break;

        extendBothEnds(state, false);
    }

    if (state.score > bestScore) {
        bestScore = state.score;
        bestPath = state.path;
    }
}

// Cut a fraction of the path from an end and re-extend
void perturbPath(PathState& state) {
    int k = (int)state.path.size();
    if (k < 10) return;

    int cutCount = max(3, (int)(k * (0.05 + 0.20 * (rng() % 1000) / 1000.0)));
    cutCount = min(cutCount, k - 2);

    if (rng() % 2 == 0) {
        state.reversePath();
    }

    for (int i = 0; i < cutCount; i++) {
        int removed = state.path.back();
        state.pos[removed] = -1;
        state.path.pop_back();
        int newEnd = state.path.back();
        state.score -= getEdgeWeight(newEnd, removed);
    }

    extendBothEnds(state, true);

    for (int r = 0; r < 30; r++) {
        if (!tryRotation(state)) {
            state.reversePath();
            if (!tryRotation(state)) break;
        }
        extendBothEnds(state, false);
    }

    if (state.score > bestScore) {
        bestScore = state.score;
        bestPath = state.path;
    }
}

// BFS to find the farthest node by total edge weight, returning node and reconstructed path
pair<int, long long> bfsFarthest(int src, vector<int>& outPath) {
    vector<long long> dist(N, -1);
    vector<int> parent(N, -1);
    queue<int> q;
    dist[src] = 0;
    q.push(src);

    int farthest = src;
    long long maxDist = 0;

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (int i = adjStart[u]; i < adjStart[u + 1]; i++) {
            int v = adjTo[i];
            if (dist[v] == -1) {
                dist[v] = dist[u] + adjW[i];
                parent[v] = u;
                q.push(v);
                if (dist[v] > maxDist) {
                    maxDist = dist[v];
                    farthest = v;
                }
            }
        }
    }

    outPath.clear();
    int curr = farthest;
    while (curr != -1) {
        outPath.push_back(curr);
        curr = parent[curr];
    }
    return {farthest, maxDist};
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    startTime = chrono::steady_clock::now();

    const char* tlEnv = getenv("TIME_LIMIT_MS");
    if (tlEnv) TIME_LIMIT = atoi(tlEnv);

    cin >> N >> M;

    vector<vector<pair<int,int>>> tmpAdj(N);
    for (int i = 0; i < M; i++) {
        int u, v, w;
        cin >> u >> v >> w;
        tmpAdj[u].push_back({v, w});
        tmpAdj[v].push_back({u, w});
    }

    // Sort adjacency lists by weight descending
    for (int i = 0; i < N; i++) {
        sort(tmpAdj[i].begin(), tmpAdj[i].end(), [](const pair<int,int>& a, const pair<int,int>& b) {
            return a.second > b.second;
        });
    }

    // Build CSR representation
    adjStart = new int[N + 1];
    adjStart[0] = 0;
    for (int i = 0; i < N; i++) {
        adjStart[i + 1] = adjStart[i] + (int)tmpAdj[i].size();
    }
    int totalEdges = adjStart[N];
    adjTo = new int[totalEdges];
    adjW = new int[totalEdges];
    for (int i = 0; i < N; i++) {
        int off = adjStart[i];
        for (int j = 0; j < (int)tmpAdj[i].size(); j++) {
            adjTo[off + j] = tmpAdj[i][j].first;
            adjW[off + j] = tmpAdj[i][j].second;
        }
    }
    tmpAdj.clear();
    tmpAdj.shrink_to_fit();

    // Collect candidate starting vertices:
    // 1. Extreme endpoints from BFS diameter finding (also optimal for trees)
    vector<int> startNodes;
    vector<int> bfsPath1, bfsPath2;
    auto [f1, d1] = bfsFarthest(0, bfsPath1);
    auto [f2, d2] = bfsFarthest(f1, bfsPath2);
    if (d2 > bestScore) {
        bestScore = d2;
        bestPath = bfsPath2;
    }
    startNodes.push_back(f1);
    startNodes.push_back(f2);
    vector<int> bfsPath3;
    auto [f3, d3] = bfsFarthest(f2, bfsPath3);
    if (d3 > bestScore) {
        bestScore = d3;
        bestPath = bfsPath3;
    }
    startNodes.push_back(f3);

    // 2. Leaf nodes (degree 1)
    for (int i = 0; i < N; i++) {
        if (adjStart[i + 1] - adjStart[i] == 1) {
            startNodes.push_back(i);
        }
    }

    // 3. Nodes with highest weighted degree
    vector<pair<long long, int>> degList;
    for (int u = 0; u < N; u++) {
        long long sumW = 0;
        for (int i = adjStart[u]; i < adjStart[u + 1]; i++) sumW += adjW[i];
        degList.push_back({sumW, u});
    }
    sort(degList.rbegin(), degList.rend());
    for (int i = 0; i < min(N, 100); i++) {
        startNodes.push_back(degList[i].second);
    }

    // Remove duplicates
    sort(startNodes.begin(), startNodes.end());
    startNodes.erase(unique(startNodes.begin(), startNodes.end()), startNodes.end());

    PathState state(N);

    // Phase 1: Search from quality starting nodes (first 40% of time)
    int phase1End = TIME_LIMIT * 40 / 100;
    int startIdx = 0;
    while (elapsedMs() < phase1End) {
        int startNode;
        if (startIdx < (int)startNodes.size()) {
            startNode = startNodes[startIdx++];
        } else {
            if (rng() % 2 == 0) {
                startNode = degList[rng() % min(N, 200)].second;
            } else {
                startNode = rng() % N;
            }
        }
        searchPath(startNode, state);
    }

    // Phase 2: Perturbation and improvement of the best path found (next 45% of time)
    int phase2End = TIME_LIMIT * 85 / 100;
    while (elapsedMs() < phase2End) {
        state.clear();
        for (int i = 0; i < (int)bestPath.size(); i++) {
            state.path.push_back(bestPath[i]);
            state.pos[bestPath[i]] = i;
        }
        state.score = bestScore;

        perturbPath(state);
    }

    // Phase 3: Final quick random restarts with remaining time
    while (elapsedMs() < TIME_LIMIT) {
        int startNode;
        if (rng() % 2 == 0 && !startNodes.empty()) {
            startNode = startNodes[rng() % startNodes.size()];
        } else {
            startNode = rng() % N;
        }
        searchPath(startNode, state);
    }

    // Output result
    cout << bestPath.size() << "\n";
    for (int i = 0; i < (int)bestPath.size(); i++) {
        if (i) cout << " ";
        cout << bestPath[i];
    }
    cout << "\n";

    delete[] adjStart;
    delete[] adjTo;
    delete[] adjW;

    return 0;
}
