// ============================================================================
// Heuristic search for a heavy simple path in an undirected weighted graph.
//
// Input : n m, then m lines "u v w" (0-indexed vertices, edge weight w).
// Output: number of vertices on the path, then the vertices in order.
//
// Overview (see DOCUMENTATION.md for details):
//   Phase 1 - build many initial paths (DFS-forest diameter + greedy growth
//             + Posa rotations + local improvement), keep the best.
//   Phase 2 - iterated local search: cut off a tail of the best path, regrow
//             it randomly, locally improve, keep the result if not worse.
// ============================================================================
 
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>
using namespace std;

// ----------------------------------------------------------------------------
// PARAMETERS TO BE FINE TUNED
// ----------------------------------------------------------------------------
const double TIME_LIMIT = 29.9;                  // total time budget (seconds)
const double PHASE1_TIME_LIMIT = TIME_LIMIT / 2.75; // end of construction phase
const double IMPROVE_TIME_LIMIT = TIME_LIMIT / 4;   // no more improve() rounds after this
const int BUILD_TRIES = 100;                     // rotation attempts in phase 1
const int KICK_TRIES = 20;                       // rotation attempts in phase 2

// Penalties for "free degree" used when choosing the next vertex / DFS order.
const double DEGREE_PENALTIES[] = {0, 5, 15, 30};
const int NUM_PENALTIES = 4;

// ----------------------------------------------------------------------------
// Global state
// ----------------------------------------------------------------------------
int numVertices, numEdges;
int minDirtyIdx;        // Smallest path index whose content changed since last sync
long long curWeight;    // Total weight of the current path

// adjacency[v] = sorted list of (neighbor, weight); no multi-edges
vector<vector<pair<int, int>>> adjacency;
vector<int> path;              // Current path (sequence of vertices)
vector<int> pos;               // Position of vertex in path (-1 if not on path)
vector<int> weightToTarget;    // Scratch array: weight of edge (target, v), 0 if none (used in improve())

mt19937 rng(7);
auto startTime = chrono::steady_clock::now();

// ----------------------------------------------------------------------------
// Utilities
// ----------------------------------------------------------------------------

// Seconds elapsed since program start.
double elapsed() {
    return chrono::duration<double>(chrono::steady_clock::now() - startTime).count();
}

// Weight of edge (a, b) via binary search; 0 if the edge does not exist.
int getWeight(int a, int b) {
    auto it = lower_bound(adjacency[a].begin(), adjacency[a].end(), b,
        [](const pair<int, int>& edge, int v) { return edge.first < v; });
    return (it != adjacency[a].end() && it->first == b) ? it->second : 0;
}

// Weight of the path edge between path[i] and path[i+1]; 0 if out of range.
int pathEdgeWeight(int i) {
    return (i >= 0 && i + 1 < (int)path.size()) ? getWeight(path[i], path[i + 1]) : 0;
}

// Number of neighbors of u that are not on the path.
int freeDeg(int u) {
    int deg = 0;
    for (auto& edge : adjacency[u]) {
        deg += (pos[edge.first] < 0);
    }
    return deg;
}

// ----------------------------------------------------------------------------
// Basic path operations (keep `pos` and `curWeight` consistent)
// ----------------------------------------------------------------------------
// Append vertex v to the end of the path.
void pushVertex(int v) {
    if (!path.empty()) {
        curWeight += getWeight(path.back(), v);
    }
    pos[v] = path.size();
    path.push_back(v);
}

// Remove and return the last vertex of the path.
int popVertex() {
    int v = path.back();
    curWeight -= pathEdgeWeight(path.size() - 2);
    pos[v] = -1;
    path.pop_back();
    return v;
}

// Reverse path[i..j] (inclusive) and update weight and positions.
void reverseSubpath(int i, int j) {
    curWeight -= pathEdgeWeight(i - 1) + pathEdgeWeight(j);
    reverse(path.begin() + i, path.begin() + j + 1);
    for (int t = i; t <= j; t++) {
        pos[path[t]] = t;
    }
    curWeight += pathEdgeWeight(i - 1) + pathEdgeWeight(j);
    minDirtyIdx = min(minDirtyIdx, i);
}

// Reverse the whole path.
void flipPath() {
    reverseSubpath(0, path.size() - 1);
}

// Replace the current path by newPath, rewriting only indices >= `from`
// (the prefix [0, from) is assumed to be identical in both paths).
void setPath(const vector<int>& newPath, long long newWeight, int from = 0) {
    for (int i = from; i < (int)path.size(); i++) {
        pos[path[i]] = -1;
    }
    path.resize(newPath.size());
    copy(newPath.begin() + from, newPath.end(), path.begin() + from);
    for (int i = from; i < (int)path.size(); i++) {
        pos[path[i]] = i;
    }
    curWeight = newWeight;
}
 
// ----------------------------------------------------------------------------
// Path growth
// ----------------------------------------------------------------------------
 
// Greedily extend the end of the path. Each candidate neighbor is scored by
// edge weight, penalized by its free degree (and heavily if it would be a
// dead end), plus a small random noise.
void grow(double degPenalty) {
    for (;;) {
        int bestNext = -1;
        double bestScore = -1e18;
        for (auto& edge : adjacency[path.back()]) {
            if (pos[edge.first] < 0) {
                int candidate = edge.first, weight = edge.second;
                int freeDegree = freeDeg(candidate);
                double score = weight - degPenalty * freeDegree
                               - (freeDegree == 0) * 1000 + rng() % 20;
                if (score > bestScore) {
                    bestScore = score;
                    bestNext = candidate;
                }
            }
        }
        if (bestNext < 0) return;
        pushVertex(bestNext);
    }
}
 
// Posa rotation: reverse a suffix of the path so that a different vertex
// becomes the endpoint, which may allow further growth.
// Prefers rotations whose new endpoint still has free neighbors.
bool rotateEnd() {
    int last = path.size() - 1;
    vector<int> goodCandidates, allCandidates;
    for (auto& edge : adjacency[path[last]]) {
        int i = pos[edge.first];
        if (i < 0 || i >= last - 1) continue;
        allCandidates.push_back(i);
        if (freeDeg(path[i + 1]) > 0) {
            goodCandidates.push_back(i);
        }
    }
    vector<int>& candidates = goodCandidates.empty() ? allCandidates : goodCandidates;
    if (candidates.empty()) return false;
 
    int splitIdx = candidates[rng() % candidates.size()];
    reverseSubpath(splitIdx + 1, last);
    return true;
}
 
// Grow the path end, alternating rotations and greedy growth.
// The try counter resets whenever a new longest length is reached.
void growEnd(double degPenalty, int maxTries) {
    grow(degPenalty);
    for (int t = 0, maxLen = path.size(); t < maxTries && elapsed() < TIME_LIMIT; t++) {
        if (!rotateEnd()) {
            if (path.size() == 1) return;
            // Try dropping the last vertex and rotating again
            int removed = popVertex();
            if (!rotateEnd()) {
                pushVertex(removed);
                return;
            }
        }
        grow(degPenalty);
        if ((int)path.size() > maxLen) {
            maxLen = path.size();
            t = 0;
        }
    }
}
 
// Extend one end, flip the path, extend the other end; stop after two
// consecutive passes without any progress.
void extendBothEnds(double degPenalty, int maxTries) {
    for (int stuck = 0; stuck < 2;) {
        int prevLen = path.size();
        growEnd(degPenalty, maxTries);
        flipPath();
        stuck = ((int)path.size() > prevLen) ? 0 : stuck + 1;
    }
}
 
// ----------------------------------------------------------------------------
// Local improvement
// ----------------------------------------------------------------------------
 
// Local improvement of the path from index `from` onward:
//   1. Replace a single vertex:  a -> c -> b   becomes   a -> x -> b
//   2. Insert 1 or 2 free vertices between consecutive vertices a -> b.
// Returns true if anything improved.
bool improve(int from) {
    bool improved = false;
    int len = path.size();
 
    // --- 1. Vertex substitution ---
    for (int i = max(from, 1); i + 1 < len; i++) {
        int prev = path[i - 1], current = path[i], next = path[i + 1];
        int oldWeight = getWeight(prev, current) + getWeight(current, next);
        int bestWeight = oldWeight, bestX = -1;
 
        // Mark weights of edges into `next` for O(1) lookup
        for (auto& edge : adjacency[next]) weightToTarget[edge.first] = edge.second;
        for (auto& edge : adjacency[prev]) {
            int x = edge.first, weight = edge.second;
            if (pos[x] < 0 && weightToTarget[x] && weight + weightToTarget[x] > bestWeight) {
                bestWeight = weight + weightToTarget[x];
                bestX = x;
            }
        }
        for (auto& edge : adjacency[next]) weightToTarget[edge.first] = 0;
 
        if (bestX >= 0) {
            pos[current] = -1;
            pos[bestX] = i;
            path[i] = bestX;
            curWeight += bestWeight - oldWeight;
            improved = true;
        }
    }
 
    // --- 2. Insertion of 1 or 2 free vertices (builds a new path) ---
    vector<int> newPath(path.begin(), path.begin() + from);
    for (int i = from; i < len; i++) {
        int a = path[i];
        newPath.push_back(a);
        if (i + 1 == len) break;
 
        int b = path[i + 1];
        int oldWeight = getWeight(a, b);
        int bestWeight = oldWeight, bestX = -1, bestY = -1;
 
        for (auto& edge : adjacency[b]) weightToTarget[edge.first] = edge.second;
        for (auto& edgeA : adjacency[a]) {
            if (pos[edgeA.first] < 0) {
                int x = edgeA.first, weightAX = edgeA.second;
                // Insert a single vertex: a -> x -> b
                if (weightToTarget[x] && weightAX + weightToTarget[x] > bestWeight) {
                    bestWeight = weightAX + weightToTarget[x];
                    bestX = x;
                    bestY = -1;
                }
                // Insert two vertices: a -> x -> y -> b
                for (auto& edgeX : adjacency[x]) {
                    int y = edgeX.first, weightXY = edgeX.second;
                    if (pos[y] < 0 && weightToTarget[y]
                        && weightAX + weightXY + weightToTarget[y] > bestWeight) {
                        bestWeight = weightAX + weightXY + weightToTarget[y];
                        bestX = x;
                        bestY = y;
                    }
                }
            }
        }
        for (auto& edge : adjacency[b]) weightToTarget[edge.first] = 0;
 
        if (bestX >= 0) {
            newPath.push_back(bestX);
            pos[bestX] = 0; // temporary "used" marker; real positions set below
            curWeight += bestWeight - oldWeight;
            improved = true;
            if (bestY >= 0) {
                newPath.push_back(bestY);
                pos[bestY] = 0;
            }
        }
    }
 
    path = newPath;
    for (int i = from; i < (int)path.size(); i++) {
        pos[path[i]] = i;
    }
    return improved;
}
 
// ----------------------------------------------------------------------------
// Initial path construction
// ----------------------------------------------------------------------------
 
// Build a DFS forest (neighbors ordered by weight minus degree penalty plus
// noise, random roots) and return the heaviest path inside it (tree diameter).
vector<int> dfsPath(double degPenalty) {
    vector<int> parent(numVertices, -1), parentWeight(numVertices, 0);
    vector<int> nextNeighborIdx(numVertices, 0), visited(numVertices, 0);
    vector<int> bestChild(numVertices, -1), secondChild(numVertices, -1);
    vector<int> visitOrder, dfsStack, roots(numVertices);
    vector<long long> bestDown(numVertices, 0), secondDown(numVertices, 0);
    // sortedAdj[v] = (priority, neighbor), ascending priority = visited first
    vector<vector<pair<int, int>>> sortedAdj(numVertices);
 
    for (int v = 0; v < numVertices; v++) {
        for (auto& edge : adjacency[v]) {
            int priority = -(int)(edge.second - degPenalty * adjacency[edge.first].size() + rng() % 20);
            sortedAdj[v].push_back({priority, edge.first});
        }
        sort(sortedAdj[v].begin(), sortedAdj[v].end());
    }
 
    for (int v = 0; v < numVertices; v++) roots[v] = v;
    shuffle(roots.begin(), roots.end(), rng);
 
    // Iterative DFS over all components
    for (int root : roots) {
        if (!visited[root]) {
            visited[root] = 1;
            visitOrder.push_back(root);
            dfsStack.push_back(root);
            while (!dfsStack.empty()) {
                int v = dfsStack.back();
                if (nextNeighborIdx[v] == (int)sortedAdj[v].size()) {
                    dfsStack.pop_back();
                    continue;
                }
                int u = sortedAdj[v][nextNeighborIdx[v]++].second;
                if (visited[u]) continue;
                visited[u] = 1;
                parent[u] = v;
                parentWeight[u] = getWeight(v, u);
                visitOrder.push_back(u);
                dfsStack.push_back(u);
            }
        }
    }
 
    // Tree DP (children before parents): for each vertex keep the two heaviest
    // downward branches.
    for (int i = numVertices - 1; i >= 0; i--) {
        int v = visitOrder[i], u = parent[v];
        if (u < 0) continue;
        long long down = bestDown[v] + parentWeight[v];
        if (down > bestDown[u]) {
            secondDown[u] = bestDown[u];
            secondChild[u] = bestChild[u];
            bestDown[u] = down;
            bestChild[u] = v;
        } else if (down > secondDown[u]) {
            secondDown[u] = down;
            secondChild[u] = v;
        }
    }
 
    // Vertex where two heaviest branches combine into the heaviest path
    int bestTop = 0;
    for (int x = 0; x < numVertices; x++) {
        if (bestDown[x] + secondDown[x] > bestDown[bestTop] + secondDown[bestTop]) {
            bestTop = x;
        }
    }
 
    // Reconstruct: second branch (reversed), then top, then best branch
    vector<int> result;
    for (int x = secondChild[bestTop]; x >= 0; x = bestChild[x]) result.push_back(x);
    reverse(result.begin(), result.end());
    for (int x = bestTop; x >= 0; x = bestChild[x]) result.push_back(x);
    return result;
}
 
// ----------------------------------------------------------------------------
// Main
// ----------------------------------------------------------------------------
int main() {
    scanf("%d %d", &numVertices, &numEdges);
    adjacency.resize(numVertices);
    pos.assign(numVertices, -1);
    weightToTarget.assign(numVertices, 0);
 
    // Read edges (self-loops ignored)
    for (int i = 0, u, v, w; i < numEdges; i++) {
        scanf("%d %d %d", &u, &v, &w);
        if (u != v) {
            adjacency[u].push_back({v, w});
            adjacency[v].push_back({u, w});
        }
    }
 
    // Remove multi-edges (keep the heaviest) and sort adjacency lists by neighbor id
    for (auto& neighbors : adjacency) {
        sort(neighbors.begin(), neighbors.end(), [](const pair<int, int>& x, const pair<int, int>& y) {
            return x.first != y.first ? x.first < y.first : x.second > y.second;
        });
        neighbors.erase(unique(neighbors.begin(), neighbors.end(), [](const pair<int, int>& x, const pair<int, int>& y) {
            return x.first == y.first;
        }), neighbors.end());
    }
 
    vector<int> bestPath = {0};
    long long bestWeight = 0;
 
    // ---- Phase 1: construct initial paths and keep the best ----
    for (int t = 0; elapsed() < PHASE1_TIME_LIMIT; t++) {
        double penalty = DEGREE_PENALTIES[t % NUM_PENALTIES];
        vector<int> initPath = dfsPath(penalty);
        long long initWeight = 0;
        for (int i = 0; i + 1 < (int)initPath.size(); i++) {
            initWeight += getWeight(initPath[i], initPath[i + 1]);
        }
        setPath(initPath, initWeight);
        extendBothEnds(penalty, BUILD_TRIES);
        for (int round = 0; round < 5 && improve(0) && elapsed() < IMPROVE_TIME_LIMIT; round++) {
            extendBothEnds(penalty, BUILD_TRIES);
        }
        if (curWeight > bestWeight) {
            bestWeight = curWeight;
            bestPath = path;
        }
    }
 
    // ---- Phase 2: iterated local search (cut the tail, regrow, improve) ----
    setPath(bestPath, bestWeight);
    int iter = 0;
    while (elapsed() < TIME_LIMIT && ++iter) {
        // Randomly work on either end of the path
        if (rng() % 2) {
            flipPath();
            reverse(bestPath.begin(), bestPath.end());
        }
        minDirtyIdx = path.size();
        if (rng() % 2) {
            rotateEnd();
        }
 
        // Choose how much of the path to keep: usually cut a short tail
        // (up to 100 vertices), occasionally (1 in 20) a long one (up to 5000).
        int keepIdx = max(0, (int)path.size() - 2 - (int)(rng() % (rng() % 20 ? 100 : 5000)));
        // Make sure the new endpoint still has a free neighbor
        while (keepIdx > 0 && freeDeg(path[keepIdx]) == 0) {
            keepIdx--;
        }
        while ((int)path.size() > keepIdx + 1) {
            popVertex();
        }
 
        minDirtyIdx = min(minDirtyIdx, keepIdx + 1);
        growEnd(DEGREE_PENALTIES[rng() % NUM_PENALTIES], KICK_TRIES);
 
        // Full improvement every 100 iterations, otherwise only the changed region
        if (iter % 100 == 0) {
            minDirtyIdx = 0;
            improve(0);
        } else {
            improve(max(max(minDirtyIdx - 2, (int)path.size() - 1000), 0));
        }
 
        // Accept if not worse (copy only the changed suffix), else roll back
        int syncFrom = max(minDirtyIdx - 3, 0);
        if (curWeight >= bestWeight) {
            bestWeight = curWeight;
            bestPath.resize(path.size());
            copy(path.begin() + syncFrom, path.end(), bestPath.begin() + syncFrom);
        } else {
            setPath(bestPath, bestWeight, syncFrom);
        }
    }
 
    // Output
    printf("%d\n", (int)bestPath.size());
    for (int v : bestPath) printf("%d ", v);
    printf("\n");
}