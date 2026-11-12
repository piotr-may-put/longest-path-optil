#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
using namespace std;

const double TIME_LIMIT = 25.0;
const int BUILD_TRIES = 100;
const int KICK_TRIES = 20;

int n, m;
int minDirtyIdx; // Najmniejszy indeks, od którego ścieżka uległa zmianie
long long curWeight;

vector<vector<pair<int, int>>> adj;
vector<int> path;
vector<int> pos;       // Pozycja wierzchołka w ścieżce (-1 jeśli poza ścieżką)
vector<int> edgeToNext; // Tablica pomocnicza do szybkiego sprawdzania krawędzi w improve()

mt19937 rng(7);
auto startTime = chrono::steady_clock::now();

double elapsed() {
    return chrono::duration<double>(chrono::steady_clock::now() - startTime).count();
}

int getWeight(int a, int b) {
    auto it = lower_bound(adj[a].begin(), adj[a].end(), b,
        [](const pair<int, int>& e, int v) { return e.first < v; });
    return (it != adj[a].end() && it->first == b) ? it->second : 0;
}

int pathEdgeWeight(int i) {
    return (i >= 0 && i + 1 < (int)path.size()) ? getWeight(path[i], path[i + 1]) : 0;
}

int freeDeg(int u) {
    int deg = 0;
    for (auto& e : adj[u]) {
        deg += (pos[e.first] < 0);
    }
    return deg;
}

void pushVertex(int v) {
    if (!path.empty()) {
        curWeight += getWeight(path.back(), v);
    }
    pos[v] = path.size();
    path.push_back(v);
}

int popVertex() {
    int v = path.back();
    curWeight -= pathEdgeWeight(path.size() - 2);
    pos[v] = -1;
    path.pop_back();
    return v;
}

void reverseSubpath(int i, int j) {
    curWeight -= pathEdgeWeight(i - 1) + pathEdgeWeight(j);
    reverse(path.begin() + i, path.begin() + j + 1);
    for (int t = i; t <= j; t++) {
        pos[path[t]] = t;
    }
    curWeight += pathEdgeWeight(i - 1) + pathEdgeWeight(j);
    minDirtyIdx = min(minDirtyIdx, i);
}

void flipPath() {
    reverseSubpath(0, path.size() - 1);
}

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

// Zachłanne przedłużanie końca ścieżki z karą za wysoki stopień swobody
void grow(double degPenalty) {
    for (;;) {
        int bestNext = -1;
        double bestScore = -1e18;
        for (auto& e : adj[path.back()]) {
            if (pos[e.first] < 0) {
                int u = e.first, w = e.second, d = freeDeg(u);
                double score = w - degPenalty * d - (d == 0) * 1000 + rng() % 20;
                if (score > bestScore) {
                    bestScore = score;
                    bestNext = u;
                }
            }
        }
        if (bestNext < 0) return;
        pushVertex(bestNext);
    }
}

// Rotacja Pósy odwracająca sufiks ścieżki, aby odblokować możliwość dalszego wzrostu
bool rotateEnd() {
    int last = path.size() - 1;
    vector<int> goodCandidates, allCandidates;
    for (auto& e : adj[path[last]]) {
        int i = pos[e.first];
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

void growEnd(double degPenalty, int maxTries) {
    grow(degPenalty);
    for (int t = 0, maxLen = path.size(); t < maxTries && elapsed() < TIME_LIMIT; t++) {
        if (!rotateEnd()) {
            if (path.size() == 1) return;
            int v = popVertex();
            if (!rotateEnd()) {
                pushVertex(v);
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

// Lokalna poprawa ścieżki: podmiana pojedynczych wierzchołków oraz wstawianie 1-2 nowych
bool improve(int from) {
    bool improved = false;
    int len = path.size();

    // 1. Podmiana wierzchołka: a -> c -> b na a -> x -> b
    for (int i = max(from, 1); i + 1 < len; i++) {
        int a = path[i - 1], c = path[i], b = path[i + 1];
        int oldW = getWeight(a, c) + getWeight(c, b);
        int bestW = oldW, bestX = -1;

        for (auto& e : adj[b]) edgeToNext[e.first] = e.second;
        for (auto& e : adj[a]) {
            int x = e.first, w = e.second;
            if (pos[x] < 0 && edgeToNext[x] && w + edgeToNext[x] > bestW) {
                bestW = w + edgeToNext[x];
                bestX = x;
            }
        }
        for (auto& e : adj[b]) edgeToNext[e.first] = 0;

        if (bestX >= 0) {
            pos[c] = -1;
            pos[bestX] = i;
            path[i] = bestX;
            curWeight += bestW - oldW;
            improved = true;
        }
    }

    // 2. Wstawianie 1 lub 2 wolnych wierzchołków pomiędzy sąsiednie a -> b
    vector<int> newPath(path.begin(), path.begin() + from);
    for (int i = from; i < len; i++) {
        int a = path[i];
        newPath.push_back(a);
        if (i + 1 == len) break;

        int b = path[i + 1];
        int oldW = getWeight(a, b);
        int bestW = oldW, bestX = -1, bestY = -1;

        for (auto& e : adj[b]) edgeToNext[e.first] = e.second;
        for (auto& e : adj[a]) {
            if (pos[e.first] < 0) {
                int x = e.first, wx = e.second;
                if (edgeToNext[x] && wx + edgeToNext[x] > bestW) {
                    bestW = wx + edgeToNext[x];
                    bestX = x;
                    bestY = -1;
                }
                for (auto& f : adj[x]) {
                    int y = f.first, wy = f.second;
                    if (pos[y] < 0 && edgeToNext[y] && wx + wy + edgeToNext[y] > bestW) {
                        bestW = wx + wy + edgeToNext[y];
                        bestX = x;
                        bestY = y;
                    }
                }
            }
        }
        for (auto& e : adj[b]) edgeToNext[e.first] = 0;

        if (bestX >= 0) {
            newPath.push_back(bestX);
            pos[bestX] = 0;
            curWeight += bestW - oldW;
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

void extendBothEnds(double degPenalty, int maxTries) {
    for (int stuck = 0; stuck < 2;) {
        int prevLen = path.size();
        growEnd(degPenalty, maxTries);
        flipPath();
        stuck = ((int)path.size() > prevLen) ? 0 : stuck + 1;
    }
}

// Budowa lasu DFS i znalezienie w nim najcięższej ścieżki (średnicy drzewa)
vector<int> dfsPath(double degPenalty) {
    vector<int> parent(n, -1), parentWeight(n, 0), headIdx(n, 0), seen(n, 0);
    vector<int> bestChild1(n, -1), bestChild2(n, -1), order, stack, roots(n);
    vector<long long> maxDown1(n, 0), maxDown2(n, 0);
    vector<vector<pair<int, int>>> sortedAdj(n);

    for (int v = 0; v < n; v++) {
        for (auto& e : adj[v]) {
            int priority = -(int)(e.second - degPenalty * adj[e.first].size() + rng() % 20);
            sortedAdj[v].push_back({priority, e.first});
        }
        sort(sortedAdj[v].begin(), sortedAdj[v].end());
    }

    for (int v = 0; v < n; v++) roots[v] = v;
    shuffle(roots.begin(), roots.end(), rng);

    for (int r : roots) {
        if (!seen[r]) {
            seen[r] = 1;
            order.push_back(r);
            stack.push_back(r);
            while (!stack.empty()) {
                int v = stack.back();
                if (headIdx[v] == (int)sortedAdj[v].size()) {
                    stack.pop_back();
                    continue;
                }
                int u = sortedAdj[v][headIdx[v]++].second;
                if (seen[u]) continue;
                seen[u] = 1;
                parent[u] = v;
                parentWeight[u] = getWeight(v, u);
                order.push_back(u);
                stack.push_back(u);
            }
        }
    }

    // Programowanie dynamiczne na drzewie DFS (dwie najdłuższe gałęzie w dół z każdego u)
    for (int i = n - 1; i >= 0; i--) {
        int v = order[i], u = parent[v];
        if (u < 0) continue;
        long long d = maxDown1[v] + parentWeight[v];
        if (d > maxDown1[u]) {
            maxDown2[u] = maxDown1[u];
            bestChild2[u] = bestChild1[u];
            maxDown1[u] = d;
            bestChild1[u] = v;
        } else if (d > maxDown2[u]) {
            maxDown2[u] = d;
            bestChild2[u] = v;
        }
    }

    int bestLca = 0;
    for (int x = 0; x < n; x++) {
        if (maxDown1[x] + maxDown2[x] > maxDown1[bestLca] + maxDown2[bestLca]) {
            bestLca = x;
        }
    }

    vector<int> res;
    for (int x = bestChild2[bestLca]; x >= 0; x = bestChild1[x]) res.push_back(x);
    reverse(res.begin(), res.end());
    for (int x = bestLca; x >= 0; x = bestChild1[x]) res.push_back(x);
    return res;
}

int main() {
    scanf("%d %d", &n, &m);
    adj.resize(n);
    pos.assign(n, -1);
    edgeToNext.assign(n, 0);

    for (int i = 0, u, v, w; i < m; i++) {
        scanf("%d %d %d", &u, &v, &w);
        if (u != v) {
            adj[u].push_back({v, w});
            adj[v].push_back({u, w});
        }
    }

    // Usunięcie krawędzi wielokrotnych (zostaje najcięższa) i posortowanie list sąsiedztwa
    for (auto& neighbors : adj) {
        sort(neighbors.begin(), neighbors.end(), [](const pair<int, int>& x, const pair<int, int>& y) {
            return x.first != y.first ? x.first < y.first : x.second > y.second;
        });
        neighbors.erase(unique(neighbors.begin(), neighbors.end(), [](const pair<int, int>& x, const pair<int, int>& y) {
            return x.first == y.first;
        }), neighbors.end());
    }

    double penalties[] = {0, 5, 15, 30};
    vector<int> bestPath = {0};
    long long bestWeight = 0;

    // Faza 1: Konstrukcja początkowych ścieżek i wybór najlepszej
    for (int t = 0; t < 8 && elapsed() < TIME_LIMIT / 4; t++) {
        double penalty = penalties[t % 4];
        vector<int> initPath = dfsPath(penalty);
        long long w = 0;
        for (int i = 0; i + 1 < (int)initPath.size(); i++) {
            w += getWeight(initPath[i], initPath[i + 1]);
        }
        setPath(initPath, w);
        extendBothEnds(penalty, BUILD_TRIES);
        for (int r = 0; r < 5 && improve(0) && elapsed() < TIME_LIMIT / 4; r++) {
            extendBothEnds(penalty, BUILD_TRIES);
        }
        if (curWeight > bestWeight) {
            bestWeight = curWeight;
            bestPath = path;
        }
    }

    // Faza 2: Iteracyjne przeszukiwanie lokalne (częściowe cofanie i odbudowa ścieżki)
    setPath(bestPath, bestWeight);
    int iter = 0;
    while (elapsed() < TIME_LIMIT && ++iter) {
        if (rng() % 2) {
            flipPath();
            reverse(bestPath.begin(), bestPath.end());
        }
        minDirtyIdx = path.size();
        if (rng() % 2) {
            rotateEnd();
        }

        int keepIdx = max(0, (int)path.size() - 2 - (int)(rng() % (rng() % 20 ? 100 : 5000)));
        while (keepIdx > 0 && freeDeg(path[keepIdx]) == 0) {
            keepIdx--;
        }
        while ((int)path.size() > keepIdx + 1) {
            popVertex();
        }

        minDirtyIdx = min(minDirtyIdx, keepIdx + 1);
        growEnd(penalties[rng() % 4], KICK_TRIES);

        if (iter % 100 == 0) {
            minDirtyIdx = 0;
            improve(0);
        } else {
            improve(max(max(minDirtyIdx - 2, (int)path.size() - 1000), 0));
        }

        int syncFrom = max(minDirtyIdx - 3, 0);
        if (curWeight >= bestWeight) {
            bestWeight = curWeight;
            bestPath.resize(path.size());
            copy(path.begin() + syncFrom, path.end(), bestPath.begin() + syncFrom);
        } else {
            setPath(bestPath, bestWeight, syncFrom);
        }
    }

    printf("%d\n", (int)bestPath.size());
    for (int v : bestPath) printf("%d ", v);
    printf("\n");
}