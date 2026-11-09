#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
using namespace std;

double TL = 25.0;
int BT = 100, KT = 20;
int n, m, lo;
long long cw;
vector<vector<pair<int, int>>> g;
vector<int> p, pos, mk;
mt19937 rng(7);
auto T0 = chrono::steady_clock::now();
double now() { return chrono::duration<double>(chrono::steady_clock::now() - T0).count(); }

int W(int a, int b) {
    auto it = lower_bound(g[a].begin(), g[a].end(), b, [](const pair<int, int>& e, int v) { return e.first < v; });
    return it != g[a].end() && it->first == b ? it->second : 0;
}

int E(int i) { return i >= 0 && i + 1 < (int)p.size() ? W(p[i], p[i + 1]) : 0; }

int freeDeg(int u) {
    int d = 0;
    for (auto& e : g[u]) d += pos[e.first] < 0;
    return d;
}

void push(int v) {
    if (!p.empty()) cw += W(p.back(), v);
    pos[v] = p.size();
    p.push_back(v);
}

int pop() {
    int v = p.back();
    cw -= E(p.size() - 2);
    pos[v] = -1;
    p.pop_back();
    return v;
}

void rev(int i, int j) {
    cw -= E(i - 1) + E(j);
    reverse(p.begin() + i, p.begin() + j + 1);
    for (int t = i; t <= j; t++) pos[p[t]] = t;
    cw += E(i - 1) + E(j);
    lo = min(lo, i);
}

void flip() { rev(0, p.size() - 1); }

void setPath(const vector<int>& np, long long w, int from = 0) {
    for (int i = from; i < (int)p.size(); i++) pos[p[i]] = -1;
    p.resize(np.size());
    copy(np.begin() + from, np.end(), p.begin() + from);
    for (int i = from; i < (int)p.size(); i++) pos[p[i]] = i;
    cw = w;
}

void grow(double c) {
    for (;;) {
        int best = -1;
        double bs = -1e18;
        for (auto& e : g[p.back()]) if (pos[e.first] < 0) {
            int u = e.first, w = e.second, d = freeDeg(u);
            double s = w - c * d - (d == 0) * 1000 + rng() % 20;
            if (s > bs) bs = s, best = u;
        }
        if (best < 0) return;
        push(best);
    }
}

bool rotate() {
    int k = p.size() - 1;
    vector<int> good, all;
    for (auto& e : g[p[k]]) {
        int i = pos[e.first];
        if (i < 0 || i >= k - 1) continue;
        all.push_back(i);
        if (freeDeg(p[i + 1]) > 0) good.push_back(i);
    }
    vector<int>& c = good.empty() ? all : good;
    if (c.empty()) return false;
    rev(c[rng() % c.size()] + 1, k);
    return true;
}

void growEnd(double c, int tries) {
    grow(c);
    for (int t = 0, mx = p.size(); t < tries && now() < TL; t++) {
        if (!rotate()) {
            if (p.size() == 1) return;
            int v = pop();
            if (!rotate()) { push(v); return; }
        }
        grow(c);
        if ((int)p.size() > mx) mx = p.size(), t = 0;
    }
}

bool improve(int from) {
    bool any = false;
    int k = p.size();
    for (int i = max(from, 1); i + 1 < k; i++) {
        int a = p[i - 1], c = p[i], b = p[i + 1], old = W(a, c) + W(c, b), cur = old, bx = -1;
        for (auto& e : g[b]) mk[e.first] = e.second;
        for (auto& e : g[a]) {
            int x = e.first, w = e.second;
            if (pos[x] < 0 && mk[x] && w + mk[x] > cur) cur = w + mk[x], bx = x;
        }
        for (auto& e : g[b]) mk[e.first] = 0;
        if (bx >= 0) pos[c] = -1, pos[bx] = i, p[i] = bx, cw += cur - old, any = true;
    }
    vector<int> q(p.begin(), p.begin() + from);
    for (int i = from; i < k; i++) {
        int a = p[i];
        q.push_back(a);
        if (i + 1 == k) break;
        int b = p[i + 1], old = W(a, b), cur = old, bx = -1, by = -1;
        for (auto& e : g[b]) mk[e.first] = e.second;
        for (auto& e : g[a]) if (pos[e.first] < 0) {
            int x = e.first, wx = e.second;
            if (mk[x] && wx + mk[x] > cur) cur = wx + mk[x], bx = x, by = -1;
            for (auto& f : g[x]) {
                int y = f.first, wy = f.second;
                if (pos[y] < 0 && mk[y] && wx + wy + mk[y] > cur) cur = wx + wy + mk[y], bx = x, by = y;
            }
        }
        for (auto& e : g[b]) mk[e.first] = 0;
        if (bx >= 0) {
            q.push_back(bx), pos[bx] = 0, cw += cur - old, any = true;
            if (by >= 0) q.push_back(by), pos[by] = 0;
        }
    }
    p = q;
    for (int i = from; i < (int)p.size(); i++) pos[p[i]] = i;
    return any;
}

void extend(double c, int tries) {
    for (int stuck = 0; stuck < 2;) {
        int k = p.size();
        growEnd(c, tries);
        flip();
        stuck = (int)p.size() > k ? 0 : stuck + 1;
    }
}

vector<int> dfsPath(double c) {
    vector<int> par(n, -1), pw(n, 0), it(n, 0), seen(n, 0), b1(n, -1), b2(n, -1), order, st, rs(n);
    vector<long long> d1(n, 0), d2(n, 0);
    vector<vector<pair<int, int>>> h(n);
    for (int v = 0; v < n; v++) {
        for (auto& e : g[v]) h[v].push_back({-(int)(e.second - c * g[e.first].size() + rng() % 20), e.first});
        sort(h[v].begin(), h[v].end());
    }
    for (int v = 0; v < n; v++) rs[v] = v;
    shuffle(rs.begin(), rs.end(), rng);
    for (int r : rs) if (!seen[r]) {
        seen[r] = 1, order.push_back(r), st.push_back(r);
        while (!st.empty()) {
            int v = st.back();
            if (it[v] == (int)h[v].size()) { st.pop_back(); continue; }
            int u = h[v][it[v]++].second;
            if (seen[u]) continue;
            seen[u] = 1, par[u] = v, pw[u] = W(v, u), order.push_back(u), st.push_back(u);
        }
    }
    for (int i = n - 1; i >= 0; i--) {
        int v = order[i], u = par[v];
        if (u < 0) continue;
        long long d = d1[v] + pw[v];
        if (d > d1[u]) d2[u] = d1[u], b2[u] = b1[u], d1[u] = d, b1[u] = v;
        else if (d > d2[u]) d2[u] = d, b2[u] = v;
    }
    int v = 0;
    for (int x = 0; x < n; x++) if (d1[x] + d2[x] > d1[v] + d2[v]) v = x;
    vector<int> a;
    for (int x = b2[v]; x >= 0; x = b1[x]) a.push_back(x);
    reverse(a.begin(), a.end());
    for (int x = v; x >= 0; x = b1[x]) a.push_back(x);
    return a;
}

int main() {
    scanf("%d %d", &n, &m);
    g.resize(n);
    pos.assign(n, -1);
    mk.assign(n, 0);
    for (int i = 0, u, v, w; i < m; i++) {
        scanf("%d %d %d", &u, &v, &w);
        if (u != v) g[u].push_back({v, w}), g[v].push_back({u, w});
    }
    for (auto& a : g) {
        sort(a.begin(), a.end(), [](const pair<int, int>& x, const pair<int, int>& y) { return x.first != y.first ? x.first < y.first : x.second > y.second; });
        a.erase(unique(a.begin(), a.end(), [](const pair<int, int>& x, const pair<int, int>& y) { return x.first == y.first; }), a.end());
    }
    double cs[] = {0, 5, 15, 30};
    vector<int> best = {0};
    long long bw = 0;
    for (int t = 0; t < 8 && now() < TL / 4; t++) {
        vector<int> d = dfsPath(cs[t % 4]);
        long long w = 0;
        for (int i = 0; i + 1 < (int)d.size(); i++) w += W(d[i], d[i + 1]);
        setPath(d, w);
        extend(cs[t % 4], BT);
        for (int r = 0; r < 5 && improve(0) && now() < TL / 4; r++) extend(cs[t % 4], BT);
        if (cw > bw) bw = cw, best = p;
    }
    setPath(best, bw);
    int it = 0;
    while (now() < TL && ++it) {
        if (rng() % 2) flip(), reverse(best.begin(), best.end());
        lo = p.size();
        if (rng() % 2) rotate();
        int i = max(0, (int)p.size() - 2 - (int)(rng() % (rng() % 20 ? 100 : 5000)));
        while (i > 0 && freeDeg(p[i]) == 0) i--;
        while ((int)p.size() > i + 1) pop();
        lo = min(lo, i + 1);
        growEnd(cs[rng() % 4], KT);
        if (it % 100 == 0) lo = 0, improve(0);
        else improve(max(max(lo - 2, (int)p.size() - 1000), 0));
        int s = max(lo - 3, 0);
        if (cw >= bw) {
            bw = cw;
            best.resize(p.size());
            copy(p.begin() + s, p.end(), best.begin() + s);
        } else setPath(best, bw, s);
    }
    printf("%d\n", (int)best.size());
    for (int v : best) printf("%d ", v);
    printf("\n");
}
