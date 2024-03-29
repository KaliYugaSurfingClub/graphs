#include <bits/stdc++.h>
#include "Graph/Graph.h"

using namespace std;

vector<Graph<int, int>>  f() {
    vector<Graph<int, int>> vec;
    Graph<int, int> g;
    g.add_vertex(1, 1);
    vec.push_back(std::move(g));
    return vec;
}

int main() {
    vector<Graph<int, int>> g = f();

    return 0;
}
