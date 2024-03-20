#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <map>
#include <algorithm>
#include <complex>
//#include <bits/stdc++.h>
#include "Graph/Graph.h"

using namespace std;

vector<string> explode(const string &str, const char &ch) {
    string next;
    vector<string> result;

    // For each character in the string
    for (string::const_iterator it = str.begin(); it != str.end(); it++) {
        // If we've hit the terminal character
        if (*it == ch) {
            // If we have some characters accumulated
            if (!next.empty()) {
                // Add them to the result vector
                result.push_back(next);
                next.clear();
            }
        } else {
            // Accumulate the next character into the sequence
            next += *it;
        }
    }
    if (!next.empty())
        result.push_back(next);
    return result;
}


class City {
public:
    City(const string &name, int x, int y) : name_(name), x_(x), y_(y) {}

public:
    string name_;
    int x_;
    int y_;
};

int main() {
    Graph<int, int> g;
    for (size_t i = 0; i < 2; ++i) {
        int start, end, w;
        cin >> start >> end >> w;
        g.add_vertex(start, start).add_vertex(end, end).add_edge(start, end, w);
    }

    auto p = g.get_shortest_path(1, 2);
    for (auto i: p.path) {
        cout << i << " ";
    }

}
