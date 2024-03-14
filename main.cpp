#include <iostream>
#include <bits/stdc++.h>
#include "Edge/Edge.h"
#include "Node/Node.h"

using namespace std;

struct Point {
    Point(int x, int y) : x(x), y(y) {}

    int x = 0;
    int y = 0;
};


class City {
public:
    City(const string &name, int x, int y) : name_(name), point_(x, y) {}

public:
    string name_;
    Point point_;
};

template<typename Id>
bool find_path(map<Id, Node<Id> *> graph, Id start, Id end) {
    static set<Id> visited;

    if (start == end) {
        return true;
    }

    visited.insert(start);

    for (Edge<Id> *edge : graph.at(start)->children) {
        Id next_vertex = edge->adjacent_node_->id_;
        if (visited.find(next_vertex) == visited.end()) {
            if (find_path(graph, next_vertex, end)) {
                return true;
            }
        }
    }
    return false;
}

const int INF = 1e9;

template<typename Id>
map<Id, int> bfs(map<Id, Node<Id> *> graph, Id start) {
    queue<Id> next_vertexes;
    map<Id, int> distances;
    for (const auto &node : graph) {
        distances[node.first] = INF;
    }

    distances[start] = 0;
    next_vertexes.push(start);

    while (!next_vertexes.empty()) {
        Id curr_id = next_vertexes.front();
        next_vertexes.pop();

        for (Edge<Id> *edge : graph[curr_id]->children) {
            Id next_id = edge->adjacent_node_->id_;
            if (distances[next_id] == INF) {
                next_vertexes.push(next_id);
                distances[next_id] = distances[curr_id] + 1;
            }
        }
    }

    return distances;
}


int main() {
    map<string, Node<string> *> graph;

    for (size_t i = 0; i < 8; ++i) {
        string dep, arrive;
        int weight;
        cin >> dep >> arrive >> weight;

        auto *node_arrive = (graph.find(arrive) != graph.end()) ? graph[arrive] : new Node<string>(arrive);
        graph[arrive] = node_arrive;

        auto *node_dep = (graph.find(dep) != graph.end()) ? graph[dep] : new Node<string>(dep);
        graph[dep] = node_dep;

        graph[dep]->children.insert(new Edge<string>(weight, node_arrive));
    }

    for (auto iter1 = graph.begin(); iter1 != graph.end(); ++iter1) {
        for (auto iter2 = iter1->second->children.begin(); iter2 != iter1->second->children.end(); ++iter2) {
            cout << iter1->first << '-' << (*iter2)->adjacent_node_->id_ << endl;
        }
    }

    cout << find_path(graph, string("AVTOZ"), string("m")) << endl;

    auto distances = bfs(graph, string("AVTOZ"));

    return 0;
}
