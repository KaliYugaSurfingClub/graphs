#include <iostream>
#include <bits/stdc++.h>
#include "Edge/Edge.h"
#include "Node/Node.h"
#include "Graph/Graph.h"

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

//template<typename Id>
//bool find_path(map<Id, Node<Id> *> graph, Id start, Id end) {
//    static set<Id> visited;
//
//    if (start == end) {
//        return true;
//    }
//
//    visited.insert(start);
//
//    for (Edge<Id> *edge : graph.at(start)->edges_) {
//        Id next_vertex = edge->adjacent_node_->id_;
//        if (visited.find(next_vertex) == visited.end()) {
//            if (find_path(graph, next_vertex, end)) {
//                return true;
//            }
//        }
//    }
//    return false;
//}
//
//const int INF = 1e9;
//
//template<typename Id>
//map<Id, int> bfs(map<Id, Node<Id> *> graph, Id start) {
//    queue<Id> next_vertexes;
//    map<Id, int> distances;
//    for (const auto &node : graph) {
//        distances[node.first] = INF;
//    }
//
//    distances[start] = 0;
//    next_vertexes.push(start);
//
//    while (!next_vertexes.empty()) {
//        Id curr_id = next_vertexes.front();
//        next_vertexes.pop();
//
//        for (Edge<Id> *edge : graph[curr_id]->edges_) {
//            Id next_id = edge->adjacent_node_->id_;
//            if (distances[next_id] == INF) {
//                next_vertexes.push(next_id);
//                distances[next_id] = distances[curr_id] + 1;
//            }
//        }
//    }
//
//    return distances;
//}

Graph<string, string> f() {
    Graph<string, string> graph;

    for (size_t i = 0; i < 8; ++i) {
        string dep, arrive;
        int weight;
        cin >> dep >> arrive >> weight;

        graph
                .add_vertex(dep, dep)
                .add_vertex(arrive, arrive)
                .add_edge(dep, arrive, weight);
    }
    return graph;
}


int main() {

    Graph<string, string> graph = f();

    return 0;
}
