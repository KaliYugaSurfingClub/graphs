#ifndef GRAPHS_GRAPH_H
#define GRAPHS_GRAPH_H

#include <bits/stdc++.h>
#include "../Edge/Edge.h"
#include "../Node/Node.h"

using namespace std;

template<typename Id>
class Graph {
public:
    bool vertex_exists(Id id) {
        return adjacency_list.find(id) == adjacency_list.end();
    }

    Graph<Id> &add_vertex(Id id) {
        if (vertex_exists(id)) {
            adjacency_list[id];
            return *this;
        }
        throw runtime_error("the vertex already exists");
    }

    Graph<Id> &add_edge(Id departure, Id arrive, int weight) {
        if (vertex_exists(departure) && vertex_exists(arrive)) {
            adjacency_list[departure]->children.insert(new Edge(arrive, weight));
            return *this;
        }
        throw runtime_error("There is not vertex");
    }

private:
    map<Id, Node<Id> *> adjacency_list;
};


#endif //GRAPHS_GRAPH_H
