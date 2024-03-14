#ifndef GRAPHS_GRAPH_H
#define GRAPHS_GRAPH_H

#include <bits/stdc++.h>
#include "../Edge/Edge.h"
#include "../Node/Node.h"

using namespace std;

template<typename Id, typename Value>
class Graph {
private:
    using Node = Node<Id, Value>;
    using Edge = Edge<Id, Value>;

public:
    bool vertex_exists(const Id &id) {
        return adjacency_list_.find(id) != adjacency_list_.end();
    }

    vector<Id> get_all_vertexes_id() {
        vector<Id> res(adjacency_list_.size());
        size_t i = 0;
        for (const auto &map_item : adjacency_list_) {
            res[i++] = map_item.first;
        }
        return std::move(res);
    }

    Graph<Id, Value> &add_vertex(const Id &id, Value value_ptr) {
        adjacency_list_[id] = new Node(id, value_ptr);
        return *this;
    }

    Graph<Id, Value> &add_edge(const Id departure, const Id arrive, int weight) {
        if (vertex_exists(departure) && vertex_exists(arrive)) {
            Edge *edge_ptr = new Edge(adjacency_list_[arrive], weight);
            adjacency_list_[departure]->edges_.push_back(edge_ptr);
            return *this;
        }
        throw runtime_error("There is not vertex");
    }

    bool path_is_exists(const Id &start, const Id &end) {
        if (!vertex_exists(start) || !vertex_exists(end)) {
            throw runtime_error("There is not vertex");
        }

        set<Id> visited;
        queue<Id> next_vertexes;
        visited.insert(start);
        next_vertexes.push(start);

        while (!next_vertexes.empty()) {
            Node *curr_vertex = adjacency_list_[next_vertexes.front()];
            next_vertexes.pop();

            for (Edge *edge : curr_vertex->edges_) {
                const Id &id = edge->adjacent_node_->id_;
                if (id == end) {
                    return true;
                }

                if (visited.find(id) == visited.end()) {
                    visited.insert(id);
                    next_vertexes.push(id);
                }
            }
        }

        return false;

    }

    Node *operator[](const Id &id) {
        return adjacency_list_.at(id);
    }

    ~Graph() {
        while (!adjacency_list_.empty()) {
            auto node_to_del = adjacency_list_.begin();
            adjacency_list_.erase(node_to_del->first);
            delete node_to_del->second;
        }
    }

private:
    map<Id, Node *> adjacency_list_;
    static const int INF = 1e9;
};


#endif //GRAPHS_GRAPH_H
