#ifndef GRAPHS_GRAPH_H
#define GRAPHS_GRAPH_H

#include <bits/stdc++.h>
#include "../Edge/Edge.h"
#include "../Node/Node.h"

using namespace std;

//специализация для int
template<typename Id, typename Value, typename Len = int>
class Graph {
public:
    struct Edge {
        Edge(Id adjacent_node, Len weight) : adjacent_node_(adjacent_node), weight_(weight) {}

        Len weight_;
        Id adjacent_node_;
    };

    struct Node {
        Id id_;
        Value value_;
        vector<Edge *> edges_;

        Node(const Id &id, Value value) : id_(id), value_(value) {}

        ~Node() {
            while (!edges_.empty()) {
                auto to_del = edges_.begin();
                Edge *ptr_t_del = *to_del;
                edges_.erase(to_del);
                delete ptr_t_del;
            }
        }
    };

    struct Dijkstra_node {
        Len distance = INF;
        Id from;
        Id id;

        Dijkstra_node() = default;

        Dijkstra_node(Len d, Id f, Id i) : distance(d), id(i), from(f) {}


        bool operator>(const Dijkstra_node &other) const {
            return this->distance < other.distance;
        }

        bool operator<(const Dijkstra_node &other) const {
            return this->distance > other.distance;
        }
    };

    struct Shortest_path {
        vector<Id> path;
        Len length;
        bool exists;
    };


public:
    bool vertex_exists(const Id &id);

    Graph<Id, Value, Len> &add_vertex(const Id &id, Value value_ptr);
    Graph<Id, Value, Len> &add_edge(const Id &departure, const Id &arrive, Len weight);

    bool path_exists(const Id &start, const Id &end);
    map<Id, Dijkstra_node> get_dijkstra_graph(const Id &start);
    Shortest_path get_shortest_path(const Id &start, const Id &end);

    Node *operator[](const Id &id) {
        return adjacency_list_.at(id);
    }

    ~Graph();

private:
    map<Id, Node *> adjacency_list_;
    static const int INF = 1e9;
};

template<typename Id, typename Value, typename Len>
Graph<Id, Value, Len>::~Graph() {
    while (!adjacency_list_.empty()) {
        auto node_to_del = adjacency_list_.begin();
        Node *node_ptr_to_del = node_to_del->second;
        adjacency_list_.erase(node_to_del->first);
        delete node_ptr_to_del;
    }
}

template<typename Id, typename Value, typename Len>
bool Graph<Id, Value, Len>::vertex_exists(const Id &id) {
    return adjacency_list_.count(id);
}

template<typename Id, typename Value, typename Len>
bool Graph<Id, Value, Len>::path_exists(const Id &start, const Id &end) {
    if (!vertex_exists(start) || !vertex_exists(end)) {
        return false;
    }

    set<Id> visited;
    queue<Id> next_vertexes;
    visited.insert(start);
    next_vertexes.push(start);

    while (!next_vertexes.empty()) {
        Node *curr_vertex = adjacency_list_[next_vertexes.front()];
        next_vertexes.pop();

        for (Edge *edge : curr_vertex->edges_) {
            const Id &id = edge->adjacent_node_;
            if (id == end) {
                return true;
            }

            if (!visited.count(id)) {
                visited.insert(id);
                next_vertexes.push(id);
            }
        }
    }

    return false;

}

template<typename Id, typename Value, typename Len>
Graph<Id, Value, Len> &
Graph<Id, Value, Len>::add_edge(const Id &departure, const Id &arrive, Len weight) {
    if (vertex_exists(departure) && vertex_exists(arrive)) {
        Edge *edge_ptr = new Edge(arrive, weight);
        adjacency_list_[departure]->edges_.push_back(edge_ptr);
        return *this;
    }
    throw runtime_error("There is not vertex (add edge)");
}

template<typename Id, typename Value, typename Len>
Graph<Id, Value, Len> &
Graph<Id, Value, Len>::add_vertex(const Id &id, Value value_ptr) {
    if (!vertex_exists(id)) {
        adjacency_list_[id] = new Node(id, value_ptr);
    }
    return *this;
}

template<typename Id, typename Value, typename Len>
typename Graph<Id, Value, Len>::Shortest_path
Graph<Id, Value, Len>::get_shortest_path(const Id &start, const Id &end) {
    if (!vertex_exists(start) || !vertex_exists(end)) {
        throw runtime_error("There is not vertex (dij)");
    }

    map<Id, Dijkstra_node> distances(get_dijkstra_graph(start));
    vector<Id> res;

    if (distances.at(end).distance == INF) {
        return {res, distances.at(end).distance, false};
    }

    Id curr_vertex = distances.at(end).id;
    while (curr_vertex != start) {
        res.push_back(curr_vertex);
        curr_vertex = distances.at(curr_vertex).from;
    }

    res.push_back(start);
    reverse(res.begin(), res.end());

    return {std::move(res), distances.at(end).distance, true};
}

template<typename Id, typename Value, typename Len>
map<Id, typename Graph<Id, Value, Len>::Dijkstra_node>
Graph<Id, Value, Len>::get_dijkstra_graph(const Id &start) {
    //todo не уверен на счет node
    priority_queue<Dijkstra_node> next_vertexes;
    map<Id, Dijkstra_node> distances;
    for (auto i: adjacency_list_) {
        distances[i.first];
    }

    next_vertexes.emplace(0, start, start);
    distances[start] = {0, start, start};

    while (!next_vertexes.empty()) {
        Dijkstra_node nearest_dij_node = next_vertexes.top();
        Len distance_to_nearest = nearest_dij_node.distance;
        Id nearest_id = nearest_dij_node.id;
        next_vertexes.pop();

        for (Edge *edge: adjacency_list_[nearest_id]->edges_) {
            const Id &id = edge->adjacent_node_;
            const Len new_distance = distance_to_nearest + edge->weight_;

            if (distances[nearest_id].distance != distance_to_nearest) {
                continue;
            }

            if (distances[id].distance > new_distance) {
                distances[id] = {new_distance, nearest_id, id};
                next_vertexes.emplace(new_distance, id, id);
            }
        }
    }

    return std::move(distances);
}

#endif //GRAPHS_GRAPH_H
