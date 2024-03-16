#ifndef GRAPHS_GRAPH_H
#define GRAPHS_GRAPH_H

#include <bits/stdc++.h>
#include "../Edge/Edge.h"
#include "../Node/Node.h"

using namespace std;

template<typename Id, typename Value, typename Len = int>
class Graph {
public:
//    using Node = Node<Id, Value, Len>;
//    using Edge = Edge<Id, Value, Len>;

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
                edges_.erase(to_del);
                delete *to_del;
            }
        }
    };


public:
    bool vertex_exists(const Id &id) {
        return adjacency_list_.find(id) != adjacency_list_.end();
    }

    vector<Id> get_all_vertexes_id() {
        vector<Id> res(adjacency_list_.size());
        size_t i = 0;
        for (const auto &map_item: adjacency_list_) {
            res[i++] = map_item.first;
        }
        return std::move(res);
    }

    Graph<Id, Value, Len> &add_vertex(const Id &id, Value value_ptr) {
        if (!vertex_exists(id)) {
            adjacency_list_[id] = new Node(id, value_ptr);
        }
        return *this;
    }

    Graph<Id, Value, Len> &add_edge(const Id &departure, const Id &arrive, Len weight) {
        if (vertex_exists(departure) && vertex_exists(arrive)) {
            Edge *edge_ptr = new Edge(arrive, weight);
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

            for (Edge *edge: curr_vertex->edges_) {
                const Id &id = edge->adjacent_node_;
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


    map<Id, Dijkstra_node> get_dijkstra_graph(const Id &start) {
        if (!vertex_exists(start)) {
            throw runtime_error("There is not vertex");
        }

        //todo не уверен на счет node
        priority_queue<Dijkstra_node> next_vertexes;
        map<Id, Dijkstra_node> distances;

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

    //сделать структуру
    tuple<vector<Id>, Len, bool> get_shortest_path(const Id &start, const Id &end) {
        map<Id, Dijkstra_node> distances(get_dijkstra_graph(start));
        vector<Id> res;

        if (distances.at(end).distance == INF) {
            return {res, distances.at(end).distance, true};
        }

        Id curr_vertex = distances.at(end).id;
        while (curr_vertex != start) {
            res.push_back(curr_vertex);
            curr_vertex = distances.at(curr_vertex).from;
        }

        res.push_back(start);
        reverse(res.begin(), res.end());

        return {res, distances.at(end).distance, true};
    }

    Node *operator[](const Id &id) {
        return adjacency_list_.at(id);
    }

    ~Graph() {
        while (!adjacency_list_.empty()) {
            auto node_to_del = adjacency_list_.begin();
            Node *node_ptr_to_del = node_to_del->second;
            adjacency_list_.erase(node_to_del->first);
            delete node_ptr_to_del;
        }
    }

public:
    map<Id, Node *> adjacency_list_;
    static const int INF = 1e9;
};


#endif //GRAPHS_GRAPH_H
