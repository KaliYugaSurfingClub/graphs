#ifndef GRAPHS_NODE_H
#define GRAPHS_NODE_H

#include <bits/stdc++.h>

using namespace std;

template<typename Id, typename Value>
class Edge;

template<typename Id, typename Value>
struct Node {
    Id id_;
    Value value_;
    vector<Edge<Id, Value> *> edges_;

    Node(const Id &id, Value value) : id_(id), value_(value) {}

    ~Node() {
        while (!edges_.empty()) {
            auto to_del = edges_.begin();
            edges_.erase(to_del);
            delete *to_del;
        }
    }
};


#endif //GRAPHS_NODE_H
