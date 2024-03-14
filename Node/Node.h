#ifndef GRAPHS_NODE_H
#define GRAPHS_NODE_H

#include <bits/stdc++.h>

using namespace std;

template<typename Id>
class Edge;

template<typename Id>
struct Node {
    Id id_;
    set<Edge<Id> *> children;

    Node(Id id) {
        id_ = id;
    }

    ~Node() {
        while (!children.empty()) {
            auto to_del = children.begin();
            children.erase(to_del);
            delete *to_del;
        }
    }
};


#endif //GRAPHS_NODE_H
