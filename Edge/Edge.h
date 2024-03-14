#ifndef GRAPHS_EDGE_H
#define GRAPHS_EDGE_H

template<typename Id, typename Value>
class Node;

template<typename Id, typename Value>
struct Edge {
    Edge(Node<Id, Value> *adjacent_node, int weight) : adjacent_node_(adjacent_node), weight_(weight) {}

    int weight_;
    Node<Id, Value> *adjacent_node_;
};


#endif
