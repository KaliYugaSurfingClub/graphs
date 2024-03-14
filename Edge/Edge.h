#ifndef GRAPHS_EDGE_H
#define GRAPHS_EDGE_H

template<typename Id>
class Node;

template<typename Id>
struct Edge {
    Edge(int weight, Node<Id> *adjacent_node) : weight_(weight), adjacent_node_(adjacent_node) {}

    int weight_;
    Node<Id> *adjacent_node_;
};


#endif
