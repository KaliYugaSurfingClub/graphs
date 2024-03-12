#include <iostream>
#include <bits/stdc++.h>

using namespace std;

class User {
public:
    User(const string &name, int age) : name_(name), age_(age) {}
    string name_;
    int age_;
};

template<typename T>
class Graph {
public:
    void add(T *node1, T *node2) {
        ad_list_[node1].push_back(node2);
        ad_list_[node2].push_back(node1);
    }

    ~Graph() {
        for (auto iter = ad_list_.cbegin(); iter != ad_list_.cend(); ++iter) {
            delete iter->first;
        }
    }

private:
    unordered_map<T*, list<T*>> ad_list_;
};


int main() {
    const vector<User*> users_vec = {
        new User("sasha", 0),
        new User("sda", 1),
        new User("ror", 2),
        new User("asdfd", 3),
        new User("svd", 4),
        new User("sadf", 5),
        new User("sadg", 6),
        new User("sdfa", 7),
    };

    Graph<User> graph;
    vector<pair<int, int>> vec = {
        {0, 4}, {4, 1}, {2, 1}, {5, 2},
        {4, 5}, {4, 6}, {7, 5}, {6, 7}
    };

    for (const auto &pa : vec) {
        graph.add(users_vec[pa.first], users_vec[pa.second]);
    }

    return 0;
}
