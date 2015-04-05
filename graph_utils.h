#ifndef GRAPH_UTILS
#define GRAPH_UTILS

#include <vector>
#include "status_defs.h"
#include <iostream>
#include <assert.h>
#include <sstream>

namespace graph_utils {

// Node class for representing a graph.
template<typename T>
class Node {
public:
  Node(T value, int id) : value(value), id(id) { }

  // WARNING: This method does not check for duplicate neighbors.
  // For that, use add_neighbor_safe.
  void add_neighbor(Node<T>* node, double weight);

  void add_neighbor_safe(Node<T>* node, double weight);

  void remove_neighbor(Node<T>* node);
  
  void remove_neighbor(int node_id);

  int id;
  T value;
  std::vector<Node<T>*> neighbors;
  std::vector<double> weights;
};

template<typename T, bool is_directed>
class Graph {
public:
  Graph(std::vector<T> values, std::vector<std::pair<int, int>> edges,
        std::vector<double> weights);

  void add_edge(int id1, int id2, double weight);

  void remove_edge(int id1, int id2);

  bool has_edge(int id1, int id2, double* weight);

  std::string to_string() const;

  friend std::ostream& operator<<(std::ostream& Str,
      const Graph<T, is_directed>& v) {
    for (int i = 0; i < v.nodes.size(); i++) {
      Str << v.nodes[i]->value << ": ";
      for (auto neighbor : v.nodes[i]->neighbors) {
        Str << neighbor->value << " ";
      }
      Str << std::endl;
    }
    return Str;
  }

  std::vector<std::unique_ptr<Node<T>>> nodes;
};

} // namespace graph_utils

#endif
