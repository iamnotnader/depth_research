#ifndef GRAPH_UTILS
#define GRAPH_UTILS

#include <vector>
#include "status_defs.h"
#include <iostream>
#include <assert.h>
#include <sstream>

namespace graph_utils {

// Node class for representing a graph.
// TODO(daddy): Make it so values are stored in a separate array rather than
// in the nodes themselves. That way more nodes can fit in memory/cache.
template<typename ValueType, typename WeightType>
class Node {
public:
  Node(ValueType value, int id) : value(value), id(id) { }

  // WARNING: This method does not check for duplicate neighbors.
  // For that, use add_neighbor_safe.
  void add_neighbor(Node<ValueType,WeightType>* node, WeightType weight);

  void add_neighbor_safe(Node<ValueType,WeightType>* node, WeightType weight);

  void remove_neighbor(Node<ValueType,WeightType>* node);
  
  void remove_neighbor(int node_id);

  int id;
  ValueType value;
  std::vector<Node<ValueType,WeightType>*> neighbors;
  std::vector<WeightType> weights;
};

template<typename ValueType, typename WeightType, bool is_directed>
class Graph {
public:
  Graph(const std::vector<ValueType>& values,
        const std::vector<std::pair<int, int>>& edges,
        const std::vector<WeightType>& weights);

  void add_edge(int id1, int id2, WeightType weight);

  void remove_edge(int id1, int id2);

  bool has_edge(int id1, int id2, WeightType* weight);

  std::string to_string() const;

  friend std::ostream& operator<<(std::ostream& Str,
      const Graph<ValueType,WeightType,is_directed>& v) {
    for (int i = 0; i < v.nodes.size(); i++) {
      Str << v.nodes[i].value << ": ";
      for (auto neighbor : v.nodes[i].neighbors) {
        Str << neighbor->value << " ";
      }
      Str << std::endl;
    }
    return Str;
  }

  std::vector<Node<ValueType,WeightType>> nodes;
};

} // namespace graph_utils

#endif
