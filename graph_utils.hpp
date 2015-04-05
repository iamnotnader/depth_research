#include "graph_utils.h"

#include <vector>
#include "status_defs.h"
#include <iostream>
#include <assert.h>
#include <sstream>

using std::vector;
using status_defs::Status;
using std::cout;
using std::endl;
using std::tuple;
using std::string;
using std::pair;

namespace graph_utils {

// Node class for representing a graph.
template<typename ValueType, typename WeightType>
void Node<ValueType,WeightType>::add_neighbor(Node<ValueType,WeightType>* node,
                                              WeightType weight) {
  neighbors.push_back(node);
  weights.push_back(weight);
}

template<typename ValueType, typename WeightType>
void Node<ValueType,WeightType>::add_neighbor_safe(
    Node<ValueType,WeightType>* node, WeightType weight) {
  for (int i = 0; i < neighbors.size(); i++) {
    assert(neighbors[i]->id != node->id);
  }
  neighbors.push_back(node);
  weights.push_back(weight);
}

template<typename ValueType, typename WeightType>
void Node<ValueType,WeightType>::remove_neighbor(
    Node<ValueType,WeightType>* node) {
  int node_index = -1;
  for (int i = 0; i < neighbors.size(); i++) {
    if (neighbors[i] == node) {
      node_index = i;
      break;
    }
  }
  if (node_index != -1) {
    neighbors.erase(neighbors.begin() + node_index);
    weights.erase(weights.begin() + node_index);
  }
}

template<typename ValueType, typename WeightType>
void Node<ValueType,WeightType>::remove_neighbor(int node_id) {
  int node_index = -1;
  for (int i = 0; i < neighbors.size(); i++) {
    if (neighbors[i].id == node_id) {
      node_index = i;
      break;
    }
  }
  if (node_index != -1) {
    neighbors.erase(neighbors.begin() + node_index);
    weights.erase(weights.begin() + node_index);
  }
}

template<typename ValueType, typename WeightType, bool is_directed>
Graph<ValueType,WeightType,is_directed>::Graph(vector<ValueType> values,
    vector<pair<int, int>> edges, vector<WeightType> weights) {
  assert(edges.size() == weights.size());
  for (size_t i = 0; i < values.size(); i++) {
    nodes.push_back(std::unique_ptr<Node<ValueType,WeightType>>(
          new Node<ValueType,WeightType>(i, values[i])));
  }
  for (size_t i = 0; i < edges.size(); i++) {
    const pair<int, int>& edge = edges[i];
    add_edge(edge.first, edge.second, weights[i]);
  }
}

template<typename ValueType, typename WeightType, bool is_directed>
void  Graph<ValueType,WeightType,is_directed>::add_edge(int id1, int id2,
    WeightType weight) {
  assert(id1 < nodes.size());
  assert(id2 < nodes.size());

  auto& nodes = this->nodes;
  // If the graph is undirected, we add an edge in both directions.
  nodes[id1]->add_neighbor_safe(nodes[id2].get(), weight);
  if (!is_directed) {
    nodes[id2]->add_neighbor_safe(nodes[id1].get(), weight);
  }
}

template<typename ValueType, typename WeightType, bool is_directed>
void Graph<ValueType,WeightType,is_directed>::remove_edge(int id1, int id2) {
  nodes[id1]->remove_neighbor(nodes[id2].get());
  if (!is_directed) {
    nodes[id2]->remove_neighbor(nodes[id1].get());
  }
}

template<typename ValueType, typename WeightType, bool is_directed>
bool Graph<ValueType,WeightType,is_directed>::has_edge(int id1, int id2,
    WeightType* weight) {
  auto& n1 = this->nodes[id1];
  for (size_t i = 0; i < n1->neighbors.size(); i++) {
    if (n1->neighbors[i]->id == id2) {
      *weight = n1->weights[i];
      return true;
    }
  }
  return false;
}

template<typename ValueType, typename WeightType, bool is_directed>
string Graph<ValueType,WeightType,is_directed>::to_string() const {
  std::ostringstream oss;
  oss << *this;
  return oss.str();
}

} // namespace graph_utils
