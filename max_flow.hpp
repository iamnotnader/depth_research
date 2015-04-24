#ifndef MAX_FLOW_HPP
#define MAX_FLOW_HPP

#include "max_flow.h"
#include <limits>
#include <algorithm>
#include "logging_utils.h"

namespace max_flow {

using std::cout;
using status_defs::Status;
using std::vector;
using graph_utils::Graph;
using std::pair;
using graph_utils::Node;

namespace internal { 

template<typename ValueType, typename EdgeType>
EdgeCapacity<EdgeType>* get_edge_between(
    Node<ValueType,EdgeCapacity<EdgeType>>* node_from,
    Node<ValueType,EdgeCapacity<EdgeType>>* node_to) {
  for (int i = 0; i < node_from->neighbors.size(); i++) {
    if (node_from->neighbors[i] == node_to) {
      return &(node_from->weights[i]);
    }
  }
  return nullptr;
}

template<typename ValueType, typename EdgeType>
bool find_augmenting_path_random_dfs(
    Node<ValueType,EdgeCapacity<EdgeType>>* source,
    Node<ValueType,EdgeCapacity<EdgeType>>* sink,
    vector<Node<ValueType,EdgeCapacity<EdgeType>>*>* path,
    vector<bool>& visited) {
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  if (source == nullptr || sink == nullptr) {
    return false;
  }
  path->push_back(source);
  visited[source->id] = true;
  if (source == sink) {
    return true;
  }
  vector<NODE*>& neighbors = const_cast<vector<NODE*>&>(source->neighbors);
  const vector<EdgeCapacity<EdgeType>>& weights = source->weights; 
  vector<int> indices;
  for (int i = 0; i < neighbors.size(); i++) {
    indices.push_back(i);
  }

  // Always trying to visit the sink first provides a small speed-up.
  for (size_t i = 0; i < neighbors.size(); i++) {
      if (neighbors[i]->id != 1) {
        continue;
      }
      if (weights[i].residual() > 0) {
        bool is_augmenting_path = find_augmenting_path_random_dfs(
                                      neighbors[i], sink, path, visited);
        if (is_augmenting_path) {
          return true;
        }
      }
  }
  // Visiting nodes in random order provides a huge speed-up.
  static unsigned long x=123456789, y=362436069, z=521288629;
  auto fast_random = [](int i) { 
    unsigned long t;
    x ^= x << 16;
    x ^= x >> 5;
    x ^= x << 1;

    t = x;
    x = y;
    y = z;
    z = t ^ x ^ y;

    return z % i;
  };
  std::random_shuffle(indices.begin(), indices.end(), fast_random);
  for (size_t p = 0; p < indices.size(); p++) {
    int i = indices[p];
    if (weights[i].residual() > 0 &&
        !visited[neighbors[i]->id]) {
      // explore this node
      bool is_augmenting_path = find_augmenting_path_random_dfs(
                                    neighbors[i], sink, path, visited);
      if (is_augmenting_path) {
        return true;
      }
    }
  }
  path->pop_back();
  return false;
}

// Tries to find an augmenting path. Puts the path in "path" and returns true
// if an augmenting path exists. Otherwise, leaves "path" empty and returns
// false.
template<typename ValueType, typename EdgeType>
bool find_augmenting_path(
    Node<ValueType,EdgeCapacity<EdgeType>>* source,
    Node<ValueType,EdgeCapacity<EdgeType>>* sink,
    vector<Node<ValueType,EdgeCapacity<EdgeType>>*>* path,
    vector<bool>& visited) {
  std::fill(visited.begin(), visited.end(), false);
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  return find_augmenting_path_random_dfs<ValueType, EdgeType>(
      source, sink, path, visited);
}

}

// Computes the minimum cut of an undirected graph.
// returns edges and weights by reference.
template<typename ValueType, typename EdgeType>
double compute_max_flow(Graph<ValueType,EdgeCapacity<EdgeType>,false>* g) {
  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  auto& nodes = g->nodes;
  NODE* source = &nodes[0];
  NODE* sink = &nodes[1];

  vector<NODE*> path;
  vector<bool> visited(nodes.size());
  while (true) {
    path.clear();

    bool exists_augmenting_path =
      internal::find_augmenting_path<ValueType>(source, sink, &path, visited);

    if (!exists_augmenting_path) {
      LOG3("Computing maximum flow by summing outflows from source.");
      // If no augmenting path exists, compute the maximum flow and return.
      double max_flow = 0.0;
      for (int i = 0; i < source->neighbors.size(); i++) {
        LOG3("flow: " << source->weights[i].flow << " capacity: "
             << source->weights[i].capacity << " from: " << source->id
             << " to: " << source->neighbors[i]->id);
        assert(source->weights[i].flow >= 0);
        max_flow += source->weights[i].flow;
      }
      return max_flow;
    }

    // Otherwise, push as much flow as we can through the augmenting path.
    double flow_to_add = std::numeric_limits<double>::max();

    // Find the edge with the smallest extra capacity (aka residual).
    for (int i = 0; i < path.size()-1; i++) {
      NODE* node_from = path[i];
      NODE* node_to = path[i+1];
      EdgeCapacity<EdgeType>* edge = internal::get_edge_between(node_from, node_to);

      assert(edge != nullptr);
      flow_to_add = std::min(flow_to_add, edge->residual());
    }

    // Add the flow we found to all the edges in the augmenting path.
    // Note that we need to modify BOTH the outgoing edge from each node AND
    // the incoming edge. In particular, for the outgoing edge we have:
    //    new_flow = old_flow + flow_to_add
    // For the incoming edge we have:
    //    new_flow = old_flow - flow_to_add
    for (int i = 0; i < path.size()-1; i++) {
      NODE* node_from = path[i];
      NODE* node_to = path[i+1];
      EdgeCapacity<EdgeType>* edge_forward =
          internal::get_edge_between(node_from, node_to);
      EdgeCapacity<EdgeType>* edge_reverse =
          internal::get_edge_between(node_to, node_from);

      assert(edge_forward != nullptr);
      edge_forward->flow += flow_to_add;
      
      assert(edge_reverse != nullptr);
      edge_reverse->flow -= flow_to_add;
    }
  }
}

} // namespace max_flow
#endif
