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
bool path_contains_node(
    const vector<Node<ValueType,EdgeCapacity<EdgeType>>*>& path,
    Node<ValueType,EdgeCapacity<EdgeType>>* node) {

  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  for (const NODE* n : path) {
    if (n == node) {
      return true;
    }
  }
  return false;
}

// Tries to find an augmenting path. Puts the path in "path" and returns true
// if an augmenting path exists. Otherwise, leaves "path" empty and returns
// false.
//
// TODO(daddy): Consider using bfs instead of dfs, since its choice of
// augmenting path tends to lead to fewer iterations. Also recursion is
// not as performant.
template<typename ValueType, typename EdgeType>
bool find_augmenting_path(
    Node<ValueType,EdgeCapacity<EdgeType>>* source,
    Node<ValueType,EdgeCapacity<EdgeType>>* sink,
    vector<Node<ValueType,EdgeCapacity<EdgeType>>*>* path) {

  typedef Node<ValueType,EdgeCapacity<EdgeType>> NODE;
  if (source == nullptr || sink == nullptr) {
    return false;
  }
  path->push_back(source);
  if (source == sink) {
    return true;
  }
  const vector<NODE*>& neighbors = source->neighbors;
  const vector<EdgeCapacity<EdgeType>>& weights = source->weights; 
  for (int i = 0; i < neighbors.size(); i++) {
    if (weights[i].residual() > 0 &&
        !path_contains_node(*path, neighbors[i])) {
      // explore this node
      bool is_augmenting_path = find_augmenting_path(neighbors[i], sink, path);
      if (is_augmenting_path) {
        return true;
      }
    } 
  }
  path->pop_back();
  return false;
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
  while (true) {
    path.clear();

    bool exists_augmenting_path =
      internal::find_augmenting_path<ValueType>(source, sink, &path);

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
