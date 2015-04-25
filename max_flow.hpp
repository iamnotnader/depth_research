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

static vector<int> random_indices(int N) {
  vector<int> indices(N);
  for (int i = 0; i < N; i++) {
    indices[i] = i;
  }
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
  return indices;
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

  struct dfs_stack_context {
    vector<int> indices;
    int current_index;
  };
  vector<dfs_stack_context> dfs_stack;
  visited[source->id] = true;
  dfs_stack.push_back({random_indices(source->neighbors.size()), 0});
  path->push_back(source);
  while (!dfs_stack.empty()) {
    assert(dfs_stack.size() == path->size());
    NODE* current_node = path->back();
    if (current_node == sink) {
      return true;
    }

    dfs_stack_context ctx = dfs_stack.back();
    const vector<NODE*>& neighbors = current_node->neighbors;
    const vector<EdgeCapacity<EdgeType>>& weights = current_node->weights;

    // Always trying to visit the sink first provides a small speed-up.
    if (!visited[sink->id]) {
      for (int ind = ctx.current_index; ind < ctx.indices.size(); ind++) {
          int i = ctx.indices[ind];
          if (neighbors[i]->id != sink->id) {
            continue;
          }
          if (weights[i].residual() > 0) {
            path->push_back(neighbors[i]);
            return true;
          }
      }
    }

    for (; ctx.current_index < ctx.indices.size(); ctx.current_index++) {
      int i = ctx.indices[ctx.current_index];
      NODE* next_node = neighbors[i];
      const EdgeCapacity<EdgeType>& weight_edge = weights[i];
      if (visited[next_node->id]) {
        continue;
      }
      if (weight_edge.residual() <= 0) {
        continue;
      }
      // At this point, we have a useful neighbor that isn't the sink.
      // Mark as visited and push onto the stack.
      visited[next_node->id] = true;
      dfs_stack.push_back({random_indices(next_node->neighbors.size()), 0});
      path->push_back(next_node);
      break;
    }
    if (ctx.current_index >= ctx.indices.size()) {
      dfs_stack.pop_back();
      path->pop_back();
      continue;
    }
  }
  if (dfs_stack.empty()) {
    return false;
  }
  return true;
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
